
#include <Arduino.h>
// #include <map>
// #include <type_traits>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <avr/eeprom.h>
// #include <EEPROM.h> //Arduino
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include <math.h>
#include <compat/twi.h>
#include <string.h>
#include <MCP4725.h> //20240625 https://github.com/RobTillaart/MCP4725
#include <Wire.h>
// #include <wiringPiI2C.h>
// #include "i2cmaster.h"
// #include "vars_main.h"
#include "FieldDeviceProperty.h"
#include "shared_Vars.h"
#include "pin_mappings.h"
#include "const.h"
#include "LoadDefs.h"
// #include "Execute_App_Command_Module/Execute_App_Command_Module_Rev_2_00.h"
// #include "Validate_EERAM_Module/Validate_EERAM_Module.h"
// twi.h has #define TW_STATUS		(TWSR & TW_STATUS_MASK).  But TWSR, applicable for ATmega328P should be TWSR0 for ATmega328PB.
#undef TW_STATUS
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)

// Function to get and clear the MCUSR register and disable the watchdog timer.  Ref https://www.avrfreaks.net/s/topic/a5C3l000000Ua0BEAS/t152185
void __attribute__((section(".init3"), used, naked)) disable_watchdog()
{
    MCUSR = 0;     // Clear the reset flags
    wdt_disable(); // Disable the watchdog timer
}

MCP4725 DAC(0x60); // (MCP4725ADD>>1);
uint8_t printCnter = 0;
uint16_t eeprom_address = 0;

// volatile uint16_t n = 0;  //Counter used in debugging.
uint16_t Boardrevision; // Board that program will be deployed to
int X = 0;              // X position requested to move EG X=100   .Move X 100 steps
int Y = 0;              // Y position requested to move EG X=100   .Move X 100 steps
int AbsX = 0;           // X absolute position from home position
int AbsY = 0;           // Y absolute position from home position
#ifdef ISOLATED_BOARD
bool isolated_board_flag = false;
uint16_t isolated_board_factor = 1;
#endif
bool printPos = true; // Use this to determine whether or not to print series of X and Y values while in run mode - mostly (?only) for testing
bool audioOn = true;  //
int Dy;               // Used to calulate how many steps required on Y axis from last position
int Dx;               // Used to calulate how many steps required on X axis from last position

bool rndLadBit; // 0 or 1 to indicate if last pass was for a new rung on one side or the next side is needed
uint8_t minYind;
uint8_t maxYind;
uint8_t RndNbr; //, PrevRndNbr, AltRndNbr;

volatile uint16_t StepCount;          // How many steps to be executed
uint16_t DSS_preload = Step_Rate_Max; // TCC1 compare value to get Desired Stepping Speed (DSS), Lower the number the faster the pulse train
volatile bool SteppingStatus = 0;
// bool SteppingStatus = 0;  //Shared   // 0 = No stepping in progress. 1= Stepping in progress
char buffer[20]; // Use for printint to serial (Bluetooth)
// char CurrentBuffer[BUFFER_SIZE]; //20240924 - for double buffering bluetooth messages
// char ProcessingBuffer[BUFFER_SIZE];
#ifndef NDEBUG
char debugMsg[DEBUG_MSG_LENGTH]; // Buffer for debug messages
#endif
char OpModeTxt[12];
int StepOverRatio;  // Step over ratio between Pan And tilt
uint16_t Remainder; // Used for Step Over Ratio calculation
bool Master_dir;    // Which motor need to step the most X or Y
// FstTick, OnTicks, OffTicks, AudioLength used for buzzer.
volatile uint16_t FstTick;
uint8_t OnTicks = 0;
uint8_t OffTicks = 0;
uint8_t AudioLength = 0;
uint8_t PrevAudioLength = 0; // For testing
uint16_t TJTick;
uint8_t Tick;       // Tick flag. 2Hz update time
uint8_t AccelTick;  // Tick flag. 2Hz update time
uint32_t Tod_tick;  // Tick for system time of day
uint8_t BuzzerTick; // Tick flag. 2Hz update time
uint8_t LaserTick;  // Tick flag. 20Hz update time

uint8_t Command;      // Holds the Command value fron the RS232 comms string.See documentation on the Comms data string build
uint16_t Instruction; // Holds the data value fron the RS232 comms string.See documentation on the Comms data string build

EramPos EEMEM EramPositions[MAX_NBR_MAP_PTS]; // Use eeprom for waypoints
uint8_t EEMEM EramMapTotalPoints;
uint8_t MapTotalPoints;
bool X_TravelLimitFlag = false; // Over travel flag
bool Y_TravelLimitFlag = false; // Over travel flag
// Use JogFlag to stop jogging if device is out of range.  0: allow jogging.  1: AbsX>X_MAXCOUNT; 2: AbsX<>>X_MINCOUNT; 3: AbsY > y_maxcount; 4: AbsY < 0;
uint8_t JogFlag = 0;
uint16_t y_maxCount = 500;           // = 1800;
int y_minCount = 20;                 // -220
uint8_t MapCount[2][NBR_ZONES] = {}; // MapCount[1][i] is incremental; MapCount[2][i] is cumulative of MapCount[1][i]
uint8_t Zn;
// uint8_t NoMapsRunningFlag;  //Although this is set (BASCOM), it doesn't appear to be used.
// uint16_t AppCompatibilityNo;
uint8_t GyroAddress = MPU6000_ADDRESS; // 0x69 for Board 6.13 and later (MPU6050).  0x68 for earlier boards (MPU6000).  Set with <11:4> (0x69) and <11:8> (0x69) and store in EramGyroAddress
bool GyroOnFlag = true;                // 20241203. Changed to true (was false). 20240722: Add to facilitate startup with different boards.
// Note that these addresses are 7 bit addresses.  With r/w bits these would be 0xD2/0xD3 for 0x69 and 0xD0/0xD1 for 0x68.
uint8_t EEMEM EramGyroAddress;

//----Set up for Watchdog timer------------------
uint8_t Wd_flag; // Read the flag first, as configuring the watchdog clears the WDRF bit in the MCUSR register
uint8_t Wd_byte;

//---------Commands from Bluetooth input--------------
uint8_t PanEnableFlag;  // X enable axis flag when jogging mode
uint8_t PanDirection;   // X rotation requested direction flag
uint8_t PanSpeed;       // X stepping speed flag... 1=fast 0=slow
uint8_t TiltDirection;  // Y rotation requested direction flag
uint8_t TiltEnableFlag; // Y enable axis flag when jogging mode
uint8_t TiltSpeed;      // Y stepping speed flag... 1=fast 0=slow

//---------Laser Variables------------------------
uint8_t UserLaserPower; // 20241202.  Default of 100 - could be zero?
uint8_t EEMEM EramUserLaserPower;

uint8_t MaxLaserPower;
uint8_t EEMEM EramMaxLaserPower;
uint8_t LaserPower = 0; // Final calculated value send to the DAC laser driver.  20241202 Initialise to 100.

float VoltPerStep = LINE_VOLTAGE / 4095; // Laser power per step. Could be macro constant.
// Input voltage ie 5 volts /12bit (4095) MCP4725 DAC = Voltage step per or 0.0012210012210012 Volt per step

uint8_t LaserOverTempFlag; // Laser over temp error flag
// uint8_t FlashTheLaserFlag; // Setup laser flash flag bit
bool CmdLaserOnFlag = false;

uint8_t SetupModeFlag = 0;      // Should this be set to 1 (setup mode - 0 is run mode) as default? 0 in BASCOM version.
uint8_t PrevSetupModeFlag = 10; // Initialise with silly high number so that first run will be different.
uint8_t Laser2StateFlag;        // Sets the current state if the laser 2 is working. 0=off 1=on

uint8_t EEMEM EramLaser2OperateFlag;
uint8_t Laser2OperateFlag; // Set a flag whether the user wants Laser 2 to operate  0=laser 2 off, 1=Laser2 on

// uint8_t EramLaser2TempTrip[3];
uint8_t Laser2TempTrip; // Sets the % trip value of the laser 1 temp where to turn the laser 2 on/off  100=100%,50=50% 25=25%
uint8_t EEMEM EramLaser2TempTrip;

uint8_t Laser2TempTripFlag;

uint8_t Laser2BbattTrip; // Sets the % trip value of the battery charge where to turn the laser 2 on/off  100=100%,50=50%,25=25%
uint8_t EEMEM EramLaser2BattTrip;

uint8_t Laser2BattTrip;
uint8_t Laser2BattTripFlag;

uint8_t MicroMajor;
uint8_t EEMEM EramMicroMajor;

uint8_t MicroMinor;
uint8_t EEMEM EramMicroMinor;

//----------Communication Variables--------------------
uint8_t A;
char S[10]; // 20240607: This should only need to be char[4] // Variables to decode data coming in from the RS232 data stream from the phone Commands

//----------Speed Zone Variables--------------------
// uint8_t SpeedZone[5];
// uint8_t EEMEM EramSpeedZone[5];

//----------Battery Charge Variables--------------------
uint16_t BattReadings[10];
uint8_t BattReadIndex;
uint16_t BattTotal;
uint16_t BattVoltAvg;
uint8_t BatteryVoltage; // Change this to single if you want to calc voltage on the micro
uint8_t BatteryTick;    // Battery tick sampling rate tick's
uint8_t LoopCount;

//----------Temperature Variables--------------------
uint8_t LaserTemperature;

//----------Light Sensor Variables Variables--------------------
uint8_t EEMEM EramUserLightTripLevel;
uint8_t UserLightTripLevel;

uint8_t EEMEM EramFactoryLightTripLevel; // Factory Default light setting value. Laser need to go into Lightbox
uint8_t FactoryLightTripLevel;

uint8_t EramLightTriggerOperation; // 0=24hr. 1= Day Mode 2= Night Mode
uint8_t LightTriggerOperation;

int LightLevel; // Holds the value of the ADC light sensor.  20250109.  This was long, changed to int.
uint8_t LightSensorModeFlag;
//---------Other Variables-------------------------
uint8_t BT_ConnectedFlag;

uint8_t EEMEM EramOperationMode;
uint8_t OperationMode;

uint8_t EEMEM EramActiveMapZones;
uint8_t ActiveMapZones;

uint8_t EEMEM EramActivePatterns;
uint8_t ActivePatterns;

uint8_t SendDataFlag = 0;  // Diagnostic mode send data back to user
uint8_t SendSetupDataFlag; // Send config data back to user.  20240522: Not used

uint8_t FirstTimeLaserOn;
uint8_t WarnLaserOnOnce = 0; // 20240801 Default value of 0 added - perhaps it should be 1?
uint8_t IsHome;

long Secondsfromreset; // Variable to hold the reset current count
bool SystemFaultFlag;
uint8_t Index;

uint16_t EEMEM EramLaserID;
uint16_t LaserID;

int EEMEM EramAccelTripPoint; // Accelerometer trip angle value
int AccelTripPoint;

uint8_t Zonespeed;
uint16_t CurrentSpeedZone;

uint16_t MapRunning;
uint8_t PatternRunning = 0;

uint8_t Counter50ms;
// Counters to use for periodic printing while debugging.
uint32_t JM_n = 0;
// uint32_t MM_n=0;

//---------MPU6050 IMU  & Temperature, then HC-05 bluetooth module-----------------
union
{                // Accel_Z to be used to access any of Z_accel, int or the subcomponents
    int Z_accel; // Accelerometer uses 2 bytes of memory. Data is read from the chip in 2 lots of 1 byte packets
    struct
    {
        uint8_t Zl_accel; // Stored memory of first byte data from accelerometer
        uint8_t Zh_accel; // Stored memory of second byte data from accelerometer
    };
} Accel_Z;

bool Z_AccelFlag = false;
bool Z_AccelFlagPrevious = false;

union
{
    int Acceltemp;
    struct
    {
        uint8_t L_acceltemp;
        uint8_t H_acceltemp;
    };
} Accel;
// int Acceltemp;  //Could have used a union/struct approach to store Acceltemp as an int and the L and H parts as bytes in the same memory
// uint8_t L_acceltemp;
// uint8_t H_acceltemp;

// Stack variables not used in C implementation.  Leave them in with place holder values so as not (initially) to break something elsewhere
// uint16_t Hw_stack;
// uint16_t Sw_stack;
// Hw_stack = 1;
// Sw_stack = 1;
// uint16_t Frame_size;
uint16_t ResetSeconds;
bool received39;
int Vertices[3][MAX_NBR_VERTICES]; // Columns: X, Y, Slope?
// int Perimeter[2][MAX_NBR_PERIMETER_PTS];
// uint8_t NbrPerimeterPts;
int res[2]; // Global variables to be used in cartesian/polar conversions:Input and results.
// volatile char ReceivedData[BUFFER_SIZE];
// char RecdDataConst[BUFFER_SIZE];
// volatile bool DataInBufferFlag = false; // Flag for there is data in the RS232 comms buffer
char Buffer1[BUFFER_SIZE];
char Buffer2[BUFFER_SIZE];
volatile bool bufferFlag = false; // false -> Buffer1, true -> Buffer2
volatile bool Buffer1Ready = false;
volatile bool Buffer2Ready = false;
volatile int DataCount = 0;
volatile uint8_t CommandLength = 0;
// uint8_t cnter = 0;

// 20240726 Tuning parameters - possibly not required in production version
uint16_t EEMEM Eram_Step_Rate_Min;
uint16_t Step_Rate_Min = 2000;
uint16_t EEMEM Eram_Step_Rate_Max;
uint16_t Step_Rate_Max = STEP_RATE_MAX;
uint8_t EEMEM Eram_Rho_Min;
uint8_t Rho_Min = 10;
uint8_t EEMEM Eram_Rho_Max;
uint8_t Rho_Max = 250;
uint8_t EEMEM Eram_Nbr_Rnd_Pts;
uint8_t Nbr_Rnd_Pts = 20;
uint8_t EEMEM Eram_Tilt_Sep;
uint8_t Tilt_Sep = 5; // 20241209 Previously 1.  Needs testing.
#ifdef WIGGLY_PTS
uint8_t nbrWigglyPts = 0;
#endif
uint8_t EEMEM EramSpeedScale;
uint8_t SpeedScale = 30; // Low value sets higher interrupt rate so higher speed.  30 seems to be a good fast value. Tune with <16:percentage> message.
uint8_t EEMEM EramLaserHt;
uint8_t LaserHt = 50; // Units are decimetres.  50 => 5metres

// Debugging flags
volatile bool timer3_isr_triggered = false;
volatile bool watchdog_isr_triggered = false;

// uint8_t Max_Nbr_Perimeter_Pts = 100;
void HomeAxis();
void DoHouseKeeping();
void StopTimer1();
void Audio2(uint8_t cnt, uint8_t OnPd, uint8_t OffPd);                        // Declaration of Audio2 without debugInfo
void Audio2(uint8_t cnt, uint8_t OnPd, uint8_t OffPd, const char *debugInfo); // Declaration of Audio2, overloaded, with debugInfo
void Audio3();
void uartPrintFlash(const __FlashStringHelper *message);
void printPerimeterStuff(const char *prefix, int a, int b, uint8_t c = 0, uint8_t d = 0);
void StopSystem();
void testWatchDog(uint8_t indicator);

void setupPeripherals()
{
    // Set relevant pins as output
    DDRD |= (1 << X_ENABLEPIN) | (1 << Y_ENABLEPIN) | (1 << X_DIR) | (1 << Y_DIR) | (1 << X_STEP) | (1 << Y_STEP);
    DDRE |= (1 << BUZZER) | (1 << LASER2) | (1 << FAN);
    // Input pins
    DDRB &= ~(1 << TILT_STOP) | (1 << PAN_STOP);
    // Configure the ADC
    ADMUX = (1 << REFS0);                                              // Use AVCC as the reference voltage
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable the ADC and set the prescaler to 128 (auto)
}

void setupWatchdog()
{
    cli();       // Disable global interrupts
    wdt_reset(); // Reset the watchdog timer
    // Set up the watchdog timer for a 2-second timeout
    WDTCSR = (1 << WDCE) | (1 << WDE);                             // Enable configuration changes
    WDTCSR = (1 << WDE) | (1 << WDIE) | (1 << WDP2) | (1 << WDP1); // Set timeout to 2 seconds and enable interrupt
    // WDTCSR = (1 << WDE) | (1 << WDP2) | (1 << WDP1); // WDIE bit enables interrupt.  Try without it.
    sei(); // Enable global interrupts
    // uartPrint("Watchdog timer configured for 2-second timeout\n");
}
uint8_t IsCharWaiting()
{
    // Check if the Receive Complete (RXC0) bit is set in the UART Status Register (UCSR0A)
    // RXC0 is bit 7, so we shift right 7 places and & with 1 to get the value of that bit
    if ((UCSR0A >> 7) & 1)
    {
        return 1; // Data is available
    }
    else
    {
        return 0; // No data
    }
}
char WaitKey()
{
    while (!(UCSR0A & (1 << RXC0)))
        ;        // Wait for data to be received
    return UDR0; // Get and return received data from buffer
}
void StartBuzzerInProgMode()
{
    if (BuzzerTick == 0)
    {
        PORTE |= (1 << BUZZER); // Set BUZZER pin to HIGH
    }
    else if (BuzzerTick >= 1)
    {
        PORTE &= ~(1 << BUZZER); // Set BUZZER pin to LOW
    }
}

void TickCounter_50ms_isr()
{
    Counter50ms++;
    TJTick++;
    // if (TJTick % 40 == 0)
    // {
    //     snprintf(debugMsg, DEBUG_MSG_LENGTH, "About to call Audio3. TJTick %d", TJTick);
    //     uartPrint(debugMsg);
    // }
    // Audio3();
    if (SetupModeFlag == 1 && IsHome == 1 && WarnLaserOnOnce == 0)
    {
        // StartBuzzerInProgMode();
        BuzzerTick++;
        LaserTick++;

        if (BuzzerTick >= ONE_SECOND)
        {
            BuzzerTick = 0;
        }

        if (LaserTick >= ONE_SECOND)
        {
            LaserTick = 0;
        }
    }

    if (Counter50ms >= 10)
    {
        BatteryTick++;
        Tod_tick++;
        Tick++;
        AccelTick++;
        // FlashTheLaserFlag ^= 1;
        Secondsfromreset++;
        Counter50ms = 0;
    }
    // wdt_reset(); // Reset the watchdog timer.  Not needed. This is called in ISR(TIMER3_COMPA_vect)
}

uint16_t readADC(uint8_t channel)
{
    // Select the ADC channel
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Clear the MUX bits and set them to the channel number
    // Start a conversion
    ADCSRA |= (1 << ADSC);
    // Wait for the conversion to complete
    while (ADCSRA & (1 << ADSC))
        ;
    // Read the conversion result
    return ADC;
}

// Initialise uart and set HC-05 to connect to any address.
void uart_init(uint16_t ubrr)
{
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter and receiver interrupt.
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    // Add a delay before sending AT commands
    _delay_ms(2000);
    // Put HC-05 into AT command mode
    uartPrintFlash(F("AT\r"));
    _delay_ms(1000); // Wait for HC-05 to respond
    // Set HC-05 to master role
    uartPrintFlash(F("AT+ROLE=1\r"));
    _delay_ms(1000); // Wait for HC-05 to respond
}
// Serial write/read functions
void uartPutChar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;     // Wait for empty transmit buffer
    UDR0 = c; // Put data into buffer, sends the data
}
void uartPrint(const char *message)
{
    while (*message)
    {
        uartPutChar(*message++);
    }
    uartPutChar('\r');
    uartPutChar('\n');
    debugMsg[0] = '\0'; // Clear the global buffer
}
void uartPrintFlash(const __FlashStringHelper *message)
{
    const char *p = reinterpret_cast<const char *>(message);
    while (pgm_read_byte(p) != 0)
    {
        uartPutChar(pgm_read_byte(p++));
    }
    // uartPutChar('\r');
    // uartPutChar('\n');
}
int uartAvailable(void)
{
    return (UCSR0A & (1 << RXC0)); // Return non-zero if data is available to read
}
char uartGetChar(void)
{
    while (!(UCSR0A & (1 << RXC0)))
        ;        /* Wait for data to be received */
    return UDR0; /* Get and return received data from buffer */
}

// ISR(USART0_RX_vect) {
//     char c = UDR0;
//     if (c == '\r' || c == '\n') {
//         // Do nothing
//     } else {
//         if (DataCount == 0 && c != '<') {
//             // Do nothing
//         } else {
//             ReceivedData[DataCount] = c;
//             DataCount++;
//             if (DataCount >= BUFFER_SIZE) { // Prevent buffer overflow
//                 DataCount = 0;
//             }
//             if (c == '>') { // Set the flag when a complete command is received
//                 ReceivedData[DataCount] = '\0'; // Add null terminator
//                 DataInBufferFlag = true;
//                 CommandLength = DataCount; // Store the length of the command
//                 DataCount = 0;
//             }
//         }
//     }
// }
// 20240924.  This version to be used with double buffering CheckBlueTooth().  Reverted to old for short term.
ISR(USART0_RX_vect)
{
    char c = UDR0; // Read the received character from the UART data register
    // uartPutChar(c);  // 20240929: Echo the received data back to the serial monitor for testing.
    if (c == '\r' || c == '\n')
    {
        // Ignore carriage return and newline characters
    }
    else
    {
        char *currentBuffer = bufferFlag ? Buffer2 : Buffer1;

        if (DataCount == 0 && c != '<')
        {
            // Ignore characters until the start of a command ('<') is received
        }
        else
        {
            currentBuffer[DataCount] = c; // Store the received character in the buffer
            DataCount++;                  // Increment the buffer index

            if (DataCount >= BUFFER_SIZE)
            { // Prevent buffer overflow
                DataCount = 0;
            }

            if (c == '>')
            {                                    // Check if the end of a command is received
                currentBuffer[DataCount] = '\0'; // Add null terminator to the buffer
                if (bufferFlag)
                {
                    Buffer2Ready = true;
                }
                else
                {
                    Buffer1Ready = true;
                }
                bufferFlag = !bufferFlag; // Switch buffer
                DataCount = 0;            // Reset the buffer index
            }
        }
    }
}
// void processReceivedData() {
//     if (ReceivedData[CommandLength] == '\0') {
//         memcpy(RecdDataConst, (const char*)ReceivedData, BUFFER_SIZE);
//         uartPrint(RecdDataConst);
//     }
// }

void ProcessError()
{
    if (Z_AccelFlag)
    {
        Audio2(2, 1, 1); //,"PEAF");
        StopSystem();
        PORTE &= ~(1 << FAN);
        SetLaserVoltage(0);
        // Audio2(2,1,1);
        return;
    }
    if (LaserOverTempFlag)
    {
        Audio2(3, 1, 2); //,"PELas");
        return;
    }
    if (X_TravelLimitFlag || Y_TravelLimitFlag) // 20250114 These are never set (except initialised to false)
    {
        uartPrintFlash(F("TravelLimit breached \n"));
        Audio2(4, 1, 1); //,"PETravel");
        return;
    }
}

void ProcessBuffer(char *buffer)
{
    // Trim leading and trailing whitespace
    char *start = buffer;
    while (isspace(*start))
        start++;
    char *end = start + strlen(start) - 1;
    while (end > start && isspace(*end))
        end--;
    *(end + 1) = '\0';

    char *token = strchr(start, '<'); // Find the start of the command
    while (token != NULL)
    {
        char *end = strchr(token, '>'); // Find the end of the command
        if (end != NULL)
        {
            *end = '\0';                      // Replace '>' with '\0' to end the string
            char *colon = strchr(token, ':'); // Find the start of the instruction
            if (colon != NULL)
            {
                *colon = '\0';                 // Replace ':' with '\0' to separate command and instruction
                Command = atoi(token + 1);     // Convert the command to an integer
                Instruction = atoi(colon + 1); // Convert the instruction to an integer

                // snprintf(debugMsg, DEBUG_MSG_LENGTH, "Cmd: %d, Inst: %d", Command, Instruction);
                // uartPrint(debugMsg);
                DecodeCommsData(); // Process the command and instruction
                // uartPrintFlash(F("Finished DecodeCommsData \n"));
            }
            token = strchr(end + 1, '<'); // Find the start of the next command
        }
        else
        {
            break; // No more complete commands in the buffer
        }
    }
    memset(buffer, 0, BUFFER_SIZE);
}

void CheckBlueTooth()
{
    if (Buffer1Ready || Buffer2Ready)
    { // We have got something
        if (Buffer1Ready)
        {
            // uartPrint("Buffer1Ready");
            ProcessBuffer(Buffer1);
            Buffer1Ready = false;
        }
        if (Buffer2Ready)
        {
            // uartPrint("Buffer2Ready");
            ProcessBuffer(Buffer2);
            Buffer2Ready = false;
        }
    }
}

void printToBT(uint8_t cmd, uint16_t inst)
{
    char printToBTMsg[20];
    sprintf(printToBTMsg, "<%02d:%04x>", cmd, inst);
    uartPrint(printToBTMsg);
    _delay_ms(20);
}

void printToBT(uint8_t cmd, int inst) // 20241225 Overload printToBT to allow int as 2nd argument.
{
    char printToBTMsg[20];
    sprintf(printToBTMsg, "<%02d:%04x>", cmd, inst);
    uartPrint(printToBTMsg);
    _delay_ms(20);
}

void printToBT(uint8_t cmd, long inst) // Overload further to cover the printToBT(25, LightLevel); case.  There's likely a better solution.
{
    char printToBTMsg[20];
    sprintf(printToBTMsg, "<%02d:%08lx>", cmd, inst);
    uartPrint(printToBTMsg);
    _delay_ms(20);
}

// Initialize I2C
void i2c_init(void)
{
    // Set the bit rate to 100 kHz
    TWSR0 = 0x00;
    TWBR0 = ((F_CPU / 100000L) - 16) / 2;
}

// Send a byte via I2C and check for errors
int i2c_wbyte(uint8_t data)
{
    TWDR0 = data;                       // Load data into the TWI data register
    TWCR0 = (1 << TWINT) | (1 << TWEN); // Start transmission
    while (!(TWCR0 & (1 << TWINT)))
        ; // Wait for transmission to complete

    // Check if the byte was sent and acknowledged
    if ((TWSR0 & 0xF8) != TW_MT_DATA_ACK)
    {
        return -1; // Error: Data not sent or not acknowledged
    }
    return 0; // Success
}

// Send a STOP condition
int i2c_stop(void)
{
    TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); // Transmit STOP condition
    while (TWCR0 & (1 << TWSTO))
        ;     // Wait for STOP condition to be transmitted
    return 0; // Assuming STOP always succeeds, might not need error handling
}

// void i2c_wbyte(uint8_t data) { //Constructed to mirror BASCOM which doesn't start or stop the bus
//     // Send data
//     TWDR0 = data;
//     TWCR0 = (1<<TWINT) | (1<<TWEN);
//     while (!(TWCR0 & (1<<TWINT)));
// }

// void i2c_init(void) {
//     // Set the bit rate to 100 kHz
//     TWSR0 = 0x00;
//     TWBR0 = ((F_CPU / 100000L) - 16) / 2;
// }

// void i2c_stop(void) {
//     // Transmit STOP condition
//     TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
//     while(TWCR0 & (1<<TWSTO));
// }

uint8_t i2c_start(uint8_t address)
{
    TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // TWINT:7; TWSTA: 5; TWEN: 2; Hence: 0b10100100  // Send START condition
    while (!(TWCR0 & (1 << TWINT)))
        ;
    TWDR0 = address;
    TWCR0 = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR0 & (1 << TWINT)))
        ;
    // Check if the device acknowledged the READ / WRITE mode
    uint8_t twst = TW_STATUS & 0xF8; // 0xF8: 11111000
    // testLoopUart("Conditional exit from i2c_start");
    if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK))
    { // TW_MT_SLA_ACK: 0x18 = 0b11100; TW_MR_SLA_ACK: 0x40 = 0b1000000;
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "i2c initiation failed");
        // uartPrint(debugMsg);
        return 1;
    }
    else
    {
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "i2c initiation succesful");
        // uartPrint(debugMsg);
        return 0;
    }
}

uint8_t i2c_rbyte(uint8_t ack)
{
    // Send ACK or NACK
    if (ack)
    {
        TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Send ACK
    }
    else
    {
        TWCR0 = (1 << TWINT) | (1 << TWEN); // Send NACK
    }
    while (!(TWCR0 & (1 << TWINT)))
        ;         // Wait for the read operation to complete
    return TWDR0; // Return the received byte
}
void StepperDriverISR()
{
    // uint8_t x_dir_state = (PIND & (1 << X_DIR)) != 0; //Could use PORTD rather rathan PIND.  But as all (that are used) are configured for output, either can be used.
    // bool x_dir_state = (PIND & (1 << X_DIR)) == 0; 20240629 Replace with something easier to follow but also opposite sign.
    bool x_dir_state = (PIND & (1 << X_DIR)) ? 1 : 0;
    // bool y_dir_state = (PIND & (1 << Y_DIR)) == 0; //20240629.  Changed != to ==
    bool y_dir_state = (PIND & (1 << Y_DIR)) ? 1 : 0;

    if (StepCount > 0)
    { // StepCount is uint16_t so can't be negative. So this is equivalent to if(StepCount !=0).
        StepCount--;
        if (Dx > Dy)
        {
            StepOverRatio += Dy;
            if (StepOverRatio >= Dx)
            {
                StepOverRatio -= Dx;
                Remainder = 0;
            }
            else
            {
                Remainder = 1;
            }
        }
        else
        {
            StepOverRatio += Dx;
            if (StepOverRatio >= Dy)
            {
                StepOverRatio -= Dy;
                Remainder = 0;
            }
            else
            {
                Remainder = 1;
            }
        }

        OCR1A = DSS_preload;

        if (SetupModeFlag == 0)
        {
            if (StepCount < START_RAMPING_COUNT)
            {
                if (OCR1A < 65525)
                {
                    OCR1A += RAMPING_STEPS;
                }
                else
                {
                    if (OCR1A > DSS_preload)
                    {
                        OCR1A -= RAMPING_STEPS;
                    }
                }
            }
        }
        // if (Remainder < 1) {20240623: Logic changed to deal with zero delta (ie one axis movement) cases.
        if (Remainder < 1 && Dx != 0 && Dy != 0)
        {
            PORTD |= (1 << X_STEP) | (1 << Y_STEP); // Turn on X and Y step pins.
            _delay_us(1);
            PORTD &= ~((1 << X_STEP) | (1 << Y_STEP)); // Turn off X and Y step pins.
            _delay_us(1);
            AbsX += x_dir_state ? 1 : -1;
            AbsY += y_dir_state ? 1 : -1;
        }
        else
        {
            if (Master_dir == 0)
            {                           // 20240606: Changed from 1 to 0.  20240619: Master_dir is now aligned with axis (where 0 = X).
                PORTD |= (1 << X_STEP); // Turn on X step pins.
                _delay_us(1);
                PORTD &= ~(1 << X_STEP); // Turn off X step pins.
                _delay_us(1);            // BASCOM used a 1us pulse.  Surely 1ms is fine?
                // AbsX += (~x_dir_state & 1) - (x_dir_state & 1);  // Update AbsX
                AbsX += x_dir_state ? 1 : -1;
            }
            else
            {
                PORTD |= (1 << Y_STEP); // Turn on Y step pins.
                _delay_us(1);
                PORTD &= ~(1 << Y_STEP); // Turn off Y step pins.
                _delay_us(1);
                // AbsY += (~y_dir_state & 1) - (y_dir_state & 1);  // Update AbsY
                AbsY += y_dir_state ? 1 : -1;
            }
        }
    }
    else
    {
        StopTimer1();
        SteppingStatus = 0;
    }
}
// Setup ISRs
volatile uint16_t timer1Counter = 0;
volatile bool TC1ind = false;
// #ifdef HOME_AXIS
// void CheckTimer1(uint8_t n, uint16_t cntThreshold)
// {
//     if (TC1ind)
//     {
//         timer1Counter++;
//         TC1ind = false;
//         if (timer1Counter > cntThreshold)
//         {
//             timer1Counter = 0;
//             snprintf(debugMsg, DEBUG_MSG_LENGTH, "CT1 n: %d", n);
//             uartPrint(debugMsg);
//         }
//     }
// }
// #endif
ISR(TIMER1_COMPA_vect)
{
    TC1ind = true;
    timer1Counter++;
    StepperDriverISR();
}
// ISR for Timer3 Compare A vector
ISR(TIMER3_COMPA_vect)
{
    wdt_reset(); // Reset the watchdog timer
    TickCounter_50ms_isr();
    timer3_isr_triggered = true; // Set flag to indicate Timer3 ISR was triggered
    // uartPrintFlash(F("T3 i\n"));
}

ISR(WDT_vect)
{
    watchdog_isr_triggered = true; // Set flag to indicate watchdog ISR was triggered
    // The system will reset after this ISR completes
    // This ISR will be called when the watchdog timer times out. Without any code, the system will reset.
}
void GetBatteryVoltage()
{
    unsigned int SensorReading = 0;
    BattTotal = BattTotal - BattReadings[BattReadIndex];
    // uartPrintFlash(F("Read ADC 1"));
    SensorReading = readADC(1);
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "SR1: %d", SensorReading);
    // uartPrint(debugMsg);

    BattReadings[BattReadIndex] = SensorReading;
    BattTotal = BattTotal + SensorReading;
    BattReadIndex++;
    if (BattReadIndex > NUM_BATT_READINGS)
    {
        BattReadIndex = 1;
    }

    BattVoltAvg = BattTotal / NUM_BATT_READINGS;

    if (BattVoltAvg <= 221)
    {
        BatteryVoltage = 98; // 9.8 Volts     =0
    }
    else if (BattVoltAvg <= 330)
    {
        BatteryVoltage = 100; // 10.0 Volts   =12%
    }
    else if (BattVoltAvg <= 430)
    {
        BatteryVoltage = 105; // 10.5 Volts   =25%
    }
    else if (BattVoltAvg <= 567)
    {
        BatteryVoltage = 110; // 11.0 Volts   =50%
    }
    else if (BattVoltAvg <= 702)
    {
        BatteryVoltage = 115; // 11.5 Volts   =65%
    }
    else if (BattVoltAvg <= 832)
    {
        BatteryVoltage = 120; // 12.0 Volts   =80%
    }
    else if (BattVoltAvg <= 939)
    {
        BatteryVoltage = 124; // 12.4 Volts   =100%
    }
    else
    {
        BatteryVoltage = 124; // 12.4 Volts
    }
}

void SetLaserVoltage(uint16_t voltage)
{
    // DAC.setVoltage(4.8); // For a 12-bit DAC, 2048 is mid-scale.  Use DAC.setMaxVoltage(5.1);
    static uint16_t prevVoltage = 0;
    uint16_t thisVoltage = voltage;
    if ((voltage < 256) && (voltage > 2))
    { // If a uint8_t value has been assigned rather than 12bit, make it 12 bit.  But not if it's zero.
        thisVoltage = (voltage << 4);
    }
#ifdef BASE_PRINT
    if (prevVoltage != thisVoltage)
    {
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "In SetLV.  voltage: %u, prevVoltage: %u", thisVoltage, prevVoltage);
        uartPrint(debugMsg);
    }
#endif
    // 20241205 Imported from BASCOM version, previously not here.
    if (voltage > 0 and BatteryTick > 4)
    { // Laser on and sample every 2 sec's
        GetBatteryVoltage();
        BatteryTick = 0;
    }
    DAC.setValue(thisVoltage);
    prevVoltage = thisVoltage;
}

void firstOn()
{
    uint8_t firstTimeOn = eeprom_read_byte(&EramFirstTimeOn);
    if (firstTimeOn == 0xFF)
    { // EEPROM is erased to 0xFF
        LoadEramDefaults();
        SetLaserVoltage(0);
        // initDACMCP4725();
        uartPrintFlash(F("AT+ENLOG0\r")); // Turn off logging
        firstTimeOn = 0;                  // 20240717: Reset first time on.  Not sure where this was done in BASCOM.
        eeprom_update_byte(&EramFirstTimeOn, 0);
    }
}

// Version without debugInfo
void Audio2(uint8_t cnt, uint8_t OnPd, uint8_t OffPd)
{
    Audio2(cnt, OnPd, OffPd, nullptr); // Call the overloaded version with nullptr for debugInfo
}
// Overloaded version with debugInfo
void Audio2(uint8_t cnt, uint8_t OnPd, uint8_t OffPd, const char *debugInfo)
{
    // void Audio2(uint8_t cnt, uint8_t OnPd, uint8_t OffPd, const __FlashStringHelper* debugInfo) {
    OnTicks = OnPd;
    OffTicks = OffPd;
    AudioLength = cnt * (OnTicks + OffTicks); // Could be cnt * OnTicks + (cnt-1)*OffTicks;
    if (FstTick == 0)
        FstTick = TJTick; // Set FstTick if starting pattern. Don't do anything if a pattern is already running.
}
void Audio3()
{ // Call this in ISR to implement buzzer when it has been setup by Audio2().
    if (TJTick % 40 == 0)
    {
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "A3 audioOn: %d, TJTick %d, FstTick %d, AudioLength %d, ", audioOn, TJTick, FstTick, AudioLength);
        uartPrint(debugMsg);
    }
    if (audioOn)
    {
        static bool BuzzerOn = false;
        if (PrevAudioLength != AudioLength)
        {
            PrevAudioLength = AudioLength;
        }
        // If FstTick is not set, there's nothing to do.
        if ((FstTick == 0) || (TJTick - FstTick >= AudioLength) || (TJTick < FstTick))
        { // If the pattern has completed or TJTick has rolled over to zero, reset FstTick to zero and return.
            FstTick = 0;
            PORTE &= ~(1 << BUZZER); // BUZZER LOW
            return;
        }

        if ((TJTick - FstTick) % (OnTicks + OffTicks) <= OnTicks)
        {
            if (!BuzzerOn)
            {
                BuzzerOn = true;
            }
        }
        else
        {
            if (BuzzerOn)
            {
                BuzzerOn = false;
            }
        }
        if (BuzzerOn)
            PORTE |= (1 << BUZZER); // BUZZER HIGH
        else
            PORTE &= ~(1 << BUZZER); // BUZZER LOW
    }
    else
    {
        PORTE &= ~(1 << BUZZER); // BUZZER LOW
    }
}

void WaitAfterPowerUp()
{
    if (FirstTimeLaserOn == 1)
    {
        _delay_ms(5); // 20240717.  What's this for?
        FirstTimeLaserOn = 0;
    }
}
// Sensor interactions
void WarnLaserOn()
{
    if (WarnLaserOnOnce == 1)
    {
        Audio2(2, 1, 8, "WLO");
        WarnLaserOnOnce = 0;
    }
}

uint8_t ProgLaserPower(uint8_t lp)
{
    if (PROGMODE_EXTRA_LASERPOWER + lp > 255)
        return 255;
    else
        return LaserPower + PROGMODE_EXTRA_LASERPOWER;
}

void StartLaserFlickerInProgMode()
{
    if (LaserTick == 0)
    {
        SetLaserVoltage(0); // Off 150ms
    }
    else if (LaserTick == 3)
    {
        SetLaserVoltage(ProgLaserPower(LaserPower));
        Audio2(2, 2, 1); //,"Flick");
    }
}

void GetLaserTemperature()
{
    unsigned int SensorReading = 0;
    unsigned int Result = 0;
    uint8_t LoopCount = 0;

    for (LoopCount = 1; LoopCount <= NUM_TEMP_READINGS; LoopCount++)
    {
        // uartPrintFlash(F("Read ADC 0 "));
        SensorReading = readADC(0);
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "SR0: %d", SensorReading);
        // uartPrint(debugMsg);
        Result += SensorReading;
    }

    Result /= NUM_TEMP_READINGS; // calculate the average:

    // Calculate the LaserTemperature directly from the Result using the linear function
    // Ref sheet LaserTemp of alg.xlsm.  That shows a close to linear relationship between the previously
    // used lookups.
    // y = 0.1114x - 33.316
    // R² = 0.9923
    // Also see C:\D\NonSML\AviTech\Avitech_Flexware\FromGM\LaserTempReading.veg or .mp4 for calibration.
    LaserTemperature = 0.1114 * Result - 33.316;

    // Ensure the LaserTemperature is within the valid range
    if (LaserTemperature < 25)
    {
        LaserTemperature = 25;
    }
    else if (LaserTemperature > 60)
    {
        LaserTemperature = 60;
    }
}

void ReadAccelerometer()
{
    static uint16_t cnt = 0;
    cnt++;
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x3F);                          // 0x3F is the high byte of the Z accelerometer register
    Wire.endTransmission(false);               // false to send a restart, keeping the connection active
    Wire.requestFrom(GyroAddress, (uint8_t)4); // Request 4 bytes: Z high, Z low, Temp high, Temp low
    if (Wire.available() == 4)
    {                                    // If 4 bytes were returned
        Accel_Z.Zh_accel = Wire.read();  // Read Z high byte
        Accel_Z.Zl_accel = Wire.read();  // Read Z low byte
        Accel.H_acceltemp = Wire.read(); // Read Temp high byte
        Accel.L_acceltemp = Wire.read(); // Read Temp low byte
    }
    // Convert temperature from raw values to degrees Celsius
    int16_t rawTemp = (Accel.H_acceltemp << 8) | Accel.L_acceltemp;
    Accel.Acceltemp = (rawTemp / 340.0) + 36.53;
    // if(!(cnt % 10000)){
    //     snprintf(debugMsg, DEBUG_MSG_LENGTH,"Accel %d, Accel h %02x, l %02x, Temp H %02x, L %02x", Accel_Z.Z_accel, Accel_Z.Zh_accel, Accel_Z.Zl_accel, Accel.H_acceltemp, Accel.L_acceltemp);
    //     uartPrint(debugMsg);
    // }
}
void DecodeAccelerometer()
{
    static uint16_t DA_cnt = 0;
    static uint16_t DA_cnt2 = 0;
    bool printFlag = false;
    if (GyroOnFlag)
    {
        if (OperationMode == 0) // OperationMode == 0 is field i.
        {                       // Field-1
            DA_cnt++;
            if (Accel_Z.Z_accel < AccelTripPoint)
            {
                DA_cnt2++;
                Z_AccelFlag = true;
                SystemFaultFlag = true;
#ifndef ISOLATED_BOARD
                snprintf(debugMsg, DEBUG_MSG_LENGTH, "ATP err DA_cnt: %u, DA_cnt2: %u, Z: %d, ATP: %d", DA_cnt, DA_cnt2, Accel_Z.Z_accel, AccelTripPoint);
                uartPrint(debugMsg);
#endif
            }
            else
            {
                Z_AccelFlag = false;
                SystemFaultFlag = false;
            }
        }
        // Similar if conditions for OperationMode 1, 2, 3, 4.  But they need to be reviewed.
        // For example most (BASCOM) have If Z_accel < Acceltrippoint Then  but OpMode = 3 has If Z_accel > Acceltrippoint Then
        // Hold accelrometer in fault state to allow head to settle from vibration
        // If the flag is not set (ie laser OK) but was previously set, set it to true.  Only if it remains unset (by the above) after a period (AccellTick >= 10), reboot the system using the wd timer.
        if (Z_AccelFlagPrevious && !Z_AccelFlag)
        {
            if (AccelTick < 10)
            {
                Z_AccelFlag = true;
                SystemFaultFlag = true;
                return; // Exit the function before AccelTick and Z_AccelFlagPrevious are reset.  Allows for transient errors not to trigger a reset.
            }
            else
            {
                uartPrintFlash(F("ST3 DA \n"));
                StopTimer3();
                testWatchDog(3); // Stop Timer3 to prevent the watchdog timer from being reset
                // _delay_ms(WATCHDOG_DELAY); //' Wait for watchdog to overflow and system will reboot
                for (uint8_t a; a < 250; a++)
                {
                    testWatchDog(4 + a);
                    _delay_ms(500);
                }
            }
        }
        Z_AccelFlagPrevious = Z_AccelFlag;
        AccelTick = 0;
    }
}
void ClearSerial()
{
    while (UCSR0A & (1 << RXC0))
    {
        (void)UDR0;
    }
}

void GetLightLevel()
{
    // long X;
    long Ylocal;
    // uartPrintFlash(F("Read ADC 2 "));
    LightLevel = readADC(2);
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "SR2: %d", LightLevel);
    // uartPrint(debugMsg);
    LightLevel >>= 2;
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "LightLev after bit shift: %d", LightLevel);
    // uartPrint(debugMsg);
    if (LightTriggerOperation == 0)
    { // Run 24hr
        LightSensorModeFlag = 0;
    }

    if (LightTriggerOperation == 1)
    {                                    // Run Daytime only
        Ylocal = UserLightTripLevel + 5; // Light sensor hysteresis value

        if (LightLevel < UserLightTripLevel)
        {
            LightSensorModeFlag = 1;
        }

        if (LightLevel > Ylocal)
        {
            LightSensorModeFlag = 0;
        }
    }

    if (LightTriggerOperation == 2)
    {                                    // Run Night time only
        Ylocal = UserLightTripLevel - 5; // Light sensor hysteresis value

        if (LightLevel < UserLightTripLevel)
        { // Its night timer so run the laser
            LightSensorModeFlag = 0;
        }

        if (LightLevel > Ylocal)
        { // Its day time so turn the laser off
            LightSensorModeFlag = 1;
        }
    }
}

uint8_t adaptiveLaserPower()
{
#ifdef ADAPTIVELY_BRIGHT
    if (LightLevel < LOW_BRIGHT)
        return LOW_BRIGHT_LASER_PERCENTAGE;
    else if (LightLevel < MED_BRIGHT)
        return MED_BRIGHT_LASER_PERCENTAGE;
    else
        return 100;
#endif
#ifndef ADAPTIVELY_BRIGHT
    return 100;
#endif
}

void MrSleepyTime()
{
    uint8_t Xlocal; //, Ylocal;

    // Ylocal = 0;
    Xlocal = 0;
    LoopCount = 0;
    Audio2(4, 1, 20);
    // _delay_ms(4200); // Wait while the buzzer sounds
    // Audio2(1, 20, 1);
    // _delay_ms(1100); // Wait while the buzzer sounds

    SetLaserVoltage(0);
    PORTE &= ~(1 << FAN); // Fan = Off;

    StopTimer1();

    SteppingStatus = 0;
    PORTD |= (1 << X_ENABLEPIN); // Disable X - TMC2130 enable is low
    PORTD |= (1 << Y_ENABLEPIN); // Disable Y

    while (LightSensorModeFlag == 1)
    {
        GetLightLevel();
        if (LightSensorModeFlag == 0) // If 0 then light level has been triggered check again in 5 seconds to comfirm
        {
            _delay_ms(5000);
            GetLightLevel(); // This will reset LightSensorModeFlag to 1 if it had been set to 0 as a result of transient rather than persistent light.
        }

        // if (Tick > 10) //Tick is not reset other than in DoHouseKeeping() and that is not called if this while loop is running.
        if (TJTick % 40 == 0)
        {
            Audio2(1, 1, 1); // Brief beep before restarting while loop.
        }
    }

    uartPrintFlash(F("ST3 MrST \n"));
    StopTimer3();              // Watchdog reset
    _delay_ms(WATCHDOG_DELAY); //' Wait for watchdog to overflow and system will reboot
}

void ResetTiming()
{
    if (SetupModeFlag == 1)
    {
        Secondsfromreset = 0;
    }

    if (SetupModeFlag == 0)
    {
        if (Secondsfromreset > ResetSeconds)
        {
            Secondsfromreset = 0;
            BT_ConnectedFlag = 0;
            SetLaserVoltage(0); // 20250116
            HomeAxis();
        }
    }
}
void CalibrateLightSensor()
{
    SetLaserVoltage(0);
    GetLightLevel();
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "LL in CalibrateLightSensor: %d", LightLevel);
    uartPrint(debugMsg);
    printToBT(25, LightLevel); // Send current light level reading
}
void ThrottleLaser()
{
    float Result = 0.0;
    float L_power = 0.0;
    unsigned int SensorReading = readADC(0); // 20241205 Laser temperture
    uint8_t B_volt = 0;
    uint8_t Y;
    uint8_t X;

    if (BatteryVoltage < 98)
        B_volt = 25;
    else if (BatteryVoltage < 100)
        B_volt = 50;
    else if (BatteryVoltage < 105)
        B_volt = 70;
    else if (BatteryVoltage < 110)
        B_volt = 90;
    else
        B_volt = 100;

    if (Laser2OperateFlag == 1)
    {                           // 20241205 Ignore this case.
        Y = Laser2BattTrip + 5; // Add hysteresis value if battery is low    eg 10.5 +0.5  = 11.0 for the reset level
        if (BatteryVoltage < Laser2BattTrip)
            Laser2BattTripFlag = 1;
        if (BatteryVoltage > Y)
            Laser2BattTripFlag = 0;

        X = Laser2TempTrip - 2; // Add hysteresis value if the main laser is too hot. Reset value is 50-2=48deg's for reset back on
        if (LaserTemperature > Laser2TempTrip)
            Laser2TempTripFlag = 1;
        if (LaserTemperature < X)
            Laser2TempTripFlag = 0;

        if (Laser2TempTripFlag == 0 && Laser2BattTripFlag == 0)
            Laser2StateFlag = 1;
        else
            Laser2StateFlag = 0;
    }

    // Calculate the laser power directly from the sensor reading using the quadratic function
    // Ref sheet LaserTemp of Alg.xlsm - this correlation directly from sensor reading to percentage of laser power.
    // y = -0.0087x^2 + 12.361x - 4268.5
    // R² = 0.9996
    L_power = -0.0087 * SensorReading * SensorReading + 12.361 * SensorReading - 4268.5; // 20241205: Why is SensorReading used rather than LaserTemperature?
    if (SensorReading < 710)
        L_power = 100; // 20241205: Add constraints.  Magic numbers from Google Sheets.
    if (SensorReading > 792)
        L_power = 37;
    // From Google Sheets: -4269 + 12.4x + -8.74E-03x^2
    //  Ensure l_power (a percentage*100 of max voltage) is within a valid range
    if (L_power < 0)
        L_power = 0;
    else if (L_power > 100)
        L_power = 100;
    if (B_volt < L_power)
        L_power = B_volt;              // Ensure L_power is less than battery voltage.
    Result = L_power / 100.0f;         // Convert to %age
    L_power = Result * UserLaserPower; // Convert to voltage

    L_power = adaptiveLaserPower() * L_power / 100.0f;

    if (L_power > UserLaserPower)
        LaserPower = UserLaserPower;
    else if (L_power < 0)
        LaserPower = 0;
    else
        LaserPower = static_cast<uint8_t>(L_power);
#ifdef THROTTLE
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "LL: %d, SR: %u, Res: %d.%02d, L_power: %d.%02d, LP: %u", LightLevel, SensorReading, static_cast<int>(Result), static_cast<int>((Result - static_cast<int>(Result)) * 100), static_cast<int>(L_power), static_cast<int>((L_power - static_cast<int>(L_power)) * 100), LaserPower);
    uartPrint(debugMsg);
#endif
    if (LaserTemperature > 55)
    {
        LaserPower = 0;
        LaserOverTempFlag = 1;
        SystemFaultFlag = true;
    }
    else
    {
        LaserOverTempFlag = 0;
        SystemFaultFlag = false;
        // 20241221 Check imu in case SystemFaultFlag should still be set.
        DecodeAccelerometer();
    }

    if (LaserPower > UserLaserPower)
        LaserPower = UserLaserPower; // 20241205: Ensure LaserPower is less than UserLaserPower.

#ifdef THROTTLE
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "BV: %d, B_volt: %d, SR: %d,  LaserPower: %d", BatteryVoltage, B_volt, SensorReading, LaserPower);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "B_volt: %u,SR: %u, ULP: %u, MLP: %u, LP: %u", B_volt, SensorReading, UserLaserPower, MaxLaserPower, LaserPower);
    uartPrint(debugMsg);
#endif
}

void initMPU()
{
    uint8_t error = 0;
    Wire.begin(); // Initialize I2C
    // Read the WHO_AM_I register
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x75);                    // WHO_AM_I register address
    error = Wire.endTransmission(false); // Restart condition
    if (error)
    {
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "Error b4 WHO_AM_I: %d", error);
        uartPrintFlash(F("Error b4 WHO_AM_I: "));
        uartPrint(error);
        // uartPrintFlash(F("\n"));
        // uartPrint("MPU 5");
    }
    else
    {
        Wire.requestFrom(GyroAddress, (uint8_t)1); // Request 1 byte
        if (Wire.available())
        {
            uint8_t whoAmI = Wire.read(); // Read WHO_AM_I byte
            // uartPrintFlash(F("Accel Chip & Gyro address: "));
            // snprintf(debugMsg, DEBUG_MSG_LENGTH, "%d, %02x", whoAmI, GyroAddress);
            // uartPrint(debugMsg);
        }
        else
        {
            uartPrintFlash(F("WHO_AM_I read failed"));
        }
    }
    // Wake up the MPU
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x6B); // Power management register
    Wire.write(0x00); // Set to zero (wakes up the MPU-6050/6000)
    error = Wire.endTransmission(true);
    if (error)
    {
        uartPrintFlash(F("Error waking MPU: "));
        uartPrint(error);
    }
    else
    {
        // uartPrintFlash(F("MPU awake \n"));
    }
    _delay_ms(300);
    // Reset signal paths
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x68); // Signal path reset register
    Wire.write(0x03); // Reset all signal paths
    error = Wire.endTransmission(true);
    if (error)
    {
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "Error resetting signal paths: %d", error);
        uartPrintFlash(F("MPU 2 \n"));
    }
    else
    {
        // uartPrintFlash(F("Signal paths reset  \n"));
    }
    _delay_ms(300);
    // Set the slowest sampling rate

    Wire.beginTransmission(GyroAddress);
    Wire.write(0x19); // Sample rate divider
    Wire.write(0xFF); // 255 for the slowest sample rate
    error = Wire.endTransmission(true);
    if (error)
    {
        uartPrintFlash(F("Error setting sample rate: \n"));
        uartPrint(error);
        // uartPrint("MPU 3");
    }
    else
    {
        // uartPrintFlash(F("Sample rate set \n"));
    }

    // Set full scale reading to ±16g
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x1C); // Accelerometer configuration register
    Wire.write(0x18); // ±16g full scale
    error = Wire.endTransmission(true);
    if (error)
    {
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "Error setting full scale: %d", error);
        uartPrintFlash(F("MPU 4 \n"));
    }
    else
    {
        // uartPrintFlash(F("Full scale set \n"));
    }
}

void setupTimer3()
{
    cli();      // Disable global interrupts
    TCCR3A = 0; // Clear Timer/Counter Control Registers
    TCCR3B = 0;
    TCNT3 = 0;                           // Initialize counter value to 0
    OCR3A = 780;                         // Set the compare value to 780 for ~50ms interval with 1024 prescaler
    TCCR3B |= (1 << WGM32);              // CTC mode
    TCCR3B |= (1 << CS32) | (1 << CS30); // 1024 prescaler
    TIMSK3 |= (1 << OCIE3A);             // Enable Timer3 compare interrupt
    sei();                               // Enable global interrupts
}

void StopTimer3()
{
    uartPrintFlash(F("In ST3\n"));
    TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30)); // Clear all clock select bits to stop the timer
}
void StopTimer1()
{
    // uartPrintFlash(F("ST1 \n"));
    // Clear all CS1 bits to stop Timer1
    TCCR1B &= ~(1 << CS12);
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);
}
void setupTimer1()
{
    TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode (Clear timer on compare)
    TCCR1B |= (1 << CS12);  // Set up and start Timer1. 20240609: (1 << CS12) | (1 << CS10); for a prescaler of 1024. (1 << CS12); for 256.
    OCR1A = DSS_preload;    // Set the compare value to desired value.  DSS_preload is order 100.  2*7812 for testing - 1 second period
    // With a prescalar of 256 and compare value of 100, frequency: 16MHz/256 ~ 64kHz/100 ~ 640Hz which is a period of about 1.6ms.
    TIMSK1 |= (1 << OCIE1A); // Enable the compare match interrupt
}

void TurnOnGyro()
{
    Command = 11;
    Instruction = 4;
    DecodeCommsData();
}
uint8_t GetZone(uint8_t i)
{
    uint16_t Opzone;
    uint8_t z = 0;
    if (i < MapTotalPoints) // Ensure i is within the valid range
    {
        Opzone = eeprom_read_word((uint16_t *)((uintptr_t)&EramPositions[i].EramY));
        uint8_t zoneBits = Opzone >> 12; // Extract the upper 4 bits
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "GetZone: i: %d, Opzone: %d, zoneBits: %d", i, Opzone, zoneBits);
        // uartPrint(debugMsg);

        switch (zoneBits)
        {
        case 1:
            z = 1;
            break;
        case 2:
            z = 2;
            break;
        case 4:
            z = 3;
            break;
        case 8:
            z = 4;
            break;
        default:
            z = 0;
            break;
        }

        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "GetZone: i: %d, z: %d", i, z);
        // uartPrint(debugMsg);
        return z;
    }
    else
    {
        // uartPrintFlash(F("GetZone: i out of range."));
        return 0;
    }
}

void getMapPtCounts()
{ // Get the number of vertices (user specified) in each zone. Populate MapCount[2,n] with incremental and cumulative values.
    uint8_t MapIndex;
    uint8_t z = 0, Prev_z = 0; // Prev_i_1;
    for (MapIndex = 1; MapIndex <= MapTotalPoints; MapIndex++)
    {                                  // 20240628: Start at MapIndex = 1.  Pass (MapIndex -1) to GetZone as that looks up EramMapPositions[] which is zero based.
        z = GetZone(MapIndex - 1) - 1; // Zone index of MapIndex (the 2nd index) is zero based - so from 0 to 3. Stored points from 0 to MapTotalPoints-1.
        if (z > 0)                     // Test for NOT the first zone
        {                              // If i == 0 (ie 1st zone), don't do anything. Assignments for that zone either when Prev_i = 0 or at total when there is only one zone.
            if (z != Prev_z)
            { // Do something when you have the first point of a new zone.
                if (Prev_z == 0)
                {
                    MapCount[0][Prev_z] = MapIndex; // Incremental is 1 greater than cumulative as incremental adds the first point of the zone as an additional last point.
                    MapCount[1][Prev_z] = MapIndex - 1;
                }
                else
                {
                    MapCount[1][Prev_z] = MapIndex - 1;
                    MapCount[0][Prev_z] = MapIndex - MapCount[1][Prev_z - 1]; // 20250110.  MapIndex is that for the first point of the next zone.  So MapIndex less cumulative to last zone is correct.
                }
                while (z > Prev_z + 1) // Allow for case when a zone is not used. eg 0 and 1 used for autofill and 3 for path but 2 not used.  In that case you'd have z = 3, Prev_z = 1
                {
                    MapCount[1][Prev_z + 1] = MapCount[1][Prev_z];
                    MapCount[0][Prev_z + 1] = 1; // Hopefully this isn't used - but consistent with other zones if there are n distinct points then this is recorded as n+1 to allow for the repeated first point
                    Prev_z = Prev_z + 1;
                    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "MI %d, z %d, NewPrev_z %d, MC0 %d, MC1 %d", MapIndex, z, Prev_z, MapCount[0][Prev_z], MapCount[1][Prev_z]);
                    // uartPrint(debugMsg);
                }
                // snprintf(debugMsg, DEBUG_MSG_LENGTH, "If z!=Prev_z: MI %d, z %d, Prev_z %d, MC0 %d, MC1 %d", MapIndex, z, Prev_z, MapCount[0][Prev_z], MapCount[1][Prev_z]);
                // uartPrint(debugMsg);
            }
        }
        Prev_z = z;
    }
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "MI %d, z %d, MTP %d", MapIndex, z, MapTotalPoints);
    // uartPrint(debugMsg);
    if (MapIndex >= MapTotalPoints)      // Do something if it's the last stored point (so MapIndex is MapTotalPoints) - in which case it hasn't been covered above.
    {                                    // The loop over MapIndex, ending at MapIndex <=MapTotalPoints, means that MapIndex ends that loop at MapTotalPoints + 1.
        MapCount[1][z] = MapTotalPoints; // Consider 4 stored points in a single zone.  This would be entered when MapIndex == 4.  4 is the correct number of cumulative specified points.
        if (z == 0)
        {
            MapCount[0][z] = MapTotalPoints + 1; // This would be 5 which is the number of specified points (4) plus 1 for the first one being repeated.
        }
        else
        {
            MapCount[0][z] = MapTotalPoints - MapCount[1][z - 1] + 1;
        }
    }

#ifdef TEST_MAPCOUNT
    for (uint8_t z = 0; z < NBR_ZONES; z++)
    {
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "z: %d, MC0z: %d, MC1z: %d", z, MapCount[0][z], MapCount[1][z]);
        uartPrint(debugMsg);
    }
#endif
}

void LoadZoneMap(uint8_t zn)
{
    //---------Gets vertices for given zone from EEPROM.
    // The Y dat MSB 4 bits are the operation zone and the 16 bits LSB is the map point number
    // 1111  1111111111111111
    //  ^             ^------------ Map point number   eg point 1, point 40 etc
    //  ^--------------------- Operation Zone     eg Zone 0 to 4
    uint8_t MapIndex, MI, zn_1;
    float res = 0.0;
    zn_1 = zn; // 20240701: Had used zn-1.  But (zn-1) now passed as argument to this function from .
    // int temp;

#ifdef BASE_PRINT
    if (MapTotalPoints == 0)
    {
        uartPrint("LZM: Zero points");
        return;
    }
    else
    {
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "LZM: MapTotalPoints: %d \r\n", MapTotalPoints);
        uartPrint(debugMsg);
    }
#endif
    for (MapIndex = 0; MapIndex < MapCount[0][zn_1] - 1; MapIndex++)
    { // Upper limit.  See notes below in next loop. With n-1 distinct points, there are n points in total
        // and they are indexed from 0 to n-2 - hence 0 to < n -1.
        // Consider 2 zones, each with 4 distinct vertices.  MC[0][0] = MC[0][1] = 5 (ie allowing for duplicating the first vertex).  MC[1][0] = 4; MC[1][1] = 8
        // In each case, the for loop is: for (MapIndex = 0; MapIndex < 4; MapIndex++)
        if (zn_1 == 0)
        {                  //
            MI = MapIndex; // MI values: 0, 1, 2, 3
        }
        else
        {
            MI = MapIndex + MapCount[1][zn_1 - 1]; // The index in EramPositions is across all zones whereas the loop index here is for a single zone
            // MI values: 0, 1, 2, 3 each + 4 => 4, 5, 6, 7
        }
        Vertices[0][MapIndex] = eeprom_read_word(&EramPositions[MI].EramX);
        Vertices[1][MapIndex] = eeprom_read_word(&EramPositions[MI].EramY) & 0x0FFF;
#ifdef LOG_PRINT // GHOST
        printPerimeterStuff("V0i, V1i", Vertices[0][MapIndex], Vertices[1][MapIndex]);
#endif
    }
    // Above loads distinct vertices.  MapIndex is now equal to MapCount[0][zn-1] as that is why the for loop has exited.  So set that vertex equal to the first one (index 0).
    Vertices[0][MapIndex] = Vertices[0][0];
    Vertices[1][MapIndex] = Vertices[1][0];
#ifdef LOG_PRINT // GHOST
    printPerimeterStuff("V0i, V1i", Vertices[0][MapIndex], Vertices[1][MapIndex]);
#endif
    // Get the slope of each segment & store in Vertices[2][i]
    for (MapIndex = 1; MapIndex < MapCount[0][zn_1]; MapIndex++)
    { // MapCount[0][zn] is a count of the number of specified vertices, including the last repeated one, in zone 1.
        // If there are 4 distinct points, then MapCount[0][zn] would be 5. Segments are 0:1, 1:2, 2:3, 3:4  So start from 1 and consider MapIndex and (MapIndex - 1) as the segment endpoints.
        if (abs(Vertices[1][MapIndex] - Vertices[1][MapIndex - 1]) > MIN_PERIMETER_TILT)
        { // If the difference in Ys is big enough, get slope.
            float num = static_cast<float>(Vertices[0][MapIndex] - Vertices[0][MapIndex - 1]) * 10;
            float den = static_cast<float>(Vertices[1][MapIndex] - Vertices[1][MapIndex - 1]);
            if (abs(den) >= 1)
            {
                res = static_cast<float>(num) / den;
            }
            else
            {
                // uartPrint("Abs(den)<1");
            }
            Vertices[2][MapIndex - 1] = res;
        }
        else
        {
            Vertices[2][MapIndex - 1] = DEF_SLOPE; // Set an extreme value where delta(x) is potentially large and delta(y) is small.
        }
    }
}

void floatToString(char *buffer, float value, int precision)
{
    dtostrf(value, 0, precision, buffer);
}

int getCartFromTilt(int t)
{                                            // Estimate a cartesian distance to the laser point based only on tilt (in steps) and LaserHt
    float a = (float)t / TILT_STEPS_PER_RAD; // Angle in radians.
    float b = tan(a);
    float c = (float)LaserHt / (b * 10); // LaserHt is in decimetres rather than metres.  So divide by 10 to get back to metres.
    return (int)(c + 0.5);               // rounding to the nearest integer before casting.  Return value is in metres.
}
int getTiltFromCart(int rho)
{
    // float a = (float)LaserHt/((float)rho*10.0); //10.0 due to LaserHt being in decimetres, not metres.
    // float b = atan2(LaserHt,rho*10);
    float b = atan2(static_cast<float>(LaserHt), static_cast<float>(rho) * 10.0f); // 20241205 Arguments were reverse ordered.  Arguments also now cast to float.
#ifdef GHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "rho %d, 100*b %d, LaserHt %d", rho, static_cast<int>(100 * b), LaserHt);
    uartPrint(debugMsg);
#endif
    return (int)(b * TILT_STEPS_PER_RAD + 0.5); // rounding to the nearest integer before casting
}
int getNextTiltVal(int thisTilt, uint8_t dirn)
{ // 20240629: TILT_SEP is target separation in Cartesian space between ladder rungs.
    // getNextTiltVal() should calculate the required tilt value to get the specified separation.  TILT_SEP will need calibration.
    int rho = getCartFromTilt(thisTilt);
    if (dirn == 1)
    {
        rho = rho - Tilt_Sep;
    }
    else
    {
        rho = rho + Tilt_Sep;
    }
    return getTiltFromCart(rho);
}
void getCart(int p, int t, int (&thisres)[2])
{ // Take pan(t) and tilt(t) for a polar point and return the cartesian equivalent (thisres)
    int r = getCartFromTilt(t);
    thisres[0] = r * cos(p / PAN_STEPS_PER_RAD); // 1st coord (x) is r at pan = 0 and 2nd coord (y) is zero.
    thisres[1] = r * sin(p / PAN_STEPS_PER_RAD);
}
void getPolars(int c1, int c2, int thisRes[2])
{ // Take cartesian coordinates as input, return, via thisRes[2], polars.
    // 20241205 Change r from long to double.
    double r = sqrt(static_cast<double>(c1) * c1 + static_cast<double>(c2) * c2); // Get the distance from the laser to the point by Pythagoras.
#ifdef xGHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "c1 %d, c2 %d, r %d", c1, c2, static_cast<int>(r));
    uartPrint(debugMsg);
#endif
    // First argument of atan2 is opp, second is adj.  Ref https://cplusplus.com/reference/cmath/atan2/.
    double angle = atan2(static_cast<double>(c2), static_cast<double>(c1)); // Pan is atan(c2/c1).  atan2 assumes arguments are doubles.
    int temp = static_cast<int>(angle * PAN_STEPS_PER_RAD);                 // pan
    thisRes[0] = temp;
    thisRes[1] = getTiltFromCart(static_cast<int>(r)); // tilt.
}
int GetPanPolar(int TiltPolar, int PanCart)
{ // Get the number of pan steps for a specified tilt angle (steps) and cartesian pan distance.
    int rho = getCartFromTilt(TiltPolar);
    return PAN_STEPS_PER_RAD * PanCart / rho;
}
void printPerimeterStuff(const char *prefix, int a, int b, uint8_t c, uint8_t d)
{
    // Using sprintf for safer string formatting and concatenation
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "%s(%d, %d) :(%d,%d)", prefix, a, b, c, d);
    uartPrint(debugMsg);
    _delay_ms(100);
}

void getExtremeTilt(uint8_t nbrZnPts, int &minTilt, int &maxTilt)
{
    // Initialize minY and maxY with extreme values
    minTilt = Vertices[1][0];
    maxTilt = Vertices[1][0];
    // Iterate through the vertices
    for (int i = 1; i < nbrZnPts; i++)
    {
        int y = Vertices[1][i]; // Assuming y values are stored in the second column of Vertices
        if (y < minTilt)
        {
            minTilt = y;
        }
        if (y > maxTilt)
        {
            maxTilt = y;
        }
        if (minTilt < getTiltFromCart(MAX_RANGE))
            minTilt = getTiltFromCart(MAX_RANGE);
        if (maxTilt > getTiltFromCart(MIN_RANGE))
            maxTilt = getTiltFromCart(MIN_RANGE);
    }
}
uint16_t getNbrRungs(int maxTilt, int minTilt, int &rhoMin)
{ //, int &rhoMax, int &rhoMin){
    rhoMin = getCartFromTilt(maxTilt);
    int rhoMax = getCartFromTilt(minTilt);
    int temp = static_cast<uint16_t>((static_cast<int>(rhoMax) - static_cast<int>(rhoMin)) / static_cast<int>(Tilt_Sep));
#ifdef xGHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Ints: rhoMax: %d rhoMin: %d tilt_sep: %d nbrRungs: %d ", static_cast<int>(rhoMax), static_cast<int>(rhoMin), static_cast<int>(Tilt_Sep));
    uartPrint(debugMsg);

    snprintf(debugMsg, DEBUG_MSG_LENGTH, "rhoMax: %d rhoMin: %d minTilt: %d maxTilt: %d tilt_sep: %d nbrRungs: %d ", rhoMax, rhoMin, minTilt, maxTilt, Tilt_Sep, temp);
    uartPrint(debugMsg);
#endif
    return (uint16_t)temp;
}

uint8_t getInterceptSegment(uint8_t nbrZnPts, int tilt, uint8_t fstInd)
{ // Get the segment of the perimeter that the tilt value falls in.
    uint8_t i = 0;
    uint8_t j = 0;
    for (i = fstInd; i < nbrZnPts; i++)
    {
        if (i == nbrZnPts - 1)
            j = 0;
        else
            j = i + 1; // 20250121: Previously j was always i+1.
        if ((Vertices[1][i] <= tilt && Vertices[1][j] > tilt) ||
            (Vertices[1][i] > tilt && Vertices[1][j] <= tilt))
        {
            // return i;
            break;
        }
    }
#ifdef GHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Tilt %d, i %d, fstInd %d, V1i %d, V1i+1 %d", tilt, i, fstInd, Vertices[1][i], Vertices[1][i + 1]);
    uartPrint(debugMsg);
#endif
    if (i < nbrZnPts)
        return i; // 20241221: Trap case when no segment is found.
    else
        return nbrZnPts - 1;
}

int sign(int x)
{
    return (x > 0) - (x < 0);
}

void PolarInterpolate(int last[2], int nxt[2], int num, int den, int (&res)[2])
{
    uint16_t temp = num * abs(nxt[0] - last[0]);
    res[0] = last[0] + temp / den * sign(nxt[0] - last[0]);
    temp = num * abs(nxt[1] - last[1]);
    res[1] = last[1] + temp / den * sign(nxt[1] - last[1]);
#ifdef DEBUG
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "PI: num: %d, den: %d, x0: %d, y0: %d, x1: %d, y1: %d, res0: %d, res1: %d", num, den, last[0], last[1], nxt[0], nxt[1], res[0], res[1]);
    uartPrint(debugMsg);
#endif
}
// Take 2 polar specified points (last and nxt), convert to cartesian (c1, c2), interpolate (num/den), and return polars of interpolated point (thisRes).
void CartesianInterpolate(int last[2], int nxt[2], int num, int den, int (&res)[2])
{
    int c1[2], c2[2];              // The cartesian end points.
    getCart(last[0], last[1], c1); // Puts Cartesian coords in c1 from pan (last[0]) and tilt (last[1])
    getCart(nxt[0], nxt[1], c2);
#ifdef xGHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "CI: num: %d, den: %d, l0: %d, l1: %d, n0: %d, n1: %d, c10: %d, c20: %d, c11: %d, c21: %d", num, den, last[0], last[1], nxt[0], nxt[1], c1[0], c2[0], c1[1], c2[1]);
    uartPrint(debugMsg);
#endif
    uint32_t temp = static_cast<uint32_t>(num) * static_cast<uint32_t>(abs(c2[0] - c1[0])); // 20241204: Changed to uint32_t from int. Though note sign() used below.
    res[0] = c1[0] + static_cast<int32_t>(temp / den) * sign(c2[0] - c1[0]);
    temp = static_cast<uint32_t>(num) * static_cast<uint32_t>(abs(c2[1] - c1[1]));
    res[1] = c1[1] + static_cast<int32_t>(temp / den) * sign(c2[1] - c1[1]);
// Use the cartesian values stored in res and write the corresponding polar values to the same variable.
#ifdef xGHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "CI: res0 %d, res1 %d", res[0], res[1]);
    uartPrint(debugMsg);
#endif
    getPolars(res[0], res[1], res);
#ifdef xGHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "CI Polars: res0 %d, res1 %d", res[0], res[1]);
    uartPrint(debugMsg);
#endif
}

void midPt(int tilt, uint8_t seg, int (&res)[2])
{
    // uint8_t ratio = 0;
    int den = 0;
    int num = abs(tilt - Vertices[1][seg]);
    // snprintf(debugMsg, DEBUG_MSG_LENGTH,"S0 %d, S1 %d, diff %d", Vertices[1][seg], Vertices[1][seg+1],abs(Vertices[1][seg]-Vertices[1][seg+1]));
    // uartPrint(debugMsg);
    den = abs(Vertices[1][seg] - Vertices[1][seg + 1]);
    if (den > 0)
    {
        int last[2] = {Vertices[0][seg], Vertices[1][seg]};
        int nxt[2] = {Vertices[0][seg + 1], Vertices[1][seg + 1]};
        // PolarInterpolate(last, nxt, num, den, res);
        CartesianInterpolate(last, nxt, num, den, res); // Calculates res[2] from last[2], nxt[2], num and den.
    }
    else
    {
        res[0] = Vertices[0][seg];
        res[1] = Vertices[1][seg];
    }
// snprintf(debugMsg, DEBUG_MSG_LENGTH,"S0 %d, S1 %d, Tilt %d, den %d, seg %d, ratio %d%%, X %d, Y %d",Vertices[1][seg], Vertices[1][seg+1], tilt,den, seg,ratio, res[0],res[1]);
#ifdef xGHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "midPt: S0 %d, S1 %d, Tilt %d, num %d, den %d, seg %d, X %d, Y %d", Vertices[1][seg], Vertices[1][seg + 1], tilt, num, den, seg, res[0], res[1]);
    uartPrint(debugMsg);
#endif
}

int crossProduct(int pt1[], int pt2[])
{
    return pt1[0] * pt2[1] - pt1[1] * pt2[0];
}

void vectorDiff(int pt1[], int pt2[], int vec[])
{
    vec[0] = pt2[0] - pt1[0];
    vec[1] = pt2[1] - pt1[1];
}

uint16_t length(int pt[])
{
    uint32_t l = pt[0] * pt[0] + pt[1] * pt[1];
    return sqrt(l);
}

bool testConvex(uint8_t zone, int pt[])
{
    bool inside = true;
    uint8_t vertexCount = MapCount[0][zone] - 1; // For a quadrilateral, for example, there are 4 vertices but MapCount[0][z] is 5 as it includes allowance for the repeated 1st point.
    // There are 4 segments to be tested  with the end point of the last being the (repeated) first vertex.
    for (uint8_t i = 0; i < vertexCount; i++)
    {
        int next = i + 1; //(i + 1) % vertexCount; // I don't think % vertexCount is necessary?
        int vec1[2], vec2[2];
        int vertex1[2] = {Vertices[0][i], Vertices[1][i]};
        int vertex2[2] = {Vertices[0][next], Vertices[1][next]};
        vectorDiff(vertex1, vertex2, vec1);
        vectorDiff(vertex1, pt, vec2);
        int CP = crossProduct(vec1, vec2);
        uint16_t l = length(vec1);
        bool CPTest = (CP < CONVEX_TOLERANCE / l);
#ifdef xGHOST // TEST_CONVEX
        if (!CPTest)
        {
            snprintf(debugMsg, DEBUG_MSG_LENGTH, "zone: %d, vertex: %d, CP: %d, l: %d, X: %d, Y: %d, CPTest: %d", zone, i, CP, l, pt[0], pt[1], CPTest);
            uartPrint(debugMsg);
        }
#endif
        if (CP < CONVEX_TOLERANCE / l)
        {
            inside = false;
            // break;
        }
    }
    return inside;
}

uint16_t cartDistance(int pt1[2], int pt2[2])
{
    uint32_t dx = static_cast<uint32_t>(abs(pt2[0] - pt1[0]));
    uint32_t dy = static_cast<uint32_t>(abs(pt2[1] - pt1[1]));
#ifdef DEBUG
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "dx %lu, dy %lu", dx, dy);
    uartPrint(debugMsg);
#endif
    uint32_t sqr = dx * dx + dy * dy;
    double dist = sqrt(sqr);
#ifdef DEBUG
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "pt20 %d, pt10 %d, pt21 %d, pt11 %d, sqr %lu, dist %u", pt2[0], pt1[0], pt2[1], pt1[1], sqr, static_cast<uint16_t>(dist));
    uartPrint(debugMsg);
#endif

    return static_cast<uint16_t>(dist); // Convert the distance to uint16_t
}

uint16_t distance(int pt1[2], int pt2[2])
{
    uint16_t dx = static_cast<uint16_t>(abs(pt2[0] - pt1[0]));
    uint16_t dy = static_cast<uint16_t>(abs(pt2[1] - pt1[1]));
    // uint16_t dist = std::max(dx,dy);
    uint16_t dist = (dx > dy) ? dx : dy;
#ifdef DEBUG
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "pt20 %d, pt10 %d, pt21 %d, pt11 %d, dist %u, dx %u, dy %u", pt2[0], pt1[0], pt2[1], pt1[1], dist, dx, dy);
    uartPrint(debugMsg);
#endif
    return dist;
}
bool getXY(uint8_t pat, uint8_t zn, uint8_t &ind, bool newPatt, uint8_t rhoMin, uint16_t nbrRungs, int minTilt, int maxTilt)
{
#ifdef WIGGLY_PTS
    static uint8_t wigglyPt = 0; // Tested but not used.  wigglyPt = 0 effectively removes this functionality.
#endif
    static int thisRes[2], tilt = 0;
    static int nextRes[2], beginRes[2], endRes[2];
    static bool startRung = true; // This is an indicator of being at one side or the other of a rung.
    static bool endRung = false;  // endRung is used to indicate that you've arrived at the end of a rung after perhaps several interpolated points along the rung
    static uint8_t rung = 0;
    // static uint8_t lastZn = 0;

    static uint8_t nbrSegPts = 0;
    static uint8_t seg = 0, fstSeg = 0, sndSeg = 0;
    static uint16_t segPt = 0; // Counter for dense filler points within a segment.
    static int pt1[2];
    static int pt2[2];

    static uint8_t rungMidPtCnt = 0; // 20241219.  Count of mid points passed while traversing a rung.

    if (newPatt)
    { // Test for a new pattern and, if so, reset seg and turn off laser to go to start of pattern (which might also be new zone)
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "getXY; newPatt true. X: %d, Y: %d, AbsX: %d, AbsY: %d", X, Y, AbsX, AbsY);
        // uartPrint(debugMsg);
        seg = 0;
        CmdLaserOnFlag = false; // 20241229.  Laser off if moving to first point of pattern (and hence also zone)
        SetLaserVoltage(0);
        endRung = true;
    }
    else
    {
        if (!SystemFaultFlag)
        {
            CmdLaserOnFlag = true;
            SetLaserVoltage(LaserPower);
        }
    }
#ifdef TEST_PATH_MODE
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "zn: %d, seg: %d, nbrSegPts: %u, segPt: %d, Nbr_Rnd_Pts: %d, MPC[0]: %d", zn, seg, nbrSegPts, segPt, Nbr_Rnd_Pts, MapCount[0][zn]);
    uartPrint(debugMsg);
#endif

    if (zn == PATH_ZONE)
    {
        if (seg < MapCount[0][zn] - 1) //  If there are n vertices, then MC[0][z] = n + 1. But seg is zero based.
        {
#ifdef TEST_PATH_MODE
            snprintf(debugMsg, DEBUG_MSG_LENGTH, "Loading V0i: %d, V1i: %d, for seg: %d", Vertices[0][seg], Vertices[1][seg], seg);
            uartPrint(debugMsg);
#endif
            seg++;
        }
        else
        {
#ifdef TEST_PATH_MODE
            snprintf(debugMsg, DEBUG_MSG_LENGTH, "Seg >=MC[0] so put X: %d, and Y: %d to AbsX: %d, and AbsY: %d. MC: %d, seg: %d", X, Y, AbsX, AbsY, MapCount[0][zn], seg);
            uartPrint(debugMsg);
#endif
            seg = 0;
            // X = AbsX; //20250121 Why was (X,Y) set to (AbsX,AbsY)?
            // Y = AbsY;
        }
        X = Vertices[0][seg];
        Y = Vertices[1][seg];
        if (seg == 0)
        {
            SetLaserVoltage(0);
            CmdLaserOnFlag = false;
        }
        else
        {
            SetLaserVoltage(LaserPower);
            CmdLaserOnFlag = true;
        }
        ind++;
    }
    else // The general autofill case
    {
        if (seg == 0 && segPt == 0) // 20250108.  Turn laser off if going to first vertex of a zone.  Should be off as a result of newPatt condition in any case
        {
            SetLaserVoltage(0);
            CmdLaserOnFlag = false;
        }
        else
        {
            if (!SystemFaultFlag)
            {
                SetLaserVoltage(LaserPower);
                CmdLaserOnFlag = true;
            }
        }
        // First traverse the boundary.
        if (seg < MapCount[0][zn]) // This conditional determines if the next point is on the boundary or a rung.
        {                          // Use seg to count segments.  Use segPt to count intermediate points in a segment.  This does the boundary, perhaps with wiggly points.
            if (segPt == 0)
            { // First point of segment - which must be a zone vertex.  Get the two vertices  which define the segment and the number of interpolated, excluding wiggly, points.
                pt1[0] = Vertices[0][seg];
                pt1[1] = Vertices[1][seg];
                // if ((seg == MapCount[0][zn] - 1) && Nbr_Rnd_Pts >= 0) // Vertices[i][MapCount[0][zn]] should be the first point again. Explicitly place that.
                // 2nd condition deals with path mode where first point should not be repeated.
                if (seg == MapCount[0][zn] - 1)
                {
                    pt2[0] = Vertices[0][0];
                    pt2[1] = Vertices[1][0];
                }
                else
                {
                    pt2[0] = Vertices[0][seg + 1];
                    pt2[1] = Vertices[1][seg + 1];
                }
                nbrSegPts = static_cast<uint8_t>(distance(pt1, pt2) / SEG_LENGTH); // This is the number of dense points to be placed along the segment, excluding wiggly points.
                                                                                   // #ifdef DEBUG
                // snprintf(debugMsg, DEBUG_MSG_LENGTH, "Distance: %u, nbrSegPts: %u", distance(pt1, pt2), nbrSegPts);
                // uartPrint(debugMsg);
                // #endif
                X = Vertices[0][seg]; // Set the next target point to the first vertex of the segment.
                Y = Vertices[1][seg];
                // 20241221: These having aberrant values caused ghost points.  Try this to fix the problem.  Neither thisRes[] nor nextResp[]
                // are (or should be) used on the boundary.  Keep it as it fixed the problem.
                thisRes[0] = X;
                thisRes[1] = Y;
                nextRes[0] = X;
                nextRes[1] = Y;

                segPt++; // 20241120. Added.  Increment segPt here so that the first point of the segment is not repeated.
                // snprintf(debugMsg, DEBUG_MSG_LENGTH, "SegPt == 0 case. X: %d, Y: %d, AbsX: %d, AbsY: %d", X, Y, AbsX, AbsY);
                // uartPrint(debugMsg);
            }
            else // segPt>0 case indicates that this is a point on the segment not being an end point (zone vertex)
            {
#ifdef WIGGLY_PTS
                // 20241119: WigglyBorder_
                if (wigglyPt == 0)
#endif
                { // Move to the next dense segment point.
                    if (nbrSegPts > 0)
                    {
                        CartesianInterpolate(pt1, pt2, segPt, (nbrSegPts + 1), res);
                        X = res[0];
                        Y = res[1];
                    }
                    else
                    { // 20250109 May need to do something but not sure yet.
                    }
                }
#ifdef WIGGLY_PTS
                else
                { // For wiggly points get a few randomly positioned around the start of the (dense) segment.
                    X = res[0] + (rand() % (2 * X_WIGGLY_BORDER_RANGE + 1)) - X_WIGGLY_BORDER_RANGE;
                    Y = res[1] + (rand() % (2 * Y_WIGGLY_BORDER_RANGE + 1)) - Y_WIGGLY_BORDER_RANGE;
                }
                wigglyPt++; // 20241129: With #define NBR_WIGGLY_POINTS 0, this increment means that no wiggly points are used.
#endif
                // snprintf(debugMsg, DEBUG_MSG_LENGTH, "segPt: %d, X: %d, Y: %d, AbsX: %d, AbsY: %d", segPt, X, Y, AbsX, AbsY);
                // uartPrint(debugMsg);
            }
#ifdef WIGGLY_PTS
            if (wigglyPt > nbrWigglyPts)
            { // Reset wigglyPt and segPt after NBR_WIGGLY_POINTS around a segment end point.
                wigglyPt = 0;
                segPt++;
            }
#endif
#ifndef WIGGLY_PTS
            segPt++; //
#endif
            if (segPt >= nbrSegPts)
            { // Having traversed the interpolated segment points, move to the next segment
                seg++;
                segPt = 0;
            }
        }
        else // if (Nbr_Rnd_Pts > 0)
        {    // Now deal with the rungs - above is just boundary.
#ifdef xGHOST
            snprintf(debugMsg, DEBUG_MSG_LENGTH, "ind: %d, startRung: %d, endRung: %d>", ind, startRung, endRung);
            uartPrint(debugMsg);
#endif
            // If ind  is zero and this else clause is entered, then this is the first required rung.  So need to calculate both end points of rung.
            // if ((ind == 0) || (endRung))
            if (endRung)
            {
                if (startRung)
                {
                    RndNbr = rand() % nbrRungs; //
                    if (pat == 4)
                    { // Sequentially cross the zone with rungs, not randomly
                        RndNbr = rung++;
                        if (RndNbr == nbrRungs)
                            rung = 0;
                    }
                    if (RndNbr == 0)
                        RndNbr = 1;
                    tilt = getTiltFromCart(rhoMin + RndNbr * Tilt_Sep); // Argument is Cartesian offset from laser.  Get tilt angle for this.
                    // 20250121
                    if (tilt < minTilt)
                    {
#ifdef GHOST
                        uartPrintFlash(F("minTilt err\n"));
#endif
                        tilt = minTilt;
                    }
                    if (tilt > maxTilt)
                    {
#ifdef GHOST
                        uartPrintFlash(F("maxTilt err\n"));
#endif
                        tilt = maxTilt;
                    }
                    fstSeg = getInterceptSegment(MapCount[0][zn] - 1, tilt, 0);          // First segment which includes specified/chosen tilt
                    sndSeg = getInterceptSegment(MapCount[0][zn] - 1, tilt, fstSeg + 1); // Opposite segment which includes specified/chosen tilt (relies on zone being convex)
#ifdef GHOST
                    snprintf(debugMsg, DEBUG_MSG_LENGTH, "rhoMin: %d, Rnd: %d, Tilt_Sep: %d>", rhoMin, RndNbr, Tilt_Sep);
                    uartPrint(debugMsg);
#endif
                    // For each boundary segment get the interpolated point in the segment needed for the new rung.
                    midPt(tilt, fstSeg, thisRes); // Intercept of pan for given tilt with fstSeg
                    midPt(tilt, sndSeg, nextRes); // Intercept of pan for given tilt with sndSeg
                    // if (pat == 3) //20250119: Q&D to stop ghosts.
                    // {                                                                         // For pat 3, "rungs" are not horizontal.  Just randomly chosen points from each side of the zone.  So get a different value for nextRes[].
                    //     int tilt2 = getTiltFromCart(rhoMin + rand() % nbrRungs * Tilt_Sep);   // Get a new, independently random, tilt value (tilt2).
                    //     fstSeg = getInterceptSegment(MapCount[0][zn] - 1, tilt2, 0);          // Get the segment with this pan intercept on side 1.  Although this intercept won't be used, fstSeg needs to be calculated for use in the next calc.
                    //     sndSeg = getInterceptSegment(MapCount[0][zn] - 1, tilt2, fstSeg + 1); // Get the segment with this pan intercept on side 2.
                    //     midPt(tilt2, sndSeg, nextRes);                                        // Note that although fstSeg is recalculated, thisRes[] is not.
                    // }
                } // When starting a new pass, whether with startRung true or false, set beginning and end points of rung.
                ind++;
                rungMidPtCnt = 0;
                if (startRung)
                { // First call in  1st direction.  End points as calculated above
                    endRes[0] = nextRes[0];
                    endRes[1] = nextRes[1];
                    beginRes[0] = thisRes[0];
                    beginRes[1] = thisRes[1];
                }
                else
                { // First call in  reverse direction.  Beginning point is end point of previous rung, end point is beginning point of new rung.
                    endRes[0] = thisRes[0];
                    endRes[1] = thisRes[1];
                    beginRes[0] = res[0]; // res[] is last point targeted.  If this is the start of a new rung in reverse direction, last point of old rung is starting point (possibly?).
                    beginRes[1] = res[1];
                }
#ifdef GHOST
                snprintf(debugMsg, DEBUG_MSG_LENGTH, "sr: %d, ind: %d, endX: %d, endY: %d, bX: %d, bY: %d>", startRung, ind, endRes[0], endRes[1], beginRes[0], beginRes[1]);
                uartPrint(debugMsg);
#endif
            }

            uint16_t num = abs(rungMidPtCnt * PAN_SEP);
            uint16_t den = abs(beginRes[0] - endRes[0]);
            rungMidPtCnt++;

            if (num < den)
            {
                CartesianInterpolate(beginRes, endRes, num, den, res);
                endRung = false;
            }
            else
            {
                // uartPrintFlash(F("Set end point"));
                res[0] = endRes[0];
                res[1] = endRes[1];
                rungMidPtCnt = 0;
                startRung = !startRung;
                endRung = true;
            }
            X = res[0];
            Y = res[1];
            // snprintf(debugMsg, DEBUG_MSG_LENGTH, "RungMidPtCnt: %d, num: %d, den: %d>", rungMidPtCnt, num, den);
            // uartPrint(debugMsg);
        }
    }

#ifdef LOG_PRINT
    static int LastX = 0;
    static int LastY = 0;
    if ((X != LastX || Y != LastY) && printPos)
    {
#ifdef TEST_LASER_POWER
        if (CmdLaserOnFlag)
            uartPrintFlash(F("On \n"));
        else
            uartPrintFlash(F("Off \n"));
#endif
#ifdef GHOST
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "ind %d, seg %d, SegPt %d, X: %d, Y: %d", ind, seg, segPt, X, Y);
        uartPrint(debugMsg);
#endif
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "X: %d, Y: %d", X, Y);
        uartPrint(debugMsg);
        printToBT(34, AbsX);
        printToBT(35, AbsY);
        LastX = X;
        LastY = Y;
    }
#endif
    int pt[2] = {X, Y};                         // Create an array with X and Y values to pass to testConvex()
    if (!testConvex(zn, pt) && zn != PATH_ZONE) // Test that autofill point is internal to zone.  Not necessary for boundary?
    {
        uartPrintFlash(F("fail convex \n"));
    }
    // lastZn = zn;
    if (zn == PATH_ZONE)
        return false;
    else
        return startRung; // Use this to set speed and (possibly) laser on.
}

void ProcessCoordinates()
{
#ifdef ISOLATED_BOARD
    // if (isolated_board_flag)
    {
        AbsX = X; // Increment position, simulating movement of the stepper motors, at specified period.
        AbsY = Y;
    }
    StepCount = 0; // Put StepCount and SteppingStatus to zero #ifdef ISOLATED_BOARD so that motors never run.
    SteppingStatus = 0;
    return;
#endif
    StepOverRatio = 0;

    Dx = X - AbsX; // Distance to move
    Dy = Y - AbsY; // Distance to move

    if ((Dx == 0) && (Dy == 0))
    { // 20240623: Explicitly exclude this case.
        StepCount = 0;
    } // Do nothing
    else
    {
        if (Dx > 0)
        {
            PORTD |= (1 << X_DIR); // Set X_DIR pin high
        }
        if (Dx < 0)
        {                           // 20240622 Saw PortD.4 oscillating in testing jogging panning.  This (ie excluding the 0 case) probably won't fix but worth trying.
            PORTD &= ~(1 << X_DIR); // Set X_DIR pin low
            Dx = Dx * -1;           // Convert all distances to a positive number
        }

        if (Dy > 0)
        {
            PORTD |= (1 << Y_DIR); // Set Y_DIR pin high
        }
        if (Dy < 0)
        {
            PORTD &= ~(1 << Y_DIR); // Set Y_DIR pin low
            Dy = Dy * -1;
        }

        if (Dx > Dy)
        {
            Master_dir = 0; // leading motor moves the most
            StepOverRatio = (Dy == 0) ? Dx : Dx / Dy;
            StepCount = Dx;
        }
        else
        {
            Master_dir = 1;
            // StepOverRatio = Dy / Dx;
            StepOverRatio = (Dx == 0) ? Dy : Dy / Dx;
            StepCount = Dy;
        }
    }
}

void MoveMotor(uint8_t axis, int steps, uint8_t waitUntilStop)
{
    if (axis == 0)
    { // pan
        X = steps;
    }
    else
    { // tilt
        Y = steps;
    }
    ProcessCoordinates();
    // CheckTimer1(0, 20);
    DSS_preload = HOMING_SPEED;
    SteppingStatus = 1;
    setupTimer1();
    if (waitUntilStop == 1)
    { // So, if waitUntilStop is not 1, execution returns ?  Yes returns to HomeMotor() where there is a (blocking) {while limit switch is low} condition.
// MoveMotor() is only called with waitUntilStop = 0 by HomeMotor() - ie when unit searching for limit switch (both pan & tilt separately)
#ifdef ISOLATED_BOARD
        DoHouseKeeping();
#endif
        while (SteppingStatus == 1)
        {
#ifdef DEBUG61
            static uint16_t cnt = 0;              // Test calling DHK in this blocking loop.
            static unsigned long prevTime = 0;    // Store the previous time
            unsigned long currentTime = millis(); // now(); // Get the current time

            if (cnt == DEBUG61_INTERVAL)
            {
                cnt = 0;
                unsigned long interval = currentTime - prevTime;                                                                  // Calculate the interval
                prevTime = currentTime;                                                                                           // Update the previous time
                snprintf(debugMsg, DEBUG_MSG_LENGTH, "Test DHK in MM. Interval: %lu ms, NbrPds: %d", interval, DEBUG61_INTERVAL); // Report the interval
                uartPrint(debugMsg);
            }
            cnt++;
#endif
            DoHouseKeeping();
            // do nothing while motor moves.  SteppingStatus is set to 0 at end of stepper ISR when StepCount !>0 (==0).
        }
    }
}

void StopSystem()
{
    StopTimer1();
    SteppingStatus = 0; // Reset stepping status
    X = AbsX;           // Set X and Y to their absolute values
    Y = AbsY;
    Dx = 0;
    Dy = 0;
}

void avoidLimits(bool axis)
{ // Don't allow jogging to take laser outside allowed range.
    // axis = false for pan and true for tilt. Set JogFlag (to one of 1,2,3,4) if limits are exceeded.
    static int loopCount = 0;
    loopCount++;
    JogFlag = 0;
    if (!axis)
    { // Pan axis case
        if (AbsX >= X_MAXCOUNT)
        {
            JogFlag = 1;
        }
        else if (AbsX <= X_MINCOUNT)
        {
            JogFlag = 2;
        }
        if ((PanDirection == 1 && JogFlag == 1) || (PanDirection == 0 && JogFlag == 2))
            PanEnableFlag = 0;
    }
    else
    { // Tilt axis
        if (AbsY >= static_cast<int>(y_maxCount))
        {
            JogFlag = 3;
        }
        else if (AbsY <= y_minCount)
        {
            JogFlag = 4;
        }
        if ((TiltDirection == 1 && JogFlag == 3) || (TiltDirection == 0 && JogFlag == 4))
            TiltEnableFlag = 0;
        _delay_ms(50);
    }
    if (JogFlag > 0)
    {
        if (!(loopCount % 2000))
        {
            snprintf(debugMsg, DEBUG_MSG_LENGTH, "JogFlag: %d, Y: %d, AbsY: %d,", JogFlag, Y, AbsY);
            uartPrint(debugMsg);
        }
    }
}

void JogMotors() // 20250107  Add and explicit stop call
{                //
    uint8_t axis = 0;
    uint8_t speed = 0;
    uint8_t dir = 0;
    static int lastX = 0;
    static int lastY = 0;
    JogFlag = 0;
    int pos = 0;
    avoidLimits(false);
    avoidLimits(true);
#ifdef NEW_APP
#ifdef LOG_PRINT
    if (X != lastX || Y != lastY)
    {
        printToBT(34, AbsX);
        printToBT(35, AbsY);
    }
#endif
#endif
    lastX = X;
    lastY = Y;

    if (PanEnableFlag == 1)
    {
        SteppingStatus = 1;
        axis = 0;
        speed = PanSpeed;
        dir = PanDirection;
        DSS_preload = (speed == 1) ? PAN_FAST_STEP_RATE : PAN_SLOW_STEP_RATE;
    }
    else
        X = AbsX;
    {
    }
    if (TiltEnableFlag == 1)
    {
        SteppingStatus = 1;
        axis = 1;
        speed = TiltSpeed;
        dir = TiltDirection;
        DSS_preload = (speed == 1) ? TILT_FAST_STEP_RATE : TILT_SLOW_STEP_RATE;
    }
    else
        Y = AbsY;

    pos = (speed == 1) ? HIGH_JOG_POS : LOW_JOG_POS; // Up to HIGH_JOG_POS steps per cycle through main loop or LOW_JOG_POS for slow.  Needs calibration. 20240629: Testing with 40:10.
#ifdef ISOLATED_BOARD
    pos = pos / isolated_board_factor;
#endif
    pos = pos * (dir ? 1 : -1);       // 20240629: See review in Avitech.rtf on this date.  Search on "Proposal to fix directions:"
    pos += (axis == 0) ? AbsX : AbsY; // 2024620: Add an amount, pos, to AbsX.  This becomes X (or Y) when MoveMotor is called. So (X - AbsX) is the increment.  When that is reached,
    // JogMotors would be called again. If the instruction (eg from <2:3>) has not been changed (eg by receipt of <2:0>) then the values set in cmd2() remain.  Accordingly pos increments
    // AbsX (which would have been incremented in previous calls to MoveMotor()) again.  So for the high speed case in which the increment passed is HIGH_JOG_POS, MoveMotor() should, given
    // the while loop, increment AbsX by HIGH_JOG_POS before returning to JogMotors then doing the same thing.  Need some debug statements to test this.

    if (PanEnableFlag == 0 && TiltEnableFlag == 0)
    { // If both pan and tilt are disabled, stop the motors.
        StopSystem();
    }
    else
    {
        setupTimer1(); // 20240620.  Could be only if necessary?

        if (axis == 0)
        {
            X = pos;
        }
        else
        {
            Y = pos;
        }
#ifdef NEW_APP
#ifdef LOG_PRINT
        if (X != lastX || Y != lastY) // Move this print here, before ProcessCoordinates() but after X or Y has been set relative to AbsX/AbsY.
        {
            // snprintf(debugMsg, DEBUG_MSG_LENGTH, "After SetupTimer1. AbsX: %d, AbsY: %d, X: %d, Y: %d", AbsX, AbsY, X, Y);
            // uartPrint(debugMsg);
            printToBT(34, AbsX);
            printToBT(35, AbsY);
        }
#endif
#endif
        ProcessCoordinates();
    }
    JogFlag = 0;
}
uint16_t CalcSpeed(bool fst)
{
    uint16_t s = 0;
    // uint8_t res = getCartFromTilt(AbsY);
    if (!fst)
    {
        int res = getCartFromTilt(AbsY);
        if (res < Rho_Min)
        {
            s = Step_Rate_Max;
        }
        else if (res > Rho_Max)
        {
            s = Step_Rate_Min;
        }
        else
        {
            long num = (long)(Rho_Max - res) * Step_Rate_Max + (long)(res - Rho_Min) * Step_Rate_Min;
            long den = (long)(Rho_Max - Rho_Min);
            s = (uint16_t)(num / den);
            snprintf(debugMsg, DEBUG_MSG_LENGTH, "num: %ld, den: %ld, s: %d", num, den, s);
            uartPrint(debugMsg);
        }
        s = static_cast<uint16_t>((static_cast<float>(s) * SpeedScale) / 100.0);
        if (s < Step_Rate_Max)
            s = Step_Rate_Max; // 20241204: Ensure that the speed is not too high.
        if (s > Step_Rate_Min)
            s = Step_Rate_Min; // 20241204: Ensure that the speed is not too low.
    }
    else
    {
        s = INTER_RUNG_SPEED;
    }
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Speed: %d, AbsY: %d", s, Y);
    uartPrint(debugMsg);
    return s;
}

void HomeMotor(uint8_t axis, int steps)
{ // Move specified motor until it reaches the relevant limit switch.
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "HM. axis:  %d,steps:  %d", axis, steps);
    // uartPrint(debugMsg);

    MoveMotor(axis, steps, 0);
    setupTimer1(); // 20240614 This added.  Shouldn't be necessary.

    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "Limit switches: P: %d, T: %d", (PINB & (1 << PAN_STOP)) != 0, (PINB & (1 << TILT_STOP)) != 0);
    // uartPrint(debugMsg);
#ifndef ISOLATED_BOARD
    if (axis == 0)
    {
        while (!(PINB & (1 << PAN_STOP)))
        { // While pan_stop pin is low do nothing while motor moves.
            // CheckTimer1(1, 10);
            // uartPrintFlash(F("HM3\n"));
            DoHouseKeeping();
        }
    }
    else
    {
        while (!(PINB & (1 << TILT_STOP)))
        { // While tilt_stop pin is low do nothing while motor moves.
            // CheckTimer1(2, 20);
            // uartPrintFlash(F("HM4\n"));
            DoHouseKeeping();
        }
    }
#endif
    StopTimer1();
    StepCount = 0;
    SteppingStatus = 0;

    if (axis == 0)
    {
        X = 0;
        AbsX = 0;
    }
    else
    {
        Y = 0;
        AbsY = 0;
    }
    // uartPrintFlash(F("HM5\n"));
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "LS end HM: P: %d, T: %d", (PINB & (1 << PAN_STOP)) != 0, (PINB & (1 << TILT_STOP)) != 0);
    // uartPrint(debugMsg);
}
void MoveLaserMotor()
{
    ProcessCoordinates();       // Drive motors to the coordinates
    DSS_preload = HOMING_SPEED; // Set speed rate
    SteppingStatus = 1;
#ifdef ISOLATED_BOARD
    SteppingStatus = 0;
#endif
    setupTimer1();
    while (SteppingStatus == 1)
    { // do nothing while motor moves
        // uartPrintFlash(F("In MLM"));
        DoHouseKeeping(); // 20240731 Add this. Perhaps to turn laser off.
    }
    StopTimer1();  // Stop the motor from stepping as sensor has been triggered.  20240615: The sensor has not been triggered. This is called when the target position is reached.
    StepCount = 0; // Clear the step count
}
void NeutralAxis()
{
    // PAN AXIS NEUTRAL
    // Neutral Position: (-4000,1000)
    // Pan motor to neutral position
    X = -4000; // Clockwise 90 deg pan
    MoveLaserMotor();
    // TILT AXIS HOME
    // Tilt motor to neutral position
    Y = 500; // Half way down
    MoveLaserMotor();
#ifdef ISOLATED_BOARD
    X = -4000;
    AbsX = X;
    Y = 500;
    AbsY = Y;
#endif
    Audio2(1, 2, 0, "Neut");
}

void HomeAxis()
{
    int Correctionstepping;
    setupTimer1();      // Probably not necessary.
    SetLaserVoltage(0); // Turn off laser
                        // *********PAN AXIS HOME****************
#ifndef ISOLATED_BOARD
    if ((PINB & (1 << PAN_STOP)))
    { // If pan_stop pin is high... "Move blade out of stop sensor at power up"
        // uartPrintFlash(F("Move from pan stop \n"));
        MoveMotor(0, -300, 1);
    }

    HomeMotor(0, 17000); // Pan motor to limit switch and set X and AbsX to 0 .
    // *********TILT AXIS HOME****************
    if ((PINB & (1 << TILT_STOP)))
    { // If tilt_stop pin is high (ie at limit). "Move blade out of stop sensor at power up"
        // uartPrintFlash(F("Move from tilt stop \n"));
        MoveMotor(1, 300, 1);
    }

    CmdLaserOnFlag = false; // 20240731
    SetLaserVoltage(0);
    HomeMotor(1, -5000); // Tilt motor homing position
#endif
#ifdef ISOLATED_BOARD
    X = 0;
    Y = 0;
    // uartPrintFlash(F("HomeAxis \n"));
#endif
    // --**Move Tilt into final position**--
    switch (OperationMode)
    {
    case 0:
        Correctionstepping = 100;
        break;
    case 1:
        Correctionstepping = -220;
        break; // 20240801: Has been -220.  But this is <0 so breaches AvoidLimits().
    case 2:
        Correctionstepping = 100;
        break;
    case 3:
        Correctionstepping = -220;
        break;
    default:
        Correctionstepping = 100;
        break;
    }

    MoveMotor(1, Correctionstepping, 1);
    CmdLaserOnFlag = false; // 20240731
    SetLaserVoltage(0);
    NeutralAxis(); // Take pan to -4000 and tilt to 500 (both magic numbers in NeutralAxis()).
    ClearSerial();
    // Audio2(2,1,1,"HA");
    CmdLaserOnFlag = false;
    SetLaserVoltage(0);
    IsHome = 1;
}
#ifdef TUNE_SPEED
void RunSweep(uint8_t zn)
#endif

#ifndef TUNE_SPEED
    void RunSweep(uint8_t zn)
{
    uint8_t PatType, ind = 0; // ind is incremented in getXY(), but only for rungs. So ind == 0 indicates being on the boundary.  It's used to count the number of rungs (and compare with Nbr_Rnd_Pts)
    int minTilt = 0, cnt = 0;
    int maxTilt = 0;
    int rhoMin = 0;
    bool runSweepStartRung = true; // 20241221.  This was startRung.  Change to runSweepStartRung to distinguish from that used (locally) in getXY().
    bool newPatt = true;
    static uint8_t lastPatt = 6; // Something that can't be valid
    static uint8_t lastZn = 10;
    SetLaserVoltage(0); // 20240727.  Off while GetPerimeter() is being calculated.
    LoadZoneMap(zn);
    printToBT(17, zn + 1); // zn passed to RunSweep is zero based.  App needs 1 based.  0 will indicate that no zone is running.
    getExtremeTilt(MapCount[0][zn], minTilt, maxTilt);
    uint16_t nbrRungs = getNbrRungs(maxTilt, minTilt, rhoMin); // rhoMin is set by this function
#ifdef GHOST
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "minTilt: %d, maxTilt: %d, rhoMin: %d", minTilt, maxTilt, rhoMin);
    uartPrint(debugMsg);
#endif
    for (PatType = 1; PatType <= 4; PatType++)
    {
        if ((lastPatt != PatType) || (zn != lastZn)) // If the zone changes but only one pattern is selected, we want the next pass to be treated as a new pattern.
            newPatt = true;
        else
        {
            newPatt = false;
            lastPatt = PatType;
            lastZn = zn;
        }
        if (ActivePatterns & (1 << PatType - 1)) // PatType runs from 1 to 4.  ActivePatterns stored in 4 least significant bits.
        {
            PatternRunning = PatType;
            printToBT(18, PatternRunning);

#ifdef LOG_PRINT // GHOST
            // static uint8_t LastPatType;
            // if (LastPatType != PatType)
            if (cnt == 0)
            {
                snprintf(debugMsg, DEBUG_MSG_LENGTH, "RS. Zone: %d Pattern: %d SpeedScale: %d Rungs: %d, X: %d, Y: %d", zn, PatType, SpeedScale, nbrRungs, X, Y);
                uartPrint(debugMsg);
            }
#endif

            while (((ind <= Nbr_Rnd_Pts) && (ActivePatterns & (1 << (PatType - 1))) && (ActiveMapZones & (1 << zn)))) //// ind is incremented in getXY(), but not for the boundary, only for rungs.
            {
#ifdef TEST_PATH_MODE
#endif
#ifdef ISOLATED_BOARD
                _delay_ms(ISOLATED_BOARD_DELAY);
                // static uint16_t cnter = 0;
                // if (!(cnter == TJTick)){
                //     cnter = TJTick;
                // }

#endif
                runSweepStartRung = getXY(PatType, zn, ind, newPatt, rhoMin, nbrRungs, minTilt, maxTilt);
                newPatt = false; // This is set on entry to the loop over PatType. Once in this while loop, it should be false until the next pattern is called.
                // if (cnt == 0 && PatType == 1)
                if (cnt == 0)
                { // 20240727:Laser off as it moves towards 0th point of pattern.
                    CmdLaserOnFlag = false;
                    SetLaserVoltage(0);
                    DSS_preload = Step_Rate_Max;
                }
                else
                {
                    if ((PatType == 2 && runSweepStartRung))
                    { // Laser off and high speed if going to start of rung in pat 2
                        CmdLaserOnFlag = false;
                        SetLaserVoltage(0);
                        DSS_preload = CalcSpeed(true); // Passing true makes the speed fast, independent of tilt angle.
                    }
                    else
                    {
                        if (!SystemFaultFlag)
                        {
                            CmdLaserOnFlag = true;
                            SetLaserVoltage(LaserPower);
                        }
                        DSS_preload = CalcSpeed(false); // Passing false makes the speed depend on tilt angle.
                    }
                }
                // if (testInternal(zn,[X,Y]) > 0) // Test that autofill point is internal to zone.  Not necessary for boundary?
                // {
                // }
                ProcessCoordinates();
                SteppingStatus = 1;
                if (!SystemFaultFlag)
                    setupTimer1();

                while (SteppingStatus == 1)
                {
                    DoHouseKeeping();
                    if (SetupModeFlag == 1)
                    { // Will only arise if SetupModeFlag is changed after RunSweep is called (which will occur if SetupModeFlag == 0 - run mode.)
                        StopTimer1();
                        StepCount = 0;
                        SteppingStatus = 0;
                        return;
                    }
                }
                cnt++; // 20241120: Count the number of times through the while loop and exit if it gets too long.
                if (cnt > 1000)
                {
                    cnt = 0;
                    break;
                }
            }
            cnt = 0;
            ind = 0;
        }
        printToBT(18, 0);   // Set current pattern to zero
        SetLaserVoltage(0); // 20250108
        CmdLaserOnFlag = false;
    }
    ResetTiming();
}
#endif

#ifdef NEW_APP
// void TraceBoundary(uint8_t zone)
// {
//     static uint8_t Current_Nbr_Rnd_Pts = 0;
//     Current_Nbr_Rnd_Pts = Nbr_Rnd_Pts; // Store Nbr_Rnd_Pts so that it can be reset after this check.  It determines the number of rungs in an autofill.
//     Nbr_Rnd_Pts = 0;                   // Set to zero so that RunSweep() only does the boundary.
//     getMapPtCounts();                  // Load zone data (count of vertices by zone) from eeprom to MapCount array
//     LoadZoneMap(zone);
//     RunSweep(zone);
//     Nbr_Rnd_Pts = Current_Nbr_Rnd_Pts; // Reset to stored value so that the stored value is used in run mode.
// }
#endif
// Define the command to variable map, initially only for TransmitData
// std::map<uint8_t, int32_t> commandToVariable = {
//     // map<uint8_t, int32_t> commandToVariable = {
//     {20, MaxLaserPower},
//     {30, Accel.Acceltemp},
//     {31, Index},
//     {32, X},
//     {33, Y},
//     {34, AbsX},
//     {35, AbsY},
//     {36, Accel_Z.Z_accel},
//     {37, Tod_tick / 2}, // Tod_tick increments every 500ms. Should this be *2 rather than /2 to get seconds?
//     {38, BattVoltAvg},
//     {39, Frame_size},
//     {42, Wd_flag},
//     {43, Boardrevision},
//     {44, LaserID},
//     {46, DSS_preload},
//     {47, AccelTripPoint},
//     {48, ResetSeconds / 2},
//     {49, OperationMode}};
// void TransmitData()
// {
//     for (const auto &entry : commandToVariable)
//     {
//         uint8_t command = entry.first;
//         int variable = entry.second;
//         printToBT(command, variable);
//     }
// }
// void TransmitData()
// { //Would prefer to use map - std not available to ATmega328PB compiler.
//     const uint8_t commands[] = {
//         20, 30, 31, 32, 33, 34, 35, 36, 37, 38, 42, 43, 44, 46, 47, 48, 49
//     };

//     const int32_t variables[] = {
//         MaxLaserPower, Accel.Acceltemp, Index, X, Y, AbsX, AbsY, Accel_Z.Z_accel,
//         Tod_tick / 2, BattVoltAvg, Wd_flag, Boardrevision, LaserID,
//         DSS_preload, AccelTripPoint, ResetSeconds / 2, OperationMode
//     };

//     for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i)
//     {
//         uint8_t command = commands[i];
//         int32_t variable = variables[i];
//         if (command == 37) {
//             variable /= 2; // Adjust Tod_tick as needed
//         }
//         printToBT(command, variable);
//     }
// }

void TransmitData()
{
    int Variables[17];
    uint8_t i;

    Variables[0] = MaxLaserPower;   // 20
    Variables[1] = Accel.Acceltemp; // 30
    Variables[2] = Index;           // 31
    Variables[3] = X;               // 32
    Variables[4] = Y;               // 33
    Variables[5] = AbsX;            // 34
    Variables[6] = AbsY;            // 35
    Variables[7] = Accel_Z.Z_accel; // 36
    Variables[8] = Tod_tick / 2;    // 37 Tod_tick increments every 500ms.  Should this be *2 rather than /2 to get seconds?
    Variables[9] = BattVoltAvg;     // 38
    // Variables[10] = Hw_stack;
    // Variables[11] = Sw_stack;
    // Variables[10] = Frame_size;
    Variables[10] = Wd_flag;          // 42
    Variables[11] = Boardrevision;    // 43
    Variables[12] = LaserID;          // 44
    Variables[13] = DSS_preload;      // 46
    Variables[14] = AccelTripPoint;   // 47
    Variables[15] = ResetSeconds / 2; // 48
    Variables[16] = OperationMode;    // 49

    if (SendDataFlag == 1)
    {
        for (i = 0; i <= 16; i++)
        {
            if (i == 0)
                printToBT(20, Variables[i]);
            else if (i <= 9)
                printToBT(i + 29, Variables[i]);
            else if (i <= 12)
                printToBT(i + 32, Variables[i]);
            else
                printToBT(i + 33, Variables[i]);
        }
    }
}

void PrintAppData()
{

    if (BT_ConnectedFlag == 1) // 20250108 Test with this removed.
    {
        printToBT(17, MapRunning);
        printToBT(18, PatternRunning);

        if (SetupModeFlag == 0)
        {
            printToBT(2, LaserTemperature);
            printToBT(3, BatteryVoltage);
            printToBT(21, UserLaserPower);
            printToBT(25, LightLevel);
            printToBT(45, LaserPower);
        }
    }
}
void PrintConfigData()
{

    uint8_t i;
    uint16_t ConfigData[12];
    uint8_t ConfigCode[12];

    // PrintZoneData();

    ConfigData[1] = ActiveMapZones;
    ConfigCode[1] = 15;

    ConfigData[2] = MapTotalPoints;
    ConfigCode[2] = 4; // 20240618: This is already printed from PrintZoneData()

    ConfigData[3] = eeprom_read_byte(&EramMicroMajor); //_version_major;
    ConfigCode[3] = 23;

    ConfigData[4] = eeprom_read_byte(&EramMicroMinor); //_version_minor;
    ConfigCode[4] = 24;

    ConfigData[5] = UserLightTripLevel;
    ConfigCode[5] = 26;

    ConfigData[6] = LightTriggerOperation;
    ConfigCode[6] = 27;

    ConfigData[7] = Laser2OperateFlag;
    ConfigCode[7] = 28;

    ConfigData[8] = FactoryLightTripLevel;
    ConfigCode[8] = 29;

    ConfigData[9] = Laser2TempTrip;
    ConfigCode[9] = 50;

    ConfigData[10] = Laser2BattTrip;
    ConfigCode[10] = 51;

    ConfigData[11] = NIGHT_TRIP_TIME_FROM_STARTUP / 2;
    ConfigCode[11] = 53;

    for (i = 1; i <= 11; i++)
    {
        printToBT(ConfigCode[i], ConfigData[i]);
        // snprintf(debugMsg, DEBUG_MSG_LENGTH, "i: %d, val: %d", i, ConfigData[i]);
        // uartPrint(debugMsg);
    }
}

void sendStatusData()
{
    TransmitData();
    PrintAppData();
    PrintConfigData();
}

void ProgrammingMode()
{
    SetLaserVoltage(0); // Turn off laser
#ifndef HOME_AXIS
    HomeAxis();
#endif
#ifdef HOME_AXIS
    uartPrintFlash(F("HomeAxis removed from ProgMode \n"));
    _delay_ms(1000);
    IsHome = 1;
#endif
    // #ifdef ISOLATED_BOARD
    //     _delay_ms(1000);
    // #endif
    // uartPrintFlash(F("ProgMode3 \n")); //This overwhelms <9:1>
    printToBT(9, 1);
}

void OperationModeSetup(int OperationMode)
{
    const char *name;
    ClearSerial();
    _delay_ms(2000); // Wait for 2 seconds.  TJ 20240614: Why?
    switch (OperationMode)
    {
    case 0:
        name = "Field-i";
        y_minCount = 20; // 20 steps with TILT_STEPS_PER_RAD 2052.0 implies tan of 0.01R.  LaserHt = 5m implies 500m range.
        y_maxCount = 1800;
        strcpy(OpModeTxt, "0:Field-i");
        break;
    case 1:
        name = "Roof-i";
        y_maxCount = 3250;
        strcpy(OpModeTxt, "1:Roof-i");
        break;
    case 2:
        name = "Orchard-i";
        y_maxCount = 2000;
        strcpy(OpModeTxt, "2:Orchard-i");
        break;
    case 3:
        name = "Eve-i";
        y_maxCount = 4095;
        strcpy(OpModeTxt, "3:Eve-i");
        break;
    case 4:
        name = "Test_i";
        y_maxCount = 2500;
        strcpy(OpModeTxt, "4:Test-i");
        break;
    }
    // testLoopUart("OMS: >switch ");
    char atCommand[50];
    sprintf(atCommand, "AT+NAME%s %d\r\n", name, LaserID);
    uartPrint(atCommand);
    // testLoopUart("OMS: end function ");
}

void DoHouseKeeping()
{
    CheckBlueTooth();
    ReadAccelerometer();
    DecodeAccelerometer();

#ifdef ISOLATED_BOARD
    // static uint16_t lastTick;
    // isolated_board_flag = false;
    // // if (TJTick >= lastTick + (ISOLATED_BOARD_INTERVAL * isolated_board_factor)) // TJTick increments every 50ms.
    // {
    //     lastTick = TJTick;
    //     X = AbsX;
    //     Y = AbsY;
    //     StepCount = 0;
    //     SteppingStatus = 0;
    //     isolated_board_flag = true; // Reset to true, so that notional movement occurs, if there has been an interval as defined above
    // }
    // Z_AccelFlag = false;
    // LaserTemperature = 50;
    // SystemFaultFlag = false;
#endif
    // avoidLimits(true); //20240731
    // avoidLimits(false);
    if (Tick > 4)
    {
        TransmitData();
        PrintAppData(); // 20241209.  Too much data in log.  Send on refresh.
        GetLaserTemperature();
        GetLightLevel(); // 20250109.  Add this here for testing.
                         // #ifdef THROTTLE
                         //         ThrottleLaser();
                         // #endif
        if (Z_AccelFlag == false)
        {
            ThrottleLaser();
        }
        Tick = 0;
    }

    if (AbsY >= TILT_SENSOR_IGNORE)
    {
        if ((PINB & (1 << TILT_STOP)))
        { // if Tilt_stop pin is high
#ifndef ISOLATED_BOARD
            uartPrintFlash(F("ST3 TL \n"));
            StopTimer3(); // 20241205: This will cause the watchdog to trigger.
            _delay_ms(WATCHDOG_DELAY);
#endif
            // uartPrintFlash(F("StopTimer3 would have been called.\r"));
        }
    }
#ifdef ISOLATED_BOARD
    wdt_reset();
#endif
    // if (SystemFaultFlag == 1) {
    if (SystemFaultFlag)
    { // 20241129
        SetLaserVoltage(0);
        StopTimer1();
        SteppingStatus = 0;
        // if (!(dhkn % 3000))
        // {
        //     snprintf(debugMsg, DEBUG_MSG_LENGTH, "DHKerr. ZFlag: %d, ZVal: %d, LaserTemp: %d", Z_AccelFlag, Accel_Z.Z_accel, LaserTemperature);
        //     uartPrint(debugMsg);
        // }
        ProcessError();
        return;
    }

    if (SetupModeFlag == 1)
    {
        WarnLaserOn();
        StartLaserFlickerInProgMode();
        // 20240725 Although this stop criterion is implemented in JogMotors(), timing indicates that it is also needed here.
        // 20250115: This was taken out after HomeAxis() no longer worked and hence would not allow switching to prog mode.
        // if (PanEnableFlag == 0 && TiltEnableFlag == 0)
        { // If both pan and tilt are disabled, stop the motors.
          // uartPrintFlash(F("Stop. No enable flags \n"));
          // StopSystem();
        }
    }

    if (SetupModeFlag == 0)
    {
        IsHome = 0;
        WaitAfterPowerUp();
        if (CmdLaserOnFlag && MapTotalPoints >= 2)
        // if (MapTotalPoints >= 2)
        {
            WarnLaserOn();
            if (!SystemFaultFlag)
            {
                SetLaserVoltage(LaserPower);
                CmdLaserOnFlag = true;
            }
            else
            {
                SetLaserVoltage(0);
                CmdLaserOnFlag = false;
                GetLightLevel();
            }
        }

        if (Tod_tick > NIGHT_TRIP_TIME_FROM_STARTUP)
        {
            if (BT_ConnectedFlag == 0 && LightSensorModeFlag == 1)
            {
                MrSleepyTime();
            }
        }
    }
}

// Property get functions:
// uint8_t getBatteryVoltageAdc()
// {
//     return static_cast<uint8_t>(BatteryVoltage);
// }

uint8_t getTimeMode()
{
    switch (LightTriggerOperation)
    {
    case 0:
        return static_cast<uint8_t>(TimeMode::always);
    case 1:
        return static_cast<uint8_t>(TimeMode::day);
    case 2:
        return static_cast<uint8_t>(TimeMode::night);
        // default:
        //     return TimeMode::always;
    }
}
uint8_t getBeamMode()
{
    // switch (????)
    // {
    // case 0:
    //     return static_cast<uint8_t>(BeamMode::continuous);
    // case 1:
    //     return static_cast<uint8_t>(BeamMode::continuousPulsing);
    // }
}
uint8_t getLocationMode()
{
    // switch (????)
    // {
    // case 0:
    //     return static_cast<uint8_t>(LocationMode::outdoor);
    // case 1:
    //     return static_cast<uint8_t>(LocationMode::indoor);
    // case 2:
    //     return static_cast<uint8_t>(LocationMode::indoorInverted);
    // }
}

// uint8_t getTripodHeight() { return LaserHt; }    // LaserHt is stored in decimetres.  Eg LaserHt = 50 is 5.0m.
// uint8_t getLineSeparation() { return Tilt_Sep; } // Tilt_Sep is metres (in Cartesian space)
// uint8_t getLinesPerPattern() { return Nbr_Rnd_Pts; }
// uint8_t getActiveMapZones() { return ActiveMapZones; }
// uint8_t getActivePatterns() { return ActivePatterns; }
// uint8_t getMaxLaserPower() { return MaxLaserPower; }
// uint8_t getUserLaserPower() { return UserLaserPower; }
// uint8_t getCurrentLaserPower() { return LaserPower; }
// uint8_t getLaserTemperature() { return LaserTemperature; } // void GetLaserTemperature() assigns a value to global uint8_t LaserTemperature
uint8_t getRandomizeSpeed() { return 0; } // TJ: I don't know what this is supposed to do.
uint8_t getSpeedScale()
{
    uint16_t r = ReScale(SpeedScale, OLD_SPEED_ZONE_MIN, OLD_SPEED_ZONE_MAX, SPEED_SCALE_MIN, SPEED_SCALE_MAX, false);
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "Invert SpeedScale %d,  r %d", SpeedScale, r);
    // uartPrint(debugMsg);
    return r;
}
uint8_t getLightSensorReading() { return LightLevel; } // LightLevel is a long.  This needs to be investigated further.

// Property send functions

void sendProperty(FieldDeviceProperty property, uint8_t value)
{
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "args to sendProp. Prop: %d, val: %d", property, value);
    // uartPrint(debugMsg);
    uint16_t result = (static_cast<uint8_t>(property) << 8) | value;
    printToBT(PROPERTY_GET_CHANNEL, result);
}

void handleGetPropertyRequest(FieldDeviceProperty property)
{
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "arg to hGPR %d", property);
    // uartPrint(debugMsg);

    switch (property)
    {
    case FieldDeviceProperty::batteryVoltAdc:
        sendProperty(property, BatteryVoltage);
        break;
    case FieldDeviceProperty::timeMode:
        sendProperty(property, getTimeMode());
        break;
    case FieldDeviceProperty::beamMode:
        sendProperty(property, getBeamMode());
        break;
    case FieldDeviceProperty::locationMode:
        sendProperty(property, getLocationMode());
        break;
    case FieldDeviceProperty::tripodHeight:
        sendProperty(property, LaserHt);
        break;
    case FieldDeviceProperty::lineSeparation:
        sendProperty(property, Tilt_Sep);
        break;
    case FieldDeviceProperty::linesPerPattern:
        sendProperty(property, Nbr_Rnd_Pts);
        break;
    case FieldDeviceProperty::activeMapZones:
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "ActZones: %d", ActiveMapZones);
        uartPrint(debugMsg);
        sendProperty(property, ActiveMapZones);
        break;
    case FieldDeviceProperty::activePatterns:
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "ActPats: %d", ActivePatterns);
        uartPrint(debugMsg);
        sendProperty(property, ActivePatterns);
        break;
    case FieldDeviceProperty::maxLaserPower:
        sendProperty(property, MaxLaserPower);
        break;
    case FieldDeviceProperty::userLaserPower:
        sendProperty(property, UserLaserPower);
        break;
    case FieldDeviceProperty::currentLaserPower:
        sendProperty(property, LaserPower);
        break;
    case FieldDeviceProperty::laserTemperature:
        sendProperty(property, LaserTemperature);
        break;
    case FieldDeviceProperty::randomizeSpeed:
        sendProperty(property, getRandomizeSpeed());
        break;
    case FieldDeviceProperty::speedScale:
        sendProperty(property, getSpeedScale());
        break;
    case FieldDeviceProperty::lightSensorReading:
        sendProperty(property, getLightSensorReading());
        break;
    case FieldDeviceProperty::deviceMode:
        sendProperty(property, SetupModeFlag); // 0 Run  mode, 1 prog mode?
        break;
    case FieldDeviceProperty::currentZoneRunning:
        sendProperty(property, MapRunning);
        break;
    case FieldDeviceProperty::currentPatternRunning:
        sendProperty(property, PatternRunning);
        break;
    case FieldDeviceProperty::microMajor:
        MicroMajor = eeprom_read_byte(&EramMicroMajor);
        sendProperty(property, MicroMajor);
        break;
    case FieldDeviceProperty::microMinor:
        MicroMinor = eeprom_read_byte(&EramMicroMinor);
        sendProperty(property, MicroMinor);
        break;
    }
}

void testWatchDog(uint8_t indicator)
{
    static uint8_t n = 0;
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "testWD, no trigger %d", indicator);
    uartPrint(debugMsg);
    // Check if Timer3 ISR was triggered
    // if (TJTick % 40 > 10)
    // {
    //     n++;
    //     if (n > 4)
    //     {
    //         snprintf(debugMsg, DEBUG_MSG_LENGTH, "In main. TJTick %d", TJTick);
    //         uartPrint(debugMsg);
    //         n = 0;
    //     }
    // }
    if (timer3_isr_triggered)
    {
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "T3 %d", indicator);
        uartPrint(debugMsg);
        timer3_isr_triggered = false; // Reset the flag
    }

    // Check if Watchdog ISR was triggered
    if (watchdog_isr_triggered)
    {
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "WD %d", indicator);
        uartPrint(debugMsg);
        watchdog_isr_triggered = false; // Reset the flag
    }
}

void setup()
{
    // wdt_disable();  //Moved to main().
    sei(); // Enable global interrupts.
    // wdt_reset(); //ref https://forum.arduino.cc/t/atmega328-locks-up-on-watchdog-reset/921269/5
    uart_init(MYUBRR); // Initialize UART with baud rate specified by macro constant
    _delay_ms(2000);   // More time to connect and not, therefore, miss serial statements.

    bool IB = false;
#ifdef ISOLATED_BOARD
    IB = true;
#endif

    snprintf(debugMsg, DEBUG_MSG_LENGTH, "In setup, IB: %d", IB);
    uartPrint(debugMsg);

    setupTimer1();
    setupTimer3();
#ifndef ISOLATED_BOARD
    TurnOnGyro();
#endif
    setupPeripherals();

    OperationModeSetup(OperationMode); // Select the operation mode the device will work under before loading data presets
    Wd_byte = MCUSR;                   // Read the Watchdog flag
    if (Wd_byte & (1 << WDRF))
    {                // there was a WD overflow. This flag is cleared of a Watchdog Config
        Wd_flag = 1; // store the flag
    }
    Wire.begin(); // Initialize I2C
    if (!DAC.begin())
    {
        uartPrintFlash(F("DAC.begin() returned false."));
        return;
    }; // DAC.begin(MCP4725ADD>>1); // Initialize MCP4725 object.  Library uses 7 bit address (ie without R/W)
    // DAC.setMaxVoltage(5.1);  //This may or may not be used.  Important if DAC.setVoltage() is called.
    firstOn();           // Load defaults to EEPROM if first time on.
    ReadEramVars();      // Reads user data from EEPROM to RAM.
    initMPU();           // 20241231 This was commented out.  It will error if the wrong address for the given board is in EEPROM (and read from preious statement)
    PORTE |= (1 << FAN); // Turn fan on.
    for (int battCount = 0; battCount < 10; battCount++)
    {                        // BatteryVoltage should be populated by this.  20241205 The limit, 10, is also the size of the relevant array. Change to use a constant?
        GetBatteryVoltage(); // Load up the battery voltage array
    }
    SetLaserVoltage(0);           // Lasers switched off
    PORTD &= ~(1 << X_ENABLEPIN); // Enable pan motor (active low ).
    PORTD &= ~(1 << Y_ENABLEPIN); // Enable tilt motor.
// Audio2(5,1,1);//,"Setup");
#ifndef ISOLATED_BOARD
    setupWatchdog(); // 20250111: Move from earlier in setup().  Ref USBASP_Q in Avitech.rtf, this date.
#endif
    // _delay_ms(2000); // 20250112 Without this delay, DecodeAccelerometer() trips as
    ReadAccelerometer();
    DecodeAccelerometer(); // 20250112: Test before any movement.
    DoHouseKeeping();
    // snprintf(debugMsg, DEBUG_MSG_LENGTH, "LS: P: %d, T: %d", (PINB & (1 << PAN_STOP)) != 0, (PINB & (1 << TILT_STOP)) != 0);
    // uartPrint(debugMsg);

    HomeAxis();
    // uartPrintFlash(F("Would normally HomeAxis here.\n"));
    sendStatusData();
}

int main()
{
    wdt_disable();
    setup();
    while (1)
    {
        switch (SetupModeFlag)
        {
        case 1:
        {
            if (PrevSetupModeFlag != SetupModeFlag)
            {
                Audio2(2, 18, 8); //,"ToMode1");
                _delay_ms(2000);
                PrevSetupModeFlag = SetupModeFlag;
            }
            JogMotors();
            break;
        }
        case 0:
        { // In run mode
            if (PrevSetupModeFlag != SetupModeFlag)
            {
                // eeprom_update_byte(&EramMapTotalPoints, MapTotalPoints);
                getMapPtCounts(); // Any need to call getMapPtCounts?  Doesn't need to be called every time.  Once at end of setup and at first time after setup.
                Audio2(3, 10, 5); //,"ToMode0");
                _delay_ms(2000);
                PrevSetupModeFlag = SetupModeFlag;
            }
            if (MapTotalPoints > 0)
            {
                for (Zn = 1; Zn <= NBR_ZONES; Zn++)
                {
                    if ((ActiveMapZones == 0) || (ActivePatterns == 0))
                        SetLaserVoltage(0);
                    if ((ActiveMapZones & (1 << (Zn - 1))) != 0)
                    {
                        if (MapCount[0][Zn - 1] > 4) // MapCount index is zero based for zone - But corresponding index in app is 1 based.
                        // 20250120 Previously if (MapCount[0][Zn - 1] > 0).  But although zero based, MC[0][z] adds a repeated first point - so MC[0][z] is 1 when there are zero points.
                        {
                            MapRunning = Zn;
                            RunSweep(Zn - 1);
                            printToBT(17, 0);   // Set MapRunning to zero after RunSweep.  It will be reset in RunSweep if that is called again.
                            SetLaserVoltage(0); // Laser needs to be off between zones.
                        }
                    }
                }
            }
            else
                SetLaserVoltage(0); // 20240718: Standby mode
            break;
        }
        case 2:
        {
            CalibrateLightSensor();
            break;
        }
        }
        DoHouseKeeping();
    }
    testWatchDog(2);
    return 0;
}

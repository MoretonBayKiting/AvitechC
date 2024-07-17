#pragma region Include files
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
    #include "shared_Vars.h"
    #include "pin_mappings.h"
    #include "const.h"
    #include "LoadDefs.h"
    // #include "Execute_App_Command_Module/Execute_App_Command_Module_Rev_2_00.h"
    // #include "Validate_EERAM_Module/Validate_EERAM_Module.h"
    // twi.h has #define TW_STATUS		(TWSR & TW_STATUS_MASK).  But TWSR, applicable for ATmega328P should be TWSR0 for ATmega328PB.
    #undef TW_STATUS
    #define TW_STATUS		(TWSR0 & TW_STATUS_MASK)

#pragma endregion Include files
#pragma region Variable definitions
MCP4725 DAC(0x60);// (MCP4725ADD>>1);
uint8_t printCnter = 0;
// uint16_t eeprom_address = 0;

// volatile uint16_t n = 0;  //Counter used in debugging.
uint16_t Boardrevision;                                   // Board that program will be deployed to
int X = 0;                                            // X position requested to move EG X=100   .Move X 100 steps
int Y = 0;                                            // Y position requested to move EG X=100   .Move X 100 steps
int AbsX = 0;                                        // X absolute position from home position
int AbsY = 0;                                         // Y absolute position from home position
int Dy;                                              // Used to calulate how many steps required on Y axis from last position
int Dx;                                              // Used to calulate how many steps required on X axis from last position

bool rndLadBit;                                        // 0 or 1 to indicate if last pass was for a new rung on one side or the next side is needed
uint8_t minYind;
uint8_t maxYind;
uint8_t RndNbr, PrevRndNbr, AltRndNbr;

volatile uint16_t StepCount;                                       // How many steps to be executed
uint16_t DSS_preload = STEP_RATE_MAX;                                     // TCC1 compare value to get Desired Stepping Speed (DSS), Lower the number the faster the pulse train
volatile bool SteppingStatus = 0;
// bool SteppingStatus = 0;  //Shared   // 0 = No stepping in progress. 1= Stepping in progress
char buffer[20];  //Use for printint to serial (Bluetooth)
#ifndef NDEBUG
char debugMsg[DEBUG_MSG_LENGTH];  // Buffer for debug messages
#endif
char OpModeTxt[12];
int StepOverRatio;                                // Step over ratio between Pan And tilt
uint16_t Remainder;                                       // Used for Step Over Ratio calculation
bool Master_dir;                                       // Which motor need to step the most X or Y
uint16_t TJTick;
uint8_t Tick;                                            // Tick flag. 2Hz update time
uint8_t AccelTick;                                       // Tick flag. 2Hz update time
uint32_t Tod_tick;                                       // Tick for system time of day
uint8_t BuzzerTick;                                      // Tick flag. 2Hz update time
uint8_t LaserTick;                                       // Tick flag. 20Hz update time

uint8_t Command;                                         // Holds the Command value fron the RS232 comms string.See documentation on the Comms data string build
uint16_t Instruction;                                     // Holds the data value fron the RS232 comms string.See documentation on the Comms data string build

EramPos EEMEM EramPositions[MAX_NBR_MAP_PTS];               // Use eeprom for waypoints
uint8_t EEMEM EramMapTotalPoints; //May not be used
uint8_t MapTotalPoints;                                 
bool X_TravelLimitFlag = false;                                // Over travel flag
bool Y_TravelLimitFlag = false;                                // Over travel flag
uint16_t y_maxCount;                                   // Max Count Y can have. This changes for different mode as more or less tilt is required in different modes
uint8_t MapCount[2][NBR_ZONES] = {}; // MapCount[1][i] is incremental; MapCount[2][i] is cumulative of MapCount[1][i]
uint8_t Zn;
// uint8_t NoMapsRunningFlag;  //Although this is set (BASCOM), it doesn't appear to be used.
// uint16_t AppCompatibilityNo;
uint8_t GyroAddress = MPU6000_ADDRESS; // 0x69 for Board 6.13 and later (MPU6050).  0x68 for earlier boards (MPU6000).  Set with <11:4> (0x69) and <11:8> (0x69) and store in EramGyroAddress
//Note that these addresses are 7 bit addresses.  With r/w bits these would be 0xD2/0xD3 for 0x69 and 0xD0/0xD1 for 0x68.
uint8_t EEMEM EramGyroAddress;

//----Set up for Watchdog timer------------------
uint8_t Wd_flag; // Read the flag first, as configuring the watchdog clears the WDRF bit in the MCUSR register
uint8_t Wd_byte;


//---------Commands from Bluetooth input--------------
uint8_t PanEnableFlag; // X enable axis flag when jogging mode
uint8_t PanDirection; // X rotation requested direction flag
uint8_t PanSpeed; // X stepping speed flag... 1=fast 0=slow
uint8_t TiltDirection; // Y rotation requested direction flag
uint8_t TiltEnableFlag; // Y enable axis flag when jogging mode
uint8_t TiltSpeed; // Y stepping speed flag... 1=fast 0=slow

//---------Laser Variables------------------------
uint8_t UserLaserPower;
uint8_t EEMEM EramUserLaserPower;

uint8_t MaxLaserPower;
uint8_t EEMEM EramMaxLaserPower;
uint8_t LaserPower = 0; // Final calculated value send to the DAC laser driver

float VoltPerStep = LINE_VOLTAGE / 4095; // Laser power per step. Could be macro constant.  
// Input voltage ie 5 volts /12bit (4095) MCP4725 DAC = Voltage step per or 0.0012210012210012 Volt per step

uint8_t LaserOverTempFlag; // Laser over temp error flag
uint8_t FlashTheLaserFlag; // Setup laser flash flag bit
bool CmdLaserOnFlag = false;

uint8_t SetupModeFlag = 0; // Should this be set to 1 (setup mode - 0 is run mode) as default? 0 in BASCOM version.

uint8_t Laser2StateFlag; // Sets the current state if the laser 2 is working. 0=off 1=on

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

//----------Communication Variables--------------------
uint8_t A;
char S[10];//20240607: This should only need to be char[4] // Variables to decode data coming in from the RS232 data stream from the phone Commands
bool DataInBufferFlag; // Flag for there is data in the RS232 comms buffer

//----------Speed Zone Variables--------------------
uint8_t SpeedZone[5];
uint8_t EEMEM EramSpeedZone[5];

//----------Battery Charge Variables--------------------
uint16_t BattReadings[10];
uint8_t BattReadIndex;
uint16_t BattTotal;
uint16_t BattVoltAvg;
uint8_t BatteryVoltage; // Change this to single if you want to calc voltage on the micro
uint8_t BatteryTick; // Battery tick sampling rate tick's
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

long LightLevel; // Holds the value of the ADC light sensor
uint8_t LightSensorModeFlag;
//---------Other Variables-------------------------
uint8_t BT_ConnectedFlag;

uint8_t EEMEM EramOperationMode;
uint8_t OperationMode;

uint8_t EramActiveMapZones;
uint8_t ActiveMapZones;

uint8_t SendDataFlag = 0;                                      // Diagnostic mode send data back to user
uint8_t SendSetupDataFlag;                                 // Send config data back to user.  20240522: Not used 

uint8_t FirstTimeLaserOn;
uint8_t WarnLaserOnOnce;
uint8_t IsHome;

long Secondsfromreset;                                     // Variable to hold the reset current count

bool SystemFaultFlag;
uint8_t Index;

uint16_t EEMEM EramLaserID;
uint16_t LaserID;

int EEMEM EramAccelTripPoint;                                 // Accelerometer trip angle value
int AccelTripPoint;

uint8_t Zonespeed;
uint16_t CurrentSpeedZone;

uint16_t MapRunning;

uint8_t Counter50ms;
//Counters to use for periodic printing while debugging.
uint32_t JM_n=0; 
uint32_t MM_n=0;

//---------MPU6050 IMU  & Temperature, then HC-05 bluetooth module-----------------
union { // Accel_Z to be used to access any of Z_accel, int or the subcomponents
    int Z_accel;  // Accelerometer uses 2 bytes of memory. Data is read from the chip in 2 lots of 1 byte packets
    struct {
        uint8_t Zl_accel;       // Stored memory of first byte data from accelerometer
        uint8_t Zh_accel;       // Stored memory of second byte data from accelerometer
    };
} Accel_Z;

uint8_t Z_AccelFlag = 0;
uint8_t Z_AccelFlagPrevious = 0;

union {
    int Acceltemp;
    struct {
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
uint16_t Frame_size;
uint16_t ResetSeconds;
bool received39;
int Vertices[3][MAX_NBR_VERTICES]; //Columns: X, Y, Slope?
int Perimeter[2][MAX_NBR_PERIMETER_PTS];
uint8_t NbrPerimeterPts;
int res[2];  //Global variables to be used in cartesian/polar conversions:Input and results.
volatile char ReceivedData[BUFFER_SIZE];
char RecdDataConst[BUFFER_SIZE];  
volatile int DataCount = 0;
volatile uint8_t CommandLength = 0;
// uint8_t cnter = 0;

#pragma endregion Variable Definitions
#pragma region Function Declarations
void HomeAxis();
void DoHouseKeeping();
void StopTimer1();
#pragma endregion Function Declarations
#pragma region Function definitions - peripherals, watchdog, uart, buzzer, timers, DAC, i2c
void setupPeripherals(){
    // Set relevant pins as output
    DDRD |= (1 << X_ENABLEPIN) | (1 << Y_ENABLEPIN) | (1 << X_DIR) | (1 << Y_DIR) | (1 << X_STEP) | (1 << Y_STEP);
    DDRE |= (1 << BUZZER) | (1 << LASER2) | (1 << FAN);
    // Input pins
    DDRB &= ~(1 << TILT_STOP) | (1 << PAN_STOP);
    // Configure the ADC
    ADMUX = (1 << REFS0); // Use AVCC as the reference voltage
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable the ADC and set the prescaler to 128 (auto)
    }

void setupWatchdog(){
    wdt_enable(WDTO_2S);  // Set the watchdog timeout to approximately 2 second
    WDTCSR |= (1 << WDIE); // Enable the watchdog interrupt
    // wdt_disable();
}

uint8_t IsCharWaiting() {
    // Check if the Receive Complete (RXC0) bit is set in the UART Status Register (UCSR0A)
    // RXC0 is bit 7, so we shift right 7 places and & with 1 to get the value of that bit
    if ((UCSR0A >> 7) & 1) {
        return 1; // Data is available
    } else {
        return 0; // No data
    }
}
char WaitKey() {
    while (!(UCSR0A & (1 << RXC0)));      // Wait for data to be received
    return UDR0;  // Get and return received data from buffer
}
void StartBuzzerInProgMode() {
    if (BuzzerTick == 0) {
        PORTE |= (1 << BUZZER); // Set BUZZER pin to HIGH
    } else if (BuzzerTick >= 1) {
        PORTE &= ~(1 << BUZZER); // Set BUZZER pin to LOW
    }
}

// eerpom_update_word and eerpom_update_byte are available in avr/eeprom.h
// void eeprom_update_word(uint16_t *eepromAddress, uint16_t newValue) {// Update EEPROM word only if the current value is different
//     uint16_t currentValue = eeprom_read_word(eepromAddress); // Read the current value from EEPROM
//     if (currentValue != newValue) {     // Compare the current value with the new value
//         eeprom_write_word(eepromAddress, newValue);
//     }
// }
// void eeprom_update_byte(uint8_t *eepromAddress, uint8_t newValue) {// Update EEPROM word only if the current value is different
//     uint16_t currentValue = eeprom_read_byte(eepromAddress); // Read the current value from EEPROM
//     if (currentValue != newValue) {     // Compare the current value with the new value
//         eeprom_write_byte(eepromAddress, newValue);
//     }
// }
void TickCounter_50ms_isr() {
    Counter50ms++;
    TJTick++;
    if (SetupModeFlag == 1 && IsHome == 1 && WarnLaserOnOnce == 0) {
        // StartBuzzerInProgMode();
        BuzzerTick++;
        LaserTick++;

        if (BuzzerTick >= ONE_SECOND) {
            BuzzerTick = 0;
        }

        if (LaserTick >= ONE_SECOND) {
            LaserTick = 0;
        }
    }

    if (Counter50ms >= 10) {
        BatteryTick++;
        Tod_tick++;
        Tick++;
        AccelTick++;
        FlashTheLaserFlag ^= 1;
        Secondsfromreset++;
        Counter50ms = 0;
    }
    wdt_reset();//Reset the watchdog timer.
}

uint16_t readADC(uint8_t channel) {
    // Select the ADC channel
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);  // Clear the MUX bits and set them to the channel number
    // Start a conversion
    ADCSRA |= (1 << ADSC);
    // Wait for the conversion to complete
    while (ADCSRA & (1 << ADSC));
    // Read the conversion result
    return ADC;
}

// Initialise uart and set HC-05 to connect to any address. 
void uart_init(uint16_t ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter and receiver interrupt.
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
    // Add a small delay before sending AT commands
    _delay_ms(2000);
    // Put HC-05 into AT command mode
    uartPrint("AT\r");
    _delay_ms(1000);  // Wait for HC-05 to respond
    // Set HC-05 to master role
    uartPrint("AT+ROLE=1\r");
    _delay_ms(1000);  // Wait for HC-05 to respond
}
// Serial write/read functions
void uartPutChar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty transmit buffer
    UDR0 = c;  // Put data into buffer, sends the data
}
void uartPrint(const char* str) {
// void uartPrint(volatile char* str) { //20240625: 
    if (strlen(str) >= DEBUG_MSG_LENGTH) {
        const char* errorMsg = "Error: Debug message too long\n";
        while (*errorMsg) {
            uartPutChar(*errorMsg++);
        }
    } else {
        while (*str) {
            uartPutChar(*str++);
        } 
        uartPutChar('\r');
        uartPutChar('\n');
    }
}
int uartAvailable(void) {
    return (UCSR0A & (1 << RXC0));  // Return non-zero if data is available to read
}
char uartGetChar(void) {
    while (!(UCSR0A & (1 << RXC0)));  /* Wait for data to be received */
    return UDR0;  /* Get and return received data from buffer */
}

// ISR(USART0_RX_vect) {
//     char c = UDR0;
//     ReceivedData[DataCount] = c;
//     DataCount++;
//     if (DataCount >= BUFFER_SIZE) { // Prevent buffer overflow
//         DataCount = 0;
//     }
//     if (c == '\n' || c == '>') { // Set the flag when a complete command is received
//         ReceivedData[DataCount] = '\0'; // Add null terminator
//         DataInBufferFlag = true;
//         CommandLength = DataCount; // Store the length of the command
//         DataCount = 0;
//     }
// }
ISR(USART0_RX_vect) {
    char c = UDR0;
    if (c == '\r' || c == '\n') {
        // Do nothing
    } else {
        if (DataCount == 0 && c != '<') {
            // Do nothing
        } else {
            ReceivedData[DataCount] = c;
            DataCount++;
            if (DataCount >= BUFFER_SIZE) { // Prevent buffer overflow
                DataCount = 0;
            }
            if (c == '>') { // Set the flag when a complete command is received
                ReceivedData[DataCount] = '\0'; // Add null terminator
                DataInBufferFlag = true;
                CommandLength = DataCount; // Store the length of the command
                DataCount = 0;
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

void ProcessError(){
    if(Z_AccelFlag){
        Audio(3);
        return;
    }
    if(LaserOverTempFlag){
        Audio(4);
        return;
    }
   if(X_TravelLimitFlag || Y_TravelLimitFlag) {
        Audio(5);
        return;
    }
}

void CheckBlueTooth() { //20240616: Search this date in Avitech.rtf for background.  But app was not sending \r\n with <10:1>. So detect > rather than null.
    
    if (DataInBufferFlag == true) { // We have got something
        // processReceivedData();
        char *token;
        // uartPrint("In CheckBlueTooth() body."); // Debugging line
        // uartPrint(ReceivedData); // Debugging line
        memcpy(RecdDataConst, (const char*)ReceivedData, BUFFER_SIZE);
        token = strchr(RecdDataConst, '<');  // Find the start of the command
        if (token != NULL) {
            // sprintf(debugMsg,"Token: %s\n", token);
            // uartPrint(debugMsg);
            Command = atoi(token + 1);// Convert the command to an integer
            token = strchr(token, ':');  // Find the start of the instruction

            if (token != NULL) {
                char *end = strchr(token, '>');
                if (end != NULL) {
                    *end = '\0'; // Replace '>' with '\0' to end the string
                    Instruction = atoi(token + 1);
                    // _delay_ms(50); //Perhaps this should be removed.
                    DecodeCommsData();  // Process the command and instruction
                }
            }
        }
        // Reset the buffer and the flag
        // memset(ReceivedData, 0, sizeof(ReceivedData));
        memset((void*)ReceivedData, 0, sizeof(ReceivedData));
        DataInBufferFlag = false;
    }
}

void uartRead(char* buffer, int length) { //20240618.  This is not called.  ISR used instead.
    int i = 0;
    while (uartAvailable() && i < length - 1) {
        char c = uartGetChar();
        buffer[i++] = c;
        // Echo the character back to the serial monitor
        uartPutChar(c);
        // If we've encountered a newline or carriage return, stop reading
        if (c == '\n' || c == '\r') {
            break;
        }
    }
    // Null-terminate the string
    buffer[i] = '\0';
}

// void testLoopUart(char* testString) {//Use this during development only.  
//     if(false){ //Put this if (false) in, with associated closing bracket, when this is not to be run.
//         for (int i = 0; i < 1000; i++) {  // Replace 1000 with the number of iterations you want
//             sprintf(debugMsg, "%s: DM:  %d", testString, i);
//             uartPrint(debugMsg);
//             CheckBlueTooth();
//             if (received39){
//                 // if (c == 'C'){
//                     uartPrint("Have read C ");
//                     received39 = false;
//                     return; // Break the loop
//                 }
//             _delay_ms(1000);  // Delay for 1 second. Adjust as needed.
//             }
//     }
// }

void printToBT(uint8_t cmd, uint16_t inst){
    char printToBTMsg[20];
    sprintf(printToBTMsg, "<%02d:%04x>", cmd,  inst);
    // sprintf(printToBTMsg, "<%02x:%04x>", cmd,  inst); //Phone app comments indicated that hex is expected.  But it doesn't in fact test for hex.
    uartPrint(printToBTMsg);
    _delay_ms(20);
}

// Initialize I2C
void i2c_init(void) {
    // Set the bit rate to 100 kHz
    TWSR0 = 0x00;
    TWBR0 = ((F_CPU / 100000L) - 16) / 2;
}

// Send a byte via I2C and check for errors
int i2c_wbyte(uint8_t data) {
    TWDR0 = data; // Load data into the TWI data register
    TWCR0 = (1<<TWINT) | (1<<TWEN); // Start transmission
    while (!(TWCR0 & (1<<TWINT))); // Wait for transmission to complete

    // Check if the byte was sent and acknowledged
    if ((TWSR0 & 0xF8) != TW_MT_DATA_ACK) {
        return -1; // Error: Data not sent or not acknowledged
    }
    return 0; // Success
}

// Send a STOP condition
int i2c_stop(void) {
    TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Transmit STOP condition
    while(TWCR0 & (1<<TWSTO)); // Wait for STOP condition to be transmitted
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

uint8_t i2c_start(uint8_t address) {
    TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  //TWINT:7; TWSTA: 5; TWEN: 2; Hence: 0b10100100  // Send START condition
    while (!(TWCR0 & (1<<TWINT)));
    TWDR0 = address;
    TWCR0 = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR0 & (1<<TWINT)));
    // Check if the device acknowledged the READ / WRITE mode
    uint8_t twst = TW_STATUS & 0xF8;  //0xF8: 11111000
    // testLoopUart("Conditional exit from i2c_start");
    if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) { // TW_MT_SLA_ACK: 0x18 = 0b11100; TW_MR_SLA_ACK: 0x40 = 0b1000000;
        // sprintf(debugMsg, "i2c initiation failed");
        // uartPrint(debugMsg);
        return 1;
        }
    else {
        // sprintf(debugMsg, "i2c initiation succesful");
        // uartPrint(debugMsg);
        return 0;
        }
    
}

uint8_t i2c_rbyte(uint8_t ack) {
    // Send ACK or NACK
    if (ack) {
        TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);  // Send ACK
    } else {
        TWCR0 = (1 << TWINT) | (1 << TWEN);  // Send NACK
    }
    while (!(TWCR0 & (1 << TWINT)));  // Wait for the read operation to complete
    return TWDR0;  // Return the received byte
}
#pragma endregion Function definitions - peripherals, watchdog, uart, buzzer, timers, DAC, i2c
#pragma region ISRs
void StepperDriverISR() {
    // uint8_t x_dir_state = (PIND & (1 << X_DIR)) != 0; //Could use PORTD rather rathan PIND.  But as all (that are used) are configured for output, either can be used.
    // bool x_dir_state = (PIND & (1 << X_DIR)) == 0; 20240629 Replace with something easier to follow but also opposite sign.
    bool x_dir_state = (PIND & (1 << X_DIR)) ? 1 : 0;
    // bool y_dir_state = (PIND & (1 << Y_DIR)) == 0; //20240629.  Changed != to ==
    bool y_dir_state = (PIND & (1 << Y_DIR)) ? 1 : 0;

    if (StepCount > 0) {  //StepCount is uint16_t so can't be negative. So this is equivalent to if(StepCount !=0).
        StepCount--;

        if (Dx > Dy) {
            StepOverRatio += Dy;
            if (StepOverRatio >= Dx) {
                StepOverRatio -= Dx;
                Remainder = 0;
            } else {
                Remainder = 1;
            }
        } else {
            StepOverRatio += Dx;
            if (StepOverRatio >= Dy) {
                StepOverRatio -= Dy;
                Remainder = 0;
            } else {
                Remainder = 1;
            }
        }

        OCR1A = DSS_preload; 

        if (SetupModeFlag == 0) {
            if (StepCount < START_RAMPING_COUNT) {
                if (OCR1A < 65525) {
                    OCR1A += RAMPING_STEPS;
                } else {
                    if (OCR1A > DSS_preload) {
                        OCR1A -= RAMPING_STEPS;
                    }
                }
            }
        }
    // if (Remainder < 1) {20240623: Logic changed to deal with zero delta (ie one axis movement) cases.
    if (Remainder < 1 && Dx!=0 && Dy != 0) {
            PORTD |= (1 << X_STEP) | (1 << Y_STEP); // Turn on X and Y step pins.
            _delay_us(1);
            PORTD &= ~((1 << X_STEP) | (1 << Y_STEP)); // Turn off X and Y step pins.
            _delay_us(1);
            AbsX += x_dir_state ? 1 : -1;
            AbsY += y_dir_state ? 1 : -1;
        } else {
            if (Master_dir == 0) { //20240606: Changed from 1 to 0.  20240619: Master_dir is now aligned with axis (where 0 = X).
                PORTD |= (1 << X_STEP); // Turn on X step pins.
                _delay_us(1);
                PORTD &= ~(1 << X_STEP); // Turn off X step pins.
                _delay_us(1); //BASCOM used a 1us pulse.  Surely 1ms is fine?
                // AbsX += (~x_dir_state & 1) - (x_dir_state & 1);  // Update AbsX 
                AbsX += x_dir_state ? 1 : -1;
            } else {
                PORTD |= (1 << Y_STEP); // Turn on Y step pins.
                _delay_us(1);
                PORTD &= ~(1 << Y_STEP); // Turn off Y step pins.
                _delay_us(1);
                // AbsY += (~y_dir_state & 1) - (y_dir_state & 1);  // Update AbsY
                AbsY += y_dir_state ? 1 : -1;
            }
        }
    } else {
        StopTimer1();
        SteppingStatus = 0;
    }
}
//Setup ISRs
ISR(TIMER1_COMPA_vect){
    StepperDriverISR();
}
// ISR for Timer3 Compare A vector
ISR(TIMER3_COMPA_vect){
    wdt_reset();  // Reset the watchdog timer
    TickCounter_50ms_isr();
}

ISR(WDT_vect){
    // This ISR will be called when the watchdog timer times out. Without any code, the system will reset.
}
#pragma endregion ISRs
#pragma region Utility functions
void GetBatteryVoltage() {
    unsigned int SensorReading = 0;
    BattTotal = BattTotal - BattReadings[BattReadIndex];
    SensorReading = readADC(1);
    BattReadings[BattReadIndex] = SensorReading;
    BattTotal = BattTotal + SensorReading;
    BattReadIndex++;
    if (BattReadIndex > NUM_BATT_READINGS) {
        BattReadIndex = 1;
    }

    BattVoltAvg = BattTotal / NUM_BATT_READINGS;

    if (BattVoltAvg <= 221) {
        BatteryVoltage = 98;        //9.8 Volts     =0
    } else if (BattVoltAvg <= 330) {
        BatteryVoltage = 100;       //10.0 Volts   =12%
    } else if (BattVoltAvg <= 430) {
        BatteryVoltage = 105;       //10.5 Volts   =25%
    } else if (BattVoltAvg <= 567) {
        BatteryVoltage = 110;       //11.0 Volts   =50%
    } else if (BattVoltAvg <= 702) {
        BatteryVoltage = 115;       //11.5 Volts   =65%
    } else if (BattVoltAvg <= 832) {
        BatteryVoltage = 120;       //12.0 Volts   =80%
    } else if (BattVoltAvg <= 939) {
        BatteryVoltage = 124;       //12.4 Volts   =100%
    } else {
        BatteryVoltage = 124;       //12.4 Volts
    }
}

// void SetLaserVoltage(uint8_t voltage) {
void SetLaserVoltage(uint16_t voltage) {
    // DAC.setVoltage(4.8); // For a 12-bit DAC, 2048 is mid-scale.  Use DAC.setMaxVoltage(5.1);  
    uint16_t thisVoltage = voltage;
    // sprintf(debugMsg,"In SetLV.  voltage: %d", voltage);
    // uartPrint(debugMsg);
    if ((voltage<256) && (voltage>2)){ //If a uint8_t value has been assigned rather than 12bit, make it 12 bit.  But not if it's zero.
        thisVoltage = (voltage<<4);
    }
    DAC.setValue(thisVoltage);
}


void firstOn(){
    uint8_t firstTimeOn = eeprom_read_byte(&EramFirstTimeOn);
    if (firstTimeOn == 0xFF) {  // EEPROM is erased to 0xFF
        LoadEramDefaults();
        SetLaserVoltage(0);
        // initDACMCP4725();
        uartPrint("AT+ENLOG0\r");  // Turn off logging
        firstTimeOn = 0; //20240717: Reset first time on.  Not sure where this was done in BASCOM.
        eeprom_update_byte(&EramFirstTimeOn, 0);
    }
}

void Audio(uint8_t pattern){
    uint8_t repeatCount = 2; //2 repeats for all but pattern 1
    uint8_t i;
    // sprintf(debugMsg,"Audio. P:%d",pattern);
    // uartPrint(debugMsg);

    if (pattern ==1) {repeatCount = 1;}  
    // if (pattern ==6) {repeatCount = 3;}  //20240625 Use 2 repeats
    if (pattern ==7) {repeatCount = 5;}  
    //20240528:  _delay_ms() takes a constant argument.  So previous strategy of passsing a parameter to it didn't work.
    for (i = 0; i < repeatCount; i++) {
        PORTE |= (1 << BUZZER); // Set BUZZER pin to HIGH
        switch (pattern) {
            case 1: _delay_ms(100); break;
            case 2: _delay_ms(50); break;   // 20240624 Originally 1ms on, 100ms off, repeat 2.  Could that be heard?
            case 3: _delay_ms(500); break;  //Initially 500.  Changed to 2500 for testing (20240601).
            case 4: _delay_ms(50); break;
            case 5: _delay_ms(50); break;
            case 6: _delay_ms(500); break;
            case 7: _delay_ms(200); break;
        }
        PORTE &= ~(1 << BUZZER); // Set BUZZER pin to LOW
        switch (pattern) {
            case 2: _delay_ms(200); break;
            case 3: _delay_ms(100); break;
            case 4: _delay_ms(150); break;
            case 5: _delay_ms(50); break;
            case 6: _delay_ms(500); break;
            case 7: _delay_ms(100); break;
        }
    }
}

void WaitAfterPowerUp() {
    if (FirstTimeLaserOn == 1) {
        _delay_ms(5); //20240717.  What's this for?
        FirstTimeLaserOn = 0;
    }
}
// Sensor interactions
void WarnLaserOn() {
    if (WarnLaserOnOnce == 1) {
        Audio(2);
        WarnLaserOnOnce = 0;
    }
}

void StartLaserFlickerInProgMode() {
    if (LaserTick == 0) {
        SetLaserVoltage(0); // Off 150ms
    } else if (LaserTick == 3) {
        SetLaserVoltage(LaserPower); // On 850ms
    }    
}

void GetLaserTemperature() {
    unsigned int SensorReading = 0;
    unsigned int Result = 0;
    uint8_t LoopCount = 0;

    for (LoopCount = 1; LoopCount <= NUM_TEMP_READINGS; LoopCount++) {
        SensorReading = readADC(0);
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
    if (LaserTemperature < 25) {
        LaserTemperature = 25;
    } else if (LaserTemperature > 60) {
        LaserTemperature = 60;
    }
}

void ReadAccelerometer() {
    static uint16_t cnt = 0;
    cnt++;
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x3F); // 0x3F is the high byte of the Z accelerometer register
    Wire.endTransmission(false); // false to send a restart, keeping the connection active
    Wire.requestFrom(GyroAddress, (uint8_t)4); // Request 4 bytes: Z high, Z low, Temp high, Temp low
    if (Wire.available() == 4) { // If 4 bytes were returned
        Accel_Z.Zh_accel = Wire.read(); // Read Z high byte
        Accel_Z.Zl_accel = Wire.read(); // Read Z low byte
        Accel.H_acceltemp = Wire.read(); // Read Temp high byte
        Accel.L_acceltemp = Wire.read(); // Read Temp low byte
    }
    // Convert temperature from raw values to degrees Celsius
    int16_t rawTemp = (Accel.H_acceltemp << 8) | Accel.L_acceltemp;
    Accel.Acceltemp = (rawTemp / 340.0) + 36.53;
    if(!(cnt % 30000)){
        sprintf(debugMsg,"Accel h %02x, l %02x, Temp H %02x, L %02x", Accel_Z.Zh_accel, Accel_Z.Zl_accel, Accel.H_acceltemp, Accel.L_acceltemp);
        uartPrint(debugMsg);
    }
}
void DecodeAccelerometer() {
    if (OperationMode == 0) { // Field-1
        if (Accel_Z.Z_accel < AccelTripPoint) {
            Z_AccelFlag = 1;
            SystemFaultFlag = true;
        } else {
            Z_AccelFlag = 0;
        }
    }

    // Similar if onditions for OperationMode 1, 2, 3, 4.  But they need to be reviewed.  For example most (BASCOM) have If Z_accel < Acceltrippoint Then  but OpMode = 3 has If Z_accel > Acceltrippoint Then 

    if (Z_AccelFlagPrevious == 1 && Z_AccelFlag == 0 && AccelTick < 10) {
        Z_AccelFlag = 1;
        SystemFaultFlag = true;
        return;
    }

    Z_AccelFlagPrevious = Z_AccelFlag;
    AccelTick = 0;
}
void ClearSerial() {
    while (UCSR0A & (1 << RXC0)) {
        (void)UDR0;
    }
}

void GetLightLevel() {
    // long X;
    long Ylocal;

    LightLevel = readADC(2); // Light sensor ADC and remove jitter
    LightLevel >>= 2;

    if (LightTriggerOperation == 0) { // Run 24hr
        LightSensorModeFlag = 0;
    }

    if (LightTriggerOperation == 1) { // Run Daytime only
        Ylocal = UserLightTripLevel + 5; // Light sensor hysteresis value

        if (LightLevel < UserLightTripLevel) {
            LightSensorModeFlag = 1;
        }

        if (LightLevel > Ylocal) {
            LightSensorModeFlag = 0;
        }
    }

    if (LightTriggerOperation == 2) { // Run Night time only
        Ylocal = UserLightTripLevel - 5; // Light sensor hysteresis value

        if (LightLevel < UserLightTripLevel) { // Its night timer so run the laser
            LightSensorModeFlag = 0;
        }

        if (LightLevel > Ylocal) { // Its day time so turn the laser off
            LightSensorModeFlag = 1;
        }
    }
}
void MrSleepyTime() {
    uint8_t Xlocal; //, Ylocal;

    // Ylocal = 0;
    Xlocal = 0;
    LoopCount = 0;

    for (Xlocal = 1; Xlocal <= 4; Xlocal++) {
        PORTE |= (1 << BUZZER); // Set BUZZER pin to HIGH
        _delay_ms(50);
        PORTE |= ~(1 << BUZZER); // Set BUZZER pin to HIGH
        _delay_ms(1000);
    }

    PORTE |= (1 << BUZZER); // Set BUZZER pin to HIGH
    _delay_ms(1000);
    PORTE |= ~(1 << BUZZER); // Set BUZZER pin to HIGH

    SetLaserVoltage(0);
    PORTE &= ~(1 << FAN);// Fan = Off;

    StopTimer1();
    // TCCR1B &= ~((1 << CS12) | (1 << CS10));
    SteppingStatus = 0;
    PORTD |= (1 << X_ENABLEPIN);  //Disable X - TMC2130 enable is low
    PORTD |= (1 << Y_ENABLEPIN);  //Disable Y

    while (LightSensorModeFlag == 1) {
        GetLightLevel();
        if (LightSensorModeFlag == 0) {
            _delay_ms(5000);
            GetLightLevel();
        }

        if (Tick > 10) {
            PORTE |= (1 << BUZZER); // Set BUZZER pin to HIGH
            _delay_ms(25);
            PORTE |= ~(1 << BUZZER); // Set BUZZER pin to HIGH
            Tick = 0;
        }
    }

    StopTimer3();
    // TCCR3B &= ~((1 << CS32) | (1 << CS30));
    _delay_ms(5000);
}

void ResetTiming() {
    if (SetupModeFlag == 1) {
        Secondsfromreset = 0;
    }

    if (SetupModeFlag == 0) {
        if (Secondsfromreset > ResetSeconds) {
            Secondsfromreset = 0;
            BT_ConnectedFlag = 0;
            HomeAxis();
        }
    }
}
void CalibrateLightSensor() {
    SetLaserVoltage(0);
    GetLightLevel();
    printToBT(25,LightLevel); // Send current light level reading
}
void ThrottleLaser() {
    // float Result = 0.0;
    // float L_power = 0.0;
    // unsigned int SensorReading = readADC(0);
    // uint8_t B_volt = 0;
    // uint8_t Y;  //
    // uint8_t X;  //

    // if (BatteryVoltage < 98) B_volt = 25;
    // else if (BatteryVoltage < 100) B_volt = 50;
    // else if (BatteryVoltage < 105) B_volt = 70;
    // else if (BatteryVoltage < 110) B_volt = 90;
    // else B_volt = 100;

    // if (Laser2OperateFlag == 1) {
    //     Y = Laser2BattTrip + 5;  //Add hysteresis value if battery is low    eg 10.5 +0.5  = 11.0 for the reset level
    //     if (BatteryVoltage < Laser2BattTrip) Laser2BattTripFlag = 1;
    //     if (BatteryVoltage > Y) Laser2BattTripFlag = 0;

    //     X = Laser2TempTrip - 2;  //Add hysteresis value if the main laser is to hot. Reset value is 50-2=48deg's for reset back on
    //     if (LaserTemperature > Laser2TempTrip) Laser2TempTripFlag = 1;
    //     if (LaserTemperature < X) Laser2TempTripFlag = 0;

    //     if (Laser2TempTripFlag == 0 && Laser2BattTripFlag == 0) Laser2StateFlag = 1;
    //     else Laser2StateFlag = 0;
    // }

    // // Calculate the laser power directly from the sensor reading using the quadratic function
    // // Ref sheet LaserTemp of Alg.xlsm - this correlation directly from sensor reading to percentage of laser power.
    // // y = -0.0087x2 + 12.361x - 4268.5
    // // R² = 0.9996
    // L_power = -0.0087 * SensorReading * SensorReading + 12.361 * SensorReading - 4268.5;
    // // Ensure l_power (a percentage*100 of max voltage) is within a valid range 
    // if (L_power < 0) L_power = 0;
    // else if (L_power > 100) L_power = 100;
    // if (B_volt < L_power) L_power = B_volt; //Ensure L_power is less than battery voltage.
    // Result = L_power / 100.0f; //Convert to %age
    // L_power = Result * MaxLaserPower; //Convert to voltage
    // Result = UserLaserPower / 100.0f; //This looks like nonsense
    // LaserPower = L_power * Result; //Scale voltage to 

    // if (LaserTemperature > 55) {
    //     LaserPower = 0;
    //     LaserOverTempFlag = 1;
    //     SystemFaultFlag = true;
    // } else {
    //     LaserOverTempFlag = 0;
    //     SystemFaultFlag = false;
    // }
    LaserPower = 200; //20240625: Don't let it go to zero (for testing)
}

void initMPU() {
    uint8_t error = 0;
    Wire.begin(); // Initialize I2C
    // Read the WHO_AM_I register
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x75); // WHO_AM_I register address
    error = Wire.endTransmission(false); // Restart condition
    if (error) {
        sprintf(debugMsg, "Error b4 WHO_AM_I: %d", error);
        uartPrint(debugMsg);
        // uartPrint("MPU 5");
    } else {
        Wire.requestFrom(GyroAddress, (uint8_t)1); // Request 1 byte
        if (Wire.available()) {
            uint8_t whoAmI = Wire.read(); // Read WHO_AM_I byte
            sprintf(debugMsg, "Accel Chip %02x Gyro address: %02x", whoAmI, GyroAddress);
            uartPrint(debugMsg);
        } else {
            uartPrint("WHO_AM_I read failed");
        }
    }
    // Wake up the MPU
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x6B); // Power management register
    Wire.write(0x00); // Set to zero (wakes up the MPU-6050/6000)
    error = Wire.endTransmission(true);
    if (error) {
        sprintf(debugMsg, "Error waking MPU: %d", error);
        uartPrint(debugMsg);
    } else {
        uartPrint("MPU awake");
    }
    _delay_ms(300);
    // Reset signal paths
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x68); // Signal path reset register
    Wire.write(0x03); // Reset all signal paths
    error = Wire.endTransmission(true);
    if (error) {
        // sprintf(debugMsg, "Error resetting signal paths: %d", error);
        uartPrint("MPU 2");
    } else {
        uartPrint("Signal paths reset");
    }
    _delay_ms(300);
    // Set the slowest sampling rate
    
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x19); // Sample rate divider
    Wire.write(0xFF); // 255 for the slowest sample rate
    error = Wire.endTransmission(true);
    if (error) {
        sprintf(debugMsg, "Error setting sample rate: %d", error);
        uartPrint(debugMsg);
        // uartPrint("MPU 3");
    } else {
        uartPrint("Sample rate set");
    }

    // Set full scale reading to ±16g
    Wire.beginTransmission(GyroAddress);
    Wire.write(0x1C); // Accelerometer configuration register
    Wire.write(0x18); // ±16g full scale
    error = Wire.endTransmission(true);
    if (error) {
        // sprintf(debugMsg, "Error setting full scale: %d", error);
        uartPrint("MPU 4");
    } else {
        uartPrint("Full scale set");
    }
}
void StartTimer1() {
    // Set prescaler to 256 and start Timer1
    TCCR1B |= (1 << CS12);  //(1 << CS12)|(1 << CS10) for 1024 prescalar (4 times slower)
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);
    OCR1A= DSS_preload;  //20240614.
}

void StartTimer3() {
    // Set prescaler to 256 and start Timer1
    TCCR3B |= (1 << CS32); 
    TCCR3B &= ~(1 << CS31);
    TCCR3B |= (1 << CS30);
    OCR3A=  780;  // Set the compare value to 781 - 1.  16MHz/1024 => 15.6kHz => 64us period.  *780=> ~50ms.
}
void StopTimer3() {
    // Clear all CS1 bits to stop Timer3
    TCCR3B &= ~(1 << CS32);
    TCCR3B &= ~(1 << CS31);
    TCCR3B &= ~(1 << CS30);
}
void StopTimer1() {
    // Clear all CS1 bits to stop Timer1
    TCCR1B &= ~(1 << CS12);
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);
}
void setupTimer1() {
    TCCR1B |= (1 << WGM12);  // Configure timer 1 for CTC mode (Clear timer on compare)
    TCCR1B |= (1 << CS12); // Set up and start Timer1. 20240609: (1 << CS12) | (1 << CS10); for a prescaler of 1024. (1 << CS12); for 256.
    OCR1A = DSS_preload; // Set the compare value to desired value.  DSS_preload is order 100.  2*7812 for testing - 1 second period
    // With a prescalar of 256 and compare value of 100, frequency: 16MHz/256 ~ 64kHz/100 ~ 640Hz which is a period of about 1.6ms. 
    TIMSK1 |= (1 << OCIE1A);  // Enable the compare match interrupt
}
void setupTimer3() {
    TCCR3B |= (1 << WGM32);  // Configure timer for CTC mode
    TCCR3B |= (1 << CS32) | (1 << CS30);  //Bits 0 and 2 set.  TCCR: Timer/Counter Control Register.  Prescaler:1024
    OCR3A = 780;  // Set the compare value to 781 - 1.  16MHz/1024 => 15.6kHz => 64us period.  *780=> ~50ms.
    TIMSK3 |= (1 << OCIE3A);  // Enable the compare match interrupt
}


#pragma endregion Utility functions
#pragma region Zone functions
uint8_t GetZone(uint8_t i) {
    uint16_t Opzone;
    if (i<=MapTotalPoints){
        // Opzone = eeprom_read_word((uint16_t*)GetPosEepromAddress(i, 1));
        // eeprom_write_word((uint16_t*)((uintptr_t)&EramPositions[i].EramY), Opzone);
        Opzone = eeprom_read_word((uint16_t*)((uintptr_t)&EramPositions[i].EramY));
        // sprintf(debugMsg,"Opzone, raw (hex), index: %04x, %d",Opzone, i);
        // uartPrint(debugMsg);
        // Opzone = eeprom_read_word(&EramPositions[i].EramY);
        Opzone >>= 12;
        // sprintf(debugMsg,"Opzone, 1st byte (hex): %04x",Opzone);
        // uartPrint(debugMsg);
        switch (Opzone) {
            case 1: return 1;
            case 2: return 2;
            case 4: return 3;
            case 8: return 4;
            default: return 0;
        }
    } else {
        uartPrint("GetZone: i out or range.");
        return 0;
    }
}
void getMapPtCounts(bool doPrint) {  //Get the number of vertices (user specified) in each zone. Populate MapCount[2,n] with incremental and cumulative values.
    uint8_t MapIndex;
    uint8_t i, Prev_i; // Prev_i_1;
    i = 0;
    Prev_i = 0;
    // sprintf(debugMsg,"in GetMapPtCounts.  MapTotalPoints: %d",MapTotalPoints);
    // uartPrint(debugMsg);
    // MapCount[0][i] is incremental amount for zone i (i 0 based); MapCount[1][i] is cumulative.  Incremental repeats 1st point at end.
    for (MapIndex = 1; MapIndex <= MapTotalPoints; MapIndex++) {// 20240628: Start at MapIndex = 1.  Pass MapIndex -1 to GetZone as that looks up EramMapPositions[] which is zero based. 
        i = GetZone(MapIndex-1)-1;  //Zone index of MapIndex (the 2nd index) is zero based - so from 0 to 3. Stored points from 0 to MapTotalPoints-1.
        if (i > 0) {//If i == 0 (ie 1st zone), don't do anything. Assignments for that zone either when Prev_i = 0 or at total when there is only one zone.
            if (i != Prev_i) { //Do something when you have the first point of a new zone.
                if (Prev_i == 0) {
                    MapCount[0][Prev_i] = MapIndex; //Incremental is 1 greater than cumulative as incremental adds the first point of the zone as an additional last point.
                    MapCount[1][Prev_i] = MapIndex - 1;
                } else { // The case where prev_i > 1
                    MapCount[1][Prev_i] = MapIndex - 1; //This is the same as the Prev_i = 1 case.
                    MapCount[0][Prev_i] = MapIndex - MapCount[1][Prev_i - 1]+ 1; //Incremental for this zone is counter less cumulative for previous plus repeated 1st point.
                }
            }
        }
        if (MapIndex == MapTotalPoints) { //This is the case i == Prev_i or i == 1. Only do something if it's the last stored point (so index is MapTotalPoints - 1).
            MapCount[1][i] = MapIndex; //Consider 4 stored points in a single zone.  This would be entered when MapIndex == 4.  4 is the correct number of cumulative specified points.
            if (i == 0) {
                MapCount[0][i] = MapIndex + 1; // This would be 5 which is the number of specified points (4) plus 1 for the first one being repeated.
            } else {
                MapCount[0][i] = MapIndex - MapCount[1][i-1] + 1;
            }
        }
        // sprintf(debugMsg,"MapIndex: %d zone: %d MC0: %d MC1: %d", MapIndex, i, MapCount[0][i],MapCount[1][i]);
        // uartPrint(debugMsg);
        // _delay_ms(100);
        Prev_i = i;
    }
}
void LoadZoneMap(uint8_t zn) {
    //---------Gets vertices for given zone from EEPROM.
    //The Y dat MSB 4 bits are the operation zone and the 16 bits LSB is the map point number
    // 1111  1111111111111111
    //  ^             ^------------ Map point number   eg point 1, point 40 etc
    //  ^--------------------- Operation Zone     eg Zone 0 to 4
    uint8_t MapIndex, MI, zn_1;
    float res = 0.0;
    zn_1 = zn; // 20240701: Had used zn-1.  But (zn-1) now passed as argument to this function.
    // int temp;

    if (MapTotalPoints == 0) {
        // uartPrint("Zero points");
        return;
    }
    else {
        // sprintf(debugMsg,"MapTotalPoints: %d",MapTotalPoints);
        // uartPrint(debugMsg);
    }
    for (MapIndex = 0; MapIndex < MapCount[0][zn_1]-1; MapIndex++) { //Upper limit.  See notes below in next loop. With n-1 distinct points, there are n points in total
    // and they are indexed from 0 to n-2 - hence 0 to < n -1.
        if (zn_1 == 0) {//
            MI = MapIndex;
        } else {
            MI = MapIndex + MapCount[1][zn_1 - 1];//The index in EramPositions is across all zones whereas the loop index here is for a single zone
        }
        Vertices[0][MapIndex] = eeprom_read_word(&EramPositions[MI].EramX);
        Vertices[1][MapIndex] = eeprom_read_word(&EramPositions[MI].EramY) & 0x0FFF;
        // sprintf(debugMsg,"V MI: %d X: %d Y: %d AddX: %p AddY: %p",MapIndex, Vertices[0][MapIndex],Vertices[1][MapIndex],
        //                                                                 (void*)&EramPositions[MI].EramX,(void*)&EramPositions[MI].EramY);
        // uartPrint(debugMsg);    
    }
    // Add a repeated vertex equal to the first vertex.  MapIndex has incremented by the "next MapIndex" statement - I think?
    Vertices[0][MapIndex] = Vertices[0][0];
    Vertices[1][MapIndex] = Vertices[1][0];
    // Get the slope of each segment & store in Vertices[2][i]
    for (MapIndex = 1; MapIndex < MapCount[0][zn_1]; MapIndex++) { //MapCount[0][zn] is a count of the number of specified vertices, including the last repeated one, in zone 1.
    // If there are 4 distinct points, then MapCount[0][zn] would be 5. Segments are 0:1, 1:2, 2:3, 3:4  So start from 1 and consider MapIndex and (MapIndex - 1) as the segment endpoints.
        if (abs(Vertices[1][MapIndex] - Vertices[1][MapIndex - 1])>MIN_PERIMETER_TILT){ //If the difference in Ys is big enough, get slope.
            float num = static_cast<float>(Vertices[0][MapIndex] - Vertices[0][MapIndex - 1]) * 10;
            float den = static_cast<float>(Vertices[1][MapIndex] - Vertices[1][MapIndex - 1]);
            if (abs(den)>=1) {
                res = static_cast<float>(num)/den;
            } else {
                // uartPrint("Abs(den)<1");
            }
            // float res  = ((float)((Vertices[0][MapIndex] - Vertices[0][MapIndex - 1]) * 10 ))/ ((float)(Vertices[1][MapIndex] - Vertices[1][MapIndex - 1]));
            Vertices[2][MapIndex - 1] = res;
            // sprintf(debugMsg, "Slope MI: %d, num %f, den %f, 100res %f",MapIndex,num, den, 100 * res);
            // sprintf(debugMsg, "Slope MI: %d, num %d, den %d, 100res %d",MapIndex,(int)num, (int)den, (int)(100 * res));
            // uartPrint(debugMsg);
        } else {
            Vertices[2][MapIndex - 1] = DEF_SLOPE ; //Set an extreme value where delta(x) is potentially large and delta(y) is small.
            // It needs to be dealt with as a speical case in GetPerimeter().
        }
    }
}
int getCartFromTilt(int t) {
    float a = (float)t/STEPS_PER_RAD;
    float b = tan(a);
    float c = LASER_HT/b;
    // sprintf(debugMsg,"100a %d, 100b %d, c %d, t %d",100*a, 100*b, c, t);
    // uartPrint(debugMsg);
    // sprintf(debugMsg,"a %0.4f, b %0.4f, c %0.4f, t %d",a, b, c, t);
    // uartPrint(debugMsg);
    return (int)(c + 0.5); // rounding to the nearest integer before casting
}
int getTiltFromCart(int rho) {
    float a = LASER_HT/(float)rho;
    float b = atan(a);
    // sprintf(debugMsg,"100a %d, b %d, rho %d",100 * a, 100 * b, rho);
    // uartPrint(debugMsg);
    // sprintf(debugMsg,"a %0.4f, b %0.4f, rho %d",a, b, rho);
    // uartPrint(debugMsg);
    return (int)(b*STEPS_PER_RAD + 0.5); // rounding to the nearest integer before casting
}
int getNextTiltVal(int thisTilt, uint8_t dirn) {//20240629: TILT_SEP is target separation in Cartesian space between ladder rungs.  
    // getNextTiltVal() should calculate the required tilt value to get the specified separation.  TILT_SEP will need calibration.
   int rho = getCartFromTilt(thisTilt);
//    sprintf(debugMsg,"rho %d, thisTilt %d", rho, thisTilt);
//    uartPrint(debugMsg);
   if (dirn == 1) {
      rho = rho - TILT_SEP;
   } else {
      rho = rho + TILT_SEP;
   }
//    sprintf(debugMsg,"From gNTV: thisTilt: %d, dirn: %d, rho: %d, newTilt: %d",thisTilt, dirn, rho, getTiltFromCart(rho));
//    uartPrint(debugMsg);
//    _delay_ms(1000);
   return getTiltFromCart(rho);
}
void getCart(int p, int t, int thisres[2]) {
   int r;
   r = getCartFromTilt(t);
   thisres[0] = r*cos(p/STEPS_PER_RAD);
   thisres[1] = r*sin(p/STEPS_PER_RAD);
}
void getPolars(int c1, int c2,int thisRes[2]) {  //Take cartesian coordinates as input, return, via thisRes[2], polars.
   long r = c1 * c1 + c2 * c2;  //Get the distance from the laser to the point by pythagoras.
   r = sqrt(r);  
   double temp = atan2((double)c2, (double)c1) * STEPS_PER_RAD;  //Pan is atan(c2/c1)
   thisRes[0] = (int)temp; //pan
   thisRes[1] = getTiltFromCart((int)r); //tilt.
}
int GetPanPolar(int TiltPolar, int PanCart){ //Get the number of pan steps for a specified tilt angle (steps) and cartesian pan distance.
    int rho = getCartFromTilt(TiltPolar);
    return STEPS_PER_RAD*PanCart/rho;
}
// void printPerimeterStuff(const char* prefix, int a, int b, uint8_t c = 0, uint8_t d = 0){
//     // Using snprintf for safer string formatting and concatenation
//     snprintf(debugMsg, sizeof(debugMsg), "%s(%d, %d) :(%d,%d)", prefix, a, b, c, d);
//     uartPrint(debugMsg);
//     _delay_ms(100);
// }
void GetPerimeter(uint8_t zn) {  //LoadZoneMap(zn) loads the vertices of a zone, specified by the user, and slope of each segment to Vertices[][].  
    // This GetPerimeter() fills out the perimeter with more points, intended to be on the existing segments, so that traversing a segment will be 
    // straighter (in Cartesian space) and, more importantly, laser traces can be denser across the zone.  Separation from one dense point to the next
    // is intended to reflect an approximately constant Cartesian tilt axis offset.
    uint8_t j = 0;
    int nextTilt = 0, lastTilt = 0; 
    bool testBit = false;
    int nbrPts =0;
    uint8_t dirn = 0;
    uint8_t zn_1 = zn; // 20240701: Had used zn-1.  But zn-1 now passed as argument to this function.

    for (uint8_t i = 0; i < MapCount[0][zn_1] - 1; i++) { //MapCount[0][zn] is the number of specified vertices, including repeated first as last, in zone z.
        Perimeter[0][j] = Vertices[0][i]; //Set the first dense perimeter point to the first specified vertex
        Perimeter[1][j] = Vertices[1][i]; //Vertices[][] holds specified vertices for the specified zone (loaded by LoadZoneMap(zn)).
        // sprintf(debugMsg,"(x,y):(%d, %d), (i, j):(%d,%d), MC %d, zn_1 %d",Perimeter[0][j], Perimeter[1][j], i, j,MapCount[0][zn_1],zn_1);
        // uartPrint(debugMsg);
        dirn = 0;
        if (Vertices[1][i+1] > Vertices[1][i]) dirn = 1; //If next y value is greater than this one, dirn = 1
        if (Vertices[1][i+1] < Vertices[1][i]) dirn = 2;
        // printPerimeterStuff("V0i, V1i", Vertices[0][i], Vertices[1][i]);
        // printPerimeterStuff("(P0j, P1j):  (i, j)", Perimeter[0][j], Perimeter[1][j], i , j);
        if ((dirn != 0) && (!(Vertices[2][i] == DEF_SLOPE))) { // dirn == 0 is the case where the tilt value doesn't change for the segment.  
            // The DEF_SLOPE case is that for minimal change in tilt.  dirn == 0 is in fact a subset of the DEF_SLOPE case.
            
            nextTilt = getNextTiltVal(Perimeter[1][j], dirn);  
            testBit = false;
            if ((nextTilt < Vertices[1][i+1] && dirn == 1) ||(nextTilt > Vertices[1][i+1] && dirn == 2)) testBit = true; //Test for room for an intermediate point
            while (testBit) { 
                j++;
                if(j>=MAX_NBR_PERIMETER_PTS) break;
                Perimeter[1][j] = nextTilt;
                Perimeter[0][j] = (Perimeter[1][j] - Vertices[1][i]) * Vertices[2][i]; // 10; //Interim (delta) x value = Delta Y * slope / 10.  (Slope is stored as actual slope * 10)
                Perimeter[0][j] = Vertices[0][i] + Perimeter[0][j]/10; // Add delta x to x(i)
                lastTilt = nextTilt;
                nextTilt = getNextTiltVal(Perimeter[1][j], dirn);
                if (nextTilt == lastTilt){ //Deal with the case where integer arithmetic rounds to no change.
                    nextTilt += (dirn==1) ? MIN_TILT_DIFF : -MIN_TILT_DIFF; //20240702 Had 1:-1.  But very slow convergence.  Need something more adaptive.  If>1 could overshoot.  Is that a problem?
                }
                // printPerimeterStuff("P0j, P1j :  (i, j)", Perimeter[0][j], Perimeter[1][j], i , j);
                if (!((nextTilt < Vertices[1][i+1] && dirn == 1) ||(nextTilt > Vertices[1][i+1] && dirn == 2))) testBit = false;//Exit if there's not room for another intermediate point
            }
        } else { //The fixed tilt, pan only case.
            int fixedPanDiff = GetPanPolar(Vertices[1][i],MAX_PAN_DIST);//
            if (abs(Vertices[0][i+1]-Vertices[0][i]) > fixedPanDiff){
                nbrPts = abs(Vertices[0][i+1]-Vertices[0][i])/fixedPanDiff; 
            }
            for (uint8_t a = 1; a<=nbrPts; a++){
                j++;
                if(j>=MAX_NBR_PERIMETER_PTS) break;
                Perimeter[0][j] = Vertices[0][i] + fixedPanDiff * ((Vertices[0][i+1]>Vertices[0][i]) ? 1 : -1) * a;
                Perimeter[1][j] = Vertices[1][i];
                // printPerimeterStuff("P0j, P1j;  (i, j)", Perimeter[0][j], Perimeter[1][j], i , j);
            }   
        }
        j++;//Increment j between segments so that intermediate points in one segment don't write over those in the next.
        if(j>=MAX_NBR_PERIMETER_PTS) break;
    }
        
    NbrPerimeterPts = j - 1; //Subtract last increment (which is nevertheless necessary so that next segment starts right)
}
// Take 2 polar specified points, convert to cartesian, interpolate ((i + 1)/n), and return polars of interpolated point.
void CartesianInterpolate(int last[2], int nxt[2], uint8_t i, int n, int thisRes[2]) {
    int c1[2], c2[2]; //The cartesian end points.

    getCart(last[0], last[1], c1);
    getCart(nxt[0], nxt[1], c2);
    // Do the Cartesian interpolation, storing results in Res (which is global)
    float ratio = (i + 1) / (float)n;
    res[0] = c1[0] + ratio * (c1[0] - c2[0]);
    res[1] = c1[1] + ratio * (c1[1] - c2[1]);
    //Use the cartesian values stored in res and write the corresponding polar values to the same variable.
    getPolars(res[0],res[1], res);
}
#pragma endregion Zone functions
#pragma region Movement functions
uint8_t getRndLadInd(uint8_t rnd) {  //Get the perimeter point reasonably close to horizontally opposite (ie panwise opposite - ie a "rung") the input point.
    uint8_t tempInd;
    if (minYind > maxYind) { // Change MinY and MaxY to ensure minY <= maxY
        tempInd = minYind;
        minYind = maxYind;
        maxYind = tempInd;
    }

    if (rnd >= minYind && rnd <= maxYind) { //
        tempInd = 2 * maxYind;
        tempInd = tempInd - rnd;
        if (tempInd > NbrPerimeterPts) {
            tempInd = tempInd - NbrPerimeterPts;
        }
    } else if (rnd < minYind) {
        tempInd = 2 * minYind;
        tempInd = tempInd - rnd;
    } else { // rnd > maxY
        tempInd = 2 * maxYind;
        tempInd = tempInd - rnd;
    }

    if (rnd == minYind || rnd == maxYind) tempInd = rnd - 1;
    if (tempInd == 0) tempInd = 2;
    if (tempInd < 1) tempInd = 1;
    if (tempInd >= NbrPerimeterPts) tempInd = NbrPerimeterPts - 1;

    return tempInd;
}
uint8_t getExtremeY(bool upDown) {//Get the index of the maximum (upDown true) or minimum (upDown false) value of the Y values of the perimeter.
    //There is a much better way to do this using modern C++ iterators etc....Ask CoPilot.
    int val = 0;
    uint8_t i, ind;
    int maxval = -30000;
    ind = 1;

    for (i = 1; i < NbrPerimeterPts; i++) {
        val = Perimeter[1][i];
        if (upDown) {
            val = -val;
        }
        if (val > maxval) {
            maxval = val;
            ind = i;
        }
    }
    return ind;
}
// Positioning and motor control 
void getXY(uint8_t ind, uint8_t pat, uint8_t z) {
    if (ind < NbrPerimeterPts) { //First cycle around the boundary - ie all (dense) perimeter points.  Perimeter[i][j] has j 0 based to NbrPerimeterPts - 1
        X = Perimeter[0][ind];
        Y = Perimeter[1][ind];
    } else {
        PrevRndNbr = RndNbr;
        RndNbr = rand() % NbrPerimeterPts; // equivalent to Rnd(NbrPerimeterPts)
        if (RndNbr == 0) {
            RndNbr = 1;
        }
        AltRndNbr = RndNbr;

        if (pat == 2) { // This is the random ladder case when the opposite side of the rung is needed.
            if (rndLadBit == 1) {
                minYind = getExtremeY(true);//
                maxYind = getExtremeY(false);
                AltRndNbr = getRndLadInd(PrevRndNbr);
                rndLadBit = 0;
            } else {
                rndLadBit = 1;
            }
        }
        X = Perimeter[0][AltRndNbr];
        Y = Perimeter[1][AltRndNbr];
    }
}
void ProcessCoordinates() {
    StepOverRatio = 0;

    Dx = X - AbsX; // Distance to move
    Dy = Y - AbsY; // Distance to move

    if ((Dx==0) && (Dy == 0)){ //20240623: Explicitly exclude this case.
        StepCount = 0;
        }//Do nothing
    else{
        if (Dx > 0) {
            PORTD |= (1 << X_DIR); // Set X_DIR pin high
        } 
        if (Dx < 0) { //20240622 Saw PortD.4 oscillating in testing jogging panning.  This (ie excluding the 0 case) probably won't fix but worth trying.
            PORTD &= ~(1 << X_DIR); // Set X_DIR pin low
            Dx = Dx * -1; // Convert all distances to a positive number
        }

        if (Dy > 0) {
            PORTD |= (1 << Y_DIR); // Set Y_DIR pin high
        } 
        if (Dy < 0) {
            PORTD &= ~(1 << Y_DIR); // Set Y_DIR pin low
            Dy = Dy * -1;
        }

        if (Dx > Dy) {
            Master_dir = 0; // leading motor moves the most
            // StepOverRatio = Dx / Dy; // X:Y slope .  20240620: What protection against Dy == 0?
            // CoPilot advises that BASCOM assigns a large number for an assignment of, for example, 40/0.  C would throw an error.
            StepOverRatio = (Dy == 0) ? Dx : Dx / Dy ;
            StepCount = Dx;
        } else {
            Master_dir = 1;
            // StepOverRatio = Dy / Dx;
            StepOverRatio = (Dx == 0) ? Dy : Dy/Dx;
            StepCount = Dy;
        }
    }
}

void MoveMotor(uint8_t axis, int steps, uint8_t waitUntilStop) {
    static uint16_t MM_n2 = 0;
    if (axis == 0) { // pan
        X = steps;
        // Y = 0; // Why change Y?  Leave it as it is if you don't want to move it.
    } else { // tilt
        // X = 0; // Why change X?  Leave it as it is if you don't want to move it.
        Y = steps;
    }
    ProcessCoordinates();
    // CheckTimer1(11);
    // DSS_preload = HOMING_SPEED;
    SteppingStatus = 1;
    StartTimer1();
    if (waitUntilStop == 1) {  //So, if waitUntilStop is not 1, execution returns ?  Yes returns to HomeMotor() where there is a (blocking) {while limit switch is low} condition.
        // MoveMotor() is only called with waitUntilStop = 0 by HomeMotor() - ie when unit searching for limit switch (both pan & tilt separately)
        while (SteppingStatus == 1) {
            // do nothing while motor moves.  SteppingStatus is set to 0 at end of stepper ISR when StepCount !>0 (==0).
            // CheckBlueTooth();  //This should not be necessary.
            if (!(MM_n % 30000) && false) { //20240624: Use && false to disable periodic printing.
                MM_n2++; //Use this to distinguish b/w apparently identical messages.
                // sprintf(debugMsg,"MM: MM_n2, mode, C, I, axis, PEF, TEF, X, AbsX, DSS, PIND: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %#04x",MM_n2, SetupModeFlag, Command, Instruction, axis, PanEnableFlag, TiltEnableFlag,X, AbsX, DSS_preload, PIND);
                // uartPrint(debugMsg);
                // _delay_ms(30);
                // sprintf(debugMsg,"MM2: X, Y, Dx, Dy, AbsX, AbsY, PIND: %d, %d, %d, %d, %d, %d, %d, %#04x",MM_n2, X, Y, Dx, Dy, AbsX, AbsY, PIND);
                // uartPrint(debugMsg);
                _delay_ms(30);
            }
            MM_n++;
        }
    }
}

void StopSystem() {
    StopTimer1();
    SteppingStatus = 0;  // Reset stepping status
    X = AbsX; // Set X and Y to their absolute values
    Y = AbsY;
}

void JogMotors(bool prnt) {//
    uint8_t axis = 0;
    uint8_t speed = 0;
    uint8_t dir = 0;
    int pos = 0;

    // BASCOM version dealt with X<=X_mincount and X>=X_maxcount....(X_MINCOUNT here).  Are those necessary?  Not for development/debugging.
    if (PanEnableFlag == 1) {
        SteppingStatus = 1;
        axis = 0;
        speed = PanSpeed;
        dir = PanDirection;
        DSS_preload = (speed == 1) ? PAN_FAST_STEP_RATE : PAN_SLOW_STEP_RATE;
        if(abs(X) >= X_MAX_COUNT ) {
            X_TravelLimitFlag = 1;   
            SystemFaultFlag = true;
            ProcessError();
        } else {  //20240717: Both flags should be reset if there is no longer an error
            X_TravelLimitFlag = false;       
            SystemFaultFlag=false;
        }
    }
    if (TiltEnableFlag == 1) {
        SteppingStatus = 1;
        axis = 1;
        speed = TiltSpeed;
        dir = TiltDirection;
        DSS_preload = (speed == 1) ? TILT_FAST_STEP_RATE : TILT_SLOW_STEP_RATE;
        if (Y<=0||Y>=y_maxCount){
            Y_TravelLimitFlag = true;
            SystemFaultFlag=true;
            ProcessError();
        } else { //20240717: Both flags should be reset if there is no longer an error
            Y_TravelLimitFlag = false;       
            SystemFaultFlag=false;
        }
    }

    pos =  (speed ==1) ? HIGH_JOG_POS:LOW_JOG_POS; //Up to 4 steps per cycle through main loop or 1 for slow.  Needs calibration. 20240629: Testing with 40:10.
    pos = pos * (dir ? 1 : -1); // 20240629: See review in Avitech.rtf on this date.  Search on "Proposal to fix directions:"
    pos += (axis == 0) ?  AbsX : AbsY; //2024620: Add an amount, pos, to AbsX.  This becomes X in MoveMotor so X - AbsX is the increment.  When that is reached, JogMotors would be called
    //  again. If the instruction (eg from <2:3>) has not been changed (eg by receipt of <2:0>) then the values set in cmd2() remain.  Accordingly pos increments AbsX (which would have 
    // been incremented in previous calls to MoveMotor()) again.  So for the high speed case in which the increment passed is 4, MoveMotor() should, given the while loop, increment AbsX 
    // by 4 before returning to JogMotors then doing the same thing.  Need some debug statements to test this.
    
    if (!(JM_n % 120000) && false) { //20240624: Use && false to disable periodic printing.
        // sprintf(debugMsg,"JM: X, Y, Dx, Dy, AbsX, AbsY, PIND: %d, %d, %d, %d, %d, %d, %#04x",X, Y, Dx, Dy, AbsX, AbsY, PIND);
        // uartPrint(debugMsg);
    }
    JM_n++;

    if (PanEnableFlag == 0 && TiltEnableFlag == 0) {  // If both pan and tilt are disabled, stop the motors.
        StopSystem();
    }
    else {
        StartTimer1();  //20240620.  Could be only if necessary?
        MoveMotor(axis, pos, 1);
    }
}

void CalcSpeedZone() {
    uint8_t Result;

    if (Y < 48) {
        Zonespeed = SpeedZone[0];
        Result = 1;
    } else if (Y < 65) {
        Zonespeed = SpeedZone[1];
        Result = 2;
    } else if (Y < 76) {
        Zonespeed = SpeedZone[2];
        Result = 3;
    } else if (Y < 125) {
        Zonespeed = SpeedZone[3];
        Result = 4;
    } else {
        Zonespeed = SpeedZone[4];
        Result = 5;
    }

    if (Result != CurrentSpeedZone) {
        if (BT_ConnectedFlag == 1) {            
            printToBT(16,Result);
            // printf("<16:%x>", Result);
            // _delay_ms(50);
        }
        CurrentSpeedZone = Result;
    }
}
uint8_t CalcSpeed() {
    float Result;
    float Ratio;
    float Gain;

    Gain = 3;
    Ratio = 0.002;

    Result = pow(Zonespeed, Gain);
    Result = Result * Ratio;
    Result = Result + STEP_RATE_MAX;

    return STEP_RATE_MAX; // Put speed to maximum for testing.
}

void HomeMotor(uint8_t axis, int steps) { //Move specified motor until it reaches the relevant limit switch.
    // static volatile uint16_t m;
    MoveMotor(axis, steps, 0);
    StartTimer1();  //20240614 This added.  Shouldn't be necessary.
    if (axis == 0) {
        while (!(PINB & (1 << PAN_STOP))){  //While pan_stop pin is low.
            // do nothing while motor moves
        }
    } else {
        while (!(PINB & (1 << TILT_STOP))) {  //While tilt_stop pin is low.
            // do nothing while motor moves
        }
    }
    StopTimer1();
    StepCount = 0;
    SteppingStatus = 0;

    if (axis == 0) {
        X = 0;
        AbsX = 0;
    } else {
        Y = 0;
        AbsY = 0;
    }
}
void MoveLaserMotor() {
    ProcessCoordinates(); // Drive motors to the coordinates
    DSS_preload = HOMING_SPEED; // Set speed rate
    SteppingStatus = 1;
    StartTimer1();
    while (SteppingStatus == 1) { // do nothing while motor moves 
    }
    StopTimer1(); // Stop the motor from stepping as sensor has been triggered.  20240615: The sensor has not been triggered. This is called when the target position is reached.
    StepCount = 0; // Clear the step count
}
void NeutralAxis() {
    // PAN AXIS NEUTRAL
    // Neutral Position: (-4000,1000)
    // Pan motor to neutral position
    X = -4000; // Clockwise 90 deg pan
    MoveLaserMotor();
    // TILT AXIS HOME
    // Tilt motor to neutral position
    Y = 500; // Half way down
    MoveLaserMotor();
    Audio(1);
}

void HomeAxis() {
    int Correctionstepping;
    StartTimer1(); //Probably not necessary.
    SetLaserVoltage(0); // Turn off laser
    // *********PAN AXIS HOME****************
    // CheckTimer1(2);
    if ((PINB & (1 << PAN_STOP))){  //If pan_stop pin is high... "Move blade out of stop sensor at power up"
        MoveMotor(0, -300, 1);
    }
    // CheckTimer1(3);
    HomeMotor(0, 17000); //Pan motor to limit switch and set X and AbsX to 0 .
    // *********TILT AXIS HOME****************
    if ((PINB & (1 << TILT_STOP))) {   //If tilt_stop pin is high (ie at limit). "Move blade out of stop sensor at power up"
        MoveMotor(1, 300, 1);
    }
    // CheckTimer1(4);
    HomeMotor(1, -5000);  //Tilt motor homing position
    // --**Move Tilt into final position**--
    // CheckTimer1(5);
    switch (OperationMode) {
        case 0: Correctionstepping = 100; break;
        case 1: Correctionstepping = -220; break;
        case 2: Correctionstepping = 100; break;
        case 3: Correctionstepping = -220; break;
        default: Correctionstepping = 100; break;
    }
    MoveMotor(1, Correctionstepping, 1);
    // CheckTimer1(6);
    NeutralAxis();  //Take pan to -4000 and tilt to 500 (both magic numbers in NeutralAxis()).
    ClearSerial();
    Audio(5);
    CmdLaserOnFlag = false;
    IsHome = 1;
}

void RunSweep(uint8_t zn) {
    uint8_t NbrPts, PatType, i;
    int last[2],nxt[2],nbrMidPts;

    LoadZoneMap(zn);
    GetPerimeter(zn);
    for (PatType = 1; PatType <= 2; PatType++) {//20240701 PatType.  Initially 2. 1:
        for (uint8_t Index = 0; Index < NbrPerimeterPts + NBR_RND_PTS; Index++) {
            // Store present point to use in interpolation
            last[0] = X;
            last[1] = Y;
            getXY(Index, PatType, zn);
            nxt[0] = X;
            nxt[1] = Y;
            // sprintf(debugMsg,"Index %d, AltRndNbr %d, x0 %d,y0 %d,x1 %d,y1 %d",Index, AltRndNbr, last[0],last[1],nxt[0],nxt[1]);
            // sprintf(debugMsg,"RN %d, x0 %d,y0 %d",AltRndNbr, last[0],last[1]);
            // uartPrint(debugMsg);
            CalcSpeedZone();
            DSS_preload = CalcSpeed();

            if (Index == 1) {
                CmdLaserOnFlag = false;
                DSS_preload = STEP_RATE_MAX;
            }

            if (Index == 2) {
                CmdLaserOnFlag = true;
            }

            // nbrMidPts = abs(nxt[0] - last[0])/ MID_PT_SEPARATION;
            // for (i = 0; i <= nbrMidPts; i++) {
                // if (nbrMidPts > 0) {
                //     if (i == nbrMidPts) {
                //         X = nxt[0];
                //         Y = nxt[1];
                //     } else {
                //         CartesianInterpolate(last,nxt,i, nbrMidPts, res);
                //         getPolars(res[1], res[2], res);  //Pass res as input 
                //         X = res[1];
                //         Y = res[2];
                //     }
                // }
                ProcessCoordinates();
                // sprintf(debugMsg,"X: %d, Y: %d, Index: %d", X, Y, Index);
                // uartPrint(debugMsg);

                SteppingStatus = 1;
                StartTimer1();

                while (SteppingStatus == 1) {
                    DoHouseKeeping();
                    if (SetupModeFlag == 1) { //Will only arise if SetupModeFlag is changed after RunSweep is called (which will occur if SetupModeFlag == 0 - run mode.)
                        StopTimer1();
                        StepCount = 0;
                        SteppingStatus = 0;
                        return;
                    }
                // }
            }
        }
    }
    ResetTiming();
}
#pragma endregion Movement functions
#pragma region Print functions
void TransmitData() {
    int Hexresult;
    char Result[5];
    int Variables[19]; // Declaring this as local causes a compile problem in BASCOM.
    uint8_t i;

    Variables[0] = MaxLaserPower;
    Variables[1] = Accel_Z.Z_accel;
    Variables[2] = Index;
    Variables[3] = X;
    Variables[4] = Y;
    Variables[5] = AbsX;
    Variables[6] = AbsY;
    Variables[7] = Accel_Z.Z_accel;
    Variables[8] = Tod_tick / 2;
    Variables[9] = BattVoltAvg;
    // Variables[10] = Hw_stack;
    // Variables[11] = Sw_stack;
    Variables[10] = Frame_size;
    Variables[11] = Wd_flag;
    Variables[12] = Boardrevision;
    Variables[13] = DSS_preload;
    Variables[15] = LaserID;
    Variables[16] = AccelTripPoint;
    Variables[17] = ResetSeconds / 2;
    Variables[18] = OperationMode;

    if (SendDataFlag == 1) {
        for (i = 0; i <= 18; i++) {
            Hexresult = Variables[i];
            sprintf(Result, "%X", Hexresult);
            if (i == 0) {
                // printToBT(20,Result);// In original MaxLaserPower has code 20, not 30. Write over 29 and sort out later.
                uartPrint(Result);  
            } else {
                // printToBT((i+29),Result);// In original MaxLaserPower has code 20, not 30. Write over 29 and sort out later.
                uartPrint(Result);  
            }
            _delay_ms(50);
        }
    }
}
void PrintZoneData() {
    uint8_t res;
    uint8_t i;


    for (i = 0; i < 5; i++) { //Print both count of map points (MapCount[0][i]) for maps and 5 speed zones
        if (i == 0){
            res = MapTotalPoints;  //This needs to be assigned.
            }
            else{
            res = MapCount[0][i];
            }
        printToBT(i + 4,res);  // Print count of map points.

        res = SpeedZone[i];
        printToBT(i + 10,res);  // Target points are 10:14 for 5 SpeedZones
        _delay_ms(50);
    }
}
void PrintAppData() {
    if (BT_ConnectedFlag == 1) {
        printToBT(17,MapRunning);  
    
        if (SetupModeFlag == 0) {
            printToBT(2, LaserTemperature);
            printToBT(3, BatteryVoltage);
            printToBT(21, UserLaserPower);
            printToBT(25, LightLevel);
            printToBT(45, LaserPower);
        }
    }
}
void PrintConfigData() {
    uint8_t i;
    uint16_t ConfigData[12];
    uint8_t ConfigCode[12];

    PrintZoneData();

    ConfigData[1] = ActiveMapZones;
    ConfigCode[1] = 15;

    ConfigData[2] = MapTotalPoints;
    ConfigCode[2] = 4;  // 20240618: This is already printed from PrintZoneData()

    ConfigData[3] = 0 ; //_version_major;
    ConfigCode[3] = 23;

    ConfigData[4] = 0 ; //_version_minor;
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

    for (i = 1; i <= 11; i++) {
        printToBT(ConfigCode[i],ConfigData[i]);
    }
}
#pragma endregion Print functions
#pragma region Mode functions
void ProgrammingMode() {
    SetLaserVoltage(0); // Turn off laser
    HomeAxis();
    printToBT(9, 1);
    _delay_ms(100);  
}

void OperationModeSetup(int OperationMode) {
    char* name;
    ClearSerial();
    _delay_ms(2000); // Wait for 2 seconds.  TJ 20240614: Why?
    switch(OperationMode) {
        case 0:
            name = "Field-i";
            y_maxCount = 1800;
            strcpy(OpModeTxt,"0:Field-i");
            break;
        case 1:
            name = "Roof-i";
            y_maxCount = 4095;
            strcpy(OpModeTxt,"1:Roof-i");
            break;
        case 2:
            name = "Orchard-i";
            y_maxCount = 2000;
            strcpy(OpModeTxt,"2:Orchard-i");
            break;
        case 3:
            name = "Eve-i";
            y_maxCount = 4095;
            strcpy(OpModeTxt,"3:Eve-i");
            break;
        case 4:
            name = "Test_i";
            y_maxCount = 2500;
            strcpy(OpModeTxt,"4:Test-i");
            break;
    }
    // testLoopUart("OMS: >switch ");
    char atCommand[50];
    sprintf(atCommand, "AT+NAME%s %d\r\n", name, LaserID);
    uartPrint(atCommand);
    // testLoopUart("OMS: end function ");
}
#pragma endregion Mode functions
void DoHouseKeeping() {

    CheckBlueTooth();
    ReadAccelerometer();
    // DecodeAccelerometer();

    if (Tick > 4) {
        TransmitData();
        PrintAppData();
        GetLaserTemperature();

        if (Z_AccelFlag == 0) {
            ThrottleLaser();
        }
        Tick = 0;
    }

    if (AbsY >= TILT_SENSOR_IGNORE) {
        if((PINB & (1 << TILT_STOP)) != 0){  //if Tilt_stop pin is on
            StopTimer3();
            // TCCR3B &= ~((1 << CS32) | (1 << CS30));
        }
    }

    if (SystemFaultFlag == 1) {
        SetLaserVoltage(0);
        StopTimer1();
        SteppingStatus = 0;
        ProcessError();
        return;
    }

    if (SetupModeFlag == 1) {
        WarnLaserOn();
        StartLaserFlickerInProgMode();
    }

    if (SetupModeFlag == 0) {
        IsHome = 0;
        WaitAfterPowerUp();
        if (CmdLaserOnFlag && MapTotalPoints >= 2) {
            WarnLaserOn();
            SetLaserVoltage(LaserPower);
        } else {
            SetLaserVoltage(0);
            GetLightLevel();
        }
        }

    if (Tod_tick > NIGHT_TRIP_TIME_FROM_STARTUP) {
        if (BT_ConnectedFlag == 0 && LightSensorModeFlag == 1) {
            MrSleepyTime();
        }
    }
}

void setup() {
    sei();  // Enable global interrupts.
    uart_init(MYUBRR);  // Initialize UART with baud rate specified by macro constant
    _delay_ms(2000); //More time to connect and not, therefore, miss serial statements.
    setupTimer1();
    setupTimer3();
    setupPeripherals();
    setupWatchdog();
    OperationModeSetup(OperationMode);     // Select the operation mode the device will work under before loading data presets
    Wd_byte = MCUSR; // Read the Watchdog flag
    if (Wd_byte & (1 << WDRF)) { // there was a WD overflow. This flag is cleared of a Watchdog Config
        Wd_flag = 1; // store the flag
    }
    Wire.begin(); // Initialize I2C
    if (!DAC.begin()){
        uartPrint("DAC.begin() returned false.");
        return;
    };  // DAC.begin(MCP4725ADD>>1); // Initialize MCP4725 object.  Library uses 7 bit address (ie without R/W)
    // DAC.setMaxVoltage(5.1);  //This may or may not be used.  Important if DAC.setVoltage() is called.
    firstOn();  //Load defaults to EEPROM if first time on.
    ReadEramVars();  //Reads user data from EEPROM to RAM.
    initMPU();
    PORTE |= (1 << FAN); //Turn fan on.
    for (int battCount = 0; battCount < 10; battCount++) {  //BatteryVoltage should be populated by this.
        GetBatteryVoltage();   //Load up the battery voltage array
    } 
    SetLaserVoltage(0);                                     //Lasers switched off
    PORTD &= ~(1 << X_ENABLEPIN); //Enable pan motor (active low ).
    PORTD &= ~(1 << Y_ENABLEPIN); //Enable tilt motor.
    Audio(1); //TJ 20240615 Had been using Audio(6)
    HomeAxis();
}
int main() {
    // "If Z_accelflag = 1 Then" //Need to implement something like this (search in BASCOM version) when IMU is working.
    static bool doPrint = true;
    // static bool firstRun = true; //Needs to be or set passed by cmd10();
    setup();
    while(1) {        
        doPrint = false;
        if (SetupModeFlag == 1) {
           JogMotors(doPrint); 
        }
        if (SetupModeFlag == 0) {  //In run mode
            // if (firstRun) {
                eeprom_update_byte(&EramMapTotalPoints, MapTotalPoints);
                getMapPtCounts(doPrint);  //Any need to call getMapPtCounts?  Doesn't need to be called every time.  Once at end of setup and at first time after setup.
                // firstRun = false;
            // }
            if (MapTotalPoints > 0){
                // uartPrint("MTP>0");
                for (Zn = 1; Zn <= NBR_ZONES; Zn++) {
                    sprintf(debugMsg,"In runsweep Zn-1: %d",Zn-1);
                    uartPrint(debugMsg);
                    if (MapCount[0][Zn-1] > 0) { //MapCount index is zero base
                        // uartPrint("Entering RunSweep");
                        RunSweep(Zn-1); //
                    }
                }
            }else SetLaserVoltage(0); // 20240718: Standby mode
        }
        if (SetupModeFlag == 2) {
            CalibrateLightSensor();
        }
        DoHouseKeeping();
    } 
    return 0;
}

// void testI2C() {
//     byte error, address;
//     int nDevices;

//     uartPrint("Scanning...");

//     nDevices = 0;
//     for(address = 1; address < 127; address++ ) {
//         Wire.beginTransmission(address);
//         error = Wire.endTransmission();

//         if (error == 0) {
//             sprintf(debugMsg,"I2C device found at address: ,%02x,   !",address);
//             uartPrint(debugMsg);
//             nDevices++;
//         }
//         else if (error==4) {
//             sprintf(debugMsg,"Unknown error at address: ,%02x,   !",address);
//             uartPrint(debugMsg);
//             nDevices++;    }    
//   }
//   if (nDevices == 0) uartPrint("No I2C devices found\n");
//   else uartPrint("done\n");

//   delay(5000); // wait 5 seconds for the next scan
// }
// void testLaserPower(){
//     // Use LaserTick as timer for testing.
//     static uint8_t lastLaserTick = 0;
//     static uint8_t cnter = 0;
//     static uint16_t l_power = 0;
//     uint8_t nbrSteps = 12;
//     if (lastLaserTick !=LaserTick){ //Only do something if LaserTick has changed (incrementes every 50ms )
//         lastLaserTick = LaserTick;
//         if (LaserTick == 0){ //Do something each second (which is the frequency that LaserTick resets to zero.)
//             if(cnter ==0) {
//                 l_power = 0;
//             }else{
//                 // l_power |= (1<<(cnter-1)); //Set one higher bit each time through. So max value will be 255 (for cnter == 8) (or 4095 for cnter == 12).
//                 l_power += MAX_LASER_VALUE/nbrSteps;
//             }
//             SetLaserVoltage(l_power);
//             // SetLaserVoltage(255); //Test with full power.
//             sprintf(debugMsg,"cnter, l_powerX, l_powerD: %d, %04x, %d", cnter, l_power,l_power);
//             uartPrint(debugMsg);
//             _delay_ms(20);
//             cnter++;
//             if (cnter == nbrSteps+1){
//                 cnter = 0;
//                 l_power = 0;
//             }
//         }
//     }
// }

// void SetLaserVoltage(uint8_t voltage) {
//     uint16_t D;
//     uint8_t Hi;
//     uint8_t Lo;
//     float Lvolt;

//     // if (voltage == 0 || Laser2OperateFlag == 0 || Laser2StateFlag == 0) {
//     //     PORTE &= ~(1 << LASER2); // Turn off Laser2
//     // } else {
//         PORTE |= (1 << LASER2); // Turn on Laser2
//     // }

//     //Lvolt = voltage / 51.0f; // Converts a byte value to a voltage (decimal) for the laser DAC to set.  EG 255/51=5volts or 127/51=2.5volts
//     Lvolt = 250 / 51.0f; //20240624. Overwrite Lvolt for testing. Just want to be sure it turns on.
//     D = Lvolt / VoltPerStep; // Calculate how many steps to set the requested voltage
//     D <<= 4; // Shift left by 4 bits
//     Hi = D >> 8; // Get the first 8 bits MSB
//     Lo = D & 0xFF; // Get the last 8 bits LSB
//     sprintf(debugMsg,"voltage, D, Hi, Lo: %d, %04x, %02x, %02x", voltage, D, Hi, Lo);
//     uartPrint(debugMsg);
//     _delay_ms(50);


//     // Start the I2C bus for the laser power DAC.
//     if (i2c_start(MCP4725) != 0) {
//         // Handle error (e.g., log error, set an error flag, etc.)
//         uartPrint("i2c_start error");
//         return; // Exit if we can't start communication
//     }

//     // Send the 1st byte
//     if (i2c_wbyte(0xC0) != 0) {  //20240624 0xC0: 0b1100 0000.  1100 is MCP4725 device code.  LSB is R/W (R=0). A2, A1, A0 (bits 1,2,3, starting from 0) are address bytes.
//     // Defaults for A2 and A1, set at manufacture, are 0. Assume that is the case.  A0 is set by logic state of A0 pin.  Assume (based on BASCOM) that this is tied low.
//         uartPrint("i2c_wbyte(0xC0) error");
//         i2c_stop(); // Attempt to properly close the I2C communication
//         return; // Exit after error
//     }

//     // Send the 2nd byte
//     if (i2c_wbyte(0x40) != 0) { //0x40: 0b0100 0000. From table 6-2 of data sheet: C2:C1:C0 = 010 =>Load configuration bits and data code to the DAC Register
//     // Ref para 6.1.2
//         uartPrint("i2c_wbyte(0x40) error");
//         i2c_stop(); // Attempt to properly close the I2C communication
//         return; // Exit after error
//     }

//     // Send the 3rd byte (MSB) 
//     if (i2c_wbyte(Hi) != 0) { //20240624 Figure 6-2 NOT 6-1 shows that 3rd byte has 8 MSBits and 4th byte has 4LSBs + 4 con't care bits.
//         uartPrint("i2c_wbyte(Hi) error");
//         i2c_stop(); // Attempt to properly close the I2C communication
//         return; // Exit after error
//     }

//     // Send the 4th byte (LSB)
//     if (i2c_wbyte(Lo) != 0) {
//         uartPrint("i2c_wbyte(Lo) error");
//         i2c_stop(); // Attempt to properly close the I2C communication
//         return; // Exit after error
//     }

//     // Stop the I2C bus
//     if (i2c_stop() != 0) {
//         // Handle error if needed
//         return; // Optional, as we're exiting the function anyway
//     }

//     if (voltage > 0 && BatteryTick > 4) {
//         GetBatteryVoltage();
//         BatteryTick = 0;
//     }
// }

// void initDACMCP4725() {
//     i2c_init(); // Initialize I2C with 400kHz bit rate
//     if (i2c_start(MCP4725)) { // Start I2C communication and send MCP4725 address
//         // The operation failed
//         sprintf(debugMsg, "Operation failed ");
//         uartPrint(debugMsg);
//     }
//     i2c_wbyte(0x0001); // Send data to set the default power to 0 volts in the EEPROM volt on startup
//     i2c_stop(); // Stop I2C communication
// }

// void Audio(uint8_t pattern) { //20240624 Refactored to be non-blocking
//     // Define the array with values for 50ms increments
//     // Format: [on time in 50ms increments, off time in 50ms increments, repeat count]
//     static const uint8_t audioTiming[6][3] = {
//         {2, 2, 1},  // Pattern 1: 100ms on, 100ms off, repeat 1 time
//         {1, 9, 2},  // Pattern 2: 50ms on, 450ms off, repeat 2 times
//         {10, 2, 2}, // Pattern 3: 500ms on, 100ms off, repeat 2 times
//         {1, 3, 2},  // Pattern 4: 50ms on, 150ms off, repeat 2 times
//         {1, 1, 2},  // Pattern 5: 50ms on, 50ms off, repeat 2 times
//         {10, 15, 4} // Pattern 6: 500ms on, 750ms off, repeat 4 times
//     };

//     static uint8_t lastCounter = 0;
//     static uint8_t currentPattern = 0;
//     static bool phase = false; // false: buzzer off, true: buzzer on
//     static uint8_t repeatCounter = 0;
//     static uint8_t phaseCounter = 0; // Tracks the duration of the current phase

//     // Check if pattern has changed or it's the first run
//     if (pattern != currentPattern) {
//         currentPattern = pattern;
//         phase = false; // Ensure buzzer starts off
//         lastCounter = Counter50ms; // Reset timing
//         repeatCounter = 0; // Reset repeat counter
//         phaseCounter = 0; // Reset phase duration counter
//     }

//     // Calculate elapsed time since last state change
//     uint8_t elapsed = (Counter50ms - lastCounter + 20) % 20; // Assuming Counter50ms increments every 50ms and resets every second

//     // Index for accessing the timing array
//     uint8_t index = pattern - 1; // Adjust for 0-based index

//     // Check if it's time to toggle the buzzer state
//     if ((phase && phaseCounter >= audioTiming[index][0]) || (!phase && phaseCounter >= audioTiming[index][1])) {
//         phase = !phase; // Toggle phase
//         lastCounter = Counter50ms; // Reset timing for next phase
//         phaseCounter = 0; // Reset phase duration counter

//         if (phase) { // If we're turning the buzzer on
//             PORTE |= (1 << BUZZER);
//         } else { // We're turning the buzzer off
//             PORTE &= ~(1 << BUZZER);
//             repeatCounter++; // Increment repeat counter after an on + off cycle
//         }
//     } else {
//         phaseCounter += elapsed; // Update phase duration counter
//     }

//     // Check if all repeats are done for the current pattern
//     if (repeatCounter >= audioTiming[index][2]) {
//         currentPattern = 0; // Reset pattern to allow re-entry or change
//     }
// }

// void checkTimer3(){
//     if(true){
//         if ((TCCR3B & ((1 << CS32) | (1 << CS31) | (1 << CS30))) == 0) {
//             uartPrint("Timer3 is stopped");
//         } 
//         else {
//             uartPrint("Timer3 is running");
//         }
//     }
// }
// void CheckTimer1(uint8_t timer1Cnt){
//     if (false){
//         if ((TCCR1B & ((1 << CS12) | (1 << CS11) | (1 << CS10))) == 0) {
//             sprintf(debugMsg,"T1 stopped: %d, %d", timer1Cnt, OCR1A);
//         } 
//         else {
//             sprintf(debugMsg,"T1 running: %d, %d", timer1Cnt, OCR1A);
//         }
//         uartPrint(debugMsg);
//     }
// }

// uintptr_t GetPosEepromAddress(uint8_t posIndex, uint8_t axis){
//     // Calculate the address offset for a position coordinate
//     if (posIndex < MAX_NBR_MAP_PTS){
//         if (axis == 0){
//             return (uintptr_t)&EramPositions[posIndex].EramX - (uintptr_t)&EramPositions[0];
//         } else
//         {
//             return (uintptr_t)&EramPositions[posIndex].EramY - (uintptr_t)&EramPositions[0];
//         }
//     }
// }
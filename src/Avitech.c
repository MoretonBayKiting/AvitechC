#pragma region Include files
    #include <stdio.h>
    #include <stdint.h>
    #include <unistd.h>
    #include <avr/eeprom.h>
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
uint8_t printCnter = 0;
uint16_t eeprom_address = 0;

uint16_t n = 0;
uint16_t Boardrevision;                                   // Board that program will be deployed to
int X = 0;                                            // X position requested to move EG X=100   .Move X 100 steps
int Y = 0;                                            // Y position requested to move EG X=100   .Move X 100 steps

bool rndLadBit;                                        // 0 or 1 to indicate if last pass was for a new rung on one side or the next side is needed
uint8_t minYind;
uint8_t maxYind;
uint8_t RndNbr, PrevRndNbr, AltRndNbr;

int AbsX = 0;                                         // X absolute position from home position
int AbsY = 0;                                         // Y absolute position from home position

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
int Dy;                                              // Used to calulate how many steps required on Y axis from last position
int Dx;                                              // Used to calulate how many steps required on X axis from last position
bool Master_dir;                                       // Which motor need to step the most X or Y
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
bool X_TravelLimitFlag;                                // Over travel flag
bool Y_TravelLimitFlag;                                // Over travel flag
uint16_t y_maxCount;                                   // Max Count Y can have. This changes for different mode as more or less tilt is required in different modes
uint8_t MapCount[2][NBR_ZONES]; // MapCount[1][i] is incremental; MapCount[2][i] is cumulative of MapCount[1][i]
uint8_t Zn;
// uint8_t NoMapsRunningFlag;  //Although this is set (BASCOM), it doesn't appear to be used.
// uint16_t AppCompatibilityNo;
uint8_t GyroAddress = 1;

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
// uint8_t EramUserLaserPower[3]; // Laser Power that the user sets up
// uint8_t DefaultEramUserLaserPower;
uint8_t UserLaserPower;
uint8_t EEMEM EramUserLaserPower;

// uint8_t EramMaxLaserPower[3]; // Store Eram laser operating power setting
// uint8_t DefaulteramMaxLaserPower;
uint8_t MaxLaserPower;
uint8_t EEMEM EramMaxLaserPower;
uint8_t EEMEM firstTimeOn;
uint8_t LaserPower; // Final calculated value send to the DAC laser driver

float VoltPerStep = LINE_VOLTAGE / 4095; // Laser power per step

uint8_t LaserOverTempFlag; // Laser over temp error flag
uint8_t FlashTheLaserFlag; // Setup laser flash flag bit
uint8_t CmdLaserOnFlag;

uint8_t SetupModeFlag = 1; // Should this be set to 1 (setup mode - 0 is run mode) as default? 0 in BASCOM version.

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
uint8_t DataInBufferFlag; // Flag for there is data in the RS232 comms buffer

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

uint8_t SendDataFlag;                                      // Diagnostic mode send data back to user
uint8_t SendSetupDataFlag;                                 // Send config data back to user.  20240522: Not used 

uint8_t FirstTimeLaserOn;
uint8_t WarnLaserOnOnce;
uint8_t IsHome;

long Secondsfromreset;                                     // Variable to hold the reset current count

uint8_t SystemFaultFlag;
uint8_t Index;

uint16_t EEMEM EramLaserID;
uint16_t LaserID;

uint16_t EEMEM EramAccelTripPoint;                                 // Accelerometer trip angle value
uint16_t AccelTripPoint;

uint8_t Zonespeed;
uint16_t CurrentSpeedZone;

uint16_t MapRunning;

uint8_t Counter50ms;

//---------MPU6050 IMU  & Temperature, then HC-05 bluetooth module-----------------
union { // Accel_Z to be used to access any of Z_accel, int or the subcomponents
    int Z_accel;  // Accelerometer uses 2 bytes of memory. Data is read from the chip in 2 lots of 1 byte packets
    struct {
        uint8_t Zl_accel;       // Stored memory of first byte data from accelerometer
        uint8_t Zh_accel;       // Stored memory of second byte data from accelerometer
    };
} Accel_Z;

uint8_t Z_AccelFlag = 0;
uint8_t Z_AccelFlagPrevious;

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

int Vertices[3][MAX_NBR_VERTICES];
int Perimeter[2][MAX_NBR_PERIMETER_PTS];
uint8_t NbrPerimeterPts;
int res[2];  //Global variables to be used in cartesian/polar conversions:Input and results.
volatile char ReceivedData[100];
volatile int DataCount = 0;
// uint8_t cnter = 0;

#pragma endregion Variable Definitions
#pragma region Function Declarations
void HomeAxis();
void DoHouseKeeping();
#pragma endregion Function Declarations
#pragma region Function definitions - peripherals, watchdog, uart, buzzer, timers, DAC, i2c
void setupPeripherals(){
    // Set relevant pits as output
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
void TickCounter_50ms_isr() {
    Counter50ms++;
    if (SetupModeFlag == 1 && IsHome == 1 && WarnLaserOnOnce == 0) {
        StartBuzzerInProgMode();
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
    // Enable receiver and transmitter and receive interrupt.
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

ISR(USART0_RX_vect) {  //20240607 ISR added for reading.
    char c = UDR0;
    // uartPutChar(c); // Debugging line
    // uartPutChar('\n'); // Debugging line
    if (c == '\n') { //// Use '\n' as the terminator. 
        ReceivedData[DataCount] = '\0'; // Null terminator
        DataInBufferFlag = 1; // Set the flag
        DataCount = 0; // Reset the counter
    } else {
        ReceivedData[DataCount] = c;
        DataCount++;
    }
}

void CheckBluetooth() {
    if (DataInBufferFlag == 1) { // We have got something
        char *token;
        token = strchr(ReceivedData, '<');  // Find the start of the command
        if (token != NULL) {
            Command = atoi(token + 1);// Convert the command to an integer
            // token = strchr(ReceivedData, ':');  // Find the start of the instruction
            token = strchr(token, ':');  // Find the start of the instruction
            if (token != NULL) {
                char *end = strchr(token, '>');
                if (end != NULL) {
                    *end = '\0'; // Replace '>' with '\0' to end the string
                }
                Instruction = atoi(token + 1);
                // Print the command and instruction for debugging
                sprintf(debugMsg, "Command: %d, Instruction: %d", Command, Instruction);
                uartPrint(debugMsg);
                DecodeCommsData();  // Process the command and instruction
            }
        }
        // Reset the buffer and the flag
        memset(ReceivedData, 0, sizeof(ReceivedData));
        DataInBufferFlag = 0;
    }
}

void uartRead(char* buffer, int length) {
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

void testLoopUart(char* testString) {
    char c; // Character to hold incoming data
    // if(false){  //Keep this (and the closing brace) if this is to be disabled.
    for (int i = 0; i < 1000; i++) {  // Replace 1000 with the number of iterations you want
        sprintf(debugMsg, "%s: DM:  %d", testString, i);
        uartPrint(debugMsg);
        while (uartAvailable()) { // Check if data is available to read
            c = uartGetChar(); //Read a single character
            _delay_ms(1);
            uartPutChar('z');
            uartPutChar(c);
            if (c == 'C'){
                    uartPrint("Have read C ");
                    ClearSerial();
                    return; // Break the loop
                }
            }
        _delay_ms(1000);  // Delay for 1 second. Adjust as needed.
        }
    // }
}

void printToBT(uint8_t cmd, uint16_t inst){
    char printToBTMsg[20];
    sprintf(printToBTMsg, "<%02d:%04x>", cmd,  inst);
    uartPrint(printToBTMsg);
    _delay_ms(20);
}

void i2c_wbyte(uint8_t data) { //Constructed to mirror BASCOM which doesn't start or stop the bus
    // Send data
    TWDR0 = data;
    TWCR0 = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR0 & (1<<TWINT)));
}

void i2c_init(void) {
    // Set the bit rate to 100 kHz
    TWSR0 = 0x00;
    TWBR0 = ((F_CPU / 100000L) - 16) / 2;
}

void i2c_stop(void) {
    // Transmit STOP condition
    TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    while(TWCR0 & (1<<TWSTO));
}

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
void stepperDriverISR() {
    // uint8_t x_dir_state = (PIND & (1 << X_DIR)) != 0; //Could use PORTD rather rathan PIND.  But as all (that are used) are configured for output, either can be used.
    uint8_t x_dir_state = (PIND & (1 << X_DIR)) == 0; 
    uint8_t y_dir_state = (PIND & (1 << Y_DIR)) != 0;

    n++;
    if (n==2000 & false){
        sprintf(debugMsg,"PD: %#02x, PB: %#02x,SC: %d,  MD: %d",PORTD, PINB, StepCount, Master_dir);
        uartPrint(debugMsg); 
        n=0;
        }
    if (StepCount > 0) {  //StepCount is uint16_t so can't be negative. So this is equivalent to StepCount !=0 .
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
    if (Remainder < 1) {
            PORTD |= (1 << X_STEP) | (1 << Y_STEP); // Turn on X and Y step pins.
            _delay_us(1);
            PORTD &= ~((1 << X_STEP) | (1 << Y_STEP)); // Turn off X and Y step pins.
            _delay_us(1);
            // Update AbsX and AbsY using bitwise operations-replaced (20240606) with ternary operator.
            // AbsX += (~x_dir_state & 1) - (x_dir_state & 1);
            // AbsY += (~y_dir_state & 1) - (y_dir_state & 1);
            AbsX += x_dir_state ? 1 : -1;
            AbsY += y_dir_state ? 1 : -1;
        } else {
            if (Master_dir == 1) { //20240606: Changed from 1 to 0.  Then back to 1.  Master_dir is not aligned with axis (where 0 = X).
                PORTD |= (1 << X_STEP); // Turn on X step pins.
                _delay_us(1);
                PORTD &= ~(1 << X_STEP); // Turn off X step pins.
                _delay_us(1); //BASCOM used a 1us pulse.  Surely 1ms is fine.
                // Update AbsX using bitwise operations
                AbsX += (~x_dir_state & 1) - (x_dir_state & 1);
            } else {
                PORTD |= (1 << Y_STEP); // Turn on Y step pins.
                _delay_us(1);
                PORTD &= ~(1 << Y_STEP); // Turn off Y step pins.
                _delay_us(1);
                // Update AbsY using bitwise operations
                AbsY += (~y_dir_state & 1) - (y_dir_state & 1);
            }
        }
    } else {
        // Stop Timer1
        SteppingStatus = 0;
    }
}
//Setup ISRs
ISR(TIMER1_COMPA_vect){
    // sprintf(debugMsg,"DSS_PL: %d",DSS_preload);
    // uartPrint(debugMsg); 
    stepperDriverISR();
}
// ISR for Timer3 Compare A vector
ISR(TIMER3_COMPA_vect){
    // uartPrint("PreWD3\r");    
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
void SetLaserVoltage(uint8_t voltage) {
    uint16_t D;
    uint8_t Hi;
    uint8_t Lo;
    float Lvolt;

    if (voltage == 0 || Laser2OperateFlag == 0 || Laser2StateFlag == 0) {
        PORTE &= ~(1 << LASER2); // Turn off Laser2
    } else {
        PORTE |= (1 << LASER2); // Turn on Laser2
    }

    Lvolt = voltage / 51.0f; // Converts a byte value to a voltage for the laser DAC to set
    D = Lvolt / VoltPerStep; // Calculate how many steps to set the requested voltage
    D <<= 4; // Shift left by 4 bits
    Hi = D >> 8; // Get the first 8 bits MSB
    Lo = D & 0xFF; // Get the last 8 bits LSB

    i2c_start(MCP4725); // Start the I2C bus for the laser power DAC.
    // From BASCOM: 
    // '1st byte sent HC0=1100 000 0   1st byte of register 1100=Device code. 000=AddressBits Lats bit A0set with I/O pin.0 R/W option
    // 2nd byte sent &H60=0110 EEPROM and Voltage output control register DAC. Sets up start up voltage , &H40=0100 Voltage output control register only.
    i2c_wbyte(0xC0); // Send the 1st byte
    i2c_wbyte(0x40); // Send the 2nd byte
    i2c_wbyte(Hi); // Send the 3rd byte (MSB)
    i2c_wbyte(Lo); // Send the 4th byte (LSB)
    i2c_stop(); // Stop the I2C bus

    if (voltage > 0 && BatteryTick > 4) {
        GetBatteryVoltage();
        BatteryTick = 0;
    }
}

void initDACMCP4725() {
    i2c_init(); // Initialize I2C with 400kHz bit rate
    if (i2c_start(MCP4725)) { // Start I2C communication and send MCP4725 address
        // The operation failed
        sprintf(debugMsg, "Operation failed ");
        uartPrint(debugMsg);
    }
    i2c_wbyte(0x0001); // Send data to set the default power to 0 volts in the EEPROM volt on startup
    i2c_stop(); // Stop I2C communication
}

void firstOn(){
    uint8_t firstTimeOn = eeprom_read_byte(&firstTimeOn);
    if (firstTimeOn == 0xFF) {  // EEPROM is erased to 0xFF
        LoadEramDefaults();
        SetLaserVoltage(0);
        initDACMCP4725();
        uartPrint("AT+ENLOG0\r");  // Turn off logging
    }
}

void Audio(uint8_t pattern){
    // uint16_t waitTime1;
    // uint16_t waitTime2;
    uint8_t repeatCount = 2; //2 repeats for all but pattern 1
    uint8_t i;
    sprintf(debugMsg,"In Audio. Pattern: %d",pattern);
    uartPrint(debugMsg);

    if (pattern ==1) {repeatCount = 1;}  
    if (pattern ==6) {repeatCount = 4;}  
    //20240528:  _delay_ms() takes a constant argument.  So previous strategy of passsing a parameter to it didn't work.
    for (i = 0; i < repeatCount; i++) {
        PORTE |= (1 << BUZZER); // Set BUZZER pin to HIGH
        switch (pattern) {
            case 1: _delay_ms(100); break;
            case 2: _delay_ms(1); break;
            case 3: _delay_ms(500); break;  //Initially 500.  Changed to 2500 for testing (20240601).
            case 4: _delay_ms(50); break;
            case 5: _delay_ms(50); break;
            case 6: _delay_ms(500); break;
        }
        PORTE &= ~(1 << BUZZER); // Set BUZZER pin to LOW
        switch (pattern) {
            case 2: _delay_ms(100); break;
            case 3: _delay_ms(100); break;
            case 4: _delay_ms(150); break;
            case 5: _delay_ms(50); break;
            case 6: _delay_ms(1500); break;
        }
    }
}

void WaitAfterPowerUp() {
    if (FirstTimeLaserOn == 1) {
        _delay_ms(5);
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
    // static uint16_t n;
    // n++;
    if (LaserTick == 0) {
        SetLaserVoltage(0); // Off 150ms
    } else if (LaserTick == 3) {
        SetLaserVoltage(LaserPower); // On 850ms
    }    
    // if (n == 1000){
    //     sprintf(debugMsg,"In StartLFIP. LaserTick, LaserPower: %d, %d",LaserTick, LaserPower);
    //     uartPrint(debugMsg);
    //     n = 0;
    // }
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
    i2c_start(MPU6050_W_PIN);
    // i2c_wbyte(Mpu6050_w);
    i2c_wbyte(0x3F);  //0x3F is the high byte of the Z accelerometer register.
    i2c_start(MPU6050_R_PIN);
    // i2c_wbyte(Mpu6050_r);
    // Ref: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
    Accel_Z.Zh_accel = i2c_rbyte(1); // Read Z high byte (0x3F) and send ACK
    Accel_Z.Zl_accel = i2c_rbyte(1); // Read Z low byte (0x40) and send ACK
    Accel.H_acceltemp = i2c_rbyte(1); // Read Temp high byte (0x41) and send ACK
    Accel.L_acceltemp = i2c_rbyte(0); // Read Temp low byte (0x42) and send NACK
    i2c_stop();
    // Acceltemp = (Accel.H_acceltemp << 8) | Accel.L_acceltemp;  //This may be the wrong way around .  Need to check endianness.
    Accel.Acceltemp = Accel.Acceltemp / 340;
    Accel.Acceltemp = Accel.Acceltemp + 36.53;
}
void DecodeAccelerometer() {
    if (OperationMode == 0) { // Field-1
        if (Accel_Z.Z_accel < AccelTripPoint) {
            Z_AccelFlag = 1;
            SystemFaultFlag = 1;
        } else {
            Z_AccelFlag = 0;
        }
    }

    // Similar if conditions for OperationMode 1, 2, 3, 4.  But they need to be reviewed.  For example most (BASCOM) have If Z_accel < Acceltrippoint Then  but OpMode = 3 has If Z_accel > Acceltrippoint Then 

    if (Z_AccelFlagPrevious == 1 && Z_AccelFlag == 0 && AccelTick < 10) {
        Z_AccelFlag = 1;
        SystemFaultFlag = 1;
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
void ProcessError() {
    if (Z_AccelFlag == 1) {
        // sprintf(debugMsg,"Audio Z_Accel");
        // uartPrint(debugMsg);
        Audio(3);
        return;
    }

    if (LaserOverTempFlag == 1) {
        // sprintf(debugMsg,"Audio LaserTemp");
        // uartPrint(debugMsg);
        Audio(4);
        return;
    }

    if (X_TravelLimitFlag == 1) {
        // sprintf(debugMsg,"Audio XLim");
        // uartPrint(debugMsg);
        Audio(5);
        return;
    }

    if (Y_TravelLimitFlag == 1) {
        // sprintf(debugMsg,"Audio YLim");
        // uartPrint(debugMsg);
        Audio(5);
        return;
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

    // stopTimer1();
    TCCR1B &= ~((1 << CS12) | (1 << CS10));
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

    // stopTimer3();
    TCCR3B &= ~((1 << CS32) | (1 << CS30));
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
    float Result = 0.0;
    float L_power = 0.0;
    unsigned int SensorReading = readADC(0);
    uint8_t B_volt = 0;
    uint8_t Y;  //
    uint8_t X;  //

    if (BatteryVoltage < 98) B_volt = 25;
    else if (BatteryVoltage < 100) B_volt = 50;
    else if (BatteryVoltage < 105) B_volt = 70;
    else if (BatteryVoltage < 110) B_volt = 90;
    else B_volt = 100;

    if (Laser2OperateFlag == 1) {
        Y = Laser2BattTrip + 5;  //Add hysteresis value if battery is low    eg 10.5 +0.5  = 11.0 for the reset level
        if (BatteryVoltage < Laser2BattTrip) Laser2BattTripFlag = 1;
        if (BatteryVoltage > Y) Laser2BattTripFlag = 0;

        X = Laser2TempTrip - 2;  //Add hysteresis value if the main laser is to hot. Reset value is 50-2=48deg's for reset back on
        if (LaserTemperature > Laser2TempTrip) Laser2TempTripFlag = 1;
        if (LaserTemperature < X) Laser2TempTripFlag = 0;

        if (Laser2TempTripFlag == 0 && Laser2BattTripFlag == 0) Laser2StateFlag = 1;
        else Laser2StateFlag = 0;
    }

    // Calculate the laser power directly from the sensor reading using the quadratic function
    // Ref sheet LaserTemp of Alg.xlsm - this correlation directly from sensor reading to percentage of laser power.
    // y = -0.0087x2 + 12.361x - 4268.5
    // R² = 0.9996
    L_power = -0.0087 * SensorReading * SensorReading + 12.361 * SensorReading - 4268.5;
    // Ensure the laser power is within the valid range
    if (L_power < 0) L_power = 0;
    else if (L_power > 100) L_power = 100;
    if (B_volt < L_power) L_power = B_volt;
    Result = L_power / 100.0f;
    L_power = Result * MaxLaserPower;
    Result = UserLaserPower / 100.0f;
    LaserPower = L_power * Result;

    if (LaserTemperature > 55) {
        LaserPower = 0;
        LaserOverTempFlag = 1;
        SystemFaultFlag = 1;
    } else {
        LaserOverTempFlag = 0;
        SystemFaultFlag = 0;
    }
}

void initMPU6050() {
    i2c_init(); // initialize I2C library

    // turn on the internal temp reading register
    i2c_start(MPU6050_W_PIN);
    i2c_wbyte(0x6B);
    i2c_wbyte(0x01);
    i2c_stop();
    _delay_ms(100);

    // reset signal accel path
    i2c_start(MPU6050_W_PIN);
    i2c_wbyte(0x68);
    i2c_wbyte(0x03);
    i2c_stop();
    _delay_ms(100);

    // Set the slowest sampling rate
    i2c_start(MPU6050_W_PIN);
    i2c_wbyte(0x19);
    i2c_wbyte(0xFF);
    i2c_stop();

    // Set full scale reading to 16g's
    i2c_start(MPU6050_W_PIN);
    i2c_wbyte(0x1C);
    i2c_wbyte(0x18);
    i2c_stop();

    // Read the WHO_AM_I register
    i2c_start(MPU6050_W_PIN);
    i2c_wbyte(WHO_AM_I_MPU6050);
    i2c_stop();

    i2c_start(MPU6050_R_PIN);
    uint8_t MPU_whoami = i2c_rbyte(0);
    i2c_stop();

    sprintf(debugMsg, "Accel Chip &H%d", MPU_whoami);
    uartPrint(debugMsg);

    sprintf(debugMsg, "Gyro Flag: %d", GyroAddress);
    uartPrint(debugMsg);
}

void startTimer1() {
    // Set prescaler to 256 and start Timer1
    TCCR1B |= (1 << CS12);  //(1 << CS12)|(1 << CS10) for 1024 prescalar (4 times slower)
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);
}

void stopTimer1() {
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
    Opzone = eeprom_read_word(&EramPositions[i].EramY);
    Opzone >>= 12;
    switch (Opzone) {
        case 1: Opzone = 1; break;
        case 2: Opzone = 2; break;
        case 4: Opzone = 3; break;
        case 8: Opzone = 4; break;
    }
    return Opzone;
}
void getMapPtCounts(bool doPrint) {
    uint8_t MapIndex;
    uint8_t I, Prev_i, Prev_i_1;
    I = 0;
    Prev_i = 0;
    for (MapIndex = 1; MapIndex <= MapTotalPoints; MapIndex++) {
        I = GetZone(MapIndex);
        if (I > 0) {
            if (I != Prev_i) {
                if (Prev_i == 1) {
                    MapCount[1][Prev_i] = MapIndex;
                    MapCount[2][Prev_i] = MapIndex - 1;
                } else {
                    MapCount[2][Prev_i] = MapIndex - 1;
                    Prev_i_1 = Prev_i - 1;
                    MapCount[1][Prev_i] = MapIndex - MapCount[2][Prev_i_1];
                    MapCount[1][Prev_i] = MapCount[1][Prev_i] + 1;
                }
            }
            if (MapIndex == MapTotalPoints) {
                if (I > 1) {
                    Prev_i = I - 1;
                }
                MapCount[2][I] = MapIndex;
                if (I == 1) {
                    MapCount[1][I] = MapIndex + 1;
                } else {
                    MapCount[1][I] = MapIndex - MapCount[2][Prev_i];
                    MapCount[1][I] = MapCount[1][I] + 1;
                }
            }
            Prev_i = I;
        }
    }
}
void LoadZoneMap(uint8_t zn) {
    //---------Gets vertices for given zone from EEPROM.
    //The Y dat MSB 4 bits are the operation zone and the 16 bits LSB is the map point number
    // 1111  1111111111111111
    //  ^             ^------------ Map point number   eg point 1, point 40 etc
    //  ^--------------------- Operation Zone     eg Zone 0 to 4
    uint8_t MapIndex, MI;
    // int temp;

    if (MapTotalPoints == 0) {
        return;
    }

    for (MapIndex = 0; MapIndex < MapCount[0][zn] - 1; MapIndex++) {
        if (zn == 0) {
            MI = MapIndex;
        } else {
            MI = MapIndex + MapCount[2][zn - 1];
        }
        Vertices[0][MapIndex] = eeprom_read_word(&EramPositions[MI].EramX);
        Vertices[1][MapIndex] = eeprom_read_word(&EramPositions[MI].EramY) & 0x0FFF;
    }
    // Add a repeated vertex equal to the first vertex.  MapIndex has incremented by the "next MapIndex" statement - I think?
    Vertices[0][MapIndex] = Vertices[0][0];
    Vertices[1][MapIndex] = Vertices[1][0];
    // Get the slope of the segment, stored in Vertices[2][i]
    for (MapIndex = 1; MapIndex < MapCount[0][zn]; MapIndex++) {
        Vertices[2][MapIndex - 1] = (Vertices[0][MapIndex] - Vertices[0][MapIndex - 1]) * 10 / (Vertices[1][MapIndex] - Vertices[1][MapIndex - 1]);
    }
}
int getCartFromTilt(int t) {
   float a;
   a = LASER_HT/tan(t/STEPS_PER_RAD);
   return (int)(a + 0.5); // rounding to the nearest integer before casting
}
int getTiltFromCart(int rho) {
   float a;
   a = atan(LASER_HT/(float)rho);
   return (int)(a*STEPS_PER_RAD + 0.5); // rounding to the nearest integer before casting
}
int getNextTiltVal(int thisTilt, uint8_t dirn) {
   int rho;
   rho = getCartFromTilt(thisTilt);
   if (dirn == 1) {
      rho = rho - TILT_SEP;
   } else {
      rho = rho + TILT_SEP;
   }
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
void GetPerimeter(uint8_t zn) {
    uint8_t I, J = 0, L, M;
    // float Scale = 50.0;
    int nextTilt;
    uint8_t testBit;
    uint8_t dirn;

    for (I = 1; I < MapCount[1][zn] - 1; I++) {
        L = I + 1;
        J++;
        Perimeter[1][J] = Vertices[1][I];
        Perimeter[2][J] = Vertices[2][I];

        dirn = 0;
        if (Vertices[2][L] > Vertices[2][I]) dirn = 1;
        if (Vertices[2][L] < Vertices[2][I]) dirn = 2;

        if (dirn != 0) {
            nextTilt = getNextTiltVal(Perimeter[2][J], dirn);
            testBit = 0;
            if (nextTilt < Vertices[2][L] && dirn == 1) testBit = 1;
            if (nextTilt > Vertices[2][L] && dirn == 2) testBit = 1;

            while (testBit == 1) {
                M = J;
                J++;
                Perimeter[2][J] = nextTilt;
                Perimeter[1][J] = abs(Perimeter[2][J] - Perimeter[2][M]);
                Perimeter[1][J] = Perimeter[1][J] * abs(Vertices[3][I]);
                Perimeter[1][J] = Perimeter[1][J] / 10;

                if (Vertices[1][L] > Vertices[1][I]) {
                    Perimeter[1][J] = Perimeter[1][M] + Perimeter[1][J];
                } else {
                    Perimeter[1][J] = Perimeter[1][M] - Perimeter[1][J];
                }

                nextTilt = getNextTiltVal(Perimeter[2][J], dirn);
                testBit = 0;
                if (nextTilt < Vertices[2][L] && dirn == 1) testBit = 1;
                if (nextTilt > Vertices[2][L] && dirn == 2) testBit = 1;
            }
        }
    }
    NbrPerimeterPts = J - 1;
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
uint8_t getRndLadInd(uint8_t rnd) {  //Get the perimeter point reasonably close to horizontally opposite (ie panwise opposite) the input point.
    uint8_t tempInd;
    if (minYind > maxYind) { // Change MinY and MaxY to ensure minY <= maxY
        tempInd = minYind;
        minYind = maxYind;
        maxYind = tempInd;
    }

    if (rnd >= minYind && rnd <= maxYind) {
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
uint8_t getExtremeY(uint8_t direction) {
    int val, maxval;
    uint8_t i, ind;
    maxval = -30000;
    ind = 1;

    for (i = 1; i < NbrPerimeterPts; i++) {
        val = Perimeter[2][i];
        if (direction == 0) {
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
    if (ind <= NbrPerimeterPts) {
        X = Perimeter[1][ind];
        Y = Perimeter[2][ind];
    } else {
        PrevRndNbr = RndNbr;
        RndNbr = rand() % NbrPerimeterPts; // equivalent to Rnd(NbrPerimeterPts)
        if (RndNbr == 0) {
            RndNbr = 1;
        }
        AltRndNbr = RndNbr;

        if (pat == 2) { // This is the random ladder case when the opposite side of the rung is needed.
            if (rndLadBit == 1) {
                minYind = getExtremeY(0);
                maxYind = getExtremeY(1);
                AltRndNbr = getRndLadInd(PrevRndNbr);
                rndLadBit = 0;
            } else {
                rndLadBit = 1;
            }
        }
        X = Perimeter[1][AltRndNbr];
        Y = Perimeter[2][AltRndNbr];
    }
}
void ProcessCoordinates() {
    StepOverRatio = 0;

    Dx = X - AbsX; // Distance to move
    Dy = Y - AbsY; // Distance to move
    // sprintf(debugMsg, "Dx, X, AbsX, Dy, Y, AbsY, PDx2: %d,%d,%d,%d,%d,%d,%d,%d", Dx, X, AbsX, Dy, Y, Dy, PIND, PORTD);  //This version used first.
    // sprintf(debugMsg, "Dx, X, AbsX, Dy, Y, AbsY, PDx2: %d,%d,%d,%d,%d,%d,%u,%u", Dx, X, AbsX, Dy, Y, Dy, PIND, PORTD);  //Could use unsigned int or
    // sprintf(debugMsg, "Dx: %d, X: %d, AbsX:%d, Dy:%d, Y:%d, AbsY:%d, PD:%#02x, PB:%#02x", Dx, X, AbsX, Dy, Y, Dy, PORTD, PINB);//hexadecimal
    // uartPrint(debugMsg);
    if (Dx > 0) {
        PORTD |= (1 << X_DIR); // Set X_DIR pin high
    } else {
        PORTD &= ~(1 << X_DIR); // Set X_DIR pin low
        Dx = Dx * -1; // Convert all distances to a positive number
    }

    if (Dy > 0) {
        PORTD |= (1 << Y_DIR); // Set Y_DIR pin high
    } else {
        PORTD &= ~(1 << Y_DIR); // Set Y_DIR pin low
        Dy = Dy * -1;
    }
    // _delay_ms(100);  20240610: Why was this here?  Removed.
    // sprintf(debugMsg, "_Dx: %d, X: %d, AbsX:%d, Dy:%d, Y:%d, AbsY:%d, PD:%#02x, PB:%#02x", Dx, X, AbsX, Dy, Y, Dy, PORTD, PINB);//hexadecimal
    // uartPrint(debugMsg);

    if (Dx > Dy) {
        Master_dir = 1; // leading motor the moves the most X
        StepOverRatio = Dx / Dy; // Slope ratio between X and Y
        StepCount = Dx;
    } else {
        Master_dir = 0;
        StepOverRatio = Dy / Dx;
        StepCount = Dy;
    }
}
void MoveMotor(uint8_t axis, int steps, uint8_t waitUntilStop) {
    if (axis == 0) { // pan
        X = steps;
        Y = 0;
    } else { // tilt
        X = 0;
        Y = steps;
    }
    ProcessCoordinates();
    DSS_preload = HOMING_SPEED;
    SteppingStatus = 1;
    startTimer1();
    if (waitUntilStop == 1) {  //So, if waitUntilStop is not 1, execution returns ?
        // MoveMotor() is only called with waitUntilStop = 0 by HomeMotor() - ie when unit searching for limit switch (both pan & tilt separately)
        while (SteppingStatus == 1) {
            // do nothing while motor moves.  SteppingStatus is set to 0 at end of stepper ISR when StepCount !>0 (==0).
        }
    }
}
void StopSystem() {
    stopTimer1();
    SteppingStatus = 0;  // Reset stepping status
    X = AbsX; // Set X and Y to their absolute values
    Y = AbsY;
}
void JogMotors(bool prnt) {
    if (PanEnableFlag == 1) {  // Check if pan is enabled
            if(prnt){
                sprintf(debugMsg, "In Jog PEF");
                uartPrint(debugMsg);
            }
        MoveMotor(0, PanSpeed, 1);
    }
    if (TiltEnableFlag == 1) {  // Check if tilt is enabled
            if(prnt){
                sprintf(debugMsg, "In Jog TEF");
                uartPrint(debugMsg);
            }
        MoveMotor(1, TiltSpeed, 1);
    }
    if (PanEnableFlag == 0 && TiltEnableFlag == 0) {  // If both pan and tilt are disabled, stop the motors.
            if(prnt){
                sprintf(debugMsg, "In Jog 0EF");
                uartPrint(debugMsg);
            }
        StopSystem();
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
            _delay_ms(50);
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
    MoveMotor(axis, steps, 0);
    if (axis == 0) {
        while (!(PINB & (1 << PAN_STOP))){  //While pan_stop pin is low.
            // do nothing while motor moves
        }
    } else {
        while (!(PINB & (1 << TILT_STOP))) {  //While tilt_stop pin is low.
            // do nothing while motor moves
        }
    }

    stopTimer1();
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
    startTimer1();
    while (SteppingStatus == 1) { // do nothing while motor moves 
    }
    stopTimer1(); // Stop the motor from stepping as sensor has been triggered
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
    SetLaserVoltage(0); // Turn off laser
    // *********PAN AXIS HOME****************
    if ((PINB & (1 << PAN_STOP))){  //If pan_stop pin is high... "Move blade out of stop sensor at power up"
    // if (!(PINB & (1 << PAN_STOP))){  //If pan_stop pin is low
        MoveMotor(0, -300, 1);
    }
    HomeMotor(0, 17000); //Pan motor to home position
    // *********TILT AXIS HOME****************
    if ((PINB & (1 << TILT_STOP))) {   //If tilt_stop pin is high (ie at limit). "Move blade out of stop sensor at power up"
        MoveMotor(1, 300, 1);
    }
    HomeMotor(1, -5000);  //Tilt motor homing position
    // --**Move Tilt into final position**--
    switch (OperationMode) {
        case 0: Correctionstepping = 100; break;
        case 1: Correctionstepping = -220; break;
        case 2: Correctionstepping = 100; break;
        case 3: Correctionstepping = -220; break;
        default: Correctionstepping = 100; break;
    }
    MoveMotor(1, Correctionstepping, 1);

    NeutralAxis();
    ClearSerial();
    Audio(5);
    CmdLaserOnFlag = 0;
    IsHome = 1;
}

void RunSweep(uint8_t zn) {
    uint8_t NbrPts, PatType, i;
    int last[2],nxt[2],nbrMidPts;

    LoadZoneMap(zn);
    GetPerimeter(zn);
    NbrPts = 30;
    for (PatType = 1; PatType <= 2; PatType++) {
        NbrPts = NbrPts + NbrPerimeterPts;
            for (uint8_t Index = 1; Index <= NbrPts; Index++) {
            // Store present point to use in interpolation
            last[0] = X;
            last[1] = Y;
            getXY(Index, PatType, zn);
            nxt[0] = X;
            nxt[1] = Y;

            CalcSpeedZone();
            DSS_preload = CalcSpeed();

            if (Index == 1) {
                CmdLaserOnFlag = 0;
                DSS_preload = STEP_RATE_MAX;
            }

            if (Index == 2) {
                CmdLaserOnFlag = 1;
            }

            nbrMidPts = abs(nxt[0] - last[0])/ MID_PT_SEPARATION;
            for (i = 0; i <= nbrMidPts; i++) {
                if (nbrMidPts > 0) {
                    if (i == nbrMidPts) {
                        X = nxt[0];
                        Y = nxt[1];
                    } else {
                        CartesianInterpolate(last,nxt,i, nbrMidPts, res);
                        getPolars(res[1], res[2], res);  //Pass res as input 
                        X = res[1];
                        Y = res[2];
                    }
                }
                ProcessCoordinates();

                SteppingStatus = 1;
                startTimer1();

                while (SteppingStatus == 1) {
                    DoHouseKeeping();
                    if (SetupModeFlag == 1) { //Will only arise if SetupModeFlag is changed after RunSweep is called (which will occur if SetupModeFlag == 0 - run mode.)
                        stopTimer1();
                        StepCount = 0;
                        SteppingStatus = 0;
                        return;
                    }
                }
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
        if (i==0){
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
    // uint16_t Hexresult;
    // char Result[5];

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
    // uint16_t Hexresult;
    // char Result[5];
    uint8_t i;
    uint16_t ConfigData[12];
    uint8_t ConfigCode[12];

    PrintZoneData();

    ConfigData[1] = ActiveMapZones;
    ConfigCode[1] = 15;

    ConfigData[2] = MapTotalPoints;
    ConfigCode[2] = 04;

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
    printToBT(9, 0);
    _delay_ms(1000);  
}

void OperationModeSetup(int OperationMode) {
    char* name;
    ClearSerial();
    _delay_ms(2000); // Wait for 2 seconds
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

    CheckBluetooth();
    // ReadAccelerometer();
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
            // stopTimer3();
            TCCR3B &= ~((1 << CS32) | (1 << CS30));
        }
    }

    if (SystemFaultFlag == 1) {
        SetLaserVoltage(0);
        stopTimer1();
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
        if (CmdLaserOnFlag == 1 && MapTotalPoints >= 2) {
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
    uart_init(MYUBRR);  // Initialize UART with baud rate specified by macro
    setupTimer1();
    setupTimer3();
    setupPeripherals();
    setupWatchdog();
    OperationModeSetup(OperationMode);     // Select the operation mode the device will work under before loading data presets
    testLoopUart("AfterOpModeSetup");
    Wd_byte = MCUSR; // Read the Watchdog flag
    if (Wd_byte & (1 << WDRF)) { // there was a WD overflow. This flag is cleared of a Watchdog Config
        Wd_flag = 1; // store the flag
    }
    initDACMCP4725();  //
    firstOn();  //Load defaults to EEPROM if first time on.
    testLoopUart("AfterFirstOn");
    ReadEramVars();  //Reads user data from EEPROM to RAM.
    PORTE |= (1 << FAN); //Turn fan on.
    for (int battCount = 0; battCount < 10; battCount++) {  //BatteryVoltage should be populated by this.
        GetBatteryVoltage();   //Load up the battery voltage array
    } 
    SetLaserVoltage(0);                                     //Lasers switched off
    _delay_ms(100); 
    // testLoopUart("Before enable pins");
    sprintf(debugMsg, "PORTD, %#02x, PINB  %#02x", PORTD, PINB);
    uartPrint(debugMsg);
    PORTD &= ~(1 << X_ENABLEPIN); //Enable pan motor (active low ).
    PORTD &= ~(1 << Y_ENABLEPIN); //Enable tilt motor.
    // sprintf(debugMsg, "PORTD, %#02x, PINB:  %#02x", PORTD, PINB);
    // uartPrint(debugMsg);
    Audio(6);
    _delay_ms(1000);
    HomeAxis();
}
int main() {
    // "If Z_accelflag = 1 Then" //Need to implement something like this (search in BASCOM version) when IMU is working.
    static uint16_t cnter = 0;
    static bool doPrint = true;
    
    setup();
    while(1) {
        cnter++;
        if (false){
            _delay_ms(1000);
        }
        else{
        
        if (cnter == 300000){
            sprintf(debugMsg,"In main <CheckBT. X, AbsX, Dx, Y, SetupModeFlag: %d, %d, %d, %d, %d", X, AbsX, Dx,Y, SetupModeFlag);
            uartPrint(debugMsg);
            cnter = 0;
        }
        CheckBluetooth();  //Should this be here?  It's called from DoHouseKeeping().  It was in both places in BASCOM.
        doPrint = false;
        n++;
        // if (n==4000){
        //     n=0;
        //     doPrint = true;
        //     }
        //     else doPrint = false;
        if (SetupModeFlag == 1) {
           JogMotors(doPrint); 
        }
        if (SetupModeFlag == 0) {  //In run mode
            if(doPrint){
                sprintf(debugMsg, "GetMapPtsCts");
                uartPrint(debugMsg);
            }            
            getMapPtCounts(doPrint);  //Any need to call doPrint?
            for (Zn = 1; Zn <= NBR_ZONES; Zn++) {
                if(doPrint){}
                    // sprintf(debugMsg, "MapCount: %d", MapCount);
                    uartPrint("MapCount");
                }      
                if (MapCount[1][Zn] > 0) {
                    RunSweep(Zn);
                }
            }
        DoHouseKeeping();
        if (SetupModeFlag == 2) {
            CalibrateLightSensor();
        }
        }
        // return 0;
    } 

    return 0;
}
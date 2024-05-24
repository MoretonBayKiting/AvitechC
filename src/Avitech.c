// **********  Include files
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPiI2C.h>
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

#include "i2cmaster.h"
#include "shared_vars.h"
#include "pin_mappings.h"
#include "const.h"
#include "vars_main.h"
#include "LoadDefs.h"
// #include "Execute_App_Command_Module/Execute_App_Command_Module_Rev_2_00.h"
// #include "Validate_EERAM_Module/Validate_EERAM_Module.h"

// Function declarations - No longer used.  Some in shared_Vars.h, others only definition required.
// void ProcessCoordinates();  //Shared
// void HomeAxis();
// void HomeMotor(uint8_t axis, int steps);
// void MoveMotor(uint8_t axis, int steps, uint8_t waitUntilStop);
// void MoveLaserMotor();
// void NeutralAxis();
// void Audio(uint8_t pattern);  //Shared
// void CheckBluetooth();
// void DecodeCommsData();  //Shared
// void JogMotors();
// void OperationModeSetup();
// void SetLaserVoltage(uint8_t voltage);  Shared
// void GetLaserTemperature();  //Declaration moved to shared_Vars.h
// void ThrottleLaser();  // Shared
// void GetBatteryVoltage();
// void RunSweep(uint8_t zn);
// void CalcSpeedZone();  //Shared
// void CalcSpeed(); //Shared
// void ResetTiming();
// void WaitAfterPowerUp();
// void WarnLaserOn();
// void StartBuzzerInProgMode();
// void StartLaserFlickerInProgMode();
// void DoHousekeeping();
// void ReadAccelerometer();
// void ProcessError();
// void TransmitData();
// void DecodeAccelerometer();
// void PrintConfigData();  Shared
// void ProgrammingMode();  //Shared
// void PrintZoneData();
// void PrintAppData();
// void InitGyro();
// void LoadZoneMap(uint8_t zn);
// void GetPerimeter(uint8_t zn);
// uint8_t GetZone(uint8_t i);
// void getXY(uint8_t ind, uint8_t pat, uint8_t z);
// void CartesianInterpolate(int last[2], int nxt[2], uint8_t i, int n, int thisRes[2]) 
// uint8_t getRndLadInd(uint8_t rnd);
// uint8_t getExtremeY(uint8_t direction);
// void getMapPtCounts();
// int getNextTiltVal(int thisTilt, int dirn);
// int getCartFromTilt(int t);
// int getTiltFromCart(int rho);
// void getPolars(int c1, int c2, int c3[2]);
// void getCart(int p, int t, int c3[2]);
// void GetLightLevel();
// void calibrateLightSensor();
// void MrSleepyTime();

//*******************************STARTUP*******************************************
// Configure pins & peripherals
void setupPeripherals(){
    // Output pins
    DDRD |= (1 << X_ENABLEPIN) | (1 << Y_ENABLEPIN) | (1 << X_DIR) | (1 << Y_DIR) | (1 << X_STEP) | (1 << Y_STEP);
    DDRE |= (1 << BUZZER) | (1 << LASER2) | (1 << FAN);
    // Input pins
    DDRB &= ~(1 << TILTSTOP) | (1 << PANSTOP);

    // SCL and SDA pins are configured by the I2C library
    //Setup ISRs
    ISR(TIMER1_COMPA_vect) {
        stepperDriverISR();
    }
    // ISR for Timer3 Compare A vector
    ISR(TIMER3_COMPA_vect) {
        wdt_reset();  // Reset the watchdog timer
        TickCounter_50ms_isr();
    }

    ISR(WDT_vect) {
        // This ISR will be called when the watchdog timer times out. Without any code, the system will reset.
    }

    wdt_enable(WDTO_2S);  // Set the watchdog timeout to approximately 2 second
    WDTCSR |= (1 << WDIE); // Enable the watchdog interrupt

    // Configure the ADC
    ADMUX = (1 << REFS0); // Use AVCC as the reference voltage
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable the ADC and set the prescaler to 128 (auto)
}
// Auxiliary functions
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
        prevRndNbr = RndNbr;
        RndNbr = rand() % NbrPerimeterPts; // equivalent to Rnd(NbrPerimeterPts)
        if (RndNbr == 0) {
            RndNbr = 1;
        }
        altRndNbr = RndNbr;

        if (pat == 2) { // This is the random ladder case when the opposite side of the rung is needed.
            if (rndLadBit == 1) {
                minYind = getExtremeY(0);
                maxYind = getExtremeY(1);
                altRndNbr = getRndLadInd(prevRndNbr);
                rndLadBit = 0;
            } else {
                rndLadBit = 1;
            }
        }
        X = Perimeter[1][altRndNbr];
        Y = Perimeter[2][altRndNbr];
    }
}
void MoveMotor(uint8_t axis, int steps, uint8_t waitUntilStop) {
    if (axis == 0) { // 0 represents pan
        X = steps;
        Y = 0;
    } else { // axis = 1 represents tilt
        X = 0;
        Y = steps;
    }

    ProcessCoordinates();
    Dss_preload = HOMING_SPEED;
    Steppingstatus = 1;
    // StartTimer1();
    TCCR1B |= (1 << CS12) | (1 << CS10);

    if (waitUntilStop == 1) {
        while (Steppingstatus == 1) {
            // do nothing while motor moves
        }
    }
}
void StopSystem() {
    // Stop Timer1
    TCCR1B &= ~(1 << CS10);
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS12);

    // Reset stepping status
    Steppingstatus = 0;

    // Set X and Y to their absolute values
    X = Absx;
    Y = Absy;
}
void JogMotors() {
    // Check if pan is enabled
    if (PanEnableFlag == 1) {
        MoveMotor(0, PanSpeed, 1);
    }
    // Check if tilt is enabled
    if (TiltEnableFlag == 1) {
        MoveMotor(1, TiltSpeed, 1);
    }
    // If both pan and tilt are disabled, stop the motors.
    if (PanEnableFlag == 0 && TiltEnableFlag == 0) {
        StopSystem();
    }
}
void CalcSpeedZone() {
    unsigned int Result;

    if (Y < 48) {
        Zonespeed = Speedzone[0];
        Result = 1;
    } else if (Y < 65) {
        Zonespeed = Speedzone[1];
        Result = 2;
    } else if (Y < 76) {
        Zonespeed = Speedzone[2];
        Result = 3;
    } else if (Y < 125) {
        Zonespeed = Speedzone[3];
        Result = 4;
    } else {
        Zonespeed = Speedzone[4];
        Result = 5;
    }

    if (Result != Currentspeedzone) {
        if (Bt_ConnectedFlag == 1) {
            printf("<16:%x>", Result);
            Sleep(50);
        }
        Currentspeedzone = Result;
    }
}
unsigned int CalcSpeed() {
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
void WaitAfterPowerUp() {
    if (FirstTimeLaserOn == 1) {
        Sleep(5);
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
void StartBuzzerInProgMode() {
    if (BuzzerTick == 0) {
        BUZZER = 1; // Start buzzer when it comes to neutral position
    } else if (BuzzerTick >= 1) {
        BUZZER = 0;
    }
}
void GetBatteryVoltage() {
    unsigned int SensorReading;
    SensorReading = 0;
    Batttotal = Batttotal - BattReadings[BattReadIndex];
    SensorReading = readADC(1);
    BattReadings[BattReadIndex] = SensorReading;
    Batttotal = Batttotal + SensorReading;
    BattReadIndex++;
    if (BattReadIndex > NUM_BATT_READINGS) {
        BattReadIndex = 1;
    }

    Battvoltavg = Batttotal / NUM_BATT_READINGS;

    if (Battvoltavg <= 221) {
        BatteryVoltage = 98;        //9.8 Volts     =0
    } else if (Battvoltavg <= 330) {
        BatteryVoltage = 100;       //10.0 Volts   =12%
    } else if (Battvoltavg <= 430) {
        BatteryVoltage = 105;       //10.5 Volts   =25%
    } else if (Battvoltavg <= 567) {
        BatteryVoltage = 110;       //11.0 Volts   =50%
    } else if (Battvoltavg <= 702) {
        BatteryVoltage = 115;       //11.5 Volts   =65%
    } else if (Battvoltavg <= 832) {
        BatteryVoltage = 120;       //12.0 Volts   =80%
    } else if (Battvoltavg <= 939) {
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
        // Laser2 = Off; // Replace with appropriate code to turn off Laser2
    } else {
        // Laser2 = On; // Replace with appropriate code to turn on Laser2
    }

    Lvolt = voltage / 51.0f; // Converts a byte value to a voltage for the laser DAC to set
    D = Lvolt / Voltperstep; // Calculate how many steps to set the requested voltage
    D <<= 4; // Shift left by 4 bits
    Hi = D >> 8; // Get the first 8 bits MSB
    Lo = D & 0xFF; // Get the last 8 bits LSB

    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Start the I2C bus
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT flag set
    TWDR = 0xC0; // 1st byte sent
    TWCR = (1<<TWINT) | (1<<TWEN); // Clear TWINT to proceed
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT flag set
    TWDR = 0x40; // 2nd byte sent
    TWCR = (1<<TWINT) | (1<<TWEN); // Clear TWINT to proceed
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT flag set
    TWDR = Hi; // Send 3rd byte MSB
    TWCR = (1<<TWINT) | (1<<TWEN); // Clear TWINT to proceed
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT flag set
    TWDR = Lo; // Send 4th byte LSB
    TWCR = (1<<TWINT) | (1<<TWEN); // Clear TWINT to proceed
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT flag set
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // Stop the I2C bus

    if (voltage > 0 && Batterytick > 4) {
        GetBatteryVoltage();
        Batterytick = 0;
    }
}
void StartLaserFlickerInProgMode() {
    if (LaserTick == 0) {
        SetLaserVoltage(0); // Off 150ms
    } else if (LaserTick == 3) {
        SetLaserVoltage(LaserPower); // On 850ms
    }
}
void CheckBluetooth() {
    Datainbufferflag = Ischarwaiting(); // Check if data is in chip buffer

    if (Datainbufferflag == 1) { // We have got something
        A = Waitkey(); // Receive data from buffer

        if (A == 60) { // < Preamble
            do {
                Command = Waitkey();
                S = S + (char)Command;
            } while (Command != 58); // : Mid Terminator

            Command = atoi(S);
            S = "";

            do {
                Instruction = Waitkey();
                S = S + (char)Instruction;
            } while (Instruction != 62); // > Terminator

            Instruction = atoi(S);
            S = "";
            DecodeCommsData();
        }
    }
}
void GetLaserTemperature() {
    unsigned int SensorReading;
    unsigned int Result;
    uint8_t LoopCount;

    SensorReading = 0;
    Result = 0;
    LoopCount = 0;

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
    I2cstart();
    I2cwbyte(Mpu6050_w);
    I2cwbyte(0x3F);

    I2cstart();
    I2cwbyte(Mpu6050_r);
    Accel_Z.Zh_accel; = I2crbyte(Ack);
    Accel_Z.Zl_accel; = I2crbyte(Ack);
    Accel.H_acceltemp = I2crbyte(Ack);
    Accel.L_acceltemp = I2crbyte(Nack);
    I2cstop();
    // Acceltemp = (Accel.H_acceltemp << 8) | Accel.L_acceltemp;  //This may be the wrong way around .  Need to check endianness.
    Acceltemp = Acceltemp / 340;
    Acceltemp = Acceltemp + 36.53;
}
void DecodeAccelerometer() {
    if (OperationMode == 0) { // Field-1
        if (Z_accel < AccelTripPoint) {
            Z_accelflag = 1;
            SystemFaultFlag = 1;
        } else {
            Z_accelflag = 0;
        }
    }

    // Similar if conditions for OperationMode 1, 2, 3, 4

    if (Z_accelflagprevious == 1 && Z_accelflag == 0 && AccelTick < 10) {
        Z_accelflag = 1;
        SystemFaultFlag = 1;
        return;
    }

    Z_accelflagprevious = Z_accelflag;
    AccelTick = 0;
}
void ProcessError() {
    if (Z_accelflag == 1) {
        Audio(3);
        return;
    }

    if (LaserOverTempFlag == 1) {
        Audio(4);
        return;
    }

    if (X_TravelLimitFlag == 1) {
        Audio(5);
        return;
    }

    if (Y_TravelLimitFlag == 1) {
        Audio(5);
        return;
    }
}
void TransmitData() {
    int Hexresult;
    char Result[5];
    int Variables[19]; // Declaring this as local causes a compile problem in BASCOM.
    uint8_t i;

    Variables[0] = MaxLaserPower;
    Variables[1] = Acceltemp;
    Variables[2] = Index;
    Variables[3] = X;
    Variables[4] = Y;
    Variables[5] = Absx;
    Variables[6] = Absy;
    Variables[7] = Z_accel;
    Variables[8] = Tod_tick / 2;
    Variables[9] = Battvoltavg;
    // Variables[10] = Hw_stack;
    // Variables[11] = Sw_stack;
    Variables[10] = Frame_size;
    Variables[11] = Wd_flag;
    Variables[12] = Boardrevision;
    Variables[13] = Dss_preload;
    Variables[15] = LaserID;
    Variables[16] = AccelTripPoint;
    Variables[17] = ResetSeconds / 2;
    Variables[18] = OperationMode;

    if (SendDataFlag == 1) {
        for (i = 0; i <= 18; i++) {
            Hexresult = Variables[i];
            sprintf(Result, "%X", Hexresult);
            if (i == 0) {
                printf("<%d:%s>", 20, Result); // In original MaxLaserPower has code 20, not 30. Write over 29 and sort out later.
            } else {
                printf("<%d:%s>", i + 29, Result); // In original MaxLaserPower has code 20, not 30. Write over 29 and sort out later.
            }
            _delay_ms(50);
        }
    }
}
void PrintZoneData() {
    uint16_t Hexresult;
    char Result[5];
    uint8_t i, j;
    char jStr[3];

    for (i = 0; i < 5; i++) { //Print both count of map points (MapCount[0][i]) for for maps and 5 speed zones
        if (i==0){
            Hexresult = MapTotalPoints;  //This needs to be assigned.
            }
            else{
            Hexresult = Mapcount[0][i];
            }
        sprintf(Result, "%X", Hexresult); //Convert hex value to string and store in result.
        
        sprintf(jStr, "%02d", i + 4); // Target points are 04 for MapTotalPoints and 05:08 for the 4 zones
        printf("<%s:%s>", jStr, Result); //Prints a string in the format "<jStr:Result>" - eg "<04:0A>".
        _delay_ms(50);
        

        Hexresult = Speedzone[i];
        sprintf(Result, "%X", Hexresult);
        sprintf(jStr, "%02d", i+10); // Target points are 10:14 for 5 SpeedZones
        printf("<%s:%s>", jStr, Result);
        _delay_ms(50);
    }
}
void PrintAppData() {
    uint16_t Hexresult;
    char Result[5];

    if (Bt_ConnectedFlag == 1) {
        printf("<17:%X>", MapRunning);
        _delay_ms(50);

        if (SetupModeFlag == 0) {
            Hexresult = LaserTemperature;
            sprintf(Result, "%X", Hexresult);
            printf("<02:%s>", Result);
            _delay_ms(50);

            Hexresult = BatteryVoltage;
            sprintf(Result, "%X", Hexresult);
            printf("<03:%s>", Result);
            _delay_ms(50);

            Hexresult = UserLaserPower;
            sprintf(Result, "%X", Hexresult);
            printf("<21:%s>", Result);
            _delay_ms(50);

            Hexresult = Lightlevel;
            sprintf(Result, "%X", Hexresult);
            printf("<25:%s>", Result);
            _delay_ms(50);

            Hexresult = LaserPower;
            sprintf(Result, "%X", Hexresult);
            printf("<45:%s>", Result);
            _delay_ms(50);
        }
    }
}
void PrintConfigData() {
    uint16_t Hexresult;
    char Result[5];
    uint8_t i;
    uint16_t ConfigData[12];
    uint8_t ConfigCode[12];

    PrintZoneData();

    ConfigData[1] = Activemapzones;
    ConfigCode[1] = 15;

    ConfigData[2] = MapTotalPoints;
    ConfigCode[2] = 04;

    ConfigData[3] = _version_major;
    ConfigCode[3] = 23;

    ConfigData[4] = _version_minor;
    ConfigCode[4] = 24;

    ConfigData[5] = Userlighttriplevel;
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
        Hexresult = ConfigData[i];
        sprintf(Result, "%X", Hexresult);
        printf("<%d:%s>", ConfigCode[i], Result);
        _delay_ms(50);
    }
}
void GetLightLevel() {
    long X;
    long Y;

    Lightlevel = readADC(2); // Light sensor ADC and remove jitter
    Lightlevel >>= 2;

    if (LightTriggerOperation == 0) { // Run 24hr
        LightSensorModeFlag = 0;
    }

    if (LightTriggerOperation == 1) { // Run Daytime only
        Y = Userlighttriplevel + 5; // Light sensor hysteresis value

        if (Lightlevel < Userlighttriplevel) {
            LightSensorModeFlag = 1;
        }

        if (Lightlevel > Y) {
            LightSensorModeFlag = 0;
        }
    }

    if (LightTriggerOperation == 2) { // Run Night time only
        Y = Userlighttriplevel - 5; // Light sensor hysteresis value

        if (Lightlevel < Userlighttriplevel) { // Its night timer so run the laser
            LightSensorModeFlag = 0;
        }

        if (Lightlevel > Y) { // Its day time so turn the laser off
            LightSensorModeFlag = 1;
        }
    }
}
void DoHousekeeping() {

    CheckBluetooth();
    ReadAccelerometer();
    DecodeAccelerometer();

    if (Tick > 4) {
        TransmitData();
        PrintAppData();
        GetLaserTemperature();

        if (Z_accelflag == 0) {
            ThrottleLaser();
        }
        Tick = 0;
    }

    if (Absy >= Tiltsensorignore) {
        if (Tiltstop == 1) {
            // stopTimer3();
            TCCR3B &= ~((1 << CS32) | (1 << CS30));
        }
    }

    if (SystemFaultFlag == 1) {
        SetLaserVoltage(0);
        // stopTimer1();
        TCCR1B &= ~((1 << CS12) | (1 << CS10));
        Steppingstatus = 0;
        ProcessError();
        return;
    }

    if (SetupModeFlag == 1) {
        WarnLaserOn();
        StartLaserFlickerInProgMode();
    }

    if (SetupModeFlag == 0) {
        Ishome = 0;

        WaitAfterPowerUp();

        if (Cmdlaseronflag == 1 && MapTotalPoints >= 2) {
            WarnLaserOn();
            SetLaserVoltage(LaserPower);
        } else {
            SetLaserVoltage(0);
            GetLightLevel();
        }
    }

    if (Tod_tick > NIGHT_TRIP_TIME_FROM_STARTUP) {
        if (Bt_ConnectedFlag == 0 && LightSensorModeFlag == 1) {
            MrSleepyTime();
        }
    }
}
void MrSleepyTime() {
    uint8_t X, Y;

    Y = 0;
    X = 0;
    LoopCount = 0;

    for (X = 1; X <= 4; X++) {
        BUZZER = On;
        Sleep(50);
        BUZZER = Off;
        Sleep(1000);
    }

    BUZZER = On;
    Sleep(1000);
    BUZZER = Off;

    SetLaserVoltage(0);
    Fan = Off;

    // stopTimer1();
    TCCR1B &= ~((1 << CS12) | (1 << CS10));
    Steppingstatus = 0;
    X_enablepin = On;
    Y_enablepin = On;

    while (LightSensorModeFlag == 1) {
        GetLightLevel();
        if (LightSensorModeFlag == 0) {
            Sleep(5000);
            GetLightLevel();
        }

        if (Tick > 10) {
            BUZZER = On;
            Sleep(25);
            BUZZER = Off;
            Tick = 0;
        }
    }

    // stopTimer3();
    TCCR3B &= ~((1 << CS32) | (1 << CS30));
    Sleep(5000);
}
void HomeMotor(uint8_t axis, int steps) {
    MoveMotor(axis, steps, 0);

    if (axis == 0) {
        while (Panstop == Off) {
            // do nothing while motor moves
        }
    } else {
        while (Tiltstop == Off) {
            // do nothing while motor moves
        }
    }

    // StopTimer1();
    TCCR1B &= ~((1 << CS12) | (1 << CS10));
    Stepcount = 0;
    Steppingstatus = 0;

    if (axis == 0) {
        X = 0;
        Absx = 0;
    } else {
        Y = 0;
        Absy = 0;
    }
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
    if (Panstop == On) {
        MoveMotor(0, -300, 1);
    }
    HomeMotor(0, 17000);
    // *********TILT AXIS HOME****************
    if (Tiltstop == On) {
        MoveMotor(1, 300, 1);
    }
    HomeMotor(1, -5000);
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
    Cmdlaseronflag = 0;
    Ishome = 1;
}
void ResetTiming() {
    if (SetupModeFlag == 1) {
        Secondsfromreset = 0;
    }

    if (SetupModeFlag == 0) {
        if (Secondsfromreset > ResetSeconds) {
            Secondsfromreset = 0;
            Bt_ConnectedFlag = 0;
            HomeAxis();
        }
    }
}
void Calibratelightsensor() {
    unsigned int Hexresult;
    char Result[4];

    SetLaserVoltage(0);
    GetLightLevel();
    Hexresult = Lightlevel; // Send current light level reading
    sprintf(Result, "%x", Hexresult);
    printf("<25:%s>", Result);
    Sleep(1000);
}
void ThrottleLaser() {
    float Result;
    float L_power;
    uint8_t B_volt;
    uint8_t Y;
    uint8_t X;

    B_volt = 0;

    if (BatteryVoltage < 98) B_volt = 25;
    else if (BatteryVoltage < 100) B_volt = 50;
    else if (BatteryVoltage < 105) B_volt = 70;
    else if (BatteryVoltage < 110) B_volt = 90;
    else B_volt = 100;

    if (Laser2OperateFlag == 1) {
        Y = Laser2BattTrip + 5;
        if (BatteryVoltage < Laser2BattTrip) Laser2BattTripFlag = 1;
        if (BatteryVoltage > Y) Laser2BattTripFlag = 0;

        X = Laser2TempTrip - 2;
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
void ProcessCoordinates() {
    Stepoverratio = 0;

    Dx = X - Absx; // Distance to move
    Dy = Y - Absy; // Distance to move

    if (Dx > 0) {
        X_dir = 1; // Set motor direction
    } else {
        X_dir = 0; // Set motor direction
        Dx = Dx * -1; // Convert all distances to a positive number
    }

    if (Dy > 0) {
        Y_dir = 1;
    } else {
        Y_dir = 0;
        Dy = Dy * -1;
    }

    if (Dx > Dy) {
        Master_dir = 1; // leading motor the moves the most X
        Stepoverratio = Dx / Dy; // Slope ratio between X and Y
        Stepcount = Dx;
    } else {
        Master_dir = 0;
        Stepoverratio = Dy / Dx;
        Stepcount = Dy;
    }
}
void MoveLaserMotor() {
    ProcessCoordinates(); // Drive motors to the coordinates
    Dss_preload = HOMING_SPEED; // Set speed rate
    Steppingstatus = 1;
    // StartTimer1();
    TCCR1B |= (1 << CS12) | (1 << CS10);

    while (Steppingstatus == 1) {
        // do nothing while motor moves
    }

    // StopTimer1(); // Stop the motor from stepping as sensor has been triggered
    TCCR1B &= ~((1 << CS12) | (1 << CS10));
    Stepcount = 0; // Clear the step count
}
void clearSerial() {
    while (UCSR0A & (1 << RXC0)) {
        (void)UDR0;
    }
}
void Audio(uint8_t pattern) {
    uint16_t waitTime1;
    uint16_t waitTime2;
    uint8_t repeatCount;
    uint8_t i;

    switch (pattern) {
        case 1: waitTime1 = 100; repeatCount = 1; break;
        case 2: waitTime1 = 1; waitTime2 = 100; repeatCount = 2; break;
        case 3: waitTime1 = 500; waitTime2 = 100; repeatCount = 2; break;
        case 4: waitTime1 = 50; waitTime2 = 150; repeatCount = 2; break;
        case 5: waitTime1 = 50; waitTime2 = 50; repeatCount = 2; break;
    }

    for (i = 0; i < repeatCount; i++) {
        BUZZER = On;
        _delay_ms(waitTime1);
        BUZZER = Off;
        _delay_ms(waitTime2);
    }
}
void ProgrammingMode() {
    uint16_t Temp = 1;
    SetLaserVoltage(0); // Turn off laser
    HomeAxis();

    printf("<09:%X>", Temp); // Tell the App that the micro is ready for programming
    _delay_ms(1000);
}
void OperationModeSetup() {
    clearSerial()
    _delay_ms(2000); // Wait for 2 seconds
    switch (OperationMode) {
        case 0:
            printf("AT+NAMEField-i %s", LaserID);
            Y_maxcount = 1800;
            Opmodetxt = "0:Field-i";
            break;
        case 1:
            printf("AT+NAMERoof-i %s", LaserID);
            Y_maxcount = 4095;
            Opmodetxt = "1:Roof-i";
            break;
        case 2:
            printf("AT+NAMEOrchard-i %s", LaserID);
            Y_maxcount = 2000;
            Opmodetxt = "2:Orchard-i";
            break;
        case 3:
            printf("AT+NAMEEve-i %s", LaserID);
            Y_maxcount = 4095;
            Opmodetxt = "3:Eve-i";
            break;
        case 4:
            printf("AT+NAMETest_i %s", LaserID);
            Y_maxcount = 2500;
            Opmodetxt = "4:Test-i";
            break;
    }
}
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
void getMapPtCounts() {
    uint8_t MapIndex;
    uint8_t I, Prev_i, Prev_i_1;
    I = 0;
    Prev_i = 0;
    for (MapIndex = 1; MapIndex <= MapTotalPoints; MapIndex++) {
        I = GetZone(MapIndex);
        if (I > 0) {
            if (I != Prev_i) {
                if (Prev_i == 1) {
                    mapCount[1][Prev_i] = MapIndex;
                    mapCount[2][Prev_i] = MapIndex - 1;
                } else {
                    mapCount[2][Prev_i] = MapIndex - 1;
                    Prev_i_1 = Prev_i - 1;
                    mapCount[1][Prev_i] = MapIndex - mapCount[2][Prev_i_1];
                    mapCount[1][Prev_i] = mapCount[1][Prev_i] + 1;
                }
            }
            if (MapIndex == MapTotalPoints) {
                if (I > 1) {
                    Prev_i = I - 1;
                }
                mapCount[2][I] = MapIndex;
                if (I == 1) {
                    mapCount[1][I] = MapIndex + 1;
                } else {
                    mapCount[1][I] = MapIndex - mapCount[2][Prev_i];
                    mapCount[1][I] = mapCount[1][I] + 1;
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
    int temp;

    if (MapTotalPoints == 0) {
        return;
    }

    for (MapIndex = 0; MapIndex < Mapcount[0][zn] - 1; MapIndex++) {
        if (zn == 0) {
            MI = MapIndex;
        } else {
            MI = MapIndex + Mapcount[2][zn - 1];
        }
        Vertices[0][MapIndex] = eeprom_read_word(&EramPositions[MI].EramX);
        Vertices[1][MapIndex] = eeprom_read_word(&EramPositions[MI].EramY) & 0x0FFF;
    }
    // Add a repeated vertex equal to the first vertex.  MapIndex has incremented by the "next MapIndex" statement - I think?
    Vertices[0][MapIndex] = Vertices[0][0];
    Vertices[1][MapIndex] = Vertices[1][0];
    // Get the slope of the segment, stored in Vertices[2][i]
    for (MapIndex = 1; MapIndex < Mapcount[0][zn]; MapIndex++) {
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
   a = atan(LASER_HT/(float)rho;);
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
   thisres[0] = r*cos(p/STEPS_PER_RAD;);
   thisres[1] = r*sin(p/STEPS_PER_RAD);
}
void getPolars(int c1, int c2,int thisRes[2]) {  //Take cartesian coordinates as input
   long r = c1 * c1 + c2 * c2;  //Get the distance from the laser to the point by pythagoras.
   r = sqrt(r);  
   double temp = atan2(double(c2) , double(c1)) * STEPS_PER_RAD;  //Pan is atan(c2/c1)
   thisRes[0] = (int)temp; //pan
   thisRes[1] = getTiltFromCart(int(r)); //tilt.
}
void GetPerimeter(uint8_t zn) {
    uint8_t I, J = 0, K, L, M;
    float Scale = 50.0;
    int nextTilt;
    uint8_t testBit;
    uint8_t dirn;

    for (I = 1; I < Mapcount[1][zn] - 1; I++) {
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
void initMPU6050() {
    i2c_init(); // initialize I2C library

    // turn on the internal temp reading register
    i2c_start(MPU6050_W);
    i2c_write(0x6B);
    i2c_write(0x01);
    i2c_stop();
    _delay_ms(100);

    // reset signal accel path
    i2c_start(MPU6050_W);
    i2c_write(0x68);
    i2c_write(0x03);
    i2c_stop();
    _delay_ms(100);

    // Set the slowest sampling rate
    i2c_start(MPU6050_W);
    i2c_write(0x19);
    i2c_write(0xFF);
    i2c_stop();

    // Set full scale reading to 16g's
    i2c_start(MPU6050_W);
    i2c_write(0x1C);
    i2c_write(0x18);
    i2c_stop();

    // Read the WHO_AM_I register
    i2c_start(MPU6050_W);
    i2c_write(WHO_AM_I_MPU6050);
    i2c_stop();

    i2c_start(MPU6050_R);
    printf("Accel Chip &H%d\n", i2c_readNak());
    i2c_stop();

    printf("Gyro Flag: %d\n", GyroAddress);
}
void initDACMCP4725() {
    TWBR = 72; // Set bit rate register (prescaler for TWI clock)
    // Start TWI
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    // Send MCP4725 address
    TWDR = MCP4725;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    // Send data
    TWDR = 0x0001; // Set the default power to 0 volts in the EEPROM volt on startup
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    // Stop TWI
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}
void stepperDriverISR() {
    if (Stepcount > 0) {
        Stepcount--;

        if (Dx > Dy) {
            Stepoverratio += Dy;
            if (Stepoverratio >= Dx) {
                Stepoverratio -= Dx;
                Remainder = 0;
            } else {
                Remainder = 1;
            }
        } else {
            Stepoverratio += Dx;
            if (Stepoverratio >= Dy) {
                Stepoverratio -= Dy;
                Remainder = 0;
            } else {
                Remainder = 1;
            }
        }

        Ocr1a = Dss_preload;

        if (SetupModeFlag == 0) {
            if (Stepcount < START_RAMPING_COUNT) {
                if (Ocr1a < 65525) {
                    Ocr1a += RAMPING_STEPS;
                } else {
                    if (Ocr1a > Dss_preload) {
                        Ocr1a -= RAMPING_STEPS;
                    }
                }
            }
        }

        if (Remainder < 1) {
            X_step = 1;
            Y_step = 1;
            usleep(1);
            X_step = 0;
            Y_step = 0;
            usleep(1);
            if (X_dir == 0) Absx--;
            if (X_dir == 1) Absx++;
            if (Y_dir == 1) Absy++;
            if (Y_dir == 0) Absy--;
        } else {
            if (Master_dir == 1) {
                X_step = 1;
                usleep(1);
                X_step = 0;
                usleep(1);
                if (X_dir == 0) Absx--;
                if (X_dir == 1) Absx++;
            } else {
                Y_step = 1;
                usleep(1);
                Y_step = 0;
                usleep(1);
                if (Y_dir == 1) Absy++;
                if (Y_dir == 0) Absy--;
            }
        }
    } else {
        // Stop Timer1
        Steppingstatus = 0;
    }
}
void TickCounter_50ms_isr() {
    // Your other code...
    Counter50ms++;

    if (SetupModeFlag == 1 && Ishome == 1 && WarnLaserOnOnce == 0) {
        Startbuzzerinprogmode();
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
        Batterytick++;
        Tod_tick++;
        Tick++;
        AccelTick++;
        Flashthelaserflag ^= 1;
        Secondsfromreset++;
        Counter50ms = 0;
    }

    wdt_reset() //Reset the watchdog timer.
}
void setupTimer1() {
    // Set up Timer1 with a prescaler of 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // Set the compare value to desired value
    OCR1A = YOUR_DESIRED_VALUE;
    // Enable the compare match interrupt
    TIMSK1 |= (1 << OCIE1A);
    // Start Timer1
    // TCCR1B |= (1 << CS10);  //Already included above.
}
void setupTimer3() {
    // Set up Timer3 with a prescaler of 1024
    TCCR3B |= (1 << CS32) | (1 << CS30);
    // Set the compare value to 781 - 1
    OCR3A = 780; //781 - 1 in the BASCOM code.
    // Enable the compare match interrupt
    TIMSK3 |= (1 << OCIE3A);
}
void setup() {
    // Other setup code...
    setupTimer1();
    setupTimer3();
    setupPeripherals();
    setupWatchdog();
    Wd_byte = MCUSR; // Read the Watchdog flag
    if (Wd_byte & (1 << WDRF)) { // there was a WD overflow. This flag is cleared of a Watchdog Config
        Wd_flag = 1; // store the flag
    }
    initDACMCP4725();  //
    initVars();  //Initialise some variables every time the unit starts
    ReadEramVars();  //Reads user data from EEPROM to RAM.
    //LoadEramDefaults();  This only to be done if the unit hasn't been used before.
    //Needs to be called conditionally somewhere.
}
void RunSweep(uint8_t zn) {
    uint8_t Nbrpts, PatType, i;
    int last[2],nxt[2]; nbrMidPts;

    LoadZoneMap(zn);
    GetPerimeter(zn);
    Nbrpts = 30;
    for (PatType = 1; PatType <= 2; PatType++) {
        NbrPts = Nbrpts + NbrPerimeterPts;
            for (uint8_t Index = 1; Index <= Nbrpts; Index++) {
            // Store present point to use in interpolation
            last[0] = X;
            last[1] = Y;
            getXY(Index, PatType, zn);
            nxt[0] = X;
            nxt[1] = Y;

            CalcSpeedZone();
            Dss_preload = CalcSpeed();

            if (Index == 1) {
                Cmdlaseronflag = 0;
                Dss_preload = STEP_RATE_MAX;
            }

            if (Index == 2) {
                Cmdlaseronflag = 1;
            }

            nbrMidPts = XlastTarget - XNextTarget;
            nbrMidPts = abs(nbrMidPts);
            nbrMidPts = nbrMidPts / MID_PT_SEPARATION;
            for (i = 0; i <= nbrMidPts; i++) {
                if (nbrMidPts > 0) {
                    if (i == nbrMidPts) {
                        X = nxt[0];
                        Y = nxt[1];
                    } else {
                        CartesianInterpolate(last,nxt,i, nbrMidPts, res);
                        getPolars(res[1], res[2], res[]);  //Pass res as input 
                        X = res[1];
                        Y = res[2];
                    }
                }
                ProcessCoordinates();

                Steppingstatus = 1;
                // startTimer1();
                TCCR1B |= (1 << CS12) | (1 << CS10);

                while (Steppingstatus == 1) {
                    DoHousekeeping();
                    if (SetupModeFlag == 1) {
                        // stopTimer1();
                        TCCR1B &= ~((1 << CS12) | (1 << CS10));
                        Stepcount = 0;
                        Steppingstatus = 0;
                        return;
                    }
                }
            }
        }
    }
    ResetTiming();
}
int main() {
    // config();  //This is not defined.
    setup();
    while(1) {
        CheckBluetooth();
        if (SetupModeFlag == 1) {
            JogMotors();
        }
        if (SetupModeFlag == 0) {
            getMapPtCounts();
            for (Zn = 1; Zn <= NBR_ZONES; Zn++) {
                if (MapCount[1][Zn] > 0) {
                    RunSweep(Zn);
                }
            }
        }
        DoHousekeeping();
        if (SetupModeFlag == 2) {
            Calibratelightsensor();
        }
        return 0;
    } 
    return 0;
}
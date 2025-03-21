// LoadDefaultsModule.c
#include "shared_Vars.h"
#include "LoadDefs.h"
#include "const.h"
#include <avr/eeprom.h>
#include <stdio.h>

uint16_t EEMEM EramResetSeconds; // NOLINT
uint8_t EEMEM EramFirstTimeOn;   // NOLINT
uint8_t NoMapsRunningFlag;
uint8_t FirstTimeOn;
// uint8_t Tick;
uint8_t GetZVal(uint8_t i);
int GetXVal(uint8_t i);
int GetYVal(uint8_t i);

void LoadEramDefaults(void)
{ // Load default values to EEPROM (only run if there is not already user data stored in EEPROM)
    uartPrint("In LoadEramDefaults()");
    eeprom_update_byte(&EramUserLaserPower, DEF_USER_LASER_POWER);
    eeprom_update_byte(&EramMaxLaserPower, DEF_MAX_LASER_POWER);
    eeprom_update_byte(&EramLaser2TempTrip, DEF_LASER_2_TEMP_TRIP);
    eeprom_update_byte(&EramLaser2BattTrip, DEF_LASER_2_BATT_TRIP);
    eeprom_update_byte(&EramLaser2OperateFlag, DEF_LASER_2_OPERATE_FLAG);
    // Map Settings
    eeprom_update_byte(&EramMapTotalPoints, DEF_MAP_TOTAL_PTS);

    eeprom_update_byte(&EramActiveMapZones, DEF_ACTIVE_MAP_ZONES);
    eeprom_update_byte(&EramActivePatterns, DEF_ACTIVE_PATTERNS);
    eeprom_update_word(&EramLaserID, DEF_LASER_ID);
    eeprom_update_word(&EramAccelTripPoint, DEF_ACCEL_TRIP_POINT);
    eeprom_update_word(&EramResetSeconds, DEF_RESET_SECONDS); // NOLINT
    eeprom_update_byte(&EramOperationMode, DEF_OPERATION_MODE);
    eeprom_update_byte(&EramFirstTimeOn, DEF_FIRST_TIME_ON);
    eeprom_update_byte(&EramUserLightTripLevel, DEF_USER_LIGHT_TRIP_LEVEL);
    eeprom_update_byte(&EramFactoryLightTripLevel, DEF_FACTORY_LIGHT_TRIP_LEVEL);
    eeprom_update_byte(&EramLightTriggerOperation, DEF_LIGHT_TRIGGER_OPERATION);
    eeprom_update_byte(&EramGyroAddress, MPU6050_ADDRESS);
    eeprom_update_byte(&Eram_Tilt_Sep, DEF_TILT_SEP);
    eeprom_update_word(&Eram_Step_Rate_Min, DEF_STEP_RATE_MIN);
    eeprom_update_word(&Eram_Step_Rate_Max, DEF_STEP_RATE_MAX);
    eeprom_update_byte(&EramSpeedScale, DEF_SPEEDSCALE);
}

void ReadEramVars(void)
{ // Transfer EEPROM user data to RAM
    // If memory is a problem, these RAM variables could be deleted and the EEPROM variables used instead.
    UserLaserPower = eeprom_read_byte(&EramUserLaserPower);
    MaxLaserPower = eeprom_read_byte(&EramMaxLaserPower);
    Laser2TempTrip = eeprom_read_byte(&EramLaser2TempTrip);
    Laser2BattTrip = eeprom_read_byte(&EramLaser2BattTrip);
    Laser2OperateFlag = eeprom_read_byte(&EramLaser2OperateFlag);
    MapTotalPoints = eeprom_read_byte(&EramMapTotalPoints);
// uartPrint(F("MapTotalPoints address: "));
// uartPrint((void*)&EramMapTotalPoints);
// uartPrint(F(" value: "));
// uartPrint(MapTotalPoints);
// uartPrint("\n");
#ifdef DEBUG
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "MapTotalPoints address:  %p value: %d", (void *)&EramMapTotalPoints, MapTotalPoints);
    uartPrint(debugMsg);
#endif
    GyroAddress = eeprom_read_byte(&EramGyroAddress);
#ifdef DEBUG
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Gyro address:  %p value: %02x", (void *)&EramGyroAddress, GyroAddress);
    uartPrint(debugMsg);
#endif
    // for(int i = 0; i < 5; i++) {
    //     SpeedZone[i] = eeprom_read_byte(&EramSpeedZone[i]);
    // }
    SpeedScale = eeprom_read_byte(&EramSpeedScale);
    LaserHt = eeprom_read_byte(&EramLaserHt);
    Tilt_Sep = eeprom_read_byte(&Eram_Tilt_Sep);
    ActiveMapZones = eeprom_read_byte(&EramActiveMapZones);
    ActivePatterns = eeprom_read_byte(&EramActivePatterns);
    LaserID = eeprom_read_word(&EramLaserID);
    AccelTripPoint = eeprom_read_word(&EramAccelTripPoint);
    ResetSeconds = eeprom_read_word(&EramResetSeconds); // NOLINT
    OperationMode = eeprom_read_byte(&EramOperationMode);
    FirstTimeOn = eeprom_read_byte(&EramFirstTimeOn);
    UserLightTripLevel = eeprom_read_byte(&EramUserLightTripLevel);
    FactoryLightTripLevel = eeprom_read_byte(&EramFactoryLightTripLevel);
    LightTriggerOperation = eeprom_read_byte(&EramLightTriggerOperation);
    Step_Rate_Min = eeprom_read_word(&Eram_Step_Rate_Min);
    Step_Rate_Max = eeprom_read_word(&Eram_Step_Rate_Max);
    SpeedScale = eeprom_read_byte(&EramSpeedScale);
}

void PrintEramVars()
{
#ifndef INCLUDE_PRINT_EEPROM
    uartPrint("PrintEramVars() not incl");
#endif
#ifdef INCLUDE_PRINT_EEPROM
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "UserLaserPower:  %p, %d", (void *)&EramUserLaserPower, eeprom_read_byte(&EramUserLaserPower));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "MaxLaserPower:  %p, %d", (void *)&EramMaxLaserPower, eeprom_read_byte(&EramMaxLaserPower));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Laser2TempTrip:  %p, %d", (void *)&EramLaser2TempTrip, eeprom_read_byte(&EramLaser2TempTrip));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Laser2BattTrip:  %p, %d", (void *)&EramLaser2BattTrip, eeprom_read_byte(&EramLaser2BattTrip));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Laser2OperateFlag:  %p, %d", (void *)&EramLaser2OperateFlag, eeprom_read_byte(&EramLaser2OperateFlag));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "MapTotalPoints:  %p, %d", (void *)&EramMapTotalPoints, eeprom_read_byte(&EramMapTotalPoints));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Gyro address: %p, %02x", (void *)&EramGyroAddress, eeprom_read_byte(&EramGyroAddress));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "First vertex address: %p", (void *)&EramPositions);
    uartPrint(debugMsg);
    MapTotalPoints = eeprom_read_byte(&EramMapTotalPoints);
    for (uint8_t i = 0; i < MapTotalPoints; i++)
    {
        // sprintf("Xi add: %p", (void *)(&EramPositions[i].EramX));
        // uartPrint(debugMsg);
        snprintf(debugMsg, DEBUG_MSG_LENGTH, "WPi: %u Zone: %d X: %d, Y: %d", i, GetZVal(i), GetXVal(i), GetYVal(i));
        uartPrint(debugMsg);
    }
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "ActiveMapZones:  %p, : %d", (void *)&EramActiveMapZones, eeprom_read_byte(&EramActiveMapZones));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "ActivePatterns:  %p, : %d", (void *)&EramActivePatterns, eeprom_read_byte(&EramActivePatterns));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "LaserID:  %p, : %d", (void *)&EramLaserID, eeprom_read_word(&EramLaserID));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "AccelTrip: %p, %d", (void *)&EramAccelTripPoint, eeprom_read_word(&EramAccelTripPoint));
    uartPrint(debugMsg);

    // ResetSeconds = eeprom_read_word(&EramResetSeconds);  //NOLINT
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "OperationMode: %p, %d", (void *)&EramOperationMode, eeprom_read_byte(&EramOperationMode));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "FirstTimeOn: %p, %d", (void *)&EramFirstTimeOn, eeprom_read_byte(&EramFirstTimeOn));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "UserLightTripLevel: %p, %d", (void *)&EramUserLightTripLevel, eeprom_read_byte(&EramUserLightTripLevel));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "FactoryLightTripLevel: %p, %d", (void *)&EramFactoryLightTripLevel, eeprom_read_byte(&EramFactoryLightTripLevel));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "LightTriggerOperation: %p, %d", (void *)&EramLightTriggerOperation, eeprom_read_byte(&EramLightTriggerOperation));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Step_Rate_Min: %p, %d", (void *)&Eram_Step_Rate_Min, eeprom_read_word(&Eram_Step_Rate_Min));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "Step_Rate_Max: %p, %d", (void *)&Eram_Step_Rate_Max, eeprom_read_word(&Eram_Step_Rate_Max));
    uartPrint(debugMsg);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "SpeedScale: %p, %d", (void *)&EramSpeedScale, eeprom_read_byte(&EramSpeedScale));
    uartPrint(debugMsg);

#endif
}

// Function to get the X value
int16_t GetXVal(uint8_t i)
{
    return (int)eeprom_read_word(&EramPositions[i].EramX);
}

// Function to get the Y value
int16_t GetYVal(uint8_t i)
{
    uint16_t rawY = eeprom_read_word(&EramPositions[i].EramY);
    int yValue = rawY & 0x0FFF; // Extract the lower 12 bits
    // Convert from 2's complement if necessary
    if (yValue & 0x0800) // Check if the sign bit (bit 11) is set
    {
        yValue |= 0xF000; // Set the upper 4 bits to maintain the sign
    }
    return yValue;
}

// Function to get the Z value
uint8_t GetZVal(uint8_t i)
{
    uint16_t rawY = eeprom_read_word(&EramPositions[i].EramY);
    return (rawY >> 12) & 0x0F; // Extract the upper 4 bits
}
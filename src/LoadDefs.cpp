// LoadDefaultsModule.c
#include "shared_Vars.h"
#include "LoadDefs.h"
#include "const.h"
#include <avr/eeprom.h>
#include <stdio.h>

uint16_t EEMEM EramResetSeconds;  //NOLINT
uint8_t EEMEM EramFirstTimeOn;  //NOLINT
uint8_t NoMapsRunningFlag;
uint8_t FirstTimeOn;
// uint8_t Tick;

void LoadEramDefaults(void) {  // Load default values to EEPROM (only run if there is not already user data stored in EEPROM)
    eeprom_update_byte(&EramUserLaserPower, DEF_USER_LASER_POWER);
    eeprom_update_byte(&EramMaxLaserPower, DEF_MAX_LASER_POWER);
    eeprom_update_byte(&EramLaser2TempTrip, DEF_LASER_2_TEMP_TRIP);
    eeprom_update_byte(&EramLaser2BattTrip, DEF_LASER_2_BATT_TRIP);
    eeprom_update_byte(&EramLaser2OperateFlag, DEF_LASER_2_OPERATE_FLAG);
    // Map Settings
    eeprom_update_byte(&EramMapTotalPoints, DEF_MAP_TOTAL_PTS);
    for(int i = 0; i < 5; i++) {
        eeprom_update_byte(&EramSpeedZone[i], DEF_SPEED);
    }
    
    eeprom_update_byte(&EramActiveMapZones, DEF_ACTIVE_MAP_ZONES);
    eeprom_update_word(&EramLaserID, DEF_LASER_ID);
    eeprom_update_word(&EramAccelTripPoint, DEF_ACCEL_TRIP_POINT);
    eeprom_update_word(&EramResetSeconds, DEF_RESET_SECONDS);  //NOLINT
    eeprom_update_byte(&EramOperationMode, DEF_OPERATION_MODE);
    eeprom_update_byte(&EramFirstTimeOn, DEF_FIRST_TIME_ON);
    eeprom_update_byte(&EramUserLightTripLevel, DEF_USER_LIGHT_TRIP_LEVEL);
    eeprom_update_byte(&EramFactoryLightTripLevel, DEF_FACTORY_LIGHT_TRIP_LEVEL);
    eeprom_update_byte(&EramLightTriggerOperation, DEF_LIGHT_TRIGGER_OPERATION);

    // // Write a few EEPROM addresses to serial for testing.
    // sprintf(debugMsg, "EramUserLaserPower: %04x", (uint16_t)&EramUserLaserPower); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramMaxLaserPower: %04x", (uint16_t)&EramMaxLaserPower); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramActiveMapZones: %04x", (uint16_t)&EramActiveMapZones); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramFirstTimeOn: %04x", (uint16_t)&EramFirstTimeOn); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramLightTriggerOperation: %04x", (uint16_t)&EramLightTriggerOperation); uartPrint(debugMsg);
    // // Read back a few values written to EEPROM
    // sprintf(debugMsg, "EramUserLaserPower: %d", eeprom_read_byte(&EramUserLaserPower)); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramMaxLaserPower: %d", eeprom_read_byte(&EramMaxLaserPower)); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramActiveMapZones: %d", eeprom_read_byte(&EramActiveMapZones)); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramFirstTimeOn: %d", eeprom_read_byte(&EramFirstTimeOn)); uartPrint(debugMsg);
    // sprintf(debugMsg, "EramLightTriggerOperation: %d", eeprom_read_byte(&EramLightTriggerOperation)); uartPrint(debugMsg);
}

void ReadEramVars(void){                // Transfer EEPROM user data to RAM
    //If memory is a problem, these RAM variables could be deleted and the EEPROM variables used instead.
    UserLaserPower = eeprom_read_byte(&EramUserLaserPower);
    MaxLaserPower = eeprom_read_byte(&EramMaxLaserPower);
    Laser2TempTrip = eeprom_read_byte(&EramLaser2TempTrip);
    Laser2BattTrip = eeprom_read_byte(&EramLaser2BattTrip);
    Laser2OperateFlag = eeprom_read_byte(&EramLaser2OperateFlag);
    MapTotalPoints = eeprom_read_byte(&EramMapTotalPoints); 
    sprintf(debugMsg,"MapTotalPonts address:  %p",(void*)&EramMapTotalPoints);
    uartPrint(debugMsg);  
    sprintf(debugMsg,"MapTotalPonts: %d",MapTotalPoints);
    uartPrint(debugMsg);  
    for(int i = 0; i < 5; i++) {
        SpeedZone[i] = eeprom_read_byte(&EramSpeedZone[i]);
    }
    ActiveMapZones = eeprom_read_byte(&EramActiveMapZones);
    LaserID = eeprom_read_word(&EramLaserID);
    AccelTripPoint = eeprom_read_word(&EramAccelTripPoint);
    ResetSeconds = eeprom_read_word(&EramResetSeconds);  //NOLINT
    OperationMode = eeprom_read_byte(&EramOperationMode);
    FirstTimeOn = eeprom_read_byte(&EramFirstTimeOn);
    UserLightTripLevel = eeprom_read_byte(&EramUserLightTripLevel);
    FactoryLightTripLevel = eeprom_read_byte(&EramFactoryLightTripLevel);
    LightTriggerOperation = eeprom_read_byte(&EramLightTriggerOperation);
}
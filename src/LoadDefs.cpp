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
    uartPrint("In LoadEramDefaults()");
    eeprom_update_byte(&EramUserLaserPower, DEF_USER_LASER_POWER);
    eeprom_update_byte(&EramMaxLaserPower, DEF_MAX_LASER_POWER);
    eeprom_update_byte(&EramLaser2TempTrip, DEF_LASER_2_TEMP_TRIP);
    eeprom_update_byte(&EramLaser2BattTrip, DEF_LASER_2_BATT_TRIP);
    eeprom_update_byte(&EramLaser2OperateFlag, DEF_LASER_2_OPERATE_FLAG);
    // Map Settings
    eeprom_update_byte(&EramMapTotalPoints, DEF_MAP_TOTAL_PTS);
    // for(int i = 0; i < 5; i++) {
    //     eeprom_update_byte(&EramSpeedZone[i], DEF_SPEED);
    // }
    
    eeprom_update_byte(&EramActiveMapZones, DEF_ACTIVE_MAP_ZONES);
    eeprom_update_word(&EramLaserID, DEF_LASER_ID);
    eeprom_update_word(&EramAccelTripPoint, DEF_ACCEL_TRIP_POINT);
    eeprom_update_word(&EramResetSeconds, DEF_RESET_SECONDS);  //NOLINT
    eeprom_update_byte(&EramOperationMode, DEF_OPERATION_MODE);
    eeprom_update_byte(&EramFirstTimeOn, DEF_FIRST_TIME_ON);
    eeprom_update_byte(&EramUserLightTripLevel, DEF_USER_LIGHT_TRIP_LEVEL);
    eeprom_update_byte(&EramFactoryLightTripLevel, DEF_FACTORY_LIGHT_TRIP_LEVEL);
    eeprom_update_byte(&EramLightTriggerOperation, DEF_LIGHT_TRIGGER_OPERATION);
    eeprom_update_byte(&EramGyroAddress, MPU6050_ADDRESS);

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
    // uartPrint(F("MapTotalPoints address: "));
    // uartPrint((void*)&EramMapTotalPoints);
    // uartPrint(F(" value: "));
    // uartPrint(MapTotalPoints);
    // uartPrint("\n");
    #ifdef DEBUG
    sprintf(debugMsg,"MapTotalPoints address:  %p value: %d",(void*)&EramMapTotalPoints, MapTotalPoints);  
    uartPrint(debugMsg);  
    #endif
    GyroAddress = eeprom_read_byte(&EramGyroAddress); 
    #ifdef DEBUG
    sprintf(debugMsg,"Gyro address:  %p value: %02x",(void*)&EramGyroAddress, GyroAddress);  
    uartPrint(debugMsg);  
    #endif
    // for(int i = 0; i < 5; i++) {
    //     SpeedZone[i] = eeprom_read_byte(&EramSpeedZone[i]);
    // }
    SpeedScale = eeprom_read_byte(&EramSpeedScale);
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

void PrintEramVars(){
    sprintf(debugMsg,"UserLaserPower:  %p, %d",(void*)&EramUserLaserPower, eeprom_read_byte(&EramUserLaserPower));  
    uartPrint(debugMsg);  
    sprintf(debugMsg,"MaxLaserPower:  %p, %d",(void*)&EramMaxLaserPower, eeprom_read_byte(&EramMaxLaserPower));  
    uartPrint(debugMsg);  
    // Laser2TempTrip = eeprom_read_byte(&EramLaser2TempTrip);
    // Laser2BattTrip = eeprom_read_byte(&EramLaser2BattTrip);
    // Laser2OperateFlag = eeprom_read_byte(&EramLaser2OperateFlag);
    sprintf(debugMsg,"MapTotalPoints:  %p, %d",(void*)&EramMapTotalPoints, eeprom_read_byte(&EramMapTotalPoints));  
    uartPrint(debugMsg);  
    sprintf(debugMsg,"Gyro address:  %02x", (void*)&EramGyroAddress, eeprom_read_byte(&EramGyroAddress));  
    uartPrint(debugMsg);  
    MapTotalPoints = eeprom_read_byte(&EramMapTotalPoints);
    for (uint8_t i=0;i<MapTotalPoints;i++){
        sprintf(debugMsg,"WPi: %u x: %u y: %u",i, (int)eeprom_read_word(&EramPositions[i].EramX), (int)(eeprom_read_word(&EramPositions[i].EramY) & 0x0FFF));  
        uartPrint(debugMsg);  
    }
    // for(int i = 0; i < 5; i++) {
    //     SpeedZone[i] = eeprom_read_byte(&EramSpeedZone[i]);
    // }
    sprintf(debugMsg,"ActiveMapZones:  %p, : %d",(void*)&EramActiveMapZones, eeprom_read_byte(&EramActiveMapZones));  
    uartPrint(debugMsg);  

    // LaserID = eeprom_read_word(&EramLaserID);
    sprintf(debugMsg, "AccelTrip: %p, %d", (void*)&EramAccelTripPoint, eeprom_read_word(&EramAccelTripPoint));
    uartPrint(debugMsg);  

    // ResetSeconds = eeprom_read_word(&EramResetSeconds);  //NOLINT
    sprintf(debugMsg, "OperationMode: %p, %d", (void*)&EramOperationMode, eeprom_read_byte(&EramOperationMode));
    uartPrint(debugMsg);  
    sprintf(debugMsg, "FirstTimeOn: %p, %d", (void*)&EramFirstTimeOn, eeprom_read_byte(&EramFirstTimeOn));
    uartPrint(debugMsg);  
    // UserLightTripLevel = eeprom_read_byte(&EramUserLightTripLevel);
    // FactoryLightTripLevel = eeprom_read_byte(&EramFactoryLightTripLevel);
    // LightTriggerOperation = eeprom_read_byte(&EramLightTriggerOperation);
}

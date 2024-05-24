// LoadDefaultsModule.c
#include "shared_vars.h"
#include "LoadDefs.h"

void LoadEramDefaults(void) {  // Load default values to EEPROM (only run if there is not already user data stored in EEPROM)
    unsigned char LoopCount;

    EramUserlaserpower = DEF_USER_LASER_POWER;
    EramMaxlaserpower = DEF_MAX_LASER_POWER;
    EramLaser2TempTrip = DEF_LASER_2_TEMP_TRIP
    EramLaser2BattTrip = DEF_LASER_2_BATT_TRIP
    EramLaser2OperateFlag = DEF_LASER_2_OPERATE_FLAG
    // Map Settings
    EramMapTotalPoints = DEF_MAP_TOTAL_PTS;
    for(int i = 0; i < 5; i++) {
        EramSpeedZone[i] = DEF_SPEED;
    }
    EramActiveMapZones = DEF_ACTIVE_MAP_ZONES;
    EramLaserID = DEF_LASER_ID;
    EramAccelTripPoint = DEF_ACCEL_TRIP_POINT
    EramResetSeconds = DEF_RESET_SECONDS
    EramOperationMode = DEF_OPERATION_MODE;
    EramFirstTimeOn = DEF_FIRST_TIME_ON
    EramUserLightTripLevel = DEF_USER_LIGHT_TRIP_LEVEL
    EramFactoryLightTripLevel = DEF_FACTORY_LIGHT_TRIP_LEVEL
    EramLightTriggerOperation = DEF_LIGHT_TRIGGER_OPERATION
}

void ReadEramVars(void){                // Transfer EEPROM user data to RAM
    //If memory is a problem, these RAM variables could be deleted and the EEPROM variables used instead.
    UserLaserPower = EramUserLaserPower
    MaxLaserPower = EramMaxLaserPower
    Laser2TempTrip = EramLaser2TempTrip
    Laser2BattTrip = EramLaser2BattTrip
    Laser2OperateFlag = EramLaser2OperateFlag
    MapTotalPoints = EramMapTotalPoints   
    for(int i = 0; i < 5; i++) {
        SpeedZone[i] = EramSpeedZone[i];
    }
    ActiveMapZones = EramActiveMapZones
    LaserID = EramLaserID
    AccelTripPoint = EramAccelTripPoint 
    ResetSeconds = EramResetSeconds
    OperationMode = EramOperationMode
    FirstTimeOn = EramFirstTimeOn
    UserLightTripLevel = EramUserLightTripLevel
    FactoryLightTripLevel = EramFactoryLightTripLevel
    LightTriggerOperation = EramLightTriggerOperation
}

void initVars(void){            // Set some initial values.
    SetupModeFlag = 0;
    NoMapsRunningFlag = 0;
    OperationModeSetup();                               // Select the operation mode the device is going to work under before loading data presets
    // AppCompatibilityNo = 1;                          //20240522.  This doesn't appear to do anything
    // LoadPositionMap();                                  //Transfer map data from EEPROM to Sram
    // LoadActiveMapZones();
    VoltPerStep = LINE_VOLTAGE / 4095;                   //Input voltage ie 5 volts /12bit (4095) MCP4725 DAC = Voltage step per or 0.0012210012210012 Volt per step
    SendDataFlag = 0;                                   //Setup to print diagnostic data from new program
    SendSetupDataFlag = 0;                              // 20240522. Not used.
    LightSensorModeFlag = 0;
    GyroAddress = 1;
}
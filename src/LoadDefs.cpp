// LoadDefaultsModule.c
#include "shared_Vars.h"
#include "LoadDefs.h"
#include "const.h"
#include <avr/eeprom.h>
#include <stdio.h>
#include <avr/pgmspace.h>
// #include <EEPROM.h>

uint16_t EEMEM EramResetSeconds; // NOLINT
uint8_t EEMEM EramFirstTimeOn;   // NOLINT
uint8_t NoMapsRunningFlag;
uint8_t FirstTimeOn;
// uint8_t Tick;
uint8_t GetZVal(uint8_t i);
int GetXVal(uint8_t i);
int GetYVal(uint8_t i);

// void LoadEramDefaults(void)
// { // Load default values to EEPROM (only run if there is not already user data stored in EEPROM)
//     // uartPrint("In LoadEramDefaults()");
//     eeprom_update_byte(&EramUserLaserPower, DEF_USER_LASER_POWER);
//     eeprom_update_byte(&EramMaxLaserPower, DEF_MAX_LASER_POWER);
//     eeprom_update_byte(&EramLaser2TempTrip, DEF_LASER_2_TEMP_TRIP);
//     eeprom_update_byte(&EramLaser2BattTrip, DEF_LASER_2_BATT_TRIP);
//     eeprom_update_byte(&EramLaser2OperateFlag, DEF_LASER_2_OPERATE_FLAG);
//     // Map Settings
//     eeprom_update_byte(&EramMapTotalPoints, DEF_MAP_TOTAL_PTS);
//     eeprom_update_byte(&EramActiveMapZones, DEF_ACTIVE_MAP_ZONES);
//     eeprom_update_byte(&EramActivePatterns, DEF_ACTIVE_PATTERNS);
//     eeprom_update_word(&EramLaserID, DEF_LASER_ID);
//     eeprom_update_byte(&Eram_Nbr_Rnd_Pts, DEF_NBR_RND_PTS);
//     eeprom_update_word(&EramAccelTripPoint, DEF_ACCEL_TRIP_POINT);
//     eeprom_update_word(&EramResetSeconds, DEF_RESET_SECONDS); // NOLINT
//     eeprom_update_byte(&EramOperationMode, DEF_OPERATION_MODE);
//     eeprom_update_byte(&EramFirstTimeOn, DEF_FIRST_TIME_ON);
//     eeprom_update_byte(&EramUserLightTripLevel, DEF_USER_LIGHT_TRIP_LEVEL);
//     eeprom_update_byte(&EramFactoryLightTripLevel, DEF_FACTORY_LIGHT_TRIP_LEVEL);
//     eeprom_update_byte(&EramLightTriggerOperation, DEF_LIGHT_TRIGGER_OPERATION);
//     eeprom_update_byte(&EramGyroAddress, MPU6050_ADDRESS);
//     eeprom_update_byte(&Eram_Tilt_Sep, DEF_TILT_SEP);
//     eeprom_update_word(&Eram_Step_Rate_Min, DEF_STEP_RATE_MIN);
//     eeprom_update_word(&Eram_Step_Rate_Max, DEF_STEP_RATE_MAX);
//     eeprom_update_byte(&EramSpeedScale, DEF_SPEEDSCALE);
//     eeprom_update_byte(&EramLaserHt, DEF_LASER_HT);
// }

// void ReadEramVars(void)
// { // Transfer EEPROM user data to RAM
//     UserLaserPower = eeprom_read_byte(&EramUserLaserPower);
//     MaxLaserPower = eeprom_read_byte(&EramMaxLaserPower);
//     Laser2TempTrip = eeprom_read_byte(&EramLaser2TempTrip);
//     Laser2BattTrip = eeprom_read_byte(&EramLaser2BattTrip);
//     Laser2OperateFlag = eeprom_read_byte(&EramLaser2OperateFlag);
//     MapTotalPoints = eeprom_read_byte(&EramMapTotalPoints);
//     GyroAddress = eeprom_read_byte(&EramGyroAddress);
//     SpeedScale = eeprom_read_byte(&EramSpeedScale);
//     Nbr_Rnd_Pts = eeprom_read_byte(&Eram_Nbr_Rnd_Pts);
//     LaserHt = eeprom_read_byte(&EramLaserHt);
//     Tilt_Sep = eeprom_read_byte(&Eram_Tilt_Sep);
//     ActiveMapZones = eeprom_read_byte(&EramActiveMapZones);
//     ActivePatterns = eeprom_read_byte(&EramActivePatterns);
//     LaserID = eeprom_read_word(&EramLaserID);
//     AccelTripPoint = eeprom_read_word(&EramAccelTripPoint);
//     ResetSeconds = eeprom_read_word(&EramResetSeconds); // NOLINT
//     OperationMode = eeprom_read_byte(&EramOperationMode);
//     FirstTimeOn = eeprom_read_byte(&EramFirstTimeOn);
//     UserLightTripLevel = eeprom_read_byte(&EramUserLightTripLevel);
//     FactoryLightTripLevel = eeprom_read_byte(&EramFactoryLightTripLevel);
//     LightTriggerOperation = eeprom_read_byte(&EramLightTriggerOperation);
//     Step_Rate_Min = eeprom_read_word(&Eram_Step_Rate_Min);
//     Step_Rate_Max = eeprom_read_word(&Eram_Step_Rate_Max);
//     SpeedScale = eeprom_read_byte(&EramSpeedScale);
// }

// void PrintEramVars()
// {
// #ifndef INCLUDE_PRINT_EEPROM
//     uartPrint(F("PrintEramVars() not incl"));
// #endif
// #ifdef INCLUDE_PRINT_EEPROM
//     char debugMsg[64]; // Reduce buffer size if possible

//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("UserLaserPower:  %p, %d"), (void *)&EramUserLaserPower, eeprom_read_byte(&EramUserLaserPower));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("MaxLaserPower:  %p, %d"), (void *)&EramMaxLaserPower, eeprom_read_byte(&EramMaxLaserPower));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Laser2TempTrip:  %p, %d"), (void *)&EramLaser2TempTrip, eeprom_read_byte(&EramLaser2TempTrip));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Laser2BattTrip:  %p, %d"), (void *)&EramLaser2BattTrip, eeprom_read_byte(&EramLaser2BattTrip));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Laser2OperateFlag:  %p, %d"), (void *)&EramLaser2OperateFlag, eeprom_read_byte(&EramLaser2OperateFlag));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("MapTotalPoints:  %p, %d"), (void *)&EramMapTotalPoints, eeprom_read_byte(&EramMapTotalPoints));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Gyro address: %p, %02x"), (void *)&EramGyroAddress, eeprom_read_byte(&EramGyroAddress));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Laser2BattTrip:  %p, %d"), (void *)&EramLaser2BattTrip, eeprom_read_byte(&EramLaser2BattTrip));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Laser2OperateFlag:  %p, %d"), (void *)&EramLaser2OperateFlag, eeprom_read_byte(&EramLaser2OperateFlag));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("First vertex address: %p"), (void *)&EramPositions);
//     uartPrint(debugMsg);
//     MapTotalPoints = eeprom_read_byte(&EramMapTotalPoints);
//     for (uint8_t i = 0; i < MapTotalPoints; i++)
//     {
//         snprintf_P(debugMsg, sizeof(debugMsg), PSTR("WPi: %u Zone: %d X: %d, Y: %d"), i, GetZVal(i), GetXVal(i), GetYVal(i));
//         uartPrint(debugMsg);
//     }
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("ActiveMapZones:  %p, : %d"), (void *)&EramActiveMapZones, eeprom_read_byte(&EramActiveMapZones));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("ActivePatterns:  %p, : %d"), (void *)&EramActivePatterns, eeprom_read_byte(&EramActivePatterns));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("LaserID:  %p, : %d"), (void *)&EramLaserID, eeprom_read_word(&EramLaserID));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("AccelTrip: %p, %d"), (void *)&EramAccelTripPoint, eeprom_read_word(&EramAccelTripPoint));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Nbr_Rnd_Pts: %p, %d"), (void *)&Eram_Nbr_Rnd_Pts, eeprom_read_byte(&Eram_Nbr_Rnd_Pts));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Tilt_Sep: %p, %d"), (void *)&Eram_Tilt_Sep, eeprom_read_byte(&Eram_Tilt_Sep));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("OperationMode: %p, %d"), (void *)&EramOperationMode, eeprom_read_byte(&EramOperationMode));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("FirstTimeOn: %p, %d"), (void *)&EramFirstTimeOn, eeprom_read_byte(&EramFirstTimeOn));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("UserLightTripLevel: %p, %d"), (void *)&EramUserLightTripLevel, eeprom_read_byte(&EramUserLightTripLevel));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("FactoryLightTripLevel: %p, %d"), (void *)&EramFactoryLightTripLevel, eeprom_read_byte(&EramFactoryLightTripLevel));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("LightTriggerOperation: %p, %d"), (void *)&EramLightTriggerOperation, eeprom_read_byte(&EramLightTriggerOperation));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Step_Rate_Min: %p, %d"), (void *)&Eram_Step_Rate_Min, eeprom_read_word(&Eram_Step_Rate_Min));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("Step_Rate_Max: %p, %d"), (void *)&Eram_Step_Rate_Max, eeprom_read_word(&Eram_Step_Rate_Max));
//     uartPrint(debugMsg);
//     snprintf_P(debugMsg, sizeof(debugMsg), PSTR("SpeedScale: %p, %d"), (void *)&EramSpeedScale, eeprom_read_byte(&EramSpeedScale));
//     uartPrint(debugMsg);

// #endif
// }

// 20250629:
typedef enum
{
    EEPROM_TYPE_BYTE,
    EEPROM_TYPE_WORD
} EepromVarType;

typedef struct
{
    const void *eeprom_addr;
    EepromVarType type;
    void *ram_addr;
    const char *name;       // For printing
    uint16_t default_value; // Store default as uint16_t (covers both byte and word)
} EepromVarInfo;

// ...existing code...

// Define all debug strings in PROGMEM
const char str_UserLaserPower[] PROGMEM = "UserLaserPower";
const char str_MaxLaserPower[] PROGMEM = "MaxLaserPower";
const char str_Laser2TempTrip[] PROGMEM = "Laser2TempTrip";
const char str_Laser2BattTrip[] PROGMEM = "Laser2BattTrip";
const char str_Laser2OperateFlag[] PROGMEM = "Laser2OperateFlag";
const char str_MapTotalPoints[] PROGMEM = "MapTotalPoints";
const char str_ActiveMapZones[] PROGMEM = "ActiveMapZones";
const char str_ActivePatterns[] PROGMEM = "ActivePatterns";
const char str_LaserID[] PROGMEM = "LaserID";
const char str_Nbr_Rnd_Pts[] PROGMEM = "Nbr_Rnd_Pts";
const char str_AccelTripPoint[] PROGMEM = "AccelTripPoint";
const char str_ResetSeconds[] PROGMEM = "ResetSeconds";
const char str_OperationMode[] PROGMEM = "OperationMode";
const char str_FirstTimeOn[] PROGMEM = "FirstTimeOn";
const char str_UserLightTripLevel[] PROGMEM = "UserLightTripLevel";
const char str_FactoryLightTripLevel[] PROGMEM = "FactoryLightTripLevel";
const char str_LightTriggerOperation[] PROGMEM = "LightTriggerOperation";
const char str_GyroAddress[] PROGMEM = "GyroAddress";
const char str_Tilt_Sep[] PROGMEM = "Tilt_Sep";
const char str_Step_Rate_Min[] PROGMEM = "Step_Rate_Min";
const char str_Step_Rate_Max[] PROGMEM = "Step_Rate_Max";
const char str_SpeedScale[] PROGMEM = "SpeedScale";
const char str_LaserHt[] PROGMEM = "LaserHt";

// Array to hold EEPROM control data
const EepromVarInfo eepromVars[] PROGMEM = {
    {&EramUserLaserPower, EEPROM_TYPE_BYTE, &UserLaserPower, str_UserLaserPower, DEF_USER_LASER_POWER},
    {&EramMaxLaserPower, EEPROM_TYPE_BYTE, &MaxLaserPower, str_MaxLaserPower, DEF_MAX_LASER_POWER},
    {&EramLaser2TempTrip, EEPROM_TYPE_BYTE, &Laser2TempTrip, str_Laser2TempTrip, DEF_LASER_2_TEMP_TRIP},
    {&EramLaser2BattTrip, EEPROM_TYPE_BYTE, &Laser2BattTrip, str_Laser2BattTrip, DEF_LASER_2_BATT_TRIP},
    {&EramLaser2OperateFlag, EEPROM_TYPE_BYTE, &Laser2OperateFlag, str_Laser2OperateFlag, DEF_LASER_2_OPERATE_FLAG},
    {&EramMapTotalPoints, EEPROM_TYPE_BYTE, &MapTotalPoints, str_MapTotalPoints, DEF_MAP_TOTAL_PTS},
    {&EramActiveMapZones, EEPROM_TYPE_BYTE, &ActiveMapZones, str_ActiveMapZones, DEF_ACTIVE_MAP_ZONES},
    {&EramActivePatterns, EEPROM_TYPE_BYTE, &ActivePatterns, str_ActivePatterns, DEF_ACTIVE_PATTERNS},
    {&EramLaserID, EEPROM_TYPE_WORD, &LaserID, str_LaserID, DEF_LASER_ID},
    {&Eram_Nbr_Rnd_Pts, EEPROM_TYPE_BYTE, &Nbr_Rnd_Pts, str_Nbr_Rnd_Pts, DEF_NBR_RND_PTS},
    {&EramAccelTripPoint, EEPROM_TYPE_WORD, &AccelTripPoint, str_AccelTripPoint, DEF_ACCEL_TRIP_POINT},
    {&EramResetSeconds, EEPROM_TYPE_WORD, &ResetSeconds, str_ResetSeconds, DEF_RESET_SECONDS},
    {&EramOperationMode, EEPROM_TYPE_BYTE, &OperationMode, str_OperationMode, DEF_OPERATION_MODE},
    {&EramFirstTimeOn, EEPROM_TYPE_BYTE, &FirstTimeOn, str_FirstTimeOn, DEF_FIRST_TIME_ON},
    {&EramUserLightTripLevel, EEPROM_TYPE_BYTE, &UserLightTripLevel, str_UserLightTripLevel, DEF_USER_LIGHT_TRIP_LEVEL},
    {&EramFactoryLightTripLevel, EEPROM_TYPE_BYTE, &FactoryLightTripLevel, str_FactoryLightTripLevel, DEF_FACTORY_LIGHT_TRIP_LEVEL},
    {&EramLightTriggerOperation, EEPROM_TYPE_BYTE, &LightTriggerOperation, str_LightTriggerOperation, DEF_LIGHT_TRIGGER_OPERATION},
    {&EramGyroAddress, EEPROM_TYPE_BYTE, &GyroAddress, str_GyroAddress, MPU6050_ADDRESS},
    {&Eram_Tilt_Sep, EEPROM_TYPE_BYTE, &Tilt_Sep, str_Tilt_Sep, DEF_TILT_SEP},
    {&Eram_Step_Rate_Min, EEPROM_TYPE_WORD, &Step_Rate_Min, str_Step_Rate_Min, DEF_STEP_RATE_MIN},
    {&Eram_Step_Rate_Max, EEPROM_TYPE_WORD, &Step_Rate_Max, str_Step_Rate_Max, DEF_STEP_RATE_MAX},
    {&EramSpeedScale, EEPROM_TYPE_BYTE, &SpeedScale, str_SpeedScale, DEF_SPEEDSCALE},
    {&EramLaserHt, EEPROM_TYPE_BYTE, &LaserHt, str_LaserHt, DEF_LASER_HT}};

#define N_EEPROM_VARS (sizeof(eepromVars) / sizeof(eepromVars[0]))

// Unified LoadEramDefaults
void LoadEramDefaults(void)
{
    for (size_t i = 0; i < N_EEPROM_VARS; ++i)
    {
        // Read the struct from PROGMEM
        EepromVarInfo var;
        memcpy_P(&var, &eepromVars[i], sizeof(EepromVarInfo));

        if (var.type == EEPROM_TYPE_BYTE)
        {
            eeprom_update_byte((uint8_t *)var.eeprom_addr, (uint8_t)var.default_value);
        }
        else
        {
            eeprom_update_word((uint16_t *)var.eeprom_addr, var.default_value);
        }
    }
}

void PrintMapPoints(void);
void PrintEramVars(void)
{
#ifdef INCLUDE_PRINT_EEPROM
    char nameBuf[32];
    for (size_t i = 0; i < N_EEPROM_VARS; ++i)
    {
        // Read the struct from PROGMEM
        EepromVarInfo var;
        memcpy_P(&var, &eepromVars[i], sizeof(EepromVarInfo));

        // Fetch the pointer to the PROGMEM string from the struct in PROGMEM
        const char *namePtr = (const char *)pgm_read_ptr(&eepromVars[i].name);
        strcpy_P(nameBuf, namePtr);

        if (var.type == EEPROM_TYPE_BYTE)
        {
            uint8_t val = eeprom_read_byte((const uint8_t *)var.eeprom_addr);
            // snprintf_P(debugMsg, sizeof(debugMsg), PSTR("%s: %p, %d"), var.name, var.eeprom_addr, val);
            snprintf_P(debugMsg, sizeof(debugMsg), PSTR("%s: %p, %d"), nameBuf, var.eeprom_addr, val);
        }
        else
        {
            uint16_t val = eeprom_read_word((const uint16_t *)var.eeprom_addr);
            // snprintf_P(debugMsg, sizeof(debugMsg), PSTR("%s: %p, %d"), var.name, var.eeprom_addr, val);
            snprintf_P(debugMsg, sizeof(debugMsg), PSTR("%s: %p, %d"), nameBuf, var.eeprom_addr, val);
        }
        uartPrint(debugMsg);
    }
    PrintMapPoints();
#endif
}
#define EEPROM_MIRROR_OFFSET 0xC0 //
#define EEPROM_MIRROR_ADDR(ptr) ((typeof(ptr))((uintptr_t)(ptr) + EEPROM_MIRROR_OFFSET))

void WriteEramMapPointsMirror(void);
void WriteEramMirrorVars(void)
{
    for (size_t i = 0; i < N_EEPROM_VARS; ++i)
    {
        EepromVarInfo var;
        memcpy_P(&var, &eepromVars[i], sizeof(EepromVarInfo));

        if (var.type == EEPROM_TYPE_BYTE)
        {
            uint8_t val = eeprom_read_byte((const uint8_t *)var.eeprom_addr);
            eeprom_update_byte(EEPROM_MIRROR_ADDR(var.eeprom_addr), val);
        }
        else
        {
            uint16_t val = eeprom_read_word((const uint16_t *)var.eeprom_addr);
            eeprom_update_word(EEPROM_MIRROR_ADDR(var.eeprom_addr), val);
        }
    }
    WriteEramMapPointsMirror();
}

void ReadEramVars(void)
{
    for (size_t i = 0; i < N_EEPROM_VARS; ++i)
    {
        EepromVarInfo var;
        memcpy_P(&var, &eepromVars[i], sizeof(EepromVarInfo));

        if (var.type == EEPROM_TYPE_BYTE)
        {
            uint8_t val = eeprom_read_byte((const uint8_t *)var.eeprom_addr);
            *((uint8_t *)var.ram_addr) = val;
        }
        else
        {
            uint16_t val = eeprom_read_word((const uint16_t *)var.eeprom_addr);
            *((uint16_t *)var.ram_addr) = val;
        }
    }
}

void PrintMapPoints(void)
{
    char debugMsg[64];
    uint8_t mapTotal = eeprom_read_byte(&EramMapTotalPoints);
    for (uint8_t i = 0; i < mapTotal; i++)
    {
        snprintf_P(debugMsg, sizeof(debugMsg), PSTR("WPi: %u Zone: %d X: %d, Y: %d"),
                   i, GetZVal(i), GetXVal(i), GetYVal(i));
        uartPrint(debugMsg);
    }
}

void WriteEramMapPointsMirror(void)
{
    uint8_t mapTotal = eeprom_read_byte(&EramMapTotalPoints);
    for (uint8_t i = 0; i < mapTotal; i++)
    {
        // Mirror X
        uint16_t x = eeprom_read_word(&EramPositions[i].EramX);
        eeprom_update_word((uint16_t *)((uintptr_t)&EramPositions[i].EramX + EEPROM_MIRROR_OFFSET), x);

        // Mirror Y
        uint16_t y = eeprom_read_word(&EramPositions[i].EramY);
        eeprom_update_word((uint16_t *)((uintptr_t)&EramPositions[i].EramY + EEPROM_MIRROR_OFFSET), y);
    }
}

// CRC
#define EEPROM_CRC_ADDR_HI (EEPROM_MIRROR_OFFSET - 2)
#define EEPROM_CRC_ADDR_LO (EEPROM_MIRROR_OFFSET - 1)
uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF)
{
    while (len--)
    {
        crc ^= (*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
// Write CRC to EEPROM_CRC_ADDR_HI/EEPROM_CRC_ADDR_LO (big-endian)
void WriteEepromConfigCRC()
{
    uint16_t crc = 0xFFFF;
    for (uint16_t addr = 0x00; addr < EEPROM_CRC_ADDR_HI; ++addr)
    {
        uint8_t val = eeprom_read_byte((uint8_t *)addr);
        crc = crc16_ccitt(&val, 1, crc);
    }
    // Store CRC at EEPROM_CRC_ADDR_HI (high byte), EEPROM_CRC_ADDR_LO (low byte)
    eeprom_update_byte((uint8_t *)EEPROM_CRC_ADDR_HI, (crc >> 8) & 0xFF);
    eeprom_update_byte((uint8_t *)EEPROM_CRC_ADDR_LO, crc & 0xFF);
    snprintf(debugMsg, DEBUG_MSG_LENGTH, "CRC : %d", crc);
    uartPrint(debugMsg);
}
// Verifying CRC
bool VerifyEepromConfigCRC()
{
    uint16_t crc = 0xFFFF;
    for (uint16_t addr = 0x00; addr < EEPROM_CRC_ADDR_HI; ++addr)
    {
        uint8_t val = eeprom_read_byte((uint8_t *)addr);
        crc = crc16_ccitt(&val, 1, crc);
    }
    uint8_t crc_hi = eeprom_read_byte((uint8_t *)EEPROM_CRC_ADDR_HI);
    uint8_t crc_lo = eeprom_read_byte((uint8_t *)EEPROM_CRC_ADDR_LO);
    uint16_t stored_crc = ((uint16_t)crc_hi << 8) | crc_lo;
    return (crc == stored_crc);
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
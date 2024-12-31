// shared_vars.h
// 20240523: Most of these shared variables are declared in Avitech.c, the main file, but used, perhaps only, in AppCmds.c.  It would probably
// be better to simply get rid of AppCmds.c.  But the BASCOM structure has been maintained.
#include "FieldDeviceProperty.h"
#include "const.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/eeprom.h>
#ifndef SHARED_VARS_H
#define SHARED_VARS_H
// #define GHOST
// #define ISOLATED_BOARD
#ifdef ISOLATED_BOARD
#define ISOLATED_BOARD_INTERVAL 10
extern bool isolated_board_flag;
extern uint16_t isolated_board_factor;
#endif
// #define THROTTLE //20241205: Use to debug laser temperature and power.
// #define BASE_PRINT
#define LOG_PRINT
#define NEW_APP
// #define DEBUG61 // Used to debug ReportVertices failure when <62:0> is being received too frequently.
#ifdef DEBUG61
#define DEBUG61_INTERVAL 100
#endif
// #define TEST_PATH_MODE
// #define TEST_FDP //FieldDeviceProperty
// #define TEST_CONVEX
// #define DEBUG
// #define PRINT_CMD9
// Variables
extern uint8_t EramUserLaserPower;
extern uint8_t UserLaserPower;
extern uint8_t EramMaxLaserPower;
extern uint8_t MaxLaserPower;
extern uint8_t LaserPower;
extern uint16_t EramLaserID;
extern uint16_t LaserID;
extern volatile bool SteppingStatus;
extern bool printPos;
extern uint8_t JogFlag;
extern uint8_t PanEnableFlag;
extern uint8_t PanDirection;
extern uint8_t PanSpeed;
extern uint8_t TiltEnableFlag;
extern uint8_t TiltDirection;
extern uint8_t TiltSpeed;
extern uint8_t EramMapTotalPoints;
extern uint8_t MapTotalPoints;
extern uint8_t Tick;
extern int X;
extern int Y;
extern int Dx;
extern int Dy;
extern int AbsX;
extern int AbsY;
extern uint8_t WarnLaserOnOnce;
extern uint8_t SetupModeFlag;
extern uint8_t PrevSetupModeFlag;
extern uint8_t EramFactoryLightTripLevel;
extern uint8_t FactoryLightTripLevel;
extern uint8_t BT_ConnectedFlag;
extern uint8_t SendDataFlag;
extern uint8_t SendSetupDataFlag;
extern uint8_t EramFirstTimeOn;
extern uint8_t SetupModeFlag;
extern long LightLevel;
extern uint8_t SendSetupDataFlag;
extern int EramAccelTripPoint;
extern int AccelTripPoint;
extern uint8_t EramOperationMode;
extern uint8_t OperationMode;
// extern uint8_t EramSpeedZone[]; //Leave out size?
// extern uint8_t SpeedZone[]; //Leave out size?
extern uint16_t DSS_preload;
extern uint8_t EramActiveMapZones;
extern uint8_t ActiveMapZones;
extern uint8_t EramActivePatterns;
extern uint8_t ActivePatterns;
extern uint16_t EramResetSeconds; // NOLINT
extern uint16_t ResetSeconds;
extern uint8_t EramLaser2OperateFlag;
extern uint8_t Laser2OperateFlag;
extern uint8_t EramLaser2BattTrip;
extern uint8_t Laser2BattTrip;
extern uint8_t EramLaser2TempTrip;
extern uint8_t Laser2TempTrip;
extern uint8_t EramUserLightTripLevel;
extern uint8_t UserLightTripLevel;
extern uint8_t EramLightTriggerOperation;
extern uint8_t LightTriggerOperation;
extern uint8_t EramFactoryLightTripLevel;
extern uint8_t FactoryLightTripLevel;
extern float VoltPerStep;
extern uint8_t LightSensorModeFlag;
extern uint8_t GyroAddress;
extern bool GyroOnFlag;
extern uint8_t EramGyroAddress;

extern uint32_t JM_n;
extern uint32_t MM_n;

// Declare the EramPos struct

typedef struct
{
    uint16_t EramX; // X map location 2 bytes
    uint16_t EramY; // Y map location 2 bytes. Zone is stored in highest 4 bits, Y value in lowest 12 bits
} EramPos;
// Declare the EramPositions array with elements of type EramPos
extern EramPos EramPositions[MAX_NBR_MAP_PTS];

// extern union {
//     int Z_accel;
//     struct {
//         uint8_t Zl_accel;
//         uint8_t Zh_accel;
//     };
// } Accel_Z;

extern uint16_t Instruction;
extern uint8_t Command;
extern bool received39;
extern char debugMsg[DEBUG_MSG_LENGTH]; // Buffer for debug messages
// 20240726 Tuning parameters - possibly not required in production version
extern uint16_t Eram_Step_Rate_Min;
extern uint16_t Step_Rate_Min;
extern uint16_t Eram_Step_Rate_Max;
extern uint16_t Step_Rate_Max;
extern uint8_t Eram_Rho_Min;
extern uint8_t Rho_Min;
extern uint8_t Eram_Rho_Max;
extern uint8_t Rho_Max;
extern uint8_t Eram_Nbr_Rnd_Pts;
extern uint8_t Nbr_Rnd_Pts;
extern uint8_t Eram_Tilt_Sep;
extern uint8_t Tilt_Sep;
extern uint8_t nbrWigglyPts;
extern uint8_t EramSpeedScale;
extern uint8_t SpeedScale;
extern uint8_t EramLaserHt;
extern uint8_t LaserHt;
extern uint8_t MapCount[2][NBR_ZONES];

// Functions
void GetLaserTemperature();
void ThrottleLaser();
void Audio2(uint8_t cnt, uint8_t OnPd, uint8_t OffPd);
void Audio2(uint8_t cnt, uint8_t OnPd, uint8_t OffPd, const char *debugInfo); // Declaration of Audio2, overloaded, with debugInfo
void ProcessCoordinates();
void ProgrammingMode();
void setupWatchdog();
void TransmitData();
void PrintAppData();
void PrintConfigData();
// void CalcSpeedZone();
uint16_t CalcSpeed();
// void SetLaserVoltage(uint8_t voltage);
void SetLaserVoltage(uint16_t voltage);
void testLaserPower();
void GetLightLevel();
void DecodeCommsData();
void ReadEramVars();
void LoadEramDefaults();
void uartPrint(const char *str);
// void uartPrintFlash(const __FlashStringHelper *message);
void printToBT(uint8_t cmd, uint16_t inst);
void printToBT(uint8_t cmd, int inst);
void printToBT(uint8_t cmd, long inst);
void StopTimer1();
void StopTimer3();
void setupTimer1();
void setupTimer3();
void initMPU();
void eeprom_update_word(uint16_t *eepromAddress, uint16_t newValue);
void PrintEramVars();
void LoadZoneMap(uint8_t zn, bool print_flag);
uint8_t GetZone(uint8_t i);
void TraceBoundary(uint8_t zone);
void GoToMapIndex();
void getMapPtCounts(bool doPrint);
void sendStatusData();
void handleGetPropertyRequest(FieldDeviceProperty property);
// void handleSetPropertyRequest(FieldDeviceProperty property, uint8_t value);
// void eeprom_update_byte(uint16_t *eepromAddress, uint16_t newValue);
#endif // SHARED_VARS_H
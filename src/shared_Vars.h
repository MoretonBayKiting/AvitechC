// shared_vars.h
// 20240523: Most of these shared variables are declared in Avitech.c, the main file, but used, perhaps only, in AppCmds.c.  It would probably 
// be better to simply get rid of AppCmds.c.  But the BASCOM structure has been maintained.
#include "const.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/eeprom.h>
#ifndef SHARED_VARS_H
#define SHARED_VARS_H
// Variables
extern uint8_t TiltSpeed;
extern uint8_t EramUserLaserPower;
extern uint8_t UserLaserPower;
extern uint8_t EramMaxLaserPower;
extern uint8_t MaxLaserPower;
extern uint16_t EramLaserID;
extern uint16_t LaserID;
extern bool SteppingStatus;
extern uint8_t PanEnableFlag;
extern uint8_t PanDirection;
extern uint8_t PanSpeed;
extern uint8_t TiltEnableFlag;
extern uint8_t TiltDirection;
extern uint8_t TiltSpeed;
extern uint8_t EramMapTotalPoints;
extern uint8_t MapTotalPoints;
extern int X;
extern int Y;
extern uint8_t WarnLaserOnOnce;
extern uint8_t SetupModeFlag;
extern uint8_t EramFactoryLightTripLevel;
extern uint8_t FactoryLightTripLevel;
extern uint8_t BT_ConnectedFlag;
extern uint8_t SendDataFlag;
extern uint8_t SendSetupDataFlag;
extern uint8_t EramFirstTimeOn;
extern uint8_t SetupModeFlag;
extern long LightLevel;
extern uint8_t SendSetupDataFlag;
extern uint16_t EramAccelTripPoint;
extern uint16_t AccelTripPoint;
extern uint8_t EramOperationMode;
extern uint8_t OperationMode;
extern uint8_t EramSpeedZone[]; //Leave out size?
extern uint8_t SpeedZone[]; //Leave out size?
extern uint16_t DSS_preload;
extern uint8_t EramActiveMapZones;
extern uint8_t ActiveMapZones;
extern uint16_t EEMEM EramResetSeconds;  //NOLINT
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
// Declare the EramPos struct

typedef struct {
    uint16_t EramX;             // X map location 2 bytes
    uint16_t EramY;             // Y map location 2 bytes. Operation data is also stored in this variable in the first 4 bits
} EramPos;
// Declare the EramPositions array with elements of type EramPos
extern EramPos EEMEM EramPositions[MAX_NBR_MAP_PTS];  //NOLINT

extern uint16_t Instruction;
extern uint8_t Command;

// Functions
void GetLaserTemperature();
void ThrottleLaser();
void Audio(uint8_t pattern);
void ProcessCoordinates();
void ProgrammingMode();
void setupWatchdog();
void PrintConfigData();
void CalcSpeedZone();
uint8_t CalcSpeed();
void SetLaserVoltage(uint8_t voltage);
void GetLightLevel();
void DecodeCommsData();
void ReadEramVars();
void LoadEramDefaults();
void uartPrint(const char* str);
void printToBT(uint8_t PrintCode, uint8_t Res);

#endif // SHARED_VARS_H
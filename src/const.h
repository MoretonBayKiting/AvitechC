// SYSTEM MATHS CONSTANTS (Don't modify)
#define ON 1
#define OFF 0

// PROJECT CONSTANTS
#define RAMPING_STEPS 1
// #define STARTUP_SPEED 100
#define START_RAMPING_COUNT 100
#define STEP_RATE_MAX 75
// #define STEP_RATE_MIN 2000
#define HOMING_SPEED 100
#define TILT_SLOW_STEP_RATE 1200
#define TILT_FAST_STEP_RATE 500
#define PAN_SLOW_STEP_RATE 400
#define PAN_FAST_STEP_RATE 105
#define LINE_VOLTAGE 5.00
#define X_MAXCOUNT 16000
#define X_MINCOUNT -16000
#define TILTSENSORIGNORE 325
#define NUM_BATT_READINGS 10
#define NUM_TEMP_READINGS 10
#define NIGHT_TRIP_TIME_FROM_STARTUP 240
// #define BT_TICKMAX 3600
#define ONE_SECOND 20
#define NBR_ZONES 4
#define MAX_NBR_VERTICES 10  //This is the maximum number of vertices per zone/map
#define MAX_NBR_MAP_PTS 40   // This is the maximum overall number of vertices
#define MAX_NBR_PERIMETER_PTS 100
#define MAX_NBR_PTS 40

// Default values to put in EEPROM (written by LoadDefs.c)
#define DEF_USER_LASER_POWER = 100
#define DEF_MAX_LASER_POWER = 120
#define DEF_LASER_2_TEMP_TRIP = 50
#define DEF_LASER_2_BATT_TRIP = 102
#define DEF_LASER_2_OPERATE_FLAG = 0
#define DEF_MAP_TOTAL_PTS = 0
#define DEF_SPEED = 50
#define DEF_ACTIVE_MAP_ZONES = 1
#define DEF_OP_MODE = 0 
#define DEF_FIRST_TIME_ON = 1
#define DEF_LASER_ID = 0

#define DEF_ACCEL_TRIP_POINT = 600
#define DEF_RESET_SECONDS = 3600

#define DEF_USER_LIGHT_TRIP_LEVEL = 10
#define DEF_FACTORY_LIGHT_TRIP_LEVEL = 10
#define DEF_LIGHT_TRIGGER_OPERATION = 1

#define MPU6050_W 0x68
#define MPU6050_R 0x69
#define WHO_AM_I_MPU6050 0x75

#define MCP4725 0x60 // I2C address of MCP4725
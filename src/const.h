// SYSTEM MATHS CONSTANTS (Don't modify)
#define ON 1
#define OFF 0

// PROJECT CONSTANTS
#define RAMPING_STEPS 1
// #define STARTUP_SPEED 100
#define START_RAMPING_COUNT 100
#define STEP_RATE_MAX 75
// #define STEP_RATE_MIN 200 //20240724 Arbitrary value
// #define RHO_MIN 10 //  20240724 Minimum cartesian distance (approx) from laser head.
// #define RHO_MAX 200 // 20240724 Maximum cartesian distance (approx) from laser head - for speed calc.
#define HOMING_SPEED 50         // 20240801 Should this be 100.  50 seems better.
#define TILT_SLOW_STEP_RATE 300 // 1500
#define TILT_FAST_STEP_RATE 70  // 1500
#define PAN_SLOW_STEP_RATE 300  // 400
#define PAN_FAST_STEP_RATE 50   // 10
#define LINE_VOLTAGE 5.00
#define X_MAXCOUNT 16000
#define X_MINCOUNT -16000
#define TILT_SENSOR_IGNORE 325 // Should this and MAX_TILT be the same (ie only one required)
// #define MAX_TILT 317 //20241204: Ref GM email: "Y shouldnt go above (physically below) Y: 317"
#define NUM_BATT_READINGS 10
#define NUM_TEMP_READINGS 10
#define NIGHT_TRIP_TIME_FROM_STARTUP 240
// #define BT_TICKMAX 3600
#define ONE_SECOND 20
#define NBR_ZONES 4
#define MAX_NBR_VERTICES 10 // This is the maximum number of vertices per zone/map
#define MAX_NBR_MAP_PTS 32  // This is the maximum overall number of vertices
// #define MAX_NBR_PERIMETER_PTS 120 //Maximum number of perimeter points for a zone.  This could probably be higher.
#define TILT_STEPS_PER_RAD 2052.0 // 36 * 57 (36 steps per degree and 57 radians per degree?)
#define PAN_STEPS_PER_RAD 2546.0  // 20240803: 4000 steps for 90°
#define MAX_RANGE 500             // Maximum range (metres) from laser to target.
#define MIN_RANGE 10              // Minimum range (metres) from laser to target.

#define MID_PT_SEPARATION 20 // Get the approximate number of mid points to insert between vertices as (x1 - x0)/MID_PT_SEPARATION
// #define  LASER_HT5.0 //Height of laser in metres.  Used for polar:cartesian conversions.
#define BUFFER_SIZE 100  // Length of buffer to use for income serial messages.
#define HIGH_JOG_POS 400 // These were, more or less, 4 & 1 in BASCOM.  But that provided very jerky motion.
#define LOW_JOG_POS 100
#define MIN_PERIMETER_TILT 2 // Minimum tilt offset between vertices to allow for.  If less, assume "slope" is zero.
#define FIXED_PAN_DIFF 200   // Steps between dense perimeter points on pan only (ie constant tilt) boundary segment
#define FIXED_TILT_DIFF 50   // Steps between dense perimeter points on tilt only (ie constant pan) boundary segment
#define MIN_TILT_DIFF 2
#define DEF_SLOPE 30000   // Extreme number for slope (inverse) when tilt values are close.
#define X_MAX_COUNT 16000 // Maximum X count. About 4 turns of the head from 0 in +ve direction
#define MAX_PAN_DIST 20   // 20240702 Approximate maximum panning distance between dense perimeter points
// #define NBR_RND_PTS 30 // Some arbitrary number of randomly selected perimeter points to traverse before the next pattern.
// Default values to put in EEPROM (written by LoadDefs.c)
#define DEF_USER_LASER_POWER 100
#define DEF_MAX_LASER_POWER 120
#define MAX_LASER_VALUE 4095 // 20240625: Use this for testing - may not be used after that.
#define DEF_LASER_2_TEMP_TRIP 50
#define DEF_LASER_2_BATT_TRIP 102
#define DEF_LASER_2_OPERATE_FLAG 0
#define DEF_MAP_TOTAL_PTS 0
#define DEF_SPEED 50
#define DEF_ACTIVE_MAP_ZONES 15 // 00001111b.
#define DEF_ACTIVE_PATTERNS 15  // 00001111b.
#define DEF_OP_MODE 0
#define DEF_FIRST_TIME_ON 1
#define DEF_LASER_ID 0
#define DEF_TILT_SEP 10 // Initially used directly as steps of tilt between rungs.  But treat it as m for Cartesian version.
#define DEF_ACCEL_TRIP_POINT 600
#define DEF_RESET_SECONDS 3600

#define DEF_USER_LIGHT_TRIP_LEVEL 32
#define DEF_FACTORY_LIGHT_TRIP_LEVEL 32
#define DEF_LIGHT_TRIGGER_OPERATION 1
#define DEF_OPERATION_MODE 0

// #define WHO_AM_I_MPU6050 0x75
// MPU pins.  The BASCOM code used a variable and set these pins to 0xD0 and 0xD1 for board version 6.12.
// These values, 0xD2 and 0xD3, are for version 6.13  Ref sub Initgyro() in BASCOM code.
// These are not physical pins (and not MCU addresses) but addresses in the relevant I2C devices (MPU and laser power controller)
// #define MPU6050_W_PIN 0xD2 //These are 8 bit addesses which already include the r/w bit in each.
// #define MPU6050_R_PIN 0xD3
#define MPU6050_ADDRESS 0x69
#define MPU6000_ADDRESS 0x68
// MCP4725 is DAC to control laser power.  Comms is via I2C.
// #define LASER_W_PIN 0xD2 //This value is made up.  Needs to be found.
#define MCP4725ADD (0x60 << 1) // I2C address of MCP4725.  This is the 7 bit address (0x60) left shifted to get the write address.
#define AUDIO_DELAY 50
#define BAUD 9600                    // 57600  //Baud rate for HC-05
#define MYUBRR F_CPU / 16 / BAUD - 1 // The formula for calculating the UBRR value is given in the AVR datasheet:
#define DEBUG_MSG_LENGTH 120
#define BUFFER_SIZE 24 // 20240924
// 20240624 Constants to be used for i2c (MCP4725 DAC)
#define TW_START_TRANSMITTED 0x08
#define TW_REP_START_TRANSMITTED 0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_DATA_ACK 0x28
#define TW_MR_SLA_ACK 0x40
#define TW_MR_DATA_ACK 0x50

#define SEG_LENGTH 100 // Length (in steps) between straightening points along boundary segments.
// 20241119: WigglyBorder_
#define X_WIGGLY_BORDER_RANGE 20
#define Y_WIGGLY_BORDER_RANGE 20
// #define NBR_WIGGLY_POINTS 0

// Various parameters written from old speed zone values to new parameters.  Use these parameters for rescaling.
#define OLD_SPEED_ZONE_MIN 0
#define OLD_SPEED_ZONE_MAX 255
#define SPEED_SCALE_MIN 130
#define SPEED_SCALE_MAX 20 // Higher speed requires a lower value of SpeedScale
#define NBR_RND_MIN 0
#define NBR_RND_MAX 300
#define TILT_SEP_MIN 1
#define TILT_SEP_MAX 30
#define PAN_SEP 100 // 20241219: Pan separation (steps) between interpolated points on rungs.
#define WIGGLY_MIN 0
#define WIGGLY_MAX 4
#define LASER_HT_MIN 40 // Decimetres.  ie 40 decimetres = 4m
#define LASER_HT_MAX 60

#define FST_STORE_PT_INDEX 128
#define REPORT_VERTICES_DELAY 20
#define REPORT_PERIOD 60 // TJTick increments every 50ms.  Report each time it increments REPORT_PERIOD times.  So 60 will give 3s report period.
#define CONVEX_TOLERANCE 100

#define PROPERTY_GET_CHANNEL 58
#define PROPERTY_SET_CHANNEL 59
#define CANT_SET_PROPERTY 255
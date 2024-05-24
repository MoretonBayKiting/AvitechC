// vars_main.h
// Define global variables used in Avitech.c which are not used elsewhere.  Those used in both Avitech.c and
// any other file are declared in shared_vars.h
#ifndef VARS_MAIN_H
#define VARS_MAIN_H

// Setup X Y co-ordinate system and stepping variables
uint16_t Boardrevision;                                   // Board that program will be deployed to
int X;                                            // X position requested to move EG X=100   .Move X 100 steps
int Y;                                            // Y position requested to move EG X=100   .Move X 100 steps
// int cart[2];                                      // Cartesian coordinates
bool rndLadBit;                                        // 0 or 1 to indicate if last pass was for a new rung on one side or the next side is needed
uint8_t minYind, maxYind;
uint8_t Rndnbr, prevRndnbr, altRndNbr;

int Absx;                                         // X absolute position from home position
int Absy;                                         // Y absolute position from home position

uint16_t Stepcount;                                       // How many steps to be executed
uint16_t Dss_preload;                                     // TCC1 compare value to get Desired Stepping Speed (DSS), Lower the number the faster the pulse train
// bool Steppingstatus;  //Shared                                   // Background stepping in progress, 0 = No stepping on progress. 1= Stepping in progress

int Stepoverratio;                                // Step over ratio between Pan And tilt
uint16_t Remainder;                                       // Used for Step Over Ratio calculation
long Dy;                                              // Used to calulate how many steps required on Y axis from last position
long Dx;                                              // Used to calulate how many steps required on X axis from last position
bool Master_dir;                                       // Which motor need to step the most X or Y
uint8_t Tick;                                            // Tick flag. 2Hz update time
uint8_t AccelTick;                                       // Tick flag. 2Hz update time
uint32_t Tod_tick;                                       // Tick for system time of day
uint8_t BuzzerTick;                                      // Tick flag. 2Hz update time
uint8_t LaserTick;                                       // Tick flag. 20Hz update time

// These are defined/declared in AppCmds.c
// uint8_t Command;                                         // Holds the command value fron the RS232 comms string.See documentation on the Comms data string build
// uint16_t Instruction;                                     // Holds the data value fron the RS232 comms string.See documentation on the Comms data string build

typedef struct {
    uint16_t EramX;             // X map location 2 bytes
    uint16_t EramY;             // Y map location 2 bytes. Operation data is also stored in this variable in the first 4 bits
} EramPosition;
EramPosition EEMEM EramPositions[MAX_NBR_MAP_PTS];               // Use eeprom for waypoints
uint8_t EEMEM EramMapTotalPoints; //May not be used
uint8_t MapTotalPoints;                                 
bool X_TravelLimitFlag;                                // Over travel flag
bool Y_TravelLimitFlag;                                // Over travel flag
uint16_t Y_maxcount;                                   // Max Count Y can have. This changes for different mode as more or less tilt is required in different modes
uint8_t Mapcount[2][NBR_ZONES]; // mapCount[1][i] is incremental; mapCount[2][i] is cumulative of mapCount[1][i]
uint8_t Zn;
uint8_t NoMapsRunningFlag;
// uint16_t AppCompatibilityNo;
uint8_t Mpu6050_w;
uint8_t Mpu6050_r;
uint8_t GyroAddress;

//----Set up for Watchdog timer------------------
uint8_t Wd_flag; // Read the flag first, as configuring the watchdog clears the WDRF bit in the MCUSR register
uint8_t Wd_byte;


//---------Commands from Bluetooth input--------------
uint8_t PanEnableFlag; // X enable axis flag when jogging mode
uint8_t PanDirection; // X rotation requested direction flag
uint8_t PanSpeed; // X stepping speed flag... 1=fast 0=slow
uint8_t TiltDirection; // Y rotation requested direction flag
uint8_t TiltEnableFlag; // Y enable axis flag when jogging mode
uint8_t TiltSpeed; // Y stepping speed flag... 1=fast 0=slow

//---------Laser Variables------------------------
// uint8_t EramUserLaserPower[3]; // Laser Power that the user sets up
// uint8_t DefaultEramUserLaserPower;
uint8_t UserLaserPower;
uint8_t EEMEM EramUserLaserPower;

// uint8_t EramMaxLaserPower[3]; // Store Eram laser operating power setting
// uint8_t DefaulteramMaxLaserPower;
uint8_t MaxLaserPower;
uint8_t EEMEM EramMaxLaserPower

uint8_t Laserpower; // Final calculated value send to the DAC laser driver

float Voltperstep; // Laser power per step

uint8_t Laserovertempflag; // Laser over temp error flag
uint8_t Flashthelaserflag; // Setup laser flash flag bit
uint8_t Cmdlaseronflag;

uint8_t SetupModeFlag;

uint8_t Laser2StateFlag; // Sets the current state if the laser 2 is working. 0=off 1=on

uint8_t EEMEM EramLaser2OperateFlag;
uint8_t Laser2OperateFlag; // Set a flag whether the user wants Laser 2 to operate  0=laser 2 off, 1=Laser2 on

// uint8_t EramLaser2TempTrip[3];
uint8_t Laser2TempTrip; // Sets the % trip value of the laser 1 temp where to turn the laser 2 on/off  100=100%,50=50% 25=25%
uint8_t EEMEM EramLaser2TempTrip;

uint8_t Laser2TempTripflag;

uint8_t Laser2BbattTrip; // Sets the % trip value of the battery charge where to turn the laser 2 on/off  100=100%,50=50%,25=25%
uint8_t EEMEM EramLaser2BattTrip;

uint8_t Laser2BattTripFlag;

//----------Communication Variables--------------------
uint8_t A;
char S[4]; // Variables to decode data coming in from the RS232 data stream from the phone commands
uint8_t DataInBufferFlag; // Flag for there is data in the RS232 comms buffer

//----------Speed Zone Variables--------------------
uint8_t SpeedZone[5];
uint8_t EEMEM EramSpeedZone[5];

//----------Battery Charge Variables--------------------
uint16_t Battreadings[10];
uint8_t Battreadindex;
uint16_t Batttotal;
uint16_t Battvoltavg;
uint8_t Batteryvoltage; // Change this to single if you want to calc voltage on the micro
uint8_t Batterytick; // Battery tick sampling rate tick's
uint8_t LoopCount;

//----------Temperature Variables--------------------
uint8_t Lasertemperature;

//----------Light Sensor Variables Variables--------------------
uint8_t EEMEM EramUserLightTripLevel;
uint8_t Userlighttriplevel;

uint8_t EEMEM EramFactoryLightTripLevel; // Factory Default light setting value. Laser need to go into Lightbox
uint8_t FactoryLightTripLevel;

uint8_t EramLightTriggerOperation; // 0=24hr. 1= Day Mode 2= Night Mode
uint8_t LightTriggerOperation;

long Lightlevel; // Holds the value of the ADC light sensor
uint8_t LightSensorModeFlag;
//---------Other Variables-------------------------
uint8_t Bt_ConnectedFlag;

uint8_t EEMEM EramOperationMode;
uint8_t OperationMode;

char OpModeTxt[11];

uint8_t EramActiveMapZones;
uint8_t Activemapzones;

uint8_t SendDataFlag;                                      // Diagnostic mode send data back to user
uint8_t SendSetupDataFlag;                                 // Send config data back to user.  20240522: Not used 

uint8_t EEMEM EramFirstTimeOn;
uint8_t FirstTimeOn;

uint8_t Firsttimelaseron;
uint8_t WarnLaserOnOnce;
uint8_t Ishome;

long Secondsfromreset;                                     // Variable to hold the reset current count

uint8_t Systemfaultflag;
uint8_t Index;

uint16_t EEMEM EramLaserID;
uint16_t LaserID;

uint16_t EEMEM EramResetSeconds;
uint16_t ResetSeconds;

int EEMEM EramAccelTripPoint;                                 // Accelerometer trip angle value
int AccelTripPoint;

uint8_t Zonespeed;
uint16_t Currentspeedzone;

uint16_t MapRunning;

uint8_t Counter50ms;

//---------MPU6050 Accelerometer & Temperature-----------------
union { // Accel_Z to be used to access any of Z_accel, int or the subcomponents
    int Z_accel;
    struct {
        uint8_t Zl_accel;
        uint8_t Zh_accel;
    };
} Accel_Z;
// int Z_accel;                                               // Accelerometer uses 2 bytes of memory. Data is read from the chip in 2 lots of 1 byte packets
// uint8_t Zl_accel;                                          // Stored memory of first byte data from accelerometer
// uint8_t Zh_accel;                                          // Stored memory of second byte data from accelerometer
uint8_t Z_accelflag;
uint8_t Z_accelflagprevious;

union {
    int Acceltemp;
    struct {
        uint8_t L_acceltemp;
        uint8_t H_acceltemp;
    };
} Accel; 
// int Acceltemp;  //Could have used a union/struct approach to store Acceltemp as an int and the L and H parts as bytes in the same memory
// uint8_t L_acceltemp;
// uint8_t H_acceltemp;

// Stack variables not used in C implementation.  Leave them in with place holder values so as not (initially) to break something elsewhere 
uint16_t Hw_stack;
uint16_t Sw_stack;
Hw_stack = 1;
Sw_stack = 1;
uint16_t Frame_size;

int Vertices[3][MAX_NBR_VERTICES];
int Perimeter[2][MAX_NBR_PERIMETER_PTS];
uint8_t NbrPerimeterPts;
int res[2];  //Global variables to be used in cartesian/polar conversions:Input and results.
// int last[2], nxt[2];

#endif // VARS_MAIN_H
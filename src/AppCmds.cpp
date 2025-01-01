#include <Arduino.h>
#include "FieldDeviceProperty.h"
#include "shared_Vars.h"
#include "pin_mappings.h"
#include "AppCmds.h"
#include "const.h"
// #include <Arduino.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// 20241206: Use new ReScale to reinterpret previously used variables (mostly SpeedZone values)
uint16_t ReScale(int32_t val, int32_t oldMin, int32_t oldMax, int32_t newMin, int32_t newMax)
{
    // Perform the calculation using floating-point arithmetic
    if (val < oldMin)
        val = oldMin;
    if (val > oldMax)
        val = oldMax;
    float ratio = (static_cast<float>(val) - static_cast<float>(oldMin)) / (static_cast<float>(oldMax) - static_cast<float>(oldMin));
    float r = ratio * (static_cast<float>(newMax) - static_cast<float>(newMin)) + static_cast<float>(newMin);
    return static_cast<uint16_t>(r);
}

void DecodeCommsData()
{

    switch (Command)
    {
    case 1:
        Cmd1();
        break; // Update laser power (in EEPROM)
    case 2:
        Cmd2();
        break; // Control pan during setup (stop/start, speed, dir)
    case 3:
        Cmd3();
        break; // Control tilt during setup (stop/start, speed, dir)
    case 4:
        Cmd4();
        break; // Update MaxLaserPower (in EEPROM)
    case 5:
        Cmd5();
        break; // Update LaserID (in EEPROM)
    case 6:
        Cmd6();
        break; // X = Instruction;  Start timer1.
    case 7:
        Cmd7();
        break; // Y = Instruction;  Start timer1.
    case 8:
        Cmd8();
        break; // Print EEPROM values to serial output.
    case 9:
        Cmd9();
        break; // Store way point
    case 10:
        Cmd10();
        break; // Setup/Run mode selection. Cold restart
    case 11:
        Cmd11();
        break; // Process the Send Diagnostic Data register or Process the full reset flag on next restart
    case 12:
        Cmd12();
        break; // Store the accelerometer trip point
    case 13:
        Cmd13();
        break; // Update OperationMode (in EEPROM)
    case 14:
        Cmd14();
        break; // Delete last way point
    case 16:
        Cmd16();
        break; // BASCOM: SpeedZone1.  Now SpeedScale - see notes with function below.
    case 17:
        Cmd17();
        break; // BASCOM: SpeedZone2.  Now Nbr_Rnd_Pts
    case 18:
        Cmd18();
        break; // BASCOM: SpeedZone3.  Now tilt_sep (rung density)
    case 19:
        Cmd19();
        break; // BASCOM: SpeedZone4.  20241209.  Wiggly points.
    case 20:
        Cmd20();
        break; // BASCOM: SpeedZone5.  LaserHt
    // case 15: Cmd15(); break;  //MinimumLaserPower - needs attention
    case 21:
        Cmd21();
        break; // Needed, as null, to listen to bespoke app.
    case 24:
        Cmd24();
        break;
    case 30:
        Cmd30();
        break; // Update ResetSeconds (in EEPROM)
    case 31:
        Cmd31();
        break; // Delete all MapTotalPoints (put EramMapTotalPoints and MapTotalPoints to zero.)
    case 32:
        Cmd32();
        break; // Update Laser2OperateFlag (in EEPROM)
    case 33:
        Cmd33();
        break; // Update Laser2BattTrip (in EEPROM)
    case 34:
        Cmd34();
        break; // Update Laser2TempTrip (in EEPROM)
    case 35:
        Cmd35();
        break; // Update UserLightTripLevel (in EEPROM)
    case 36:
        Cmd36();
        break; // Update LightTriggerOperation (in EEPROM)
    case 37:
        Cmd37();
        break; // getLightLevel()
    case 38:
        Cmd38();
        break; // Update FactoryLightTripLevel (in EEPROM)
    case 39:
        Cmd39();
        break; // 20240612: Test function set up by TJ

    // case 45: Cmd45(); break;  //Tilt_Sep
    case 46:
        Cmd46();
        break; // Step_Rate_Min
    case 47:
        Cmd47();
        break; // Step_Rate_Max
    case 48:
        Cmd48();
        break; // Rho_Min;
    case 49:
        Cmd49();
        break; // Rho_Max;
    case 50:
        Cmd50();
        break; // Nbr_Rnd_Pts

    case 52:
        Cmd52(); // ActivePatterns
        break;   // Update ActiveMapZones (in EEPROM)
    case 53:
        printPos = Instruction; // printPos is bool so True for any positive value, false if zero.
        // printPos  determines if some regular run time data is printed - not needed for normal app use.
    case 54:
        Cmd54();
        break; // Update ActivePatterns (in EEPROM)
    case 55:   // Turn audio on or off.
        audioOn = !audioOn;
        sprintf(debugMsg, "audioOn: %d", audioOn);
        uartPrint(debugMsg);
        break; //

    case PROPERTY_GET_CHANNEL:
        handleGetPropertyRequest(Instruction);
        break;
    case PROPERTY_SET_CHANNEL:
        FieldDeviceProperty prop = static_cast<FieldDeviceProperty>(Instruction >> 8); // Upper 8 bits encode property
        uint8_t value = Instruction & 0x00FF;
        handleSetPropertyRequest(prop, value);
        break;

        //     case 59:
        // #ifdef ISOLATED_BOARD
        //         isolated_board_factor = Instruction;
        // #endif
        //         break;
    case 60: // Check zones.
        GoToMapIndex();
        break; // Update ActiveMapZones (in EEPROM)
    case 61:   // ReportVertices and store MapTotalPoints.
        if (Instruction == 0)
        {
            ReportVertices(); // Report vertices back to app
        }
        break;
        if (Instruction == 1)
        {
            eeprom_update_byte(&EramMapTotalPoints, MapTotalPoints); // MapTotalPoints is set whenever a point is stored.  The app sends <61:1> when it has finished sending points.
            CmdStorePts(true);
        }
    case 62:
        printToBT(54, ActiveMapZones);
        printToBT(52, ActivePatterns);
        break;

    case 63:
        sendStatusData();
        break;

    case 64:
        TraceBoundary(Instruction); // Instruction is zone (1 based)
        break;
    }

    if ((Command >= FST_STORE_PT_INDEX) && Command <= (FST_STORE_PT_INDEX + 3 * MAX_NBR_MAP_PTS))
    {
        CmdStorePts(false);
    }
}

void Cmd1()
{
    // Incoming value 0-100.. This value is a percentage of how many % the user wants laser dimmer than the max laser power
    eeprom_update_byte(&EramUserLaserPower, Instruction);
    UserLaserPower = Instruction;
    // LaserPower = Instruction; //20241202: Temporary for testing.
    GetLaserTemperature();
    ThrottleLaser();
    Audio2(1, 2, 0); //,"AC1");
    _delay_ms(100);
}

// void resetDebugCounters(){ //Reset counters to zero so that there is a print after every message (cmd2 & cmd3) is received.
//     MM_n = 0;
//     JM_n = 0;
// }

void Cmd2()
{
    // Process Pan Stop/Start Register
    TiltEnableFlag = 0; // 20240620: Added by TJ.
    PanEnableFlag = (Instruction & 0b00000001) ? 1 : 0;
    if (PanEnableFlag == 0)
        X = AbsX; // Attempt to stop kink at direction changes
    // Process Pan Direction Register
    PanDirection = (Instruction & 0b00000010) ? 1 : 0;
    // Process Pan Speed Register
    PanSpeed = (Instruction & 0b00000100) ? 1 : 0;
    // Reset PanEnableFlag to zero if outside range.
    setupTimer1(); // 20240729: Start Timer 1 - it may have been stopped by going outside boundary.
}

void Cmd3()
{
    // Process Tilt Stop/Start Register
    PanEnableFlag = 0; // 20240620: Added by TJ.
    TiltEnableFlag = (Instruction & 0b00000001) ? 1 : 0;
    if (TiltEnableFlag == 0)
        Y = AbsY;
    // Process Tilt Direction Register
    TiltDirection = (Instruction & 0b00000010) ? 0 : 1; // 20240629: Back to 1:0. 20240622 Had the opposite previously as directions seemed to be wrong.
    // 20240629 Back to 0:1.  This saves making asymmetric change in JogMotors to this: pos = pos * (dir ? 1 : -1);
    //  Process Tilt Speed Register
    TiltSpeed = (Instruction & 0b00000100) ? 1 : 0;
    setupTimer1();
}

void Cmd4()
{
    eeprom_update_byte(&EramMaxLaserPower, Instruction);
    MaxLaserPower = Instruction; // #define DEF_MAX_LASER_POWER 120
    GetLaserTemperature();
    ThrottleLaser();
    Audio2(1, 2, 0); //,"AC4");
}

void Cmd5()
{
    eeprom_update_word(&EramLaserID, Instruction);
    LaserID = Instruction;
    Audio2(1, 2, 0); //,"AC5");
}

void Cmd6()
{
    X = Instruction;
    ProcessCoordinates();
    SteppingStatus = 1;
    Audio2(1, 2, 0); //,"AC6");
    // StartTimer1();
    TCCR1B |= (1 << CS12) | (1 << CS10);
}

void Cmd7()
{
    Y = Instruction;
    ProcessCoordinates();
    SteppingStatus = 1;
    Audio2(1, 2, 0); //,"AC7");
    // StartTimer1();
    TCCR1B |= (1 << CS12) | (1 << CS10);
}
void Cmd8()
{
    PrintEramVars();
}

void Cmd9()
{
    uint16_t OperationZone;
    int16_t ZoneY = Y; // 20240628 ZoneY is Y with zone added. Y populates the 12LSBs and zone is one of the 4MSBs.
    uartPrint("ST3 C9 \n");
    StopTimer3(); // 20240628 Don't want interrupt getting in the way of EEPROM write?
    // Get the Operation Zone data from the received data
    // OperationZone = Instruction & 0b111100000000; // Get raw Operation Zone from Instruction
    OperationZone = Instruction & 0xF00; // Get raw Operation Zone from Instruction
    OperationZone >>= 8;                 // Position the Operation Zone bits
    // Decode the binary data for Operation Zone
    switch (OperationZone)
    {
    case 1:
        OperationZone = 1;
        break;
    case 2:
        OperationZone = 2;
        break;
    case 4:
        OperationZone = 3;
        break;
    case 8:
        OperationZone = 4;
        break;
    default:
        uartPrint("Invalid Operation Zone!");
        break; // Handle invalid Operation Zone
    }
    uint8_t MapPointNumber = static_cast<uint8_t>(Instruction & 0xFF); // Get Map Point number from Instruction

    MapTotalPoints = MapPointNumber;
    eeprom_update_byte(&EramMapTotalPoints, MapTotalPoints); // 20240726 This had not been included

    if (MapPointNumber >= MAX_NBR_MAP_PTS)
    {
        uartPrint("Error: MapPointNumber out of range!");
        return; // Early return to prevent out-of-bounds access
    }
    ZoneY = ((Instruction & 0b111100000000) << 4) | abs(Y); // 20240628. eg <9:257. => 0b0001 0000 0001 .  This is point 1 in zone 1.  <9:518> =>  0b0010 0000 0110 would be point 6 in zone 2.
// Compounding with Y only works if the first byte of Y (4 bytes, first being highest) is 0 (which is where zone is encoded).
// eeprom_update_word((uint16_t*)&EramMapTotalPoints, MapPointNumber);
#ifdef PRINT_CMD9
    sprintf(debugMsg, "MapPoint: %d in zone %d with X: %d, Y: %d, Inst: %d, ZoneY: %04x", MapPointNumber, OperationZone, X, Y, Instruction, ZoneY);
    uartPrint(debugMsg);
#endif
    // Update EEPROM with X and Y values.  Note that index for EramPositions is zero based whereas app treats as 1 based.  Hence MapPointNumber-1
    uint16_t eepromAddress = (uint16_t)&EramPositions[MapPointNumber - 1].EramX;
    eeprom_update_word((uint16_t *)eepromAddress, X);
#ifdef PRINT_CMD9
    sprintf(debugMsg, "X: %04x,%d", eepromAddress, X);
    uartPrint(debugMsg);
#endif

    eepromAddress = (uint16_t)&EramPositions[MapPointNumber - 1].EramY;
    eeprom_update_word((uint16_t *)eepromAddress, ZoneY);
#ifdef PRINT_CMD9
    sprintf(debugMsg, "Y: %04x,%d", eepromAddress, Y);
    uartPrint(debugMsg);
#endif

    Audio2(1, 2, 0); //,"AC9");
    setupTimer3();   // Restart having stopped it at the start of this function.  Really?
}

void cmd10_running()
{
    eeprom_update_byte(&EramMapTotalPoints, MapTotalPoints); // Setting to run mode indicates completion of setup so store MapTotalPoints.
    WarnLaserOnOnce = 1;                                     // Enable laser warning when Run Mode button is pressed
    PrevSetupModeFlag = SetupModeFlag;
    SetupModeFlag = 0;
}
void cmd10_programming()
{
    WarnLaserOnOnce = 1; // Enable laser warning when Program Mode button is pressed
    PrevSetupModeFlag = SetupModeFlag;
    SetupModeFlag = 1;
    printToBT(9, 1); // 20240922
    // Audio2(2,2,2);//,"AC10:1");
    ProgrammingMode(); // Home machine ready for programming in points
}

void cmd10_restart()
{
    Audio2(1, 2, 0); //,"AC10:4");
    setupWatchdog();
    _delay_ms(1000);
}

void cmd10_lightSensor() // Setup light sensor mode    <10:8>
{
    Audio2(1, 2, 0); //,"AC10:8");
    SetupModeFlag = 2;
    _delay_ms(1000);
}

void cmd10_lightTrigger()
{ // Store current value to default light trigger value    <10:16>
    if (SetupModeFlag == 2 && LightLevel < 100)
    {
        eeprom_update_byte(&EramFactoryLightTripLevel, LightLevel);
        FactoryLightTripLevel = LightLevel;
        Audio2(1, 2, 0); //,"AC10:16");
        _delay_ms(1);
    }
}

void cmd10_btConnected()
{ // App telling the micro that the bluetooth is connected
    BT_ConnectedFlag = 1;
    // 20241209.  Why would SendDataFlag be zero, as was set here?  Change it to 1.  It's only used in TransmitData() and that's only called from DoHouseKeeping().
    // 20241209.  Put back to zero.  Dom has added refresh button to app.
    SendDataFlag = 0; // Output data back to application .1=Send data. 0=Don't send data used for testing only  . There just incase setup engineer forgets to turn the data dump off
    Audio2(1, 2, 0);  //,"AC10:32");
    PrintAppData();   // 20241209: Add this and TransmitData() here so that they only write to app when requested ("refresh")
    TransmitData();
    PrintConfigData(); // Send area data back to the app for user to see
}
void cmd10_btDisconnected()
{ // App telling the micro that the bluetooth is disconnected
    BT_ConnectedFlag = 0;
    Audio2(1, 2, 0); //,"AC10:64");
}

void Cmd10()
{
    switch (Instruction)
    {
    case 0:
        cmd10_running();
        printToBT(9, 0); // 20240922
        break;
    case 1:
        cmd10_programming();
        break;
    case 4:
        cmd10_restart();
        break;
    case 8:
        cmd10_lightSensor();
        break;
    case 16:
        cmd10_lightTrigger();
        break;
    case 32:
        cmd10_btConnected();
        break;
    case 64:
        cmd10_btDisconnected();
        break;
    }
}

// void Cmd10()
// { // Setup/Run mode selection. Delete all map points. Cold restart
//     uint8_t A;
//     A = Instruction;

//     if (A == 0b00000000)
//     {                                                            // Run mode   <10:0>
//         eeprom_update_byte(&EramMapTotalPoints, MapTotalPoints); // Setting to run mode indicates completion of setup so store MapTotalPoints.
//         WarnLaserOnOnce = 1;                                     // Enable laser warning when Run Mode button is pressed
//         PrevSetupModeFlag = SetupModeFlag;
//         SetupModeFlag = 0;
//         printToBT(9, 0); // 20240922
//         // PORTE |= ~(1 << BUZZER); // Set BUZZER pin to HIGH
//     }

//     if (A == 0b00000001)
//     {                        // Program Mode  <10:1>
//         WarnLaserOnOnce = 1; // Enable laser warning when Program Mode button is pressed
//         PrevSetupModeFlag = SetupModeFlag;
//         SetupModeFlag = 1;
//         printToBT(9, 1); // 20240922
//         // Audio2(2,2,2);//,"AC10:1");
//         ProgrammingMode(); // Home machine ready for programming in points
//     }

//     if (A == 0b00000100)
//     {                    // Full cold restart of device <10:4>
//         Audio2(1, 2, 0); //,"AC10:4");
//         setupWatchdog();
//         _delay_ms(1000);
//     }

//     if (A == 0b00001000)
//     {                    // Setup light sensor mode    <10:8>
//         Audio2(1, 2, 0); //,"AC10:8");
//         SetupModeFlag = 2;
//         _delay_ms(1000);
//     }
//     // --Setup light sensor mode---
//     if (A == 0b00010000)
//     { // Store current value to default light trigger value    <10:16>
//         if (SetupModeFlag == 2 && LightLevel < 100)
//         {
//             eeprom_update_byte(&EramFactoryLightTripLevel, LightLevel);
//             FactoryLightTripLevel = LightLevel;
//             Audio2(1, 2, 0); //,"AC10:16");
//             _delay_ms(1);
//         }
//     }

//     if (A == 0b00100000)
//     { // App telling the micro that the bluetooth is connected
//         BT_ConnectedFlag = 1;
//         // 20241209.  Why would SendDataFlag be zero, as was set here?  Change it to 1.  It's only used in TransmitData() and that's only called from DoHouseKeeping().
//         // 20241209.  Put back to zero.  Dom has added refresh button to app.
//         SendDataFlag = 0; // Output data back to application .1=Send data. 0=Don't send data used for testing only  . There just incase setup engineer forgets to turn the data dump off
//         Audio2(1, 2, 0);  //,"AC10:32");
//         PrintAppData();   // 20241209: Add this and TransmitData() here so that they only write to app when requested ("refresh")
//         TransmitData();
//         PrintConfigData(); // Send area data back to the app for user to see
//     }

//     if (A == 0b01000000)
//     { // App telling the micro that the bluetooth is disconnected
//         BT_ConnectedFlag = 0;
//         Audio2(1, 2, 0); //,"AC10:64");
//     }
// }

void Cmd11()
{
    // Process the Send Diagnostic Data register
    if (Instruction == 0b00000001)
    {
        SendDataFlag ^= 1; // Toggle value of SendDataFlag
        SendSetupDataFlag = 0;
        Audio2(1, 2, 0); //,"AC11:1");
    }

    // Process the full reset flag on next restart
    if (Instruction == 0b00000010)
    {
        eeprom_update_byte(&EramFirstTimeOn, 0xFF);
        Audio2(1, 2, 0); //,"AC11:2");
    }

    if (Instruction == 0b00000100)
    { // Set GyroAddress to true
        eeprom_update_byte(&EramGyroAddress, MPU6050_ADDRESS);
        GyroAddress = MPU6050_ADDRESS;
        GyroOnFlag = true;
        initMPU();
        Audio2(1, 2, 0); //,"AC11:4");
    }

    if (Instruction == 0b00001000)
    { // Set GyroAddress to false
        eeprom_update_byte(&EramGyroAddress, MPU6000_ADDRESS);
        GyroAddress = MPU6000_ADDRESS;
        GyroOnFlag = true;
        initMPU();
        Audio2(1, 2, 0); //,"AC11:8");
    }
}

void Cmd12()
{
    // Store the accelerometer trip point
    eeprom_update_word(&EramAccelTripPoint, Instruction);
    AccelTripPoint = Instruction;
    Audio2(3, 4, 2); //,"AC12");
}

void Cmd13()
{
    eeprom_update_byte(&EramOperationMode, Instruction);
    OperationMode = Instruction;
    Audio2(1, 2, 0, "AC13");
}

void Cmd14()
{ // 20240522: Delete a map point.  It's always the last map point.
    // Note that with C arrays being zero based, the point to be deleted is MapTotalPoints -1
    EramPos PosType;
    uint16_t OpZone; // Operation zone number from the Y encoded data

    // Read the map location from EEPROM into Mytype
    uint16_t eepromAddress = (uint16_t)&EramPositions[MapTotalPoints - 1];
    eeprom_read_block(&PosType, (const void *)eepromAddress, sizeof(EramPos));
    // eeprom_read_block(&PosType, &EramPositions[MapTotalPoints - 1], sizeof(EramPos));
    // Get the Y map location and operation data
    OpZone = PosType.EramY;
    OpZone >>= 12; // Bit shift the data to the correct format with a value of 1 to 4 Operating Zones

    // Decode the binary data
    switch (OpZone)
    {
    case 1:
        OpZone = 1;
        break;
    case 2:
        OpZone = 2;
        break;
    case 4:
        OpZone = 3;
        break;
    case 8:
        OpZone = 4;
        break;
    }

    printToBT(19, OpZone);
    MapTotalPoints -= 1;
    eeprom_update_byte(&EramMapTotalPoints, MapTotalPoints);
    Audio2(1, 2, 0); //,"AC14");
}

// void Cmd15(){
//     // eeprom_update_word(&EramMinimumLaserPower, Instruction);
//     // MinimumLaserPower = Instruction;
// }

void Cmd16()
{ // 20241202 Cmd16 was previously for speedzone 1. As speedzones are no longer used, use the speedzone 1 function for this general parameter.
    uint16_t NewInstruction = ReScale(Instruction, OLD_SPEED_ZONE_MIN, OLD_SPEED_ZONE_MAX, SPEED_SCALE_MIN, SPEED_SCALE_MAX);
    eeprom_update_byte(&EramSpeedScale, NewInstruction);
    SpeedScale = NewInstruction;
    Audio2(1, 2, 0); //,"AC22");
}

void Cmd17()
{ // Previously SpeedZone 2 (with 1st numbered 1)
    uint16_t NewInstruction = ReScale(Instruction, OLD_SPEED_ZONE_MIN, OLD_SPEED_ZONE_MAX, NBR_RND_MIN, NBR_RND_MAX);
    eeprom_update_byte(&Eram_Nbr_Rnd_Pts, NewInstruction);
    Nbr_Rnd_Pts = NewInstruction;
}
void Cmd18()
{
    uint16_t NewInstruction = ReScale(Instruction, OLD_SPEED_ZONE_MIN, OLD_SPEED_ZONE_MAX, TILT_SEP_MIN, TILT_SEP_MAX);
    eeprom_update_byte(&Eram_Tilt_Sep, NewInstruction);
    Tilt_Sep = NewInstruction;
}

void Cmd19()
{ // 20241231.  Change this to adjust user laser power.
    uint16_t NewInstruction = ReScale(Instruction, OLD_SPEED_ZONE_MIN, OLD_SPEED_ZONE_MAX, 0, DEF_MAX_LASER_POWER);
    UserLaserPower = NewInstruction;
    // nbrWigglyPts not stored to EEPROM.  Just for testing.  Set to 0 when declared .
}
void Cmd20()
{
    uint16_t NewInstruction = ReScale(Instruction, OLD_SPEED_ZONE_MIN, OLD_SPEED_ZONE_MAX, LASER_HT_MIN, LASER_HT_MAX);
    eeprom_update_byte(&EramLaserHt, NewInstruction);
    LaserHt = NewInstruction;
}

void Cmd21()
{ // Need to listen for this from app.  Was previously something to do with map zones.
  // eeprom_update_word(&EramResetSeconds, Instruction);
  // ResetSeconds = Instruction;
  // Audio2(1,2,0);//,"AC30");
}

void Cmd24()
{ // 20241212 Report vertices to app
    for (uint8_t zn = 1; zn <= NBR_ZONES; zn++)
    {
        LoadZoneMap(zn, true);
    }
}

void Cmd30()
{
    eeprom_update_word(&EramResetSeconds, Instruction);
    ResetSeconds = Instruction;
    Audio2(1, 2, 0); //,"AC30");
}

void Cmd31()
{
    eeprom_update_byte(&EramMapTotalPoints, 0);
    MapTotalPoints = 0;
    // sprintf(debugMsg, "eMTP: %d", eeprom_read_byte(&EramMapTotalPoints));
    // uartPrint(debugMsg);
    Audio2(2, 1, 3); //,"AC31");
}

void Cmd32()
{
    eeprom_update_byte(&EramLaser2OperateFlag, Instruction);
    Laser2OperateFlag = Instruction;
    Audio2(1, 2, 0); //,"AC32");
}

void Cmd33()
{
    eeprom_update_byte(&EramLaser2BattTrip, Instruction);
    Laser2BattTrip = Instruction;
    Audio2(1, 2, 0); //,"AC33");
}

void Cmd34()
{
    eeprom_update_byte(&EramLaser2TempTrip, Instruction);
    Laser2TempTrip = Instruction;
    Audio2(1, 2, 0); //,"AC34");
}

void Cmd35()
{
    eeprom_update_byte(&EramUserLightTripLevel, Instruction);
    UserLightTripLevel = Instruction;
    Audio2(1, 2, 0); //,"AC35");
}

void Cmd36()
{
    eeprom_update_byte(&EramLightTriggerOperation, Instruction);
    LightTriggerOperation = Instruction;
    Audio2(1, 2, 0); //,"AC36");
}

void Cmd37()
{
    uint16_t HexResult;
    char Result[5];

    SetLaserVoltage(0);
    _delay_ms(1000);

    GetLightLevel();
    HexResult = LightLevel;
    sprintf(Result, "%X", HexResult);
    printToBT(26, HexResult);
    _delay_ms(50);
    // Clear serial input buffer
    Audio2(1, 2, 0); //,"AC37");
}

void Cmd38()
{
    eeprom_update_byte(&EramFactoryLightTripLevel, Instruction);
    FactoryLightTripLevel = Instruction;
    Audio2(1, 2, 0); //,"AC38");
}

void Cmd39()
{
    received39 = true;
}

void Cmd46()
{ //
    eeprom_update_word(&Eram_Step_Rate_Min, Instruction);
    Step_Rate_Min = Instruction;
}
void Cmd47()
{ //
    eeprom_update_word(&Eram_Step_Rate_Max, Instruction);
    Step_Rate_Max = Instruction;
}
void Cmd48()
{ //
    eeprom_update_byte(&Eram_Rho_Min, Instruction);
    Rho_Min = Instruction;
}
void Cmd49()
{ //
    eeprom_update_byte(&Eram_Rho_Max, Instruction);
    Rho_Max = Instruction;
}
void Cmd50()
{ //
    eeprom_update_byte(&Eram_Nbr_Rnd_Pts, Instruction);
    Nbr_Rnd_Pts = Instruction;
}

#ifdef NEW_APP
void Cmd54()
{
    if ((Instruction < 0x10) && (Instruction >= 0))
    {
        eeprom_update_byte(&EramActiveMapZones, Instruction);
        ActiveMapZones = Instruction;
    }
    // printToBT(54, 255);
    // else
    //     printToBT(51, ActiveMapZones);
}

void Cmd52()
{
    if ((Instruction < 0x10) && (Instruction >= 0))
    {
        eeprom_update_byte(&EramActivePatterns, Instruction);
        ActivePatterns = Instruction;
    }
    // printToBT(52, 255);
    // else
    //     printToBT(52, ActivePatterns);
}
void Cmd53()
{
    sendStatusData();
}

void CmdStorePts(bool test)
{
    static uint16_t outFunctionTime = 0;
    static uint16_t inFunctionTime = 0;
    static uint16_t cnt;

    if (!test)
    {
        uint8_t z = 0;
        uint16_t OpZone = 0;
        uint8_t index = 0;
        uint16_t eepromAddress = 0;
        static uint16_t lastMillisStart = 0;
        static uint16_t lastMillisEnd = 0;

        index = (Command - FST_STORE_PT_INDEX) % MAX_NBR_MAP_PTS;
        if (index == 0)
        {
            cnt = 0;
            inFunctionTime = 0;
            outFunctionTime = 0;
            lastMillisStart = millis();
            lastMillisEnd = millis();
        }
        cnt++;
        lastMillisStart = millis();
        outFunctionTime = millis() - lastMillisEnd;

        if (Command < FST_STORE_PT_INDEX + MAX_NBR_MAP_PTS) // Instruction is zone
        {
            z = ((Instruction + 1) & 0x0F); // Store the zone in the upper 4 bits
            OpZone = 1 << (12 + Instruction);
            eepromAddress = (uint16_t)&EramPositions[index].EramY;
            eeprom_update_word((uint16_t *)eepromAddress, OpZone);
        }
        else if (Command < FST_STORE_PT_INDEX + 2 * MAX_NBR_MAP_PTS) // Instruction is Y
        {
            OpZone = eeprom_read_word((uint16_t *)((uintptr_t)&EramPositions[index].EramY));
            eepromAddress = (uint16_t)&EramPositions[index].EramY;

            // Correctly handle negative values and mask the lower 12 bits
            int16_t y_value = static_cast<int16_t>(Instruction);
            uint16_t masked_y_value = static_cast<uint16_t>(y_value) & 0x0FFF;

            eeprom_update_word((uint16_t *)eepromAddress, (OpZone & 0xF000) | masked_y_value);
        }
        else if (Command < FST_STORE_PT_INDEX + 3 * MAX_NBR_MAP_PTS) // Instruction is X
        {
            eepromAddress = (uint16_t)&EramPositions[index].EramX;
            eeprom_update_word((uint16_t *)eepromAddress, Instruction);
        }
        // uint8_t MapPointNumber = static_cast<uint8_t>(index);
        // MapTotalPoints = MapPointNumber + 1;
        MapTotalPoints = index + 1;
        _delay_ms(REPORT_VERTICES_DELAY);
        printToBT(61, 10);

        inFunctionTime = millis() - lastMillisStart;
        lastMillisEnd = millis();
    }
    else
    {
#ifdef TEST_FDP
        sprintf(debugMsg, " MTP: %d, count: %d,inFunctionTime: %d, outFunctionTime: %d", MapTotalPoints, cnt, inFunctionTime, outFunctionTime);
        uartPrint(debugMsg);
#endif
    }
}

void GoToMapIndex()
{
    // uint8_t z;
    uint16_t eepromAddress;
    // getMapPtCounts(false); // Load zone data (count of vertices by zone) from eeprom to MapCount array
    X = eeprom_read_word(&EramPositions[Instruction].EramX);
    Y = eeprom_read_word(&EramPositions[Instruction].EramY) & 0x0FFF;
    setupTimer1();
    ProcessCoordinates();
}

void ReportVertices()
{
    getMapPtCounts(false);
    for (uint8_t i = 0; i < MapTotalPoints; i++)
    {
        uint8_t n = 0;
        uint8_t z = 0;
        uint16_t eepromY = eeprom_read_word(&EramPositions[i].EramY);
        // z = (eepromY >> 12) & 0x0F - 1;      // Extract the zone from the upper 4 bits
        z = GetZone(i) - 1;                  // GetZone() returns 1 based index of zone
        uint16_t y_value = eepromY & 0x0FFF; // Extract the Y value from the lower 12 bits

        // Handle negative Y values if necessary
        if (y_value & 0x0800)
        {
            y_value |= 0xF000; // Sign-extend to 16 bits
        }

        if (z > 0)
            n = i - MapCount[1][z - 1]; // MapCount[1][z] is the cumulative number of vertices (not including doubling the first vertex) to zone z.
        else
            n = i;

        sprintf(debugMsg, "<61: i: %d, z: %d, n: %d, X: %d, Y: %d>", i, z, n, eeprom_read_word(&EramPositions[i].EramX), static_cast<int16_t>(y_value));
        uartPrint(debugMsg);
        _delay_ms(REPORT_VERTICES_DELAY);
    }
    uartPrint("<61:>");
}

void setProperty(FieldDeviceProperty property, uint8_t value)
{
    if (value == CANT_SET_PROPERTY)
    {
        sprintf(debugMsg, "Property not settable", property);
        uartPrint(debugMsg);
    }
    else
    {
    }
}

void handleSetPropertyRequest(FieldDeviceProperty property, uint8_t value)
{
#ifdef TEST_FDP
    sprintf(debugMsg, "Received by hSPR. Prop: %d, val: %d", property, value);
    uartPrint(debugMsg);
#endif
    uint8_t currentValue = 0;
    switch (property)
    {
    case FieldDeviceProperty::batteryVoltAdc:
        setProperty(property, CANT_SET_PROPERTY);
        break;
    case FieldDeviceProperty::timeMode:
        currentValue = eeprom_read_byte(&EramLightTriggerOperation);
        if (value != currentValue)
        {
            eeprom_update_byte(&EramLightTriggerOperation, value);
        }
        LightTriggerOperation = value;
        break;
    case FieldDeviceProperty::tripodHeight:
        currentValue = eeprom_read_byte(&EramLaserHt);
        if (value != currentValue)
        {
            eeprom_update_byte(&EramLaserHt, value);
        }
        LaserHt = value;
        break;
    case FieldDeviceProperty::lineSeparation:
        currentValue = eeprom_read_byte(&Eram_Tilt_Sep);
        if (value != currentValue)
        {
            eeprom_update_byte(&Eram_Tilt_Sep, value);
        }
        LaserHt = value;
        break;
    case FieldDeviceProperty::linesPerPattern:
        currentValue = eeprom_read_byte(&Eram_Nbr_Rnd_Pts);
        if (value != currentValue)
        {
            eeprom_update_byte(&Eram_Nbr_Rnd_Pts, value);
        }
        Nbr_Rnd_Pts = value;
        break;
    case FieldDeviceProperty::activePatterns:
        currentValue = eeprom_read_byte(&EramActivePatterns);
        if (value != currentValue)
        {
            eeprom_update_byte(&EramActivePatterns, value);
        }
        ActivePatterns = value;
        break;
    case FieldDeviceProperty::maxLaserPower:
        Instruction = value;
        Cmd4();
        break;
    case FieldDeviceProperty::laserTemperature:
        setProperty(property, CANT_SET_PROPERTY);
        break;
    case FieldDeviceProperty::randomizeSpeed:
        setProperty(property, CANT_SET_PROPERTY);
        break;
    case FieldDeviceProperty::speedScale:
        uint8_t newValue = ReScale(value, OLD_SPEED_ZONE_MIN, OLD_SPEED_ZONE_MAX, SPEED_SCALE_MIN, SPEED_SCALE_MAX);
        currentValue = eeprom_read_byte(&EramSpeedScale);
        if (newValue != currentValue)
        {
            eeprom_update_byte(&EramSpeedScale, value);
        }
        SpeedScale = newValue;
        break;
    case FieldDeviceProperty::lightSensorReading:
        setProperty(property, CANT_SET_PROPERTY);
        break;
    case FieldDeviceProperty::deviceMode:
        switch (value)
        {
        case running:
            cmd10_running();
            break;
        case programming:
            cmd10_programming();
            break;
        case restart:
            cmd10_restart();
            break;
        case lightSensor:
            cmd10_lightSensor();
            break;
        case lightTrigger:
            cmd10_lightTrigger();
            break;
        case btConnected:
            cmd10_btConnected();
            break;
        case btDisconnected:
            cmd10_btDisconnected();
            break;
        }
        break;
    case FieldDeviceProperty::currentZoneRunning:
        setProperty(property, CANT_SET_PROPERTY);
        break;
    case FieldDeviceProperty::currentPatternRunning:
        setProperty(property, CANT_SET_PROPERTY);
        break;
    case FieldDeviceProperty::microMajor:
        currentValue = eeprom_read_byte(&EramMicroMajor);
        if (newValue != currentValue)
        {
            eeprom_update_byte(&EramMicroMajor, value);
        }
        MicroMajor = newValue;
        break;
    case FieldDeviceProperty::microMinor:
        currentValue = eeprom_read_byte(&EramMicroMinor);
        if (newValue != currentValue)
        {
            eeprom_update_byte(&EramMicroMinor, value);
        }
        MicroMinor = newValue;
        break;
    }
}

#endif
#ifndef FIELD_DEVICE_PROPERTY_H
#define FIELD_DEVICE_PROPERTY_H

enum FieldDeviceProperty
{
    // microMajorVersion,  // uint
    // microMinorVersion,  // uint
    batteryVoltAdc,     // uint
    timeMode,           // TimeMode
    beamMode,           // BeamMode
    locationMode,       // LocationMode
    tripodHeight,       // uint in decimeters
    lineSeparation,     // [0, 100] percent
    linesPerPattern,    // int count
    activeMapZones,     // [0, 8) compressed bit array
    activePatterns,     // [0, 8) compressed bit array
    maxLaserPower,      // uint
    userLaserPower,     // [0, 100] percent
    currentLaserPower,  // uint ? percent ?
    laserTemperature,   // uint
    randomizeSpeed,     // [0, 100] percent
    speedScale,         // [0, 100] percent
    lightSensorReading, // uint
    deviceMode,         //
    currentZoneRunning,
    currentPatternRunning,
    microMajor,
    microMinor,
};

enum TimeMode
{
    always,
    day,
    night
};

enum BeamMode
{
    continuous,
    continuousPulsing
};

enum LocationMode
{
    outdoor,
    indoor,
    indoorInverted
};

enum FieldDeviceMode
{
    running,
    programming,
    // restart,
    lightSensor,
    // lightTrigger,
    // btConnected,
    // btDisconnected
};

#endif // FIELD_DEVICE_PROPERTY_H
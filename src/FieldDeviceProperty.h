#ifndef FIELD_DEVICE_PROPERTY_H
#define FIELD_DEVICE_PROPERTY_H

#include <stdint.h> // Include the header for uint8_t

// enum class FieldDeviceProperty : uint8_t
enum FieldDeviceProperty
{
    batteryVoltAdc,        // 0 uint
    timeMode,              // 1 TimeMode
    beamMode,              // 2 BeamMode
    locationMode,          // 3 LocationMode
    tripodHeight,          // 4 uint in decimeters
    lineSeparation,        // 5 [0, 100] percent
    linesPerPattern,       // 6 int count
    activeMapZones,        // 7 [0, 8) compressed bit array
    activePatterns,        // 8 [0, 8) compressed bit array
    maxLaserPower,         // 9 uint
    userLaserPower,        // 10 [0, 100] percent
    currentLaserPower,     // 11 uint ? percent ?
    laserTemperature,      // 12 uint
    randomizeSpeed,        // 13 [0, 100] percent
    speedScale,            // 14 [0, 100] percent
    lightSensorReading,    // 15 uint
    deviceMode,            // 16
    currentZoneRunning,    // 17
    currentPatternRunning, // 18
    laserID,               // 19
    microMajor,            // 20
    microMinor             // 21
};

// enum class TimeMode : uint8_t
enum TimeMode
{
    always,
    day,
    night
};

// enum class BeamMode : uint8_t
enum BeamMode
{
    continuous,
    continuousPulsing
};

// enum class LocationMode : uint8_t
enum LocationMode
{
    outdoor,
    indoor,
    indoorInverted
};

// enum class FieldDeviceMode : uint8_t
enum FieldDeviceMode
{
    running,
    programming,
    lightSensor
};

#endif // FIELD_DEVICE_PROPERTY_H
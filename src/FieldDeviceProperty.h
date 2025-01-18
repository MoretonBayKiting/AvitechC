#ifndef FIELD_DEVICE_PROPERTY_H
#define FIELD_DEVICE_PROPERTY_H

#include <stdint.h> // Include the header for uint8_t

// enum class FieldDeviceProperty : uint8_t
enum FieldDeviceProperty
{
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
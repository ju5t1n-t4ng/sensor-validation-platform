// sensor_data.h
// Sensor data structures and classes
// This headerfile defines the SensorData class. Every sensor in this project produces a SensorData object.
// Updated Phase 1: added fault tracking and sensor health  

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

// Added a sensor health status
enum SensorHealth {
    SENSOR_OK,
    SENSOR_INVALID,
    SENSOR_STALE,
    SENSOR_DISCONNECTED,
    SENSOR_UNINITIALIZED
};

struct SensorData {
    float value;                // The sensor reading value
    float lastValidValue;       // The last valid reading value
    unsigned long timestamp;    // The timestamp of each reading (ms since boot)
    unsigned long lastUpdate;   // Last time value was changed
    bool valid;                 // Whether the reading is valid
    SensorHealth health;        // The health status of the sensor
    int faultCount;             // Count of consecutive failed validations
};  

#endif  
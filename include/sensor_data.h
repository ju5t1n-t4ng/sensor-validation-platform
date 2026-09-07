// sensor_data.h
// Sensor data structures and classes
// This headerfile defines the SensorData class. Every sensor in this project produces a SensorData object.

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

struct SensorData {
    float value;                // The sensor reading value
    unsigned long timestamp;    // The timestamp of each reading (ms since boot)
    bool valid;                 // Whether the reading is valid
};

// Add a sensor health status - unfinished, expanded in Phase 2
enum SensorHealth {
    SENSOR_OK,
    SENSOR_INVALID,
    SENSOR_STALE,
    SENSOR_DISCONNECTED
};

#endif
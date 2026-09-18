// sensor_data.h
// Phase 3: added fault reason tracking and recovery state

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

// System level state
enum SystemState {
    STATE_NORMAL,
    STATE_WARNING,
    STATE_CRITICAL,
    STATE_SENSOR_FAULT,
    STATE_SAFE
};

// Reason the system entered SAFE, logged at transition
enum SafeReason {
    SAFE_NONE,
    SAFE_CRITICAL_TIMEOUT, // Sustained critical condition
    SAFE_SENSOR_FAULT, // fault persisted
    SAFE_WATCHDOG //Stalled main loop
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
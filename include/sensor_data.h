// sensor_data.h
// Phase 4: added timing metrics for performance analysis

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

enum SensorHealth {
    SENSOR_OK,
    SENSOR_INVALID,
    SENSOR_STALE,
    SENSOR_DISCONNECTED,
    SENSOR_UNINITIALIZED
};

enum SystemState {
    STATE_NORMAL,
    STATE_WARNING,
    STATE_CRITICAL,
    STATE_SENSOR_FAULT,
    STATE_SAFE
};

enum SafeReason {
    SAFE_NONE,
    SAFE_CRITICAL_TIMEOUT,
    SAFE_SENSOR_FAULT,
    SAFE_WATCHDOG
};

struct SensorData {
    float value;                
    float lastValidValue;       
    unsigned long timestamp;    
    unsigned long lastUpdate;
    bool valid;               
    SensorHealth health;        
    int faultCount;             
};  

// Timing metrics — populated by each task for Phase 5 analysis
struct TaskMetrics {
    unsigned long lastExecutionTime;  // How long the task took (ms)
    unsigned long maxExecutionTime;   // Worst case seen so far
    unsigned long missedDeadlines;    // How many times deadline was exceeded
    unsigned long deadline;           // Target deadline (ms)
};

#endif  
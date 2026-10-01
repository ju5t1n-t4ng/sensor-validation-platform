// Real-Time Sensor Acquisition, Validation & Fault-Tolerant Control Platform
// Phase 3 - FreeRTOS real-time scheduling

// Task Architechture: 
// SensorTask ──► ValidationTask ──► ControlTask
//                               └──► TelemetryTask
// WatchdogTask runs independently

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "sensor_data.h"

// Pin Definitions
const int POT_PIN = 34;
const int LED_PIN = 26;
const int MOTOR_PIN = 25;

// Validation Thresholds 
const float TEMP_MIN = -40.0;
const float TEMP_MAX = 80.0;
const float PRESSURE_MIN = 300.0;
const float PRESSURE_MAX = 1100.0;
const float TEMP_MAX_DELTA = 10.0;   // Real hardware value: ~0.5°C per 100ms
const float PRESSURE_MAX_DELTA = 50.0;  // Real hardware value: ~2.0 hPa per 100ms
const int FAULT_THRESHOLD = 3;
const unsigned long STALE_TIMEOUT = 2000;

// Operational Thresholds
const float TEMP_WARNING = 35.0;
const float TEMP_CRITICAL = 60.0;
const float PRESSURE_WARNING = 1005.0;
const float PRESSURE_CRITICAL = 980.0;
const int ANALOG_WARNING = 3000;
const int ANALOG_CRITICAL = 3800;

// Timing
const unsigned long SAFE_ENTRY_DELAY = 2000;
const unsigned long RECOVERY_DURATION = 3000;

// Task Deadlines
const unsigned long SENSOR_DEADLINE = 10;
const unsigned long CONTROL_DEADLINE = 5;
const unsigned long TELEMETRY_DEADLINE = 100;
const unsigned long WATCHDOG_DEADLINE = 100;

// Sensor Objects
Adafruit_BMP085 bmp;    // BMP180 sensor object uses the BMP085 library

// Queues
// Pass data between task safely
QueueHandle_t sensorToValidation; // Raw sensor data to validation
QueueHandle_t validationToControl;
QueueHandle_t validationToTelemetry;

// Shared System State
// Protected by mutex - multipls tasks read/write state
SemaphoreHandle_t stateMutex;
SystemState currentState = STATE_NORMAL;
SystemState previousState = STATE_NORMAL;
SafeReason safeReason = SAFE_NONE;
bool inRecovery = false;

// Timing Metrics
TaskMetrics sensorMetrics = {0, 0, 0, SENSOR_DEADLINE};
TaskMetrics controlMetrics = {0, 0, 0, CONTROL_DEADLINE};
TaskMetrics telemetryMetrics = {0, 0, 0, TELEMETRY_DEADLINE};

// Watchdog
SemaphoreHandle_t watchdogSemaphore;

// Shared sensor data
SemaphoreHandle_t sensorMutex;
SensorData temperatureData;
SensorData pressureData;
SensorData analogData;

// Timing State
unsigned long criticalEntryTime = 0;
unsigned long faultEntryTime = 0;
unsigned long recoveryStartTime = 0;

// Function Declarations
void validateSensor(SensorData &sensor, float minVal, float maxVal, float maxDelta);
SystemState determineState();
bool checkRecoveryConditions();
void enterSafe(SafeReason reason);
String healthStr(SensorHealth h);
String stateStr(SystemState s);
String safeReasonStr(SafeReason r);

// Task Declarations
void SensorTask(void *pvParameters);
void ValidationTask(void *pvParameters);
void ControlTask(void *pvParameters);
void TelemetryTask(void *pvParameters);
void WatchdogTask(void *pvParameters);

// -------------------------------------------------------------------------------------------------------
void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    Serial.begin(115200);

    Serial.println("RT Sensor Platform - Phase 4");
    Serial.println("Initializing...");

    if (!bmp.begin(0x77)) {
        Serial.println("ERROR: BMP180 sensor not detected.");
        while (1);
    }

    // Seed from first real read
    float initialTemp = bmp.readTemperature();
    float initialPressure = bmp.readPressure() / 100.0;

    temperatureData = {initialTemp, initialTemp, millis(), millis(), false, SENSOR_DISCONNECTED, 0};
    pressureData = {initialPressure, initialPressure, millis(), millis(), false, SENSOR_DISCONNECTED, 0};
    analogData = {0, 0, 0, 0, false, SENSOR_UNINITIALIZED, 0};

    Serial.print("Initial temp: "); Serial.println(initialTemp);
    Serial.print("Initial pressure: "); Serial.println(initialPressure);

    //Create synchronization primitives
    stateMutex = xSemaphoreCreateMutex();
    sensorMutex = xSemaphoreCreateMutex();
    watchdogSemaphore = xSemaphoreCreateBinary();

    // Create queues
    sensorToValidation = xQueueCreate(5, sizeof(SensorData) * 3);
    validationToControl = xQueueCreate(5, sizeof(SensorData) * 3);
    validationToTelemetry = xQueueCreate(5, sizeof(SensorData) * 3);

    // Create tasks
    xTaskCreate(SensorTask, "Sensor", 4096, NULL, 3, NULL);
    xTaskCreate(ValidationTask, "Validation", 4096, NULL, 3, NULL);
    xTaskCreate(ControlTask, "Control", 4096, NULL, 4, NULL);
    xTaskCreate(TelemetryTask, "Telemetry", 4096, NULL, 1, NULL);
    xTaskCreate(WatchdogTask, "Watchdog", 2048, NULL, 2, NULL);

    Serial.println("All tasks created. System ready.");
}

// loop is empty FreeRTOS scheduler takes over
void loop() {
    vTaskDelay(portMAX_DELAY);
}

// Sensor Task
// Reads all sensors at 10ms intervals
// Feeds raw data to validation queue
void SensorTask(void *pvParameters){
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        unsigned long taskStart = millis();
        unsigned long t = millis();

        // Read bmp 180
        float rawTemp = bmp.readTemperature();
        float rawPressure = bmp.readPressure() / 100.0;

        // Update shared sensor data under mutex
        xSemaphoreTake(sensorMutex, portMAX_DELAY);

        if (!isnan(rawTemp)) {
        temperatureData.value = rawTemp;
        temperatureData.timestamp = t;
        }
        if (!isnan(rawPressure)) {
        pressureData.value = rawPressure;
        pressureData.timestamp = t;
        }

        analogData.value = analogRead(POT_PIN);
        analogData.timestamp = t;
        analogData.valid = true;
        analogData.health = SENSOR_OK;

        xSemaphoreGive(sensorMutex);

        // Signal watchdog sensor task is alive
        xSemaphoreGive(watchdogSemaphore);

        // Track execution time
        sensorMetrics.lastExecutionTime = millis() - taskStart;
        if (sensorMetrics.lastExecutionTime > sensorMetrics.maxExecutionTime) {
        sensorMetrics.maxExecutionTime = sensorMetrics.lastExecutionTime;
        }
        if (sensorMetrics.lastExecutionTime > sensorMetrics.deadline) {
        sensorMetrics.missedDeadlines++;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

// Validation Task
// Validates sensor readings in 10ms intervals
// Feeds validated data into control and telemery queues
void ValidationTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        // Reads validated sensor data
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        SensorData localTemp = temperatureData;
        SensorData localPressure = pressureData;
        SensorData localAnalog = analogData;
        xSemaphoreGive(sensorMutex);

        // Validate local copies
        validateSensor(localTemp, TEMP_MIN, TEMP_MAX, TEMP_MAX_DELTA);
        validateSensor(localPressure, PRESSURE_MIN, PRESSURE_MAX, PRESSURE_MAX_DELTA);
        localAnalog.valid = (localAnalog.value >= 0 && localAnalog.value <= 4095);

        // Write validated data back under mutex
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        temperatureData = localTemp;
        pressureData = localPressure;
        analogData = localAnalog;
        xSemaphoreGive(sensorMutex);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

// Control Task
// 5ms interval
// Evaluates system state and controsl acuators
void ControlTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        unsigned long taskStart = millis();

        // Read validated sensor data
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        SensorData localTemp = temperatureData;
        SensorData localPressure = pressureData;
        SensorData localAnalog = analogData;
        xSemaphoreGive(sensorMutex);
        
        // Evaluate state under state mutex
        xSemaphoreTake(stateMutex, portMAX_DELAY);

        previousState = currentState;

        if (currentState == STATE_SAFE) {
            if (checkRecoveryConditions()) {
                currentState = STATE_NORMAL;
                safeReason   = SAFE_NONE;
                inRecovery   = false;
                Serial.println(">>> RECOVERY COMPLETE — returning to NORMAL <<<");
            }
        } else {
            SystemState newState = determineState();

            if (newState == STATE_CRITICAL && previousState != STATE_CRITICAL) {
                criticalEntryTime = millis();
            }
            if (newState == STATE_SENSOR_FAULT && previousState != STATE_SENSOR_FAULT) {
                faultEntryTime = millis();
            }
            if (newState == STATE_CRITICAL && millis() - criticalEntryTime >= SAFE_ENTRY_DELAY) {
                enterSafe(SAFE_CRITICAL_TIMEOUT);
            } else if (newState == STATE_SENSOR_FAULT && millis() - faultEntryTime >= SAFE_ENTRY_DELAY) {
                enterSafe(SAFE_SENSOR_FAULT);
            } else {
                currentState = newState;
            }

            if (currentState != previousState) {
            Serial.print(">>> STATE CHANGE: ");
            Serial.print(stateStr(previousState));
            Serial.print(" → ");
            Serial.println(stateStr(currentState));
            }
        }

        xSemaphoreGive(stateMutex);

        // Control actuators based on state
        switch (currentState) {
            case STATE_NORMAL:
                analogWrite(MOTOR_PIN, 200);
                digitalWrite(LED_PIN, LOW);
                break;

            case STATE_WARNING:
                analogWrite(MOTOR_PIN, 120);
                digitalWrite(LED_PIN, HIGH);
                break;

            case STATE_CRITICAL:
                analogWrite(MOTOR_PIN, 50);
                digitalWrite(LED_PIN, (millis() / 250) % 2);
                break;

            case STATE_SENSOR_FAULT:
                analogWrite(MOTOR_PIN, 0);
                digitalWrite(LED_PIN, (millis() / 100) % 2);
                break;

            case STATE_SAFE:
                analogWrite(MOTOR_PIN, 0);
                digitalWrite(LED_PIN, (millis() / 500) % 2);
                break;
        }

        controlMetrics.lastExecutionTime = millis() - taskStart;
        if (controlMetrics.lastExecutionTime > controlMetrics.maxExecutionTime) {
        controlMetrics.maxExecutionTime = controlMetrics.lastExecutionTime;
        }
        if (controlMetrics.lastExecutionTime > controlMetrics.deadline) {
        controlMetrics.missedDeadlines++;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

// Telemetry Task
// Lowest priority 100ms intervals
// Logs sensor data and task metrics to serial
// Slow serial output cannot block higher priority tasks
void TelemetryTask (void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    Serial.println("Time(ms)  | Temp  | Pressure | Analog | State        | T-Health | FC");
    Serial.println("----------|-------|----------|--------|--------------|----------|----");

    while (true) {
        // Read sensor data
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        SensorData localTemp = temperatureData;
        SensorData localPressure = pressureData;
        SensorData localAnalog = analogData;
        xSemaphoreGive(sensorMutex);

        // Read system state
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        SystemState state = currentState;
        xSemaphoreGive(stateMutex);

        // Log telemetry
        Serial.print(millis());
        Serial.print("ms  | ");
        Serial.print(localTemp.valid ? localTemp.value : -999);
        Serial.print("C  | ");
        Serial.print(localPressure.valid ? localPressure.value : -999);
        Serial.print(" hPa | ");
        Serial.print((int)localAnalog.value);
        Serial.print("   | ");
        Serial.print(stateStr(state));
        Serial.print(" | ");
        Serial.print(healthStr(localTemp.health));
        Serial.print(" | ");
        Serial.println(localTemp.faultCount);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
    }
}

// Watchdog Task
// Monitors sensor task health
// If sensor task stops feeding the semaphore, fires safe state
void WatchdogTask(void *pvParameters) {
    while (true) {
        // Wait for sensor task heartbeat, timeout if not recieved
        bool heartbeat = xSemaphoreTake(watchdogSemaphore, pdMS_TO_TICKS(500));

        if (!heartbeat) {
            Serial.println(">>> WATCHDOG: sensor task stalled");
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            if (currentState != STATE_SAFE) {
                enterSafe(SAFE_WATCHDOG);
            }
            xSemaphoreGive(stateMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Validation Function
void validateSensor(SensorData &sensor, float minVal, float maxVal, float maxDelta) {
    // Check 1: NaN check
    if (isnan(sensor.value)) {
        sensor.valid = false;
        sensor.health = SENSOR_INVALID;
        sensor.faultCount++;
        return;
    }
    // Check 2: Range validation
    if (sensor.value < minVal || sensor.value > maxVal) {
        sensor.valid = false;
        sensor.health = SENSOR_INVALID;
        sensor.faultCount++;
        return;
    }
    // Check 3: Rate of change validation
    float delta = abs(sensor.value - sensor.lastValidValue);
    if (sensor.lastValidValue != 0 && sensor.lastUpdate != 0 && delta > maxDelta) {
        sensor.valid = false;
        sensor.health = SENSOR_INVALID;
        sensor.faultCount++;
        sensor.lastValidValue = sensor.value;
        return;
    }
    // Check 4: Stale data check
    if (sensor.lastUpdate != 0 && millis() - sensor.lastUpdate > STALE_TIMEOUT) {
        sensor.valid = false;
        sensor.health = SENSOR_STALE;
        sensor.faultCount++;
        sensor.lastUpdate = millis();
        return;
    }
    sensor.valid          = true;
    sensor.health         = SENSOR_OK;
    sensor.faultCount     = 0;
    sensor.lastValidValue = sensor.value;
    sensor.lastUpdate     = millis();
}

// Enter Safe State 
void enterSafe(SafeReason reason) {
    currentState = STATE_SAFE;
    safeReason = reason;
    inRecovery = false;
    Serial.print(">>> SAFE STATE ENTERED — reason: ");
    Serial.println(safeReasonStr(reason));
}

// Recovery Conditions Function
bool checkRecoveryConditions() {
    bool sensorsHealthy = (temperatureData.health == SENSOR_OK && pressureData.health == SENSOR_OK);
    bool readingsNormal = (temperatureData.value < TEMP_WARNING && pressureData.value > PRESSURE_WARNING && analogData.value < ANALOG_WARNING);
    bool noActiveFaults = (temperatureData.faultCount == 0 && pressureData.faultCount == 0);
    bool recoveryReady = sensorsHealthy && readingsNormal && noActiveFaults;

    if (recoveryReady && !inRecovery) {
        inRecovery = true;
        recoveryStartTime = millis();
        Serial.println(">>> Recovery conditions met — monitoring...");
    }

    if (!recoveryReady && inRecovery) {
        inRecovery = false;
        Serial.println(">>> Recovery conditions lost — reset timer");
    }

    return (inRecovery && millis() - recoveryStartTime >= RECOVERY_DURATION);
}

// State Determination Function
SystemState determineState() {
    bool sensorFault = (temperatureData.faultCount >= FAULT_THRESHOLD || pressureData.faultCount >= FAULT_THRESHOLD);
    if (sensorFault) return STATE_SENSOR_FAULT;

    if (!temperatureData.valid || !pressureData.valid) return STATE_WARNING;

    if (temperatureData.value > TEMP_CRITICAL || pressureData.value < PRESSURE_CRITICAL || analogData.value > ANALOG_CRITICAL) {
        return STATE_CRITICAL;
    }

    if (temperatureData.value > TEMP_WARNING || pressureData.value < PRESSURE_WARNING || analogData.value > ANALOG_WARNING) {
        return STATE_WARNING;
    }
    
    return STATE_NORMAL;
}

// Helper Functions
String healthStr(SensorHealth h) {
    switch (h) {
        case SENSOR_OK:            return "OK        ";
        case SENSOR_INVALID:       return "INVALID   ";
        case SENSOR_STALE:         return "STALE     ";
        case SENSOR_DISCONNECTED:  return "NO SENSOR ";
        case SENSOR_UNINITIALIZED: return "UNKNOWN   ";
        default:                   return "UNKNOWN   ";
    }
}

String stateStr(SystemState s) {
    switch (s) {
        case STATE_NORMAL:       return "NORMAL      ";
        case STATE_WARNING:      return "WARNING     ";
        case STATE_CRITICAL:     return "CRITICAL    ";
        case STATE_SENSOR_FAULT: return "SENSOR_FAULT";
        case STATE_SAFE:         return "SAFE        ";
        default:                 return "UNKNOWN     ";
    }
}

String safeReasonStr(SafeReason r) {
    switch (r) {
        case SAFE_NONE:             return "NONE";
        case SAFE_CRITICAL_TIMEOUT: return "CRITICAL timeout";
        case SAFE_SENSOR_FAULT:     return "SENSOR_FAULT timeout";
        case SAFE_WATCHDOG:         return "WATCHDOG fired";
        default:                    return "UNKNOWN";
  }
}
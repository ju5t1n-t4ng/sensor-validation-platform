// Real-Time Sensor Acquisition, Validation & Fault-Tolerant Control Platform
// Phase 3 - Fault-tolerant control

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "sensor_data.h"

// Pin Definitions -------------------------------------------------------------------------------------------------------
const int POT_PIN = 34;
const int LED_PIN = 26;
const int MOTOR_PIN = 25;

// Timing Variables -------------------------------------------------------------------------------------------------------
const unsigned long SAMPLE_INTERVAL = 100;
const unsigned long STALE_TIMEOUT = 2000;
const unsigned long SAFE_ENTRY_DELAY  = 2000; // ms in CRITICAL/FAULT before SAFE
const unsigned long RECOVERY_DURATION = 3000; // ms before recovery allowed
const unsigned long WATCHDOG_TIMEOUT = 500; // ms before watchdog activates

unsigned long lastSampleTime = 0;
unsigned long criticalEntryTime = 0;
unsigned long faultEntryTime = 0;
unsigned long recoveryStartTime = 0;
unsigned long lastLoopTime = 0;

// Validation Thresholds -------------------------------------------------------------------------------------------------------
const float TEMP_MIN = -40.0;
const float TEMP_MAX = 80.0;
const float PRESSURE_MIN = 300.0;
const float PRESSURE_MAX = 1100.0;
const float TEMP_MAX_DELTA = 10.0;    // degrees C per sample current values for simulation purposes // Real hardware value: ~0.5°C per 100ms
const float PRESSURE_MAX_DELTA = 50.0;    // hPa per sample // Real hardware value: ~2.0 hPa per 100ms
const int FAULT_THRESHOLD = 3;

// Operational Thresholds ---------------------------------------------------------------------------------------------------------
const float TEMP_WARNING       = 35.0;
const float TEMP_CRITICAL      = 60.0;
const float PRESSURE_WARNING   = 1005.0;  // Below this is low pressure warning
const float PRESSURE_CRITICAL  = 980.0;   // Below this is critical
const int   ANALOG_WARNING     = 3000;
const int   ANALOG_CRITICAL    = 3800;

// Sensor Objects -------------------------------------------------------------------------------------------------------
Adafruit_BMP085 bmp;    // BMP180 sensor object uses the BMP085 library

// Sensor Data -------------------------------------------------------------------------------------------------------
SensorData temperatureData;
SensorData pressureData;
SensorData analogData;

// System State ---------------------------------------------------------------------
SystemState currentState = STATE_NORMAL;
SystemState previousState = STATE_NORMAL;
SafeReason safeReason = SAFE_NONE;
bool inRecovery = false;

// Function Declarations -------------------------------------------------------------------------------------------------------
void acquireSensors();
void validateData();
void validateSensor(SensorData &sensor, float minVal, float maxVal, float maxDelta);
void evaluateState();
SystemState determineState();
bool checkRecoveryConditions();
void enterSafe(SafeReason reason);
void controlActuators();
void logData();
String healthStr(SensorHealth h);
String stateStr(SystemState s);
String safeReasonStr(SafeReason r);

// -------------------------------------------------------------------------------------------------------
void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    Serial.begin(115200);

    Serial.println("RT Sensor Platform - Phase 3");
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

    lastLoopTime = millis();

    Serial.print("Initial temp: ");     Serial.println(initialTemp);
    Serial.print("Initial pressure: "); Serial.println(initialPressure);
    Serial.println("System ready.");
    Serial.println();
    Serial.println("Time(ms)  | Temp  | Pressure | Analog | State        | T-Health | FC");
    Serial.println("----------|-------|----------|--------|--------------|----------|----");
}

// -------------------------------------------------------------------------------------------------------
void loop() {
    unsigned long currentTime = millis();

    // Watchdog check -------------------------------------
    // If loop stalls, this executes on next execution
    if (currentTime - lastLoopTime > WATCHDOG_TIMEOUT) {
        if (currentState != STATE_SAFE) {
            enterSafe (SAFE_WATCHDOG);
        }
    }
    lastLoopTime = currentTime;

    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
        lastSampleTime = currentTime;

        acquireSensors();
        validateData();
        evaluateState();
        controlActuators();
        logData();
    }
}

// Acqusition Part -------------------------------------------------------------------------------------------------------
void acquireSensors() {
    unsigned long timestamp = millis();

    float rawTemp = bmp.readTemperature();
    if (!isnan(rawTemp)) {
        temperatureData.value = rawTemp;
        temperatureData.timestamp = timestamp;
    }

    float rawPressure = bmp.readPressure() / 100.0;
    if (!isnan(rawPressure)) {
        pressureData.value = rawPressure;
        pressureData.timestamp = timestamp;
    }

    analogData.value = analogRead(POT_PIN);
    analogData.timestamp = timestamp;
    analogData.valid = true;
    analogData.health = SENSOR_OK;
}

// Validation Part -------------------------------------------------------------------------------------------------------
void validateData() {
    validateSensor(temperatureData, TEMP_MIN, TEMP_MAX, TEMP_MAX_DELTA);
    validateSensor(pressureData, PRESSURE_MIN, PRESSURE_MAX, PRESSURE_MAX_DELTA);
    analogData.valid = (analogData.value >= 0 && analogData.value <= 4095);
}

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

// State Evaluation -------------------------------------------------------------------------------------------------------
void evaluateState() {
    previousState = currentState;

    // SAFE is a latch, only exists via reset
    if (currentState == STATE_SAFE) {
        if (checkRecoveryConditions()) {
            currentState = STATE_NORMAL;
            safeReason = SAFE_NONE;
            inRecovery = false;
            Serial.println(">>> RECOVERY COMPLETE — returning to NORMAL <<<");
        }
        return;
    }

    SystemState newState = determineState();

    // Track CRITICAL entry time
    if (newState == STATE_CRITICAL && previousState != STATE_CRITICAL) {
        criticalEntryTime = millis();
    }

    // Track SENSOR_FAULT entry time
    if (newState == STATE_SENSOR_FAULT && previousState != STATE_SENSOR_FAULT) {
        faultEntryTime = millis();
    }

    // Enter SAFE from CRITICAL after delay
    if (newState == STATE_CRITICAL && millis() - criticalEntryTime >= SAFE_ENTRY_DELAY) {
        enterSafe(SAFE_CRITICAL_TIMEOUT);
        return;
    }

    // Enter SAFE from SENSOR_FAULT after delay
    if (newState == STATE_SENSOR_FAULT && millis() - faultEntryTime >= SAFE_ENTRY_DELAY) {
        enterSafe(SAFE_SENSOR_FAULT);
        return;
    }

    currentState = newState;

    // Log state transitions
    if (currentState != previousState) {
        Serial.print(">>> STATE CHANGE: ");
        Serial.print(stateStr(previousState));
        Serial.print(" → ");
        Serial.println(stateStr(currentState));
    }
}

// Enter Safe State ---------------------------------------------------------------------------------------------
void enterSafe(SafeReason reason) {
    currentState = STATE_SAFE;
    safeReason = reason;
    inRecovery = false;

    Serial.print(">>> SAFE STATE ENTERED — reason: ");
    Serial.println(safeReasonStr(reason));
}

// Recovery Condtitions ------------------------------------------------------------------------------------------
// System must observe stable conditions for RECOVERY_DURATION before being allowed to exit SAFE

bool checkRecoveryConditions() {
    bool sensorsHealthy = (temperatureData.health == SENSOR_OK && pressureData.health == SENSOR_OK);
    bool readingsNormal = (temperatureData.value < TEMP_WARNING && pressureData.value > PRESSURE_WARNING && analogData.value < ANALOG_WARNING);
    bool noActiveFaults = (temperatureData.faultCount == 0 && pressureData.faultCount == 0);
    bool recoveryReady = sensorsHealthy && readingsNormal && noActiveFaults;

    // Start recovery timer when conditions first met
    if (recoveryReady && !inRecovery) {
        inRecovery = true;
        recoveryStartTime = millis();
        Serial.println(">>> Recovery conditions met — monitoring...");
    }

    // Reset if conditions lost during recovery window
    if (!recoveryReady && inRecovery) {
        inRecovery = false;
        Serial.println(">>> Recovery conditions lost — reset timer");
    }

    // Recovery complete if conditions held for full duration
    return (inRecovery && millis() - recoveryStartTime >= RECOVERY_DURATION);
}

// State Determination ---------------------------------------------------------------------------------------------------
SystemState determineState() {
    // SENSOR_FAULT
    bool sensorFault = (temperatureData.faultCount >= FAULT_THRESHOLD || pressureData.faultCount >= FAULT_THRESHOLD);
    if (sensorFault) return STATE_SENSOR_FAULT;

    if (!temperatureData.valid || !pressureData.valid) return STATE_WARNING;

    // CRITICAL
    if (temperatureData.value > TEMP_CRITICAL || pressureData.value < PRESSURE_CRITICAL || analogData.value > ANALOG_CRITICAL) {
        return STATE_CRITICAL;
    }

    //WARNING
    if (temperatureData.value > TEMP_WARNING || pressureData.value < PRESSURE_WARNING || analogData.value > ANALOG_WARNING) {
        return STATE_WARNING;
    }
    
    return STATE_NORMAL;
}

// Actuator Control -------------------------------------------------------------------------------------------------------
// All actuator commands flow through system state
// No sensor value ever directly commands an actuator

void controlActuators() {
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
}

// Helper Functions ----------------------------------------------------------------------------
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


// Telemetry Logging -------------------------------------------------------------------------------------------------------
void logData() {
  Serial.print(millis());
  Serial.print("ms  | ");
  Serial.print(temperatureData.valid ? temperatureData.value : -999);
  Serial.print("C  | ");
  Serial.print(pressureData.valid ? pressureData.value : -999);
  Serial.print(" hPa | ");
  Serial.print((int)analogData.value);
  Serial.print("   | ");
  Serial.print(stateStr(currentState));
  Serial.print(" | ");
  Serial.print(healthStr(temperatureData.health));
  Serial.print(" | ");
  Serial.println(temperatureData.faultCount);
}
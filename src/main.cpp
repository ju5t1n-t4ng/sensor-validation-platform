// Real-Time Sensor Acquisition, Validation & Fault-Tolerant Control Platform
// This project is a real-time sensor acquisition, validation, and fault-tolerant control platform. It is designed to acquire data from various sensors, validate the readings, and provide fault-tolerant control mechanisms based on the sensor data.
// Phase 2 — System state machine

//Architecture: loop() -> acquireSensors() -> validateData() -> evaluateState() -> controlActuators()

#include <Wire.h>
#include <Adafruit_BMP085.h>   // BMP180 uses the BMP085 library
#include "sensor_data.h"

// Pin Definitions -------------------------------------------------------------------------------------------------------
const int POT_PIN = 34;
const int LED_PIN = 26;
const int MOTOR_PIN = 25;

// Timing Variables -------------------------------------------------------------------------------------------------------
const unsigned long SAMPLE_INTERVAL = 100;    // ms between sensor readings
const unsigned long STALE_TIMEOUT = 2000;    // ms before sensor data is stale
const unsigned long SAFE_ENTRY_DELAY  = 2000;  // ms in CRITICAL before SAFE
unsigned long lastSampleTime = 0;
unsigned long criticalEntryTime = 0;    // When CRITICAL was entered

// Validation Thresholds -------------------------------------------------------------------------------------------------------
// BMP180 Sensor operating limits
const float TEMP_MIN = -40.0;
const float TEMP_MAX = 80.0;
const float PRESSURE_MIN = 300.0;
const float PRESSURE_MAX = 1100.0;

// Maximum realistic rate of change per 100ms sample interval
const float TEMP_MAX_DELTA = 10.0;    // degrees C per sample current values for simulation purposes // Real hardware value: ~0.5°C per 100ms
const float PRESSURE_MAX_DELTA = 50.0;    // hPa per sample // Real hardware value: ~2.0 hPa per 100ms

// Fault counter threshold, consecutive failed validations before fault declared
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

// Function Declarations -------------------------------------------------------------------------------------------------------
void acquireSensors();
void validateData();
void validateSensor(SensorData &sensor, float minVal, float maxVal, float maxDelta);
void evaluateState();
SystemState determineState();
void controlActuators();
void logData();
String healthStr(SensorHealth h);
String stateStr(SystemState s);

// -------------------------------------------------------------------------------------------------------
void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    Serial.begin(115200);

    Serial.println("RT Sensor Platform - Phase 2");
    Serial.println("Initializing...");

    if (!bmp.begin(0x77)) {
        Serial.println("ERROR: BMP180 sensor not detected.");
        while (1);
    }

    // Seed intial values from first sensor reading
    // Prevents false rate of change faults at startup
    float initialTemp = bmp.readTemperature();
    float initialPress = bmp.readPressure() / 100.0;

    temperatureData = {initialTemp, initialTemp, millis(), millis(), false, SENSOR_DISCONNECTED, 0};
    pressureData = {initialPress, initialPress, millis(), millis(), false, SENSOR_DISCONNECTED, 0};
    analogData = {0, 0, 0, 0, false, SENSOR_UNINITIALIZED, 0};

    Serial.println("BMP180 initialized.");
    Serial.println("System ready.");
    Serial.println();
    Serial.println("Time(ms)  | Temp(C) | Pressure(hPa) | Analog | T-Health | P-Health");
    Serial.println("----------|---------|---------------|--------|----------|----------");
}

// -------------------------------------------------------------------------------------------------------
void loop() {
    unsigned long currentTime = millis();

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
// Responsible for reading sensors, timestamping, and detectin comm failures
// Not responsible for validating the data, just acquiring it
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
    // Phase 3 will add recovery mechanism
    if (currentState == STATE_SAFE) {
        return;
    }

    SystemState newState = determineState();

    // Track when CRITICAL was first entered
    if (newState == STATE_CRITICAL && previousState != STATE_CRITICAL) {
        criticalEntryTime = millis();
    }

    // Enter SAFE if CRITICAL persists beyond delay
    if (newState == STATE_CRITICAL && millis() - criticalEntryTime >= SAFE_ENTRY_DELAY) {
        currentState = STATE_SAFE;
        Serial.println(">>> SAFE STATE ENTERED — actuators disabled <<<");
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

// State Determination ---------------------------------------------------------------------------------------------------
// Evaluated highest priority to lowest
// Only uses validated data — invalid readings cannot command state

SystemState determineState() {
    // SENSOR_FAULT
    bool sensorFault = (temperatureData.faultCount >= FAULT_THRESHOLD || pressureData.faultCount >= FAULT_THRESHOLD);
    if (sensorFault) return STATE_SENSOR_FAULT;

    // Only use valid readings for operational state
    // If sensor is invalid but below fault threshold — hold WARNING
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
            // LED flashes in CRITICAL
            digitalWrite(LED_PIN, (millis() / 250) % 2);
            break;

        case STATE_SENSOR_FAULT:
            analogWrite(MOTOR_PIN, 0);
            // Fast flash for sensor fault
            digitalWrite(LED_PIN, (millis() / 100) % 2);
            break;

        case STATE_SAFE:
            analogWrite(MOTOR_PIN, 0);
            // Slow flash for safe state
            digitalWrite(LED_PIN, (millis() / 500) % 2);
            break;
    }
}

// Helper: health status to string
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
  Serial.println(healthStr(temperatureData.health));
  Serial.print(" | FC=");
  Serial.println(temperatureData.faultCount);
}
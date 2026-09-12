// Real-Time Sensor Acquisition, Validation & Fault-Tolerant Control Platform
// This project is a real-time sensor acquisition, validation, and fault-tolerant control platform. It is designed to acquire data from various sensors, validate the readings, and provide fault-tolerant control mechanisms based on the sensor data.
// Phase 1 - Sensor validation layer

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
unsigned long lastSampleTime = 0;

// Validation Thresholds -------------------------------------------------------------------------------------------------------
// BMP180 Sensor operating limits
const float TEMP_MIN = -40.0;
const float TEMP_MAX = 80.0;
const float PRESSURE_MIN = 300.0;
const float PRESSURE_MAX = 1100.0;

// Maximum realistic rate of change per 100ms sample interval
const float TEMP_MAX_DELTA = 2.0;    // degrees C per sample
const float PRESSURE_MAX_DELTA = 10.0;    // hPa per sample

// Fault counter threshold, consecutive failed validations before fault declared
const int FAULT_THRESHOLD = 3;

// Sensor Objects -------------------------------------------------------------------------------------------------------
Adafruit_BMP085 bmp;    // BMP180 sensor object uses the BMP085 library

// Sensor Data -------------------------------------------------------------------------------------------------------
SensorData temperatureData;
SensorData pressureData;
SensorData analogData;

// Function Declarations -------------------------------------------------------------------------------------------------------
void acquireSensors();
void validateData();\
void validateSensor(SensorData &sensor, float minVal, float maxVal, float maxDelta);
void evaluateState();
void controlActuators();
void logData();

// -------------------------------------------------------------------------------------------------------
void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    Serial.begin(115200);

    Serial.println("RT Sensor Platform - Phase 0");
    Serial.println("Initializing...");

    // Initialize SensorData fields
    temperatureData = {0, 0, millis(), millis(), false, SENSOR_DISCONNECTED, 0};
    pressureData    = {0, 0, millis(), millis(), false, SENSOR_DISCONNECTED, 0};
    analogData = {0, 0, 0, 0, false, SENSOR_UNINITIALIZED, 0};

    if (!bmp.begin(0x77)) {    // Current adress for BMP180 is 0x77
        Serial.println("ERROR: BMP180 sensor not detected.");
        while (1);
    }

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

    // Temperature
    float rawTemp = bmp.readTemperature();
    if (!isnan(rawTemp)) {
        temperatureData.value = rawTemp;
        temperatureData.timestamp = timestamp;
    }
    // temperatureData.value = bmp.readTemperature();
    // temperatureData.timestamp = timestamp;
    // temperatureData.valid = true;    // Comm validation will be done in next phase

    // Pressure
    float rawPressure = bmp.readPressure() / 100.0;
    if (!isnan(rawPressure)) {
        pressureData.value = rawPressure;
        pressureData.timestamp = timestamp;
    }
    // pressureData.value = bmp.readPressure() / 100.0;    // Convert to hPa
    // pressureData.timestamp = timestamp;
    // pressureData.valid = true;

    // Analog Input, analog reads dont fail the same way, always valid at acquisition
    analogData.value = analogRead(POT_PIN);
    analogData.timestamp = timestamp;
    analogData.valid = true;
    analogData.health = SENSOR_OK;
}

// Validation Part -------------------------------------------------------------------------------------------------------
void validateData() {
    validateSensor(temperatureData, TEMP_MIN, TEMP_MAX, TEMP_MAX_DELTA);
    validateSensor(pressureData, PRESSURE_MIN, PRESSURE_MAX, PRESSURE_MAX_DELTA);
    // Analog validation is simple, just check range 0-4095 (12-bit ADC)
    analogData.valid = (analogData.value >= 0 && analogData.value <= 4095);
}

// Main Validation Function -------------------------------------------------------------------------------------------------------
// Takes a sensor by refernece, runs checks, updates health and valid flag
// Using & so changes to "sensor" are reflected in the original object

void validateSensor(SensorData &sensor, float minVal, float maxVal, float maxDelta) {

    // Check 1: NaN check - did the sensor return a non-number?
    if (isnan(sensor.value)) {
        sensor.valid = false;
        sensor.health = SENSOR_INVALID;
        sensor.faultCount++;
        return;    // No need to check further if value is NaN
    }

    // Check 2: Range validation - is the value within the expected range?
    if (sensor.value < minVal || sensor.value > maxVal) {
        sensor.valid = false;
        sensor.health = SENSOR_INVALID;
        sensor.faultCount++;
        return;
    }

    // Check 3: Rate of change validation - is this changing too fast?
    float delta = abs(sensor.value - sensor.lastValidValue);
    if (sensor.lastValidValue != 0 && sensor.lastUpdate != 0 && delta > maxDelta) {
        sensor.valid = false;
        sensor.health = SENSOR_INVALID;
        sensor.faultCount++;
        sensor.lastValidValue = sensor.value;
        return;
    }

    // Check 4: Stale data check - has the sensor stopped updating?
    if (sensor.lastUpdate != 0 && millis() - sensor.lastUpdate > STALE_TIMEOUT) {
        sensor.valid = false;
        sensor.health = SENSOR_STALE;
        sensor.faultCount++;
        sensor.lastUpdate = millis();  // Reset so next read gets a fair chance
        return;
    }

    // All checks passed
    sensor.valid          = true;
    sensor.health         = SENSOR_OK;
    sensor.faultCount     = 0;
    sensor.lastValidValue = sensor.value;
    sensor.lastUpdate     = millis();

}

// State Evaluation -------------------------------------------------------------------------------------------------------
// Place holder for now - state machine will be added in phase 2
void evaluateState() {
    // Phase 2 will add:
    // - state machine logic: NORMAL, WARNING, CRITICAL, FAULT, SAFE states
}

// Actuator Control -------------------------------------------------------------------------------------------------------
// Basic test - LED indicated any validation fault
// Phase 3 will replace with state-drive fault-tolerant control logic`
void controlActuators() {
    bool anyFault = !temperatureData.valid || !pressureData.valid;
    analogWrite(MOTOR_PIN, 127);    // Motor at half speed
    digitalWrite(LED_PIN, anyFault ? HIGH : LOW);
}

// Helper: health status to string
String healthStr(SensorHealth h) {
    switch (h) {
        case SENSOR_OK:             return "OK        ";
        case SENSOR_INVALID:        return "INVALID   ";
        case SENSOR_STALE:          return "STALE     ";
        case SENSOR_DISCONNECTED:   return "NO SENSOR ";
        case SENSOR_UNINITIALIZED:    return "UNKNOWN   ";
        default:                    return "UNKNOWN   ";
    }
}


// Telemetry Logging -------------------------------------------------------------------------------------------------------
void logData() {
  Serial.print(millis());
  Serial.print("ms  | ");
  Serial.print(temperatureData.valid ? temperatureData.value : -999);
  Serial.print("C    | ");
  Serial.print(pressureData.valid ? pressureData.value : -999);
  Serial.print(" hPa  | ");
  Serial.print((int)analogData.value);
  Serial.print("   | ");
  Serial.print(healthStr(temperatureData.health));
  Serial.print(" | ");
  Serial.println(healthStr(pressureData.health));
}
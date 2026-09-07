// Real-Time Sensor Acquisition, Validation & Fault-Tolerant Control Platform
// This project is a real-time sensor acquisition, validation, and fault-tolerant control platform. It is designed to acquire data from various sensors, validate the readings, and provide fault-tolerant control mechanisms based on the sensor data.
// Phase 0 - Baseline acquistion

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
unsigned long lastSampleTime = 0;

// Sensor Objects -------------------------------------------------------------------------------------------------------
Adafruit_BMP085 bmp;    // BMP180 sensor object uses the BMP085 library

// Sensor Data -------------------------------------------------------------------------------------------------------
SensorData temperatureData;
SensorData pressureData;
SensorData analogData;

// Function Declarations -------------------------------------------------------------------------------------------------------
void acquireSensors();
void validateData();
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

    if (!bmp.begin(0x77)) {    // Current adress for BMP180 is 0x77
        Serial.println("ERROR: BMP180 sensor not detected.");
        while (1);
    }

    Serial.println("BMP180 initialized.");
    Serial.println("System ready.");
    Serial.println();
    Serial.println("Time(ms)  | Temp(C) | Pressure(hPa) | Analog | Valid");
    Serial.println("----------|---------|---------------|--------|------");
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
    temperatureData.value = bmp.readTemperature();
    temperatureData.timestamp = timestamp;
    temperatureData.valid = true;    // Comm validation will be done in next phase

    // Pressure
    pressureData.value = bmp.readPressure() / 100.0;    // Convert to hPa
    pressureData.timestamp = timestamp;
    pressureData.valid = true;

    // Analog Input
    analogData.value = analogRead(POT_PIN);
    analogData.timestamp = timestamp;
    analogData.valid = true;
}

// Validation Part -------------------------------------------------------------------------------------------------------
// Place holder for now - will be added in pahse 1

void validateData() {
  // Phase 1 will add:
  // - range checks
  // - rate-of-change checks
  // - stale data detection
  // - communication fault detection
}

// State Evaluation -------------------------------------------------------------------------------------------------------
// Place holder for now - state machine will be added in pahse 2

void evaluateState() {
    // Phase 2 will add:
    // - state machine logic: NORMAL, WARNING, CRITICAL, FAULT, SAFE states
}

// Actuator Control -------------------------------------------------------------------------------------------------------
// Basic test - confirm hardware is working
// Phase 3 will replace with state-drive fault-tolerant control logic

void controlActuators() {
    analogWrite(MOTOR_PIN, 127);    // Motor at half speed
    digitalWrite(LED_PIN, HIGH);    // LED ON
}

// Telemetry Logging -------------------------------------------------------------------------------------------------------
void logData() {
    Serial.print(millis());
    Serial.print("ms  | ");
    Serial.print(temperatureData.value);
    Serial.print("C    | ");
    Serial.print(pressureData.value);
    Serial.print(" hPa  | ");
    Serial.print((int)analogData.value);
    Serial.print("   | ");
    Serial.println(temperatureData.valid ? "OK" : "FAULT");
}
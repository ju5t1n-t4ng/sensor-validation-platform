# Real-Time Sensor Acquisition, Validation & Fault-Tolerant Control System

A real-time embedded systems project built on the ESP32 using FreeRTOS. The system acquires data from multiple sensors, validates every measurement before trusting it, manages system health through a five-state state machine, and responds to faults safely — including sustained critical conditions, sensor failures, and watchdog timeouts.

Built as a portfolio project demonstrating practical embedded systems engineering: sensor integration, data validation pipelines, fault-tolerant control, and real-time scheduling.

---

## System Pipeline

```text
┌───────────────┐
│    Sensors    │
│  BMP180       │
│  Potentiometer│
└───────┬───────┘
        │
        ▼
┌─────────────────────┐
│  Data Acquisition   │
│  I²C / ADC          │
│  Timestamping       │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Data Validation    │
│  Range Checks       │
│  Rate-of-Change     │
│  Stale Detection    │
│  Fault Counting     │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  System State       │
│  NORMAL             │
│  WARNING            │
│  CRITICAL           │
│  SENSOR_FAULT       │
│  SAFE               │
└──────────┬──────────┘
           │
       ┌───┴────┐
       ▼        ▼
┌───────────┐ ┌────────────┐
│ Actuators │ │ Telemetry  │
│ Motor     │ │ Serial Log │
│ LED       │ │ Fault Log  │
└───────────┘ └────────────┘
```

---

## Hardware

| Component | Role |
|---|---|
| ESP32 DevKit C | Microcontroller |
| BMP180 | Temperature and pressure (I²C, 0x77) |
| Potentiometer | Simulated analog load sensor |
| LED | System state indicator |
| DC Motor | Primary actuator (stepper in Wokwi simulation) |

---

## Firmware Architecture

### Sensor Acquisition
Reads BMP180 over I²C and potentiometer via ADC at defined intervals. Every measurement is timestamped and stored in a standardized `SensorData` struct. Acquisition is responsible only for obtaining raw data — not for trusting it.

### Data Validation
Every measurement passes four checks before being marked valid:
- **Range check** — rejects physically impossible values
- **Rate-of-change check** — rejects measurements that change faster than physically realistic
- **NaN check** — rejects failed sensor reads
- **Stale detection** — rejects sensors that stop updating

A fault counter increments on each failed check and resets on a clean read. Three consecutive failures trigger `SENSOR_FAULT`.

### System State Machine
Validated data feeds a five-state machine evaluated in priority order:

| State | Condition | Motor | LED |
|---|---|---|---|
| NORMAL | All readings within range | Full speed | Off |
| WARNING | Any reading near threshold | Reduced | On |
| CRITICAL | Any reading exceeds critical threshold | Minimal | Flash 4Hz |
| SENSOR_FAULT | Fault counter ≥ 3 on any sensor | Off | Flash 10Hz |
| SAFE | CRITICAL or SENSOR_FAULT sustained > 2s | Off | Flash 1Hz |

SAFE is a hard latch — it requires recovery conditions to be met and sustained for 3 seconds before the system returns to NORMAL. Invalid sensor data never directly commands actuator behavior.

### FreeRTOS Task Architecture

| Task | Period | Priority | Role |
|---|---|---|---|
| SensorTask | 10ms | 3 | Reads all sensors |
| ValidationTask | 10ms | 3 | Validates sensor readings |
| ControlTask | 5ms | 4 | Evaluates state, commands actuators |
| TelemetryTask | 100ms | 1 | Serial logging |
| WatchdogTask | 100ms | 2 | Monitors sensor task health |

All shared data protected by mutexes. `vTaskDelayUntil()` used for deterministic periodic timing.

### Fault-Tolerant Control
- CRITICAL sustained > 2s → SAFE (reason: CRITICAL timeout)
- SENSOR_FAULT sustained > 2s → SAFE (reason: SENSOR_FAULT timeout)
- Watchdog timeout > 500ms → SAFE (reason: WATCHDOG fired)
- Recovery requires: sensors healthy + readings normal + fault counters cleared, sustained for 3 seconds
- Recovery timer resets if conditions are lost during the recovery window

---

## Validation Thresholds

| Parameter | Min | Max | Warning | Critical |
|---|---|---|---|---|
| Temperature | -40°C | 80°C | 35°C | 60°C |
| Pressure | 300 hPa | 1100 hPa | <1005 hPa | <980 hPa |
| Analog | 0 | 4095 | 3000 | 3800 |

---

## Telemetry Output

```text
Time(ms)  | Temp  | Pressure | Analog | State        | T-Health | FC
----------|-------|----------|--------|--------------|----------|----
1000ms    | 24.0C | 1013.0   | 0      | NORMAL       | OK       | 0
1100ms    | 24.0C | 1013.0   | 0      | NORMAL       | OK       | 0
```

State transitions and fault events are logged inline:
```text
>>> STATE CHANGE: NORMAL → WARNING
>>> SAFE STATE ENTERED — reason: CRITICAL timeout
>>> Recovery conditions met — monitoring...
>>> RECOVERY COMPLETE — returning to NORMAL
```

---

## Project Structure

rt-sensor-platform/
├── src/
│ └── main.cpp
├── include/
│ └── sensor_data.h
├── docs/
│ ├── architecture.md
│ └── development-log.md
├── wokwi/
│ └── diagram.json
└── README.md


---

## Development Phases

| Phase | Description | Status |
|---|---|---|
| 0 | Baseline sensor acquisition | ✅ Complete |
| 1 | Sensor validation layer | ✅ Complete |
| 2 | System state machine | ✅ Complete |
| 3 | Fault-tolerant control + recovery | ✅ Complete |
| 4 | FreeRTOS real-time scheduling | ✅ Complete |
| 5 | Performance analysis | 🔄 In progress |
| 6 | Fault injection testing | ⬜ Planned |
| 7 | Physical hardware build | ⬜ Planned |
| 8 | FPGA hardware acceleration | ⬜ Planned |

---

## Simulation

Built and tested in [Wokwi](https://wokwi.com) — a browser-based ESP32 simulator. Circuit diagram available in `wokwi/diagram.json`.

**Simulation concessions:**
- Stepper motor used in place of DC motor — PWM commands issued correctly but visual response is not representative of real DC motor behavior
- Rate-of-change thresholds relaxed for Wokwi slider behavior (real hardware values documented in code)
- Physical hardware build planned using ESP32 DevKit, BMP180, and L298N motor driver

---

## Skills Demonstrated

- Embedded C/C++ on ESP32
- I²C sensor integration
- ADC analog input
- Real-time systems with FreeRTOS
- Multi-task scheduling and prioritization
- Mutex and semaphore synchronization
- Sensor data validation pipelines
- State machine design
- Fault detection and safe-state control
- Watchdog timer implementation
- Serial telemetry logging
- Modular firmware architecture
- Hardware/software co-design (planned)

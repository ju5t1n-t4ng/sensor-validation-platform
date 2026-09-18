# Development Log — RT Sensor Platform

## Phase 0

### Issue 1 — Stepper Motor vs DC Motor
**Problem:** Wokwi has no DC motor. Stepper motor driven with `analogWrite()` — incorrect for real stepper control but works visually in simulation.
**Fix:** Accepted for simulation. DC motor + L298N driver planned for physical build.
**Lesson:** Verify simulation behavior matches real hardware before assuming design is correct.

---

## Phase 1

### Issue 2 — Stale Check Triggered Immediately on Boot
**Problem:** Both sensors immediately reported `SENSOR_STALE`, logging `-999`.
**Root Cause:** `lastUpdate` initialized to `0`. At first sample, `millis() - 0 > 500ms` was always true.
**Fix:**
- Guard condition: only run stale check if `lastUpdate != 0`
- Initialize `lastUpdate = millis()` in `setup()`
- Reset `lastUpdate = millis()` inside the stale check block

**Lesson:** Never initialize timestamp fields to `0`. Seed from `millis()` at startup.

---

## Phase 2

### Issue 3 — Rate-of-Change Check Locking Sensor Permanently
**Problem:** After any slider adjustment, sensors permanently reported `SENSOR_INVALID`.
**Root Cause:** `lastValidValue` initialized to `0`. Real reading of `24.0°C` produced delta of `24.0` against `0`, exceeding threshold. `lastValidValue` never updated so every read kept failing.
**Fix:**
- Update `lastValidValue` inside the failure block so baseline advances even on rejection
- Seed `lastValidValue` from first real sensor read in `setup()`:
```cpp
float initTemp = bmp.readTemperature();
temperatureData = {initTemp, initTemp, millis(), millis(), false, SENSOR_DISCONNECTED, 0};
```
**Lesson:** Never initialize sensor baselines to `0` unless `0` is a physically realistic value.

---

### Issue 4 — Delta Thresholds Too Tight for Wokwi
**Problem:** Any slider drag immediately triggered rate-of-change fault.
**Root Cause:** Wokwi sliders jump instantly — no physical inertia. Real thresholds (`2.0°C/100ms`) are exceeded by any slider movement.
**Fix:** Relaxed for simulation with comments documenting real-hardware values:
```cpp
const float TEMP_MAX_DELTA     = 10.0;  // Relaxed for Wokwi — real HW: ~0.5°C/100ms
const float PRESSURE_MAX_DELTA = 50.0;  // Relaxed for Wokwi — real HW: ~2.0 hPa/100ms
```
**Lesson:** Document all simulation-specific concessions so they are not carried into physical builds.

---

## Design Decisions

**Separate validation from operational thresholds** — validation limits reject physically impossible readings; operational thresholds drive system state. Keeps the two concerns independent.

**Fault counter debouncing** — three consecutive failures required before `SENSOR_FAULT`. Single noise spikes do not trigger a system fault.

**Invalid data cannot command state** — `determineState()` only evaluates operational thresholds against `sensor.value` when `sensor.valid == true`. Garbage data holds the system at `WARNING`, never commands `NORMAL` or `CRITICAL`.

**SAFE state as a latch** — once entered, requires deliberate reset to exit. Automatic recovery from a safety shutdown is itself a safety risk.

**Startup sensor seeding** — `lastValidValue` seeded from first real read at boot, not a hardcoded assumption.

---

## Open Issues

| Issue | Phase |
|---|---|
| Stepper motor used instead of DC motor | Hardware |
| Delta thresholds relaxed for Wokwi — tighten for physical build | Phase 1 |
| `SENSOR_FAULT` auto-clears on recovery — proper latch needed | Phase 3 |
| `SAFE` only latches from `CRITICAL`, not `SENSOR_FAULT` | Phase 3 |
| No watchdog timer | Phase 3 |
| No FreeRTOS task scheduling | Phase 4 |

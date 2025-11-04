# System Step-by-Step Walkthrough

This document explains how the system operates from boot to monitoring, including processing and decisions.

## 1) Boot & Initialization

- Start `Serial` at `115200`.
- Configure pins for sensors, LEDs, buzzer, button, ultrasonic, and relay.
- Configure ESP32 ADC (`12-bit`, `ADC_11db` attenuation).
- `dht.begin()` to start the DHT11 sensor.
- Attempt WiFi connection using `ssid` and `pass`.
- While not connected: blink Red LED (non-blocking) and keep trying.
- On WiFi connect: initialize Blynk, send a startup event, switch to `STATE_PREHEATING`.

## 2) Preheating (15 minutes)

- Duration: `PREHEAT_TIME = 900000 ms`.
- Visual: alternating Red/Blue (siren pattern) to indicate preheat.
- Every 10% progress: update Blynk `V0` with preheat percentage and log remaining time.
- Sample temperature and humidity at `SENSOR_READ_INTERVAL` for baseline stabilization.
- When time elapses: set Blue LED solid, prompt calibration via Blynk `V7`, move to `STATE_READY_FOR_CALIBRATION`.

## 3) Ready for Calibration

- State indicates system is ready; Blue LED solid.
- Press button (non-blocking debounce) to start calibration.
- Single beep to acknowledge calibration start.
- Transition to `STATE_CALIBRATING`.

## 4) Calibration (10 seconds)

- Duration: `CALIBRATION_TIME = 10000 ms`.
- Sample MQ135, MQ3, DHT at ~10 Hz.
- Add readings to `AdvancedSensorProcessor` (rolling window, outlier-resistant weighted average).
- Compute filtered averages at the end:
  - Convert ADC to voltage (`3.3V` reference).
  - Compute `RS` for each sensor using load resistors: `RL_MQ135 = 20.0`, `RL_MQ3 = 1.0`.
  - Compute clean-air baselines: `R0_MQ135 = RS_MQ135 / 3.6`, `R0_MQ3 = RS_MQ3 / 60.0`.
- Calibrate `EnvironmentalCompensator` with the averaged temp/humidity.
- Set Green LED and declare monitoring readiness.
- Double beep to confirm completion; update Blynk `V10` to 100%, `V7` to "System Ready".
- Transition to `STATE_MONITORING`.

## 5) Monitoring Loop (continuous)

At each cycle (`SENSOR_READ_INTERVAL = 1000 ms`):

1. Read sensors:
   - MQ135/MQ3 via `readMQSensorOptimized()` (ADC settle + read).
   - DHT temperature and humidity.
2. Filter readings:
   - Weighted average excluding outliers; track stability and trend.
3. Environmental compensation:
   - Adjust MQ readings for temp/humidity using calibrated baselines.
4. Compute ratios:
   - Voltage → `RS` → `ratio = RS / R0` per sensor.
5. Adaptive thresholds:
   - `environmentalFactor = (1 + 0.01*(temp-25)) * (1 + 0.002*(humidity-50))`.
   - Targets: `spoiledTarget = spoiledBaseline × environmentalFactor`.
   - Update: `spoiledThreshold += (target - spoiledThreshold) × adaptationRate` (`0.05`).
6. Classification:
   - `Spoiled` if `mq135Ratio <= mq135.spoiledThreshold` OR `mq3Ratio <= mq3.spoiledThreshold`.
   - `Warning` if ratio falls within warning bands (MQ135: `2.4–2.7`, MQ3: `44.0–50.0`).
   - `Fresh` otherwise.
7. UI updates:
   - LEDs: Fresh→Green, Warning→Yellow, Spoiled→Red.
   - Blynk metrics: `V1/V2` raw values; `V8/V9` ratios; `V3/V4` temp/humidity; `V5` status.
   - On status change: send corresponding Blynk event and beep `foodStatus + 1` times.
8. Logging:
   - Every ~30s: print sensor values, ratios, trends, stability, and status.
9. Safety:
   - If `temp > MAX_TEMP (80°C)`, switch to `STATE_ERROR`.
10. Proximity & Relay:
   - Read ultrasonic distance at ~10 Hz; set relay HIGH if `distance <= 10 cm`.

## 6) Error State

- Flash Red LED, disable other LEDs.
- Set Relay HIGH.
- Report via Blynk and Serial.
- Await user intervention (reset or fix conditions).

## Thresholds & Tuning

- Baseline thresholds (constructor of `AdaptiveThresholdManager`):
  - MQ135:
    - `freshMin: 2.8`, `freshMax: 3.5`, `warningMin: 2.4`, `warningMax: 2.7`, `spoiledThreshold: 2.3`.
  - MQ3:
    - `freshMin: 51.0`, `freshMax: 65.0`, `warningMin: 44.0`, `warningMax: 50.0`, `spoiledThreshold: 43.0`.
- Adaptation rate: `0.05` (slower, more stable).
- To change sensitivity: edit baselines or adaptation rate.

## Blynk Mapping

- `V0` Preheat progress
- `V1` MQ135 raw
- `V2` MQ3 raw
- `V3` Temperature
- `V4` Humidity
- `V5` Status text
- `V6` Buzzer off toggle
- `V7` Button/status prompts
- `V8` MQ135 ratio
- `V9` MQ3 ratio
- `V10` Calibration progress

## Tips & Best Practices

- Calibrate in clean, ventilated air; avoid strong odors.
- Let MQ sensors warm up fully to stabilize readings.
- Place sensors away from direct airflow to reduce noise.
- Use stable power and avoid long ADC wires to minimize interference.


# System Flowchart

This flowchart captures the high-level control flow, state transitions, and core data processing steps. Render this using Mermaid (supported by many IDEs and markdown renderers).

```mermaid
flowchart TD
  A[Power On] --> B[STATE_INITIALIZING]
  B -->|WiFi connected| C[STATE_PREHEATING]
  B -->|WiFi not connected| B

  C --> D{Preheat complete?}
  D -->|No| C
  D -->|Yes| E[STATE_READY_FOR_CALIBRATION]

  E -->|Button pressed| F[STATE_CALIBRATING]

  F --> F1[Sample MQ135, MQ3, DHT @10Hz]
  F1 --> F2[Filter readings]
  F2 --> F3[Compute RS and R0 baselines]
  F3 --> F4[Calibrate EnvironmentalCompensator]
  F4 --> G[STATE_MONITORING]

  subgraph Monitoring Loop
    G --> K1[Read sensors]
    K1 --> K2[Advanced filtering]
    K2 --> K3[Environmental compensation]
    K3 --> K4[Compute ratios]
    K4 --> K5[Update adaptive thresholds]
    K5 --> K6{Classify food}
    K6 -->|Fresh| N[LED: Green; Event: food_fresh]
    K6 -->|Warning| O[LED: Yellow; Event: food_warning_]
    K6 -->|Spoiled| P[LED: Red; Event: food_alert]
    G --> U[Ultrasonic+Relay control]
    G --> V[Status/Blynk updates]
    G --> G
  end

  G --> H{SafetyMonitor}
  H -->|Overheat| I[STATE_ERROR]
  I --> I1[Flash Red LED, Relay HIGH, Log error]
```

Notes:
- Monitoring is non-blocking, paced by `SENSOR_READ_INTERVAL` and bounded by small `delay(10)` in `loop()`.
- Adaptive thresholds follow `spoiledBaseline × environmentalFactor` with `adaptationRate = 0.05`.


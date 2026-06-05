# Autonomous-Sonar-Based-War-Machine
Embedded autonomous platform with dual-sensor fusion, servo-driven scanning, and wireless control — built on ESP8266 / NodeMCU
Overview
This project implements a fully autonomous robotic platform capable of enemy/obstacle detection, 180° sonar scanning, and simulated targeting response — all running on a NodeMCU (ESP8266) with custom firmware written in C/C++. The robot operates without human input once deployed, using onboard sensor data to make real-time decisions about movement, threat detection, and firing simulation.
The design was motivated by the challenge of building a reliable autonomous decision loop on a resource-constrained microcontroller (80 MHz, 80 KB RAM) while maintaining sub-100 ms sensor-to-actuator latency.

System Architecture



┌─────────────────────────────────────────────────────┐
│                    NodeMCU (ESP8266)                 │
│  ┌─────────────┐    ┌──────────────────────────┐    │
│  │  Forward    │    │   Servo-Mounted Sensor   │    │
│  │  HC-SR04    │    │       HC-SR04            │    │
│  │ (Detection) │    │  (180° Scanning)         │    │
│  └──────┬──────┘    └────────────┬─────────────┘    │
│         │                        │                   │
│  ┌──────▼────────────────────────▼───────────────┐  │
│  │            Sensor Fusion & Decision Engine    │  │
│  │  - Threat detection threshold logic           │  │
│  │  - Scan state machine                         │  │
│  │  - Motor command arbitration                  │  │
│  └──────────────────┬────────────────────────────┘  │
│                     │                                │
│     ┌───────────────┼───────────────┐               │
│     ▼               ▼               ▼               │
│  Drive Motors    Servo Motor    Wi-Fi TX (ESP)       │
│  (L298N bridge)  (SG90)        (Wireless telemetry) │
└─────────────────────────────────────────────────────┘




Hardware Components
ComponentModelPurposeMicrocontrollerNodeMCU ESP8266 (v3)Main compute + Wi-FiUltrasonic Sensor x2HC-SR04Forward detection + scanningServo MotorSG90Sensor sweep mountMotor DriverL298N H-BridgeDrive motor controlDC Motors x2TT Gear MotorDifferential drivePower Supply7.4V 2S LiPoMain power busVoltage RegulatorLM78055V logic rail
Power Architecture
7.4V LiPo
    ├── L298N Motor Driver (direct)
    └── LM7805 → 5V rail
                ├── NodeMCU (3.3V via onboard LDO)
                ├── HC-SR04 sensors
                └── SG90 servo

Firmware Architecture
The firmware is structured as a cooperative state machine with three concurrent logical tasks:
State Machine — Autonomous Mode
         ┌──────────┐
    ───►  │  SCAN   │ ◄─── Servo sweeps 0°→180°, sampling every 15°
         └────┬─────┘
              │ Target detected (distance < THREAT_THRESHOLD_CM)
              ▼
         ┌──────────┐
         │  TRACK   │ ── Servo locks on target, forward sensor confirms
         └────┬─────┘
              │ Confirmed threat
              ▼
         ┌──────────┐
         │  ENGAGE  │ ── Simulated fire pulse, halt drive motors
         └────┬─────┘
              │ Target lost / timeout
              ▼
         ┌──────────┐
         │ NAVIGATE │ ── Obstacle avoidance via forward HC-SR04
         └──────────┘
              │ Path clear
              └──────────► SCAN
Key Firmware Modules
firmware/
├── main.ino                  ← Setup, loop, state dispatcher
├── sensor_driver.cpp/.h      ← HC-SR04 timed pulse, filtered distance
├── servo_scanner.cpp/.h      ← Sweep control, angle-to-distance map
├── motor_control.cpp/.h      ← PWM drive, differential steering
├── decision_engine.cpp/.h    ← State machine logic, threat thresholds
└── wifi_telemetry.cpp/.h     ← UDP packet broadcast of sensor state
Sensor Reading — HC-SR04 Implementation
cpp// Blocking pulse with timeout guard — avoids watchdog reset on open air
float readDistanceCM(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
    if (duration == 0) return MAX_RANGE_CM;  // Timeout = no object

    return (duration * 0.0343f) / 2.0f;
}
Servo Scan Loop
cpp// Non-blocking servo sweep using millis() — avoids delay() blocking sensor reads
void ServScanner::update() {
    if (millis() - _lastStepTime < STEP_INTERVAL_MS) return;
    _lastStepTime = millis();

    _servo.write(_currentAngle);
    _scanMap[_currentAngle / ANGLE_STEP] = readDistanceCM(_trigPin, _echoPin);

    _currentAngle += _direction * ANGLE_STEP;
    if (_currentAngle >= 180 || _currentAngle <= 0) {
        _direction *= -1;   // Reverse sweep direction
        _scanComplete = true;
    }
}

Validation & Testing
Bench Tests Performed
TestMethodResultSensor range accuracyMeasured vs. ruler at 10/20/50/100 cm±2 cm at <100 cmServo angle accuracyProtractor verification at 0°/45°/90°/135°/180°±3°Threat detection latencyOscilloscope: trigger pin → motor stop<85 msFalse positive rate200 runs, open environment3%Wi-Fi telemetry rangeUDP packet loss test at 5/10/15m0% loss ≤10mBattery runtime7.4V LiPo under full load~47 min
Logic Analyzer Captures

I²C / PWM timing verified on servo control signal (50 Hz, 1–2 ms pulse width)
HC-SR04 echo pulse captured to confirm timeout logic on no-return scenarios
UART debug output logged at 115200 baud during all state transitions


Known Limitations & Future Work

 HC-SR04 returns spurious echoes on highly reflective surfaces (glass, metal) — Kalman filter or median filter would improve robustness
 Single-threaded cooperative loop means long servo sweeps (~900 ms full sweep) delay forward sensor refresh
 Future: migrate to FreeRTOS tasks on ESP32 for true concurrent sensor polling
 Future: replace UDP broadcast with MQTT broker for structured telemetry logging


Build & Flash
bash# Arduino CLI
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 firmware/
arduino-cli upload  --fqbn esp8266:esp8266:nodemcuv2 --port /dev/ttyUSB0 firmware/
Dependencies:

ESP8266WiFi (bundled with ESP8266 Arduino core)
Servo.h (Arduino standard library)


Lessons Learned
Sensor timing is critical on single-core MCUs. Early versions used delay() inside the servo sweep, which blocked the forward sensor from updating. This caused the robot to miss obstacles while scanning. Switching to a millis()-based non-blocking pattern fixed this entirely — a fundamental lesson in embedded real-time design that directly applies to any multi-sensor system.
Power rail noise caused spurious HC-SR04 readings. Motor PWM switching coupled onto the 5V logic rail, causing intermittent false-short-distance readings. Added 100µF bulk + 100nF bypass capacitors at each sensor VCC pin — noise dropped from ±15 cm to ±2 cm. This is a classic PDN issue that shows up in every real hardware system.

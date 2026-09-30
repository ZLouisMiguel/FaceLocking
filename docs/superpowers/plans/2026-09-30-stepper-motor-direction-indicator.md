# Stepper Motor Direction Indicator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect the existing identity-lock pipeline to an ESP8266-driven 28BYJ-48 stepper so the locked face's horizontal position commands left, center, or right motor movement over MQTT.

**Architecture:** A pure Python mapping/controller layer will turn a locked face box into a smoothed, throttled absolute angle and publish it through an injectable MQTT transport. `src/recognize.py` will feed tracker state and frame geometry into that controller and render its state in the existing OpenCV overlay. The ESP8266 firmware will subscribe to the same topic and drive ULN2003 inputs with half-step sequencing, with all deployment settings configurable at the top of the firmware.

**Tech Stack:** Python 3, unittest, OpenCV, paho-mqtt (runtime PC publisher), MicroPython on ESP8266, MQTT 3.1.1, 28BYJ-48 + ULN2003.

**Spec:** `docs/superpowers/specs/2026-09-30-stepper-motor-direction-indicator-design.md`

## Global Constraints

- Existing face detection, recognition, and `TargetTracker` behavior remains authoritative and must continue passing its current tests.
- Default MQTT topic is `face/servo/angle`; payload is an ASCII decimal absolute angle.
- Default angle range is 10°–170° with 90° neutral.
- Default ESP8266 motor GPIO tuple is `(5, 4, 0, 2)` for NodeMCU D1–D4.
- Wi-Fi credentials and deployment secrets must not be committed.
- Recognition must continue running when MQTT is unavailable.

## Review Focus

- MQTT unavailable or reconnecting: recognition continues and the overlay reports offline state.
- Target center outside or at the edges of the frame: commanded angle is clamped to the configured range.
- Noisy face boxes near center: deadband, smoothing, and throttling prevent stepper jitter.
- Temporary `LOST` versus full `SEARCHING` reset: hold position during grace loss and center only after reset.
- Invalid firmware payload or motor/network exception: ignore/recover safely and release coils on firmware failure.

### Task 1: Add the PC-side angle mapping and MQTT motor controller

**Files:**
- Create: `src/motor_control.py`
- Test: `tests/test_motor_control.py`

**Interfaces:**
- Produces `map_x_to_angle(center_x: float, frame_width: int, minimum: int = 10, maximum: int = 170, neutral: int = 90) -> int`.
- Produces `MotorCommand` with `angle`, `direction`, and `published` fields.
- Produces `FaceMotorController(publisher, ...)` with `update(state: str, box, frame_width: int, now: float | None = None) -> MotorCommand` and `close() -> None`.
- Consumes an injected publisher exposing `publish_angle(angle: int) -> bool` and `close() -> None`.

- [ ] **Step 1: Write failing tests** for left/center/right mapping and clamping; deadband and direction labels; publish throttling; `LOST` hold behavior; and one-time neutral command after a full reset.
- [ ] **Step 2: Run `\.venv\Scripts\python.exe -m unittest tests.test_motor_control -v` and verify it fails because `src.motor_control` does not exist.
- [ ] **Step 3: Implement the pure mapping and controller** with normalized center coordinates, configured exponential smoothing, deadband around neutral, minimum publish interval, and state transition handling. Keep MQTT transport injectable so tests do not require a broker.
- [ ] **Step 4: Run the focused tests and verify they pass.**
- [ ] **Step 5: Commit as `feat: add face position motor controller`.**

### Task 2: Integrate motor commands into face recognition and overlay

**Files:**
- Modify: `src/recognize.py`
- Modify: `src/face_signals.py`
- Modify: `src/main.py`
- Test: `tests/test_motor_integration.py`

**Interfaces:**
- Consumes `FaceMotorController` and `MotorCommand` from Task 1.
- Produces `MqttAnglePublisher` with configurable broker, port, topic, client id, and reconnect interval; it must degrade to an offline result rather than raise into the camera loop.
- `run_recognition` gains optional motor settings while preserving its existing callable defaults.

- [ ] **Step 1: Write failing tests** for MQTT publisher payload/configuration using a fake client, offline publish behavior, and recognition-level motor overlay formatting/state integration.
- [ ] **Step 2: Run `\.venv\Scripts\python.exe -m unittest tests.test_motor_integration -v` and verify the new tests fail because the publisher/integration does not exist.
- [ ] **Step 3: Implement `MqttAnglePublisher`** with lazy paho-mqtt import, connection/reconnect handling, `publish_angle`, and clean shutdown; implement environment-backed settings in `src/main.py` and feed tracker state, `track.box`, and `frame.shape[1]` into the controller each frame.
- [ ] **Step 4: Add an OpenCV overlay line** showing `MOTOR: LEFT/RIGHT/CENTER`, angle, and `ONLINE/OFFLINE`, without changing existing lock colors/status.
- [ ] **Step 5: Run focused tests, then the existing suite, and verify both pass.**
- [ ] **Step 6: Commit as `feat: connect face lock to mqtt motor control`.**

### Task 3: Add ESP8266 firmware and pure motor math

**Files:**
- Create: `frame-step-motor/main.py`
- Create: `frame-step-motor/umqtt_simple.py`
- Create: `frame-step-motor/motor_math.py`
- Test: `tests/test_motor_math.py`
- Modify: `frame-step-motor/connections.md`

**Interfaces:**
- `motor_math.clamp(value, minimum, maximum)`, `motor_math.angle_to_steps(angle, minimum, maximum, steps_per_rev)`, and `motor_math.parse_angle(payload, minimum, maximum)` are pure MicroPython-compatible helpers.
- Firmware subscribes to the configured topic, validates payloads, maps absolute angles to half-steps, releases coils when idle/error, and retries Wi-Fi/MQTT connections.

- [ ] **Step 1: Write failing tests** for firmware angle conversion, clamping, valid payload parsing, and invalid/non-finite payload rejection.
- [ ] **Step 2: Run `\.venv\Scripts\python.exe -m unittest tests.test_motor_math -v` and verify it fails because the helper does not exist.
- [ ] **Step 3: Implement pure helpers and firmware** using the approved D1–D4 defaults, half-step sequence, safe coil release, and configurable Wi-Fi/broker/topic/angle constants. Preserve the supplied `umqtt_simple.py` protocol implementation while adapting it to the final topic/configuration.
- [ ] **Step 4: Update the wiring document** with D1–D4/GPIO mapping, shared ground and motor-supply warning, firmware upload steps, and the fact that pins 1–4 are not the default.
- [ ] **Step 5: Run the focused tests and verify they pass.**
- [ ] **Step 6: Commit as `feat: add esp8266 stepper mqtt firmware`.**

### Task 4: Document setup and perform final verification

**Files:**
- Modify: `README.md`
- Modify: `.gitignore` only if a local secrets/config pattern is needed

- [ ] **Step 1: Add setup documentation** for installing `paho-mqtt`, setting environment variables, starting the PC tracker, uploading the three firmware files, and verifying left/center/right behavior.
- [ ] **Step 2: Run the full test suite:** `\.venv\Scripts\python.exe -m unittest discover -s tests -v`.
- [ ] **Step 3: Run `\.venv\Scripts\python.exe -m compileall -q src` and inspect `git diff` for accidental credential or unrelated-file changes.
- [ ] **Step 4: Commit as `docs: document stepper motor integration`.**


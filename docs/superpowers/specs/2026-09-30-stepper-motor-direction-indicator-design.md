# Stepper Motor Direction Indicator Design

## Goal

Extend the existing face-recognition identity-lock pipeline so a locked face's horizontal position controls a 28BYJ-48 stepper motor through an ESP8266 and MQTT. The center of the camera frame is the motor's neutral position; moving the locked face left or right moves the motor in the corresponding direction.

## Scope and source authority

The existing Python face detector, recognizer, and `TargetTracker` remain the source of truth for target identity and lock state. The downloaded `C:\Users\HP\Downloads\main.py` and `umqtt_simple.py` are reference firmware files supplied for this integration; they are not instructions that override the project request. The project will include a configured, credential-free firmware copy and the PC-side publisher needed to connect the current pipeline to it.

## Architecture

The PC application will create a small motor-control component that accepts the current frame width and locked target box, converts the target center x-coordinate into a bounded absolute angle, and publishes that angle over MQTT. The component will use a deadband, exponential smoothing, and a minimum publish interval to prevent visual noise from causing motor jitter. It will hold position during the tracker's temporary `LOST` state and command the configured center angle after the tracker fully returns to `SEARCHING`.

The ESP8266 MicroPython firmware will subscribe to the existing-compatible `face/servo/angle` topic, convert absolute angles into half-steps, and drive the ULN2003 inputs. Wi-Fi credentials, broker settings, topic, angle limits, and motor pin mapping will be constants at the top of the firmware so deployment requires configuration but no secret is committed.

## Interfaces and defaults

- MQTT broker: configurable, default `broker.emqx.io:1883` to match the supplied reference.
- MQTT topic: configurable, default `face/servo/angle` for compatibility with the supplied firmware.
- Payload: ASCII decimal angle, bounded to `10`–`170` degrees by default.
- Neutral angle: `90` degrees.
- ESP8266 motor pins: NodeMCU D1–D4, GPIO `(5, 4, 0, 2)` by default. The conflicting project note about pins 1–4 is handled by the configurable tuple and is not used as the default because GPIO1 is the serial TX pin.
- Mapping: target center at the frame center produces 90 degrees; left and right map linearly to the configured minimum and maximum angles.
- Unlocked behavior: no motor movement while acquiring a lock; after a full tracker reset, publish neutral once.

## Failure handling

- If MQTT is unavailable at startup or disconnects during operation, recognition and the on-screen overlay continue; the motor component reports the error and reconnects on later update attempts.
- Invalid or non-finite target values are ignored by the PC publisher and clamped/rejected by the firmware.
- Firmware connection failures release all motor coils before retrying.
- The PC process disconnects cleanly from MQTT and releases the camera when the loop exits.

## Verification

Unit tests will cover left/center/right mapping, clamping, deadband and publish throttling, lock/lost/searching transitions, invalid payload handling, and the firmware's pure angle/step conversion helpers where they can be tested without hardware. Existing face-locking tests must remain passing. A manual hardware check will verify D1–D4 wiring, left/right direction, neutral centering, and recovery after Wi-Fi or broker interruption.


# Face Recognition with Identity Lock

This repository detects faces, recognizes enrolled identities, keeps a stable lock on the selected person, and drives a remote stepper-motor direction indicator. A locked face left of the camera center commands the motor left; a locked face right of center commands it right.

## Requirements

- Windows camera accessible through OpenCV.
- Python environment in `.venv` with OpenCV, NumPy, MediaPipe, ONNX Runtime, and PySerial.
- `paho-mqtt` in the PC environment for publishing motor angles.
- `face_detection_yunet_2023mar.onnx` in the project root.
- `models/embedder_arcface.onnx` for ArcFace embeddings.

## Run automated tests

From this directory:

```powershell
.venv\Scripts\python.exe -m unittest discover -s tests -v
.venv\Scripts\python.exe -m compileall -q src
```

The tests cover acquisition debounce, skipped embeddings, wrong-identity rejection, temporary loss, reset/reacquisition, landmark ordering, database edge cases, and overlay colors.

## Enroll a person

```powershell
.venv\Scripts\python.exe -m src.enroll
```

Enter a name, face the camera, press `SPACE` to capture samples, then press `s` to save. Capture at least three clear samples with small changes in pose or expression. Press `q` to cancel.

For expression robustness, capture neutral, smiling, and slightly turned-face samples. Every normalized sample is now preserved instead of being averaged into one expression-specific vector. Re-enrolling an existing name appends the new samples to that identity profile.

The current working checkout is configured for camera index `2`. On the development machine, `camprobe.py` found index `1` working and index `2` unavailable, so run the probe first and change `cam_source` in `src/enroll.py` and `src/recognize.py` if needed.

## Run face recognition, identity lock, and stepper indicator

Install the PC MQTT dependency in the project environment:

```powershell
.venv\Scripts\python.exe -m pip install paho-mqtt
```

The PC tracker uses these defaults:

- Broker: `broker.emqx.io:1883`
- Topic: `face/servo/angle`
- Angle range: `10`–`170` degrees
- Neutral: `90` degrees
- Motor pin mapping on the ESP8266: D1–D4 / GPIO `5, 4, 0, 2`

Set deployment-specific values through environment variables rather than
editing code or committing credentials:

```powershell
$env:FACE_CAMERA_SOURCE = "2"
$env:FACE_MQTT_BROKER = "broker.emqx.io"
$env:FACE_MQTT_PORT = "1883"
$env:FACE_MQTT_TOPIC = "face/servo/angle"
$env:FACE_MOTOR_ENABLED = "true"
```

To run without a board or broker while testing recognition, set
`$env:FACE_MOTOR_ENABLED = "false"`. Recognition continues if MQTT is
unavailable, and the overlay reports the motor as offline.

```powershell
.venv\Scripts\python.exe -m src.recognize
```

Expected live behavior:

1. `SEARCHING / UNLOCKED` in red while no known face is stable.
2. After three consistent recognized frames, `LOCKED: <name>` in green.
3. A short detector gap shows `TARGET LOST: <name>` in orange and can reacquire the same person.
4. After a longer loss, the system resets to searching and requires a new identity lock.

When locked, the overlay also shows `MOTOR: LEFT`, `CENTER`, or `RIGHT`, the
commanded angle, and MQTT availability. A short temporary target loss holds the
last motor position; a full tracker reset commands neutral (90°) once.

Every detected face is boxed. Known faces show their enrolled name; faces that are not in the database show `Stranger` in red.

Controls:

- `q`: quit.
- `+` or `=`: loosen the recognition distance threshold.
- `-`: tighten the recognition distance threshold.

For assessment evidence, record a short video showing the target moving, another person entering the frame, the green lock remaining on the enrolled identity, and the system returning to searching after the target leaves.

## Upload the ESP8266 controller

The MicroPython firmware is in `frame-step-motor/`. Upload all three files to
the ESP8266:

- `main.py`
- `motor_math.py`
- `umqtt_simple.py`

Before uploading, set `WIFI_SSID` and `WIFI_PASSWORD` in `main.py`. Keep its
broker and topic equal to the PC settings. Wire the ULN2003 inputs as shown in
[`frame-step-motor/connections.md`](frame-step-motor/connections.md), power the
stepper from an external 5 V supply, and share ground with the ESP8266. The
firmware assumes the physical pointer is at 90° when the board resets.

## Hardware verification

1. Start the ESP8266 and confirm its serial log shows Wi-Fi and MQTT ready.
2. Start the PC tracker and enroll/recognize a target.
3. Move the locked face left of frame center and verify the motor turns left.
4. Move it right of frame center and verify the motor turns right.
5. Return the face near center and verify the motor settles at neutral.
6. If direction is reversed, set `REVERSE_DIRECTION = True` in the ESP8266 `main.py`.

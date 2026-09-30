"""ESP8266 MQTT stepper controller for the face-lock motor indicator."""

import time

import network
from machine import Pin
from umqtt_simple import MQTTClient

from motor_math import angle_to_steps, parse_angle


# Fill these values before uploading. They are deliberately not committed as
# real credentials.
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "esp8266-face-motor"
ANGLE_TOPIC = "face/servo/angle"

# NodeMCU D1, D2, D3, D4 -> ULN2003 IN1, IN2, IN3, IN4.
# GPIO1/TX is intentionally not used as a motor input.
MOTOR_PINS = (5, 4, 0, 2)

MIN_ANGLE = 10
MAX_ANGLE = 170
START_ANGLE = 90
STEPS_PER_REV = 4096
REVERSE_DIRECTION = False
STEP_DELAY_S = 0.002

HALF_STEP_SEQUENCE = (
    (1, 0, 0, 0),
    (1, 1, 0, 0),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 0),
    (0, 0, 1, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
)


motor_pins = [Pin(pin, Pin.OUT) for pin in MOTOR_PINS]
current_steps = angle_to_steps(
    START_ANGLE,
    MIN_ANGLE,
    MAX_ANGLE,
    STEPS_PER_REV,
)
sequence_index = current_steps % len(HALF_STEP_SEQUENCE)


def write_coils(pattern):
    for pin, value in zip(motor_pins, pattern):
        pin.value(value)


def release_motor():
    write_coils((0, 0, 0, 0))


def step_motor(direction):
    """Advance one half-step while preserving logical angle direction."""
    global current_steps, sequence_index

    physical_direction = -direction if REVERSE_DIRECTION else direction
    if physical_direction > 0:
        sequence_index = (sequence_index + 1) % len(HALF_STEP_SEQUENCE)
    else:
        sequence_index = (sequence_index - 1) % len(HALF_STEP_SEQUENCE)

    write_coils(HALF_STEP_SEQUENCE[sequence_index])
    current_steps += direction
    time.sleep(STEP_DELAY_S)


def move_to_angle(target_angle):
    global current_steps

    target_steps = angle_to_steps(
        target_angle,
        MIN_ANGLE,
        MAX_ANGLE,
        STEPS_PER_REV,
    )
    delta = target_steps - current_steps
    if delta == 0:
        return

    direction = 1 if delta > 0 else -1
    print("[MOTOR] angle", target_angle, "steps", abs(delta))
    for _ in range(abs(delta)):
        step_motor(direction)
    release_motor()
    print("[MOTOR] position steps", current_steps)


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        return wlan

    print("[WiFi] connecting")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    remaining = 20
    while not wlan.isconnected() and remaining > 0:
        time.sleep(1)
        remaining -= 1

    if not wlan.isconnected():
        raise RuntimeError("Wi-Fi connection failed")
    print("[WiFi] connected", wlan.ifconfig())
    return wlan


def mqtt_callback(topic, payload):
    topic_text = topic.decode() if isinstance(topic, bytes) else topic
    if topic_text != ANGLE_TOPIC:
        return

    angle = parse_angle(payload, MIN_ANGLE, MAX_ANGLE)
    if angle is None:
        print("[MQTT] ignored invalid angle", payload)
        return

    print("[MQTT] angle", angle)
    move_to_angle(angle)


def connect_mqtt():
    client = MQTTClient(
        MQTT_CLIENT_ID,
        MQTT_BROKER,
        port=MQTT_PORT,
        keepalive=60,
    )
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(ANGLE_TOPIC)
    print("[MQTT] subscribed", ANGLE_TOPIC)
    return client


def main():
    release_motor()
    while True:
        try:
            connect_wifi()
            client = connect_mqtt()
            print("[SYSTEM] ready")
            while True:
                client.check_msg()
                time.sleep(0.01)
        except Exception as exc:
            print("[SYSTEM] error", exc)
            release_motor()
            time.sleep(3)


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    release_motor()

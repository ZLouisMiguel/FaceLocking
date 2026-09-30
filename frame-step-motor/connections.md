# Stepper motor connections and deployment

The face-lock application publishes an absolute angle over MQTT. The ESP8266
receives that angle and moves the 28BYJ-48 stepper through a ULN2003 driver.
The camera frame center is neutral (90°); a locked face left of center commands
the motor left, and a locked face right of center commands it right.

## Wiring

Use a NodeMCU/Wemos D1 mini mapping:

| ESP8266 | ULN2003 |
| --- | --- |
| D1 / GPIO5 | IN1 |
| D2 / GPIO4 | IN2 |
| D3 / GPIO0 | IN3 |
| D4 / GPIO2 | IN4 |

Connect the 28BYJ-48 to the ULN2003 motor socket. Power the motor from an
appropriate external 5 V supply and connect the supply ground to ESP8266
ground. Do not power the stepper directly from an ESP8266 GPIO. The older note
about pins 1–4 is not the default: GPIO1 is the serial TX pin and should not be
used for a motor coil input.

## Firmware upload

Upload these three files to the ESP8266 filesystem using a MicroPython tool
such as `mpremote` or Thonny:

1. `main.py`
2. `motor_math.py`
3. `umqtt_simple.py`

Before uploading, edit `main.py` and set `WIFI_SSID` and `WIFI_PASSWORD`. Keep
the broker and topic equal to the PC configuration (`broker.emqx.io` and
`face/servo/angle` by default), or change both sides together.

The firmware assumes the pointer is physically at 90° when the board resets.
It releases all four coils while idle and whenever the network loop retries.


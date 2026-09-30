"""Hardware-independent helpers shared by the ESP8266 motor firmware."""

import math


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def angle_to_steps(angle, minimum=10, maximum=170, steps_per_rev=4096):
    angle = clamp(float(angle), minimum, maximum)
    return int(round((angle / 360.0) * steps_per_rev))


def parse_angle(payload, minimum=10, maximum=170):
    try:
        if isinstance(payload, bytes):
            payload = payload.decode()
        value = float(str(payload).strip())
        if not math.isfinite(value):
            return None
        return int(round(clamp(value, minimum, maximum)))
    except (TypeError, ValueError, UnicodeError):
        return None

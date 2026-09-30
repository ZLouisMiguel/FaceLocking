"""Map a locked face position to a stable stepper-motor command."""

from dataclasses import dataclass
import math
import time


SEARCHING = "SEARCHING"
LOCKED = "LOCKED"
LOST = "LOST"


def _validate_angle_range(minimum, maximum, neutral):
    if minimum > maximum or not minimum <= neutral <= maximum:
        raise ValueError("minimum <= neutral <= maximum is required")


def map_x_to_angle(
    center_x,
    frame_width,
    minimum=10,
    maximum=170,
    neutral=90,
):
    """Map a horizontal frame coordinate to a bounded absolute angle.

    The mapping is piecewise so the exact frame center always maps to the
    configured neutral angle, even when the minimum and maximum ranges are
    not symmetric around it.
    """
    if frame_width <= 0:
        raise ValueError("frame_width must be positive")
    _validate_angle_range(minimum, maximum, neutral)

    x = max(0.0, min(float(frame_width), float(center_x)))
    midpoint = float(frame_width) / 2.0
    if x <= midpoint:
        ratio = x / midpoint if midpoint else 0.0
        angle = minimum + ratio * (neutral - minimum)
    else:
        ratio = (x - midpoint) / midpoint
        angle = neutral + ratio * (maximum - neutral)
    return int(round(max(minimum, min(maximum, angle))))


def _direction_for(angle, neutral):
    if angle < neutral:
        return "LEFT"
    if angle > neutral:
        return "RIGHT"
    return "CENTER"


@dataclass(frozen=True)
class MotorCommand:
    angle: int
    direction: str
    published: bool = False
    available: bool = True


def _default_mqtt_client_factory(client_id):
    import paho.mqtt.client as mqtt

    return mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)


class MqttAnglePublisher:
    """Publish absolute angle commands without coupling recognition to MQTT."""

    def __init__(
        self,
        broker="broker.emqx.io",
        port=1883,
        topic="face/servo/angle",
        client_id="face-recognition-motor",
        keepalive=60,
        reconnect_interval=3.0,
        client_factory=None,
        clock=time.monotonic,
    ):
        self.broker = broker
        self.port = int(port)
        self.topic = topic
        self.client_id = client_id
        self.keepalive = int(keepalive)
        self.reconnect_interval = float(reconnect_interval)
        self.client_factory = client_factory or _default_mqtt_client_factory
        self.clock = clock
        self._client = None
        self.online = False
        self._next_retry_at = 0.0

    def _drop_client(self):
        client = self._client
        self._client = None
        self.online = False
        if client is None:
            return
        try:
            client.loop_stop()
        except Exception:
            pass
        try:
            client.disconnect()
        except Exception:
            pass

    def _connect(self):
        if self.online and self._client is not None:
            return True
        if self.clock() < self._next_retry_at:
            return False

        try:
            client = self.client_factory(self.client_id)
            result = client.connect(self.broker, self.port, self.keepalive)
            if result not in (None, 0):
                raise OSError(f"MQTT connect returned {result}")
            client.loop_start()
            self._client = client
            self.online = True
            self._next_retry_at = 0.0
            return True
        except Exception:
            self._drop_client()
            self._next_retry_at = self.clock() + self.reconnect_interval
            return False

    def publish_angle(self, angle):
        try:
            numeric_angle = float(angle)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(numeric_angle):
            return False
        if not self._connect():
            return False

        payload = (
            str(int(numeric_angle))
            if numeric_angle.is_integer()
            else str(numeric_angle)
        )
        try:
            result = self._client.publish(self.topic, payload, qos=0, retain=False)
            rc = getattr(result, "rc", 0)
            if rc not in (None, 0):
                raise OSError(f"MQTT publish returned {rc}")
            return True
        except Exception:
            self._drop_client()
            self._next_retry_at = self.clock() + self.reconnect_interval
            return False

    def close(self):
        self._drop_client()


class DisabledMotorPublisher:
    online = False

    def publish_angle(self, _angle):
        return False

    def close(self):
        return None


class FaceMotorController:
    """Convert tracker output into smoothed, throttled motor commands."""

    def __init__(
        self,
        publisher,
        minimum=10,
        maximum=170,
        neutral=90,
        deadband=0.05,
        smoothing=0.35,
        publish_interval=0.15,
        clock=time.monotonic,
    ):
        _validate_angle_range(minimum, maximum, neutral)
        if not 0.0 <= deadband < 0.5:
            raise ValueError("deadband must be between 0 and 0.5")
        if not 0.0 < smoothing <= 1.0:
            raise ValueError("smoothing must be greater than 0 and at most 1")
        if publish_interval < 0:
            raise ValueError("publish_interval cannot be negative")

        self.publisher = publisher
        self.minimum = minimum
        self.maximum = maximum
        self.neutral = neutral
        self.deadband = deadband
        self.smoothing = smoothing
        self.publish_interval = publish_interval
        self.clock = clock
        self._smoothed_x = None
        self._last_angle = neutral
        self._last_state = SEARCHING
        self._last_publish_at = None
        self._had_active_target = False
        self.available = bool(getattr(publisher, "online", True))

    def _command(self, angle, published=False):
        return MotorCommand(
            angle=angle,
            direction=_direction_for(angle, self.neutral),
            published=published,
            available=self.available,
        )

    def _publish_if_due(self, angle, now):
        if (
            self._last_publish_at is not None
            and now - self._last_publish_at < self.publish_interval
        ):
            return False

        try:
            published = bool(self.publisher.publish_angle(angle))
        except Exception:
            published = False

        self.available = published or self.available
        if published:
            self._last_publish_at = now
        else:
            self.available = False
        return published

    def _target_normalized_x(self, center_x, frame_width):
        if frame_width <= 0:
            raise ValueError("frame_width must be positive")
        normalized = max(0.0, min(1.0, float(center_x) / frame_width))
        if abs(normalized - 0.5) <= self.deadband:
            return 0.5
        return normalized

    def update(self, state, box, frame_width, now=None):
        """Return the desired command for one tracker update."""
        if now is None:
            now = self.clock()

        if state == LOCKED and box is not None:
            x, _y, width, _height = box
            target_x = self._target_normalized_x(x + (width / 2.0), frame_width)
            if self._smoothed_x is None:
                self._smoothed_x = target_x
            else:
                self._smoothed_x += self.smoothing * (target_x - self._smoothed_x)

            center_x = self._smoothed_x * frame_width
            angle = map_x_to_angle(
                center_x,
                frame_width,
                self.minimum,
                self.maximum,
                self.neutral,
            )
            published = self._publish_if_due(angle, now)
            self._last_angle = angle
            self._last_state = LOCKED
            self._had_active_target = True
            return self._command(angle, published)

        if state == LOST and self._had_active_target:
            self._last_state = LOST
            return self._command(self._last_angle)

        if state == SEARCHING:
            just_reset = self._had_active_target and self._last_state in (
                LOCKED,
                LOST,
            )
            self._last_state = SEARCHING
            self._smoothed_x = 0.5
            if just_reset:
                published = self._publish_if_due(self.neutral, now)
                self._last_angle = self.neutral
                self._had_active_target = False
                return self._command(self.neutral, published)
            return self._command(self._last_angle)

        return self._command(self._last_angle)

    def close(self):
        self.publisher.close()

import unittest
from types import SimpleNamespace

from src.face_signals import format_motor_status
from src.motor_control import MqttAnglePublisher, MotorCommand


class FakeMqttClient:
    def __init__(self, client_id):
        self.client_id = client_id
        self.connected_to = None
        self.started = False
        self.stopped = False
        self.disconnected = False
        self.messages = []

    def connect(self, broker, port, keepalive):
        self.connected_to = (broker, port, keepalive)
        return 0

    def loop_start(self):
        self.started = True

    def loop_stop(self):
        self.stopped = True

    def publish(self, topic, payload, qos=0, retain=False):
        self.messages.append((topic, payload, qos, retain))
        return SimpleNamespace(rc=0)

    def disconnect(self):
        self.disconnected = True


class FailingMqttClient(FakeMqttClient):
    def connect(self, broker, port, keepalive):
        raise OSError("broker unavailable")


class MqttPublisherTests(unittest.TestCase):
    def test_connects_lazily_and_publishes_ascii_angle(self):
        clients = []

        def factory(client_id):
            client = FakeMqttClient(client_id)
            clients.append(client)
            return client

        publisher = MqttAnglePublisher(
            broker="test-broker",
            port=1884,
            topic="face/servo/angle",
            client_id="test-client",
            client_factory=factory,
        )

        self.assertTrue(publisher.publish_angle(123))
        self.assertEqual(clients[0].connected_to, ("test-broker", 1884, 60))
        self.assertEqual(
            clients[0].messages,
            [("face/servo/angle", "123", 0, False)],
        )
        self.assertTrue(publisher.online)

        publisher.close()
        self.assertTrue(clients[0].stopped)
        self.assertTrue(clients[0].disconnected)

    def test_rejects_non_finite_angles_without_network_call(self):
        created = []

        def factory(client_id):
            created.append(client_id)
            return FakeMqttClient(client_id)

        publisher = MqttAnglePublisher(client_factory=factory)

        self.assertFalse(publisher.publish_angle(float("nan")))
        self.assertFalse(publisher.publish_angle(float("inf")))
        self.assertEqual(created, [])

    def test_broker_failure_is_reported_as_offline(self):
        publisher = MqttAnglePublisher(
            client_factory=lambda client_id: FailingMqttClient(client_id)
        )

        self.assertFalse(publisher.publish_angle(90))
        self.assertFalse(publisher.online)


class MotorOverlayTests(unittest.TestCase):
    def test_formats_direction_angle_and_connection_state(self):
        online = MotorCommand(170, "RIGHT", published=True, available=True)
        offline = MotorCommand(90, "CENTER", published=False, available=False)

        self.assertEqual(format_motor_status(online), "MOTOR: RIGHT 170 deg [ONLINE]")
        self.assertEqual(format_motor_status(offline), "MOTOR: CENTER 90 deg [OFFLINE]")


if __name__ == "__main__":
    unittest.main()

import unittest

from src.motor_control import FaceMotorController, map_x_to_angle


class RecordingPublisher:
    def __init__(self):
        self.angles = []
        self.closed = False

    def publish_angle(self, angle):
        self.angles.append(angle)
        return True

    def close(self):
        self.closed = True


class MotorMappingTests(unittest.TestCase):
    def test_maps_frame_edges_and_center_to_motor_range(self):
        self.assertEqual(map_x_to_angle(0, 640), 10)
        self.assertEqual(map_x_to_angle(320, 640), 90)
        self.assertEqual(map_x_to_angle(640, 640), 170)

    def test_clamps_target_centers_outside_frame(self):
        self.assertEqual(map_x_to_angle(-50, 640), 10)
        self.assertEqual(map_x_to_angle(900, 640), 170)

    def test_rejects_non_positive_frame_width(self):
        with self.assertRaises(ValueError):
            map_x_to_angle(10, 0)


class FaceMotorControllerTests(unittest.TestCase):
    def setUp(self):
        self.publisher = RecordingPublisher()
        self.controller = FaceMotorController(
            self.publisher,
            smoothing=1.0,
            deadband=0.05,
            publish_interval=0.2,
        )

    def test_reports_direction_and_publishes_locked_target(self):
        command = self.controller.update("LOCKED", (0, 0, 0, 0), 640, now=0.0)

        self.assertEqual(command.angle, 10)
        self.assertEqual(command.direction, "LEFT")
        self.assertTrue(command.published)
        self.assertEqual(self.publisher.angles, [10])

    def test_deadband_keeps_near_center_at_neutral(self):
        command = self.controller.update("LOCKED", (330, 0, 0, 0), 640, now=0.0)

        self.assertEqual(command.angle, 90)
        self.assertEqual(command.direction, "CENTER")

    def test_throttles_repeated_motor_publishes(self):
        self.controller.update("LOCKED", (0, 0, 0, 0), 640, now=0.0)
        command = self.controller.update("LOCKED", (640, 0, 0, 0), 640, now=0.1)

        self.assertFalse(command.published)
        self.assertEqual(self.publisher.angles, [10])

        command = self.controller.update("LOCKED", (640, 0, 0, 0), 640, now=0.3)
        self.assertTrue(command.published)
        self.assertEqual(self.publisher.angles, [10, 170])

    def test_holds_position_during_temporary_loss(self):
        locked = self.controller.update("LOCKED", (0, 0, 0, 0), 640, now=0.0)
        lost = self.controller.update("LOST", None, 640, now=1.0)

        self.assertEqual(lost.angle, locked.angle)
        self.assertEqual(lost.direction, "LEFT")
        self.assertFalse(lost.published)
        self.assertEqual(self.publisher.angles, [10])

    def test_returns_to_neutral_once_after_full_tracker_reset(self):
        self.controller.update("LOCKED", (0, 0, 0, 0), 640, now=0.0)
        centered = self.controller.update("SEARCHING", None, 640, now=1.0)
        repeated = self.controller.update("SEARCHING", None, 640, now=2.0)

        self.assertEqual(centered.angle, 90)
        self.assertEqual(centered.direction, "CENTER")
        self.assertTrue(centered.published)
        self.assertFalse(repeated.published)
        self.assertEqual(self.publisher.angles, [10, 90])

    def test_close_closes_the_publisher(self):
        self.controller.close()
        self.assertTrue(self.publisher.closed)


if __name__ == "__main__":
    unittest.main()

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).parents[1] / "frame-step-motor" / "motor_math.py"
SPEC = importlib.util.spec_from_file_location("motor_math", MODULE_PATH)
motor_math = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(motor_math)


class MotorMathTests(unittest.TestCase):
    def test_converts_neutral_angle_to_half_steps(self):
        self.assertEqual(motor_math.angle_to_steps(90), 1024)

    def test_clamps_angle_before_converting(self):
        self.assertEqual(motor_math.angle_to_steps(-20), 114)
        self.assertEqual(motor_math.angle_to_steps(400), 1934)

    def test_parses_and_clamps_text_or_bytes_payloads(self):
        self.assertEqual(motor_math.parse_angle("123"), 123)
        self.assertEqual(motor_math.parse_angle(b"999"), 170)

    def test_rejects_invalid_or_non_finite_payloads(self):
        self.assertIsNone(motor_math.parse_angle("not-an-angle"))
        self.assertIsNone(motor_math.parse_angle("nan"))
        self.assertIsNone(motor_math.parse_angle(b"inf"))


if __name__ == "__main__":
    unittest.main()

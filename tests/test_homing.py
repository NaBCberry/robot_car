import tempfile
import threading
import unittest
from pathlib import Path

from robot_car.roller_control.homing import HomeCancelled, home_from_config_file


class FakeActuator:
    instances = []

    def __init__(self, **_kwargs):
        self.calls = []
        self.status = 0
        type(self).instances.append(self)

    def open(self):
        self.calls.append("open")

    def enable(self):
        self.calls.append("enable")

    def home_absolute_zero(self, **_kwargs):
        self.calls.append("home")

    def cancel_home(self):
        self.calls.append("cancel")

    def read_home_status(self, timeout_s):
        self.calls.append("status")
        return self.status

    def read_position_deg(self, timeout_s):
        self.calls.append("position")
        return 0.0

    def close(self):
        self.calls.append("close")


class HomingTests(unittest.TestCase):
    def setUp(self):
        FakeActuator.instances.clear()
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "roller_control.yaml"
        self.path.write_text("""enabled: true
imu: {}
motor:
  can_interface: can0
  address: 1
  soft_limit_min_deg: -10
  soft_limit_max_deg: 10
control: {}
""", encoding="utf-8")

    def tearDown(self):
        self.directory.cleanup()

    def test_completed_home_returns_verified_position_and_closes(self):
        self.assertEqual(home_from_config_file(self.path, 1000, actuator_factory=FakeActuator), 0.0)
        self.assertEqual(FakeActuator.instances[0].calls,
                         ["open", "enable", "home", "status", "position", "close"])

    def test_cancelled_home_sends_y42_cancel_before_close(self):
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(HomeCancelled):
            home_from_config_file(self.path, 1000, cancel_event=cancel,
                                  actuator_factory=FakeActuator)
        self.assertEqual(FakeActuator.instances[0].calls,
                         ["open", "enable", "home", "cancel", "close"])


if __name__ == "__main__":
    unittest.main()

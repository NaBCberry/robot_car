import unittest

from robot_car.rollercontrold import RollerControlDaemon


class RollerControlDaemonTests(unittest.TestCase):
    def test_missing_or_invalid_feedforward_resets_to_zero(self):
        daemon = RollerControlDaemon.__new__(RollerControlDaemon)
        import threading
        daemon._feedforward_lock = threading.Lock()
        daemon._feedforward_mm_s2 = 12.5

        daemon.set_feedforward_mm_s2(None)
        self.assertEqual(daemon._feedforward(), 0.0)
        daemon.set_feedforward_mm_s2("invalid")
        self.assertEqual(daemon._feedforward(), 0.0)
        daemon.set_feedforward_mm_s2(42)
        self.assertEqual(daemon._feedforward(), 42.0)

    def test_ball_timeout_holds_motor_position_before_stopping(self):
        class FakeActuator:
            def __init__(self):
                self.hold_calls = 0
                self.stop_calls = 0

            def hold_current_position(self, **_kwargs):
                self.hold_calls += 1
                return True

            def stop(self):
                self.stop_calls += 1

        daemon = RollerControlDaemon.__new__(RollerControlDaemon)
        daemon.last_safe = False
        daemon.received_ball_state = True
        daemon.armed = True
        daemon.dry_run = False
        daemon.speed_rpm = 60
        daemon.acceleration_rpm_s = 100
        daemon.deceleration_rpm_s = 100
        daemon.actuator = FakeActuator()

        daemon._safe_stop("ball state timeout")

        self.assertEqual(daemon.actuator.hold_calls, 1)
        self.assertEqual(daemon.actuator.stop_calls, 0)


if __name__ == "__main__":
    unittest.main()

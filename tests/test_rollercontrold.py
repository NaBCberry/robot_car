import unittest
from queue import Queue

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
        daemon.last_ball_frame_id = None
        daemon.last_ball_event_age_ms = None
        daemon.accepted_ball_events = 0
        daemon.discarded_stale_ball_events = 0
        daemon.ball = None
        daemon.armed = True
        daemon.dry_run = False
        daemon.speed_rpm = 60
        daemon.acceleration_rpm_s = 100
        daemon.deceleration_rpm_s = 100
        daemon.actuator = FakeActuator()

        daemon._safe_stop("ball state timeout")

        self.assertEqual(daemon.actuator.hold_calls, 1)
        self.assertEqual(daemon.actuator.stop_calls, 0)

    def test_external_vision_queue_keeps_only_the_latest_ball_event(self):
        class Event:
            def __init__(self, frame_id):
                self.event_type = "BALL_BALANCE_STATE"
                self.frame_id = frame_id

        daemon = RollerControlDaemon.__new__(RollerControlDaemon)
        daemon._external_vision_queue = Queue(maxsize=1)

        daemon.submit_vision_event(Event(1))
        daemon.submit_vision_event(Event(2))

        self.assertEqual(daemon._external_vision_queue.get_nowait().frame_id, 2)


if __name__ == "__main__":
    unittest.main()

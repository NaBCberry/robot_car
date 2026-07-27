import unittest

from robot_car.decision.capture_target import ControlMode
from robot_car.decision.control_arbiter import ControlArbiter
from robot_car.decision.motion_target import MotionTarget
from robot_car.perception.events import VisionEvent


def ball_event(timestamp=1000):
    return VisionEvent(timestamp, "steelball", "BALL_TARGET", 0.94, {
        "stable_id": "primary_ball", "track_id": 17, "bearing_mdeg": 12000, "range_mm": 680,
    }, 8, 150, 1280, 720)


CAPTURE_CONFIG = {
    "capture": {
        "enabled": True,
        "control_mode": "MCU_TARGET_SERVO",
        "target_timeout_ms": 200,
        "feedback": {"enabled": False},
    },
}


class ControlArbiterTests(unittest.TestCase):
    def test_rdk_motion_mode_keeps_motion_command(self):
        arbiter = ControlArbiter({"capture": {"enabled": True, "control_mode": "RDK_MOTION_TARGET"}})
        motion = MotionTarget(enable=True, target_speed_mm_s=100)
        selected_motion, capture = arbiter.select(1000, motion)
        self.assertEqual(selected_motion, motion)
        self.assertIsNone(capture)

    def test_capture_mode_selects_relative_target_only(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        arbiter.handle_event(ball_event(), 1020)
        motion, capture = arbiter.select(1030, MotionTarget(enable=True))
        self.assertIsNone(motion)
        self.assertTrue(capture.target_valid)
        self.assertEqual(capture.range_mm, 680)
        self.assertFalse(capture.capture_armed)

    def test_capture_target_expires_to_explicit_safe_target(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        arbiter.handle_event(ball_event(), 1020)
        motion, capture = arbiter.select(1300, MotionTarget(enable=True))
        self.assertIsNone(motion)
        self.assertFalse(capture.target_valid)

    def test_invalid_capture_target_is_discarded_without_raising(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        invalid = VisionEvent(1000, "steelball", "BALL_TARGET", 0.94, {}, 8, 150, 1280, 720)
        arbiter.handle_event(invalid, 1020)
        _, capture = arbiter.select(1030, MotionTarget(enable=True))
        self.assertFalse(capture.target_valid)

    def test_capture_arm_is_single_high_level_event(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        arm = VisionEvent(1000, "task", "CAPTURE_ARM", 1.0, {}, 1, 150, 1, 1)
        arbiter.handle_event(arm, 1010)
        arbiter.handle_event(ball_event(), 1020)
        self.assertEqual(arbiter.consume_capture_event(), "CAPTURE_ARM")
        self.assertIsNone(arbiter.consume_capture_event())
        _, capture = arbiter.select(1030, MotionTarget(enable=True))
        self.assertTrue(capture.capture_armed)

    def test_disabled_feedback_never_claims_capture_success(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        self.assertEqual(arbiter.capture_result({"capture_state": "CAPTURED"}), "UNKNOWN")
        self.assertEqual(arbiter.capture_result({"capture_attempted": True}), "CAPTURE_ATTEMPTED")


if __name__ == "__main__":
    unittest.main()

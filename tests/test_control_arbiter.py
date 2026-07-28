import unittest

from robot_car.decision.control_arbiter import ControlArbiter
from robot_car.decision.motion_target import MotionTarget
from robot_car.perception.events import VisionEvent
from robot_car.protocol.messages import MotionMode


def ball_event(timestamp=1000):
    return VisionEvent(timestamp, "steelball", "BALL_TARGET", 0.94, {
        "stable_id": "primary_ball", "track_id": 17, "bearing_mdeg": 12000, "range_mm": 680,
    }, 8, 150, 1280, 720)


CAPTURE_CONFIG = {
    "capture": {
        "enabled": True,
        "target_timeout_ms": 200,
        "feedback": {"enabled": False},
    },
}


class ControlArbiterTests(unittest.TestCase):
    def test_non_capture_mode_keeps_fallback_motion(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        motion = MotionTarget(mode=MotionMode.LINE_FOLLOW, enabled=True)
        self.assertEqual(arbiter.select(1000, motion), motion)

    def test_capture_mode_selects_one_polar_motion_target(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        arbiter.handle_event(ball_event(), 1020)
        motion = arbiter.select(1030, MotionTarget(MotionMode.CAPTURE_TARGET_POLAR, True))
        self.assertEqual(motion.mode, MotionMode.CAPTURE_TARGET_POLAR)
        self.assertTrue(motion.enabled)
        self.assertIsNotNone(motion.capture_target)
        self.assertEqual(motion.capture_target.range_mm, 680)
        self.assertFalse(motion.capture_target.capture_armed)

    def test_expired_capture_target_becomes_explicit_safe_polar_motion(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        arbiter.handle_event(ball_event(), 1020)
        motion = arbiter.select(1300, MotionTarget(MotionMode.CAPTURE_TARGET_POLAR, True))
        self.assertEqual(motion.mode, MotionMode.CAPTURE_TARGET_POLAR)
        self.assertFalse(motion.enabled)
        self.assertIsNone(motion.capture_target)

    def test_invalid_capture_target_is_discarded_without_raising(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        invalid = VisionEvent(1000, "steelball", "BALL_TARGET", 0.94, {}, 8, 150, 1280, 720)
        arbiter.handle_event(invalid, 1020)
        motion = arbiter.select(1030, MotionTarget(MotionMode.CAPTURE_TARGET_POLAR, True))
        self.assertFalse(motion.enabled)
        self.assertIsNone(motion.capture_target)

    def test_capture_arm_is_carried_by_the_polar_motion_target(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        arm = VisionEvent(1000, "task", "CAPTURE_ARM", 1.0, {}, 1, 150, 1, 1)
        arbiter.handle_event(arm, 1010)
        arbiter.handle_event(ball_event(), 1020)
        motion = arbiter.select(1030, MotionTarget(MotionMode.CAPTURE_TARGET_POLAR, True))
        self.assertTrue(motion.capture_target.capture_armed)

    def test_disabled_feedback_never_claims_capture_success(self):
        arbiter = ControlArbiter(CAPTURE_CONFIG)
        self.assertEqual(arbiter.capture_result({"capture_state": "CAPTURED"}), "UNKNOWN")
        self.assertEqual(arbiter.capture_result({"capture_attempted": True}), "CAPTURE_ATTEMPTED")


if __name__ == "__main__":
    unittest.main()

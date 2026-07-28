import unittest

from robot_car.decision.state_machine import VehicleState, VehicleStateMachine
from robot_car.perception.events import VisionEvent
from robot_car.perception.stabilizer import EventStabilizer
from robot_car.protocol.messages import MotionMode


CONFIG = {
    "control_enabled": True,
    "initial_mode": "LINE_FOLLOW",
    "default_valid_for_ms": 200,
    "vision_timeout_ms": 500,
    "stop_hold_ms": 1000,
}


def event(event_type, timestamp=1000):
    return VisionEvent(timestamp, "test", event_type, 1.0, {}, 1, 150, 640, 480)


class StateMachineTests(unittest.TestCase):
    def test_default_line_follow_is_a_semantic_mode(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        target = machine.target(1000)
        self.assertEqual(machine.state, VehicleState.LINE_FOLLOW)
        self.assertEqual(target.mode, MotionMode.LINE_FOLLOW)
        self.assertTrue(target.enabled)

    def test_stop_is_disabled_high_level_target(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.handle_event(event("STOP"), 1000)
        self.assertEqual(machine.state, VehicleState.VISION_ASSIST)
        self.assertFalse(machine.target(1200).enabled)

    def test_slow_down_selects_mspm0_vision_assist_policy(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.handle_event(event("SLOW_DOWN"), 1000)
        target = machine.target(1001)
        self.assertEqual(target.mode, MotionMode.VISION_ASSIST)
        self.assertTrue(target.enabled)

    def test_vision_timeout_fails_safe(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.handle_event(event("INTERSECTION"), 1000)
        machine.update_safety(True, now_ms=1600)
        self.assertEqual(machine.state, VehicleState.FAILSAFE)
        self.assertFalse(machine.target(1600).enabled)

    def test_missing_vision_heartbeat_fails_after_grace(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.update_safety(True, now_ms=1000)
        machine.update_safety(True, now_ms=1600)
        self.assertEqual(machine.state, VehicleState.FAILSAFE)
        machine.handle_event(event("VISION_HEALTH", 1601), 1601)
        self.assertEqual(machine.state, VehicleState.LINE_FOLLOW)

    def test_estop_has_priority(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.update_safety(True, estop=True, now_ms=1000)
        self.assertEqual(machine.state, VehicleState.E_STOP)

    def test_control_gate_disables_motion(self):
        config = dict(CONFIG, control_enabled=False)
        machine = VehicleStateMachine(config)
        machine.start()
        target = machine.target(1000)
        self.assertFalse(target.enabled)
        self.assertEqual(target.mode, MotionMode.LINE_FOLLOW)

    def test_capture_mode_uses_polar_motion_semantics(self):
        config = dict(CONFIG, capture={"enabled": True})
        machine = VehicleStateMachine(config)
        machine.start()
        target_event = VisionEvent(1000, "steelball", "BALL_TARGET", 0.9,
                                   {"track_id": 1, "bearing_mdeg": 0, "range_mm": 300},
                                   1, 150, 640, 480)
        machine.handle_event(target_event, 1010)
        self.assertEqual(machine.state, VehicleState.CAPTURE_TARGET_POLAR)
        target = machine.target(1020)
        self.assertTrue(target.enabled)
        self.assertEqual(target.mode, MotionMode.CAPTURE_TARGET_POLAR)
        machine.handle_event(event("CAPTURE_CANCEL", 1030), 1030)
        self.assertEqual(machine.state, VehicleState.LINE_FOLLOW)


class StabilizerTests(unittest.TestCase):
    def test_requires_consecutive_observations(self):
        stabilizer = EventStabilizer(3)
        candidate = event("STOP")
        self.assertEqual(stabilizer.update([candidate], 1000, {"test"}), [])
        self.assertEqual(stabilizer.update([], 1010, {"test"}), [])
        self.assertEqual(stabilizer.update([candidate], 1020, {"test"}), [])
        self.assertEqual(stabilizer.update([candidate], 1030, {"test"}), [])
        self.assertEqual(len(stabilizer.update([candidate], 1040, {"test"})), 1)


if __name__ == "__main__":
    unittest.main()

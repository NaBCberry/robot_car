import unittest

from robot_car.decision.state_machine import VehicleState, VehicleStateMachine
from robot_car.perception.events import VisionEvent
from robot_car.perception.stabilizer import EventStabilizer


CONFIG = {
    "control_enabled": True,
    "initial_mode": "LINE_FOLLOW",
    "line_follow_speed_mm_s": 300,
    "speed_limit_mm_s": 400,
    "slow_speed_limit_mm_s": 100,
    "default_valid_for_ms": 200,
    "vision_timeout_ms": 500,
    "stop_hold_ms": 1000,
}


def event(event_type, timestamp=1000):
    return VisionEvent(timestamp, "test", event_type, 1.0, {}, 1, 150, 640, 480)


class StateMachineTests(unittest.TestCase):
    def test_default_line_follow(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        target = machine.target(1000)
        self.assertEqual(machine.state, VehicleState.LINE_FOLLOW)
        self.assertTrue(target.enable)
        self.assertEqual(target.target_speed_mm_s, 300)

    def test_stop_is_disabled_high_level_target(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.handle_event(event("STOP"), 1000)
        self.assertEqual(machine.state, VehicleState.VISION_ASSIST)
        self.assertFalse(machine.target(1200).enable)

    def test_slow_down_limits_speed(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.handle_event(event("SLOW_DOWN"), 1000)
        self.assertEqual(machine.target(1001).target_speed_mm_s, 100)

    def test_vision_timeout_fails_safe(self):
        machine = VehicleStateMachine(CONFIG)
        machine.start()
        machine.handle_event(event("INTERSECTION"), 1000)
        machine.update_safety(True, now_ms=1600)
        self.assertEqual(machine.state, VehicleState.FAILSAFE)
        self.assertFalse(machine.target(1600).enable)

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

    def test_control_gate_forces_zero(self):
        config = dict(CONFIG, control_enabled=False)
        machine = VehicleStateMachine(config)
        machine.start()
        target = machine.target(1000)
        self.assertFalse(target.enable)
        self.assertEqual(target.target_speed_mm_s, 0)


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

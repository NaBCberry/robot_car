import unittest

from robot_car.decision.balance_state import BalanceState
from robot_car.decision.control_arbiter import ControlArbiter
from robot_car.decision.motion_target import MotionTarget
from robot_car.decision.state_machine import VehicleState, VehicleStateMachine
from robot_car.perception.events import VisionEvent
from robot_car.protocol.messages import MotionMode


BALANCE_CONFIG = {"enabled": True, "state_timeout_ms": 120}


def balance_event(timestamp: int = 1000) -> VisionEvent:
    return VisionEvent(timestamp, "roller_balance", "BALL_BALANCE_STATE", 0.91, {
        "error_mm": 25,
    }, 4, 120, 1280, 720)


class BalanceControlTests(unittest.TestCase):
    def test_event_builds_valid_balance_state(self):
        state = BalanceState.from_event(balance_event(), 1020, 120)
        self.assertTrue(state.valid)
        self.assertEqual(state.error_mm, 25)

    def test_arbiter_selects_only_fresh_balance_state(self):
        arbiter = ControlArbiter({"balance": BALANCE_CONFIG})
        arbiter.handle_event(balance_event(), 1010)
        motion = arbiter.select(1020, MotionTarget(MotionMode.BALANCE_ROLLER, True, 120))
        self.assertTrue(motion.enabled)
        self.assertEqual(motion.balance_state.error_mm, 25)
        expired = arbiter.select(1140, MotionTarget(MotionMode.BALANCE_ROLLER, True, 120))
        self.assertFalse(expired.enabled)
        self.assertIsNone(expired.balance_state)

    def test_output_only_preserves_fresh_state_while_motion_is_disabled(self):
        arbiter = ControlArbiter({"balance": {**BALANCE_CONFIG, "output_only": True}})
        arbiter.handle_event(balance_event(), 1010)
        motion = arbiter.select(1020, MotionTarget(MotionMode.BALANCE_ROLLER, False, 120))
        self.assertFalse(motion.enabled)
        self.assertEqual(motion.balance_state.error_mm, 25)

    def test_state_machine_switches_to_balance_mode(self):
        machine = VehicleStateMachine({"control_enabled": True, "initial_mode": "IDLE",
                                       "default_valid_for_ms": 120, "balance": BALANCE_CONFIG})
        machine.start()
        machine.handle_event(balance_event(), 1010)
        self.assertEqual(machine.state, VehicleState.BALANCE_ROLLER)
        self.assertEqual(machine.target(1020).mode, MotionMode.BALANCE_ROLLER)


if __name__ == "__main__":
    unittest.main()

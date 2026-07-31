import unittest

from robot_car.decision.action_dispatcher import ActionDispatcher, ActionId


class ActionDispatcherTests(unittest.TestCase):
    def test_line_action_maps_to_line_follow_and_completes_at_a(self):
        dispatcher = ActionDispatcher({"control_enabled": True})
        dispatcher.request(ActionId.LINE_LAP_TO_A, now_ms=1000)
        self.assertEqual(dispatcher.motion_mode().name, "LINE_FOLLOW")
        self.assertEqual(dispatcher.snapshot(1200).status, "RUNNING")

    def test_sweep_changes_targets_and_finishes_after_negative_hold(self):
        dispatcher = ActionDispatcher({"control_enabled": True})
        dispatcher.request(ActionId.ROLLER_SWEEP, now_ms=1000)
        dispatcher.ball_error_mm = 49
        dispatcher.update(1100)
        self.assertEqual(dispatcher.phase, "RETURN_CENTER")
        self.assertEqual(dispatcher.target_mm, 0.0)
        dispatcher.ball_error_mm = 0
        dispatcher.update(1200)
        self.assertEqual(dispatcher.phase, "TO_NEGATIVE")
        self.assertEqual(dispatcher.target_mm, -50.0)
        dispatcher.ball_error_mm = -49
        dispatcher.update(1300)
        dispatcher.update(1600)
        self.assertEqual(dispatcher.snapshot(1600).status, "COMPLETE")

    def test_target_action_requires_a_measurement(self):
        dispatcher = ActionDispatcher({})
        with self.assertRaisesRegex(ValueError, "target_mm"):
            dispatcher.request(ActionId.LINE_LAP_BALANCE_TARGET, now_ms=0)

    def test_m0_checkpoint_completes_lap_action(self):
        dispatcher = ActionDispatcher({})
        dispatcher.request(ActionId.LINE_LAP_TO_A, now_ms=1000)
        dispatcher.handle_telemetry({"checkpoint": "A"}, now_ms=3200)
        self.assertEqual(dispatcher.snapshot(3200).status, "COMPLETE")
        self.assertEqual(dispatcher.snapshot(3200).reason, "checkpoint_a")


if __name__ == "__main__":
    unittest.main()

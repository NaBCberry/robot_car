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

    def test_competition_task_ids_use_task_numbers(self):
        self.assertEqual(int(ActionId.ROLLER_HOME), 1)
        self.assertEqual(int(ActionId.LINE_LAP_TO_A), 2)
        self.assertEqual(int(ActionId.ROLLER_SWEEP), 3)
        self.assertEqual(int(ActionId.LINE_TO_B_BALANCE_CENTER), 4)
        self.assertEqual(int(ActionId.LINE_LAP_BALANCE_CENTER), 5)
        self.assertEqual(int(ActionId.LINE_LAP_BALANCE_TARGET), 6)

    def test_motor_home_has_no_vehicle_motion_mode(self):
        dispatcher = ActionDispatcher({})
        dispatcher.request(ActionId.ROLLER_HOME, now_ms=1000)
        self.assertEqual(dispatcher.phase, "MOTOR_HOME")
        self.assertIsNone(dispatcher.motion_mode())

    def test_direct_task_three_progress_does_not_use_m0_balance_motion(self):
        dispatcher = ActionDispatcher({})
        dispatcher.request(ActionId.ROLLER_SWEEP, now_ms=1000)
        dispatcher.set_direct_roller_progress("TO_POSITIVE", 50.0)
        dispatcher.ball_error_mm = 50.0
        dispatcher.update(1200)
        self.assertEqual(dispatcher.phase, "DIRECT_TO_POSITIVE")
        self.assertEqual(dispatcher.target_mm, 50.0)

    def test_line_balance_actions_keep_m0_on_line_follow(self):
        for action in (ActionId.LINE_TO_B_BALANCE_CENTER,
                       ActionId.LINE_LAP_BALANCE_CENTER,
                       ActionId.LINE_LAP_BALANCE_TARGET):
            dispatcher = ActionDispatcher({})
            parameters = {"target_mm": 30} if action == ActionId.LINE_LAP_BALANCE_TARGET else {}
            dispatcher.request(action, parameters, now_ms=1000)
            self.assertEqual(dispatcher.motion_mode().name, "LINE_FOLLOW")

    def test_m0_checkpoint_completes_lap_action(self):
        dispatcher = ActionDispatcher({})
        dispatcher.request(ActionId.LINE_LAP_TO_A, now_ms=1000)
        dispatcher.handle_telemetry({"checkpoint": "A"}, now_ms=3200)
        self.assertEqual(dispatcher.snapshot(3200).status, "COMPLETE")
        self.assertEqual(dispatcher.snapshot(3200).reason, "checkpoint_a")


if __name__ == "__main__":
    unittest.main()

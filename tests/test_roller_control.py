import unittest

from robot_car.roller_control.control import (BallState, BallStateEstimator, LinearTable, Pid,
                                              PidParameters, RollerController)


def pid(limit):
    return Pid(PidParameters(1.0, 0.0, 0.0, limit, 10.0))


class RollerControlTests(unittest.TestCase):
    def test_estimator_filters_velocity_and_acceleration(self):
        estimator = BallStateEstimator(velocity_alpha=1.0, acceleration_alpha=1.0)
        estimator.update(1000, 0)
        state = estimator.update(1100, 5)
        self.assertEqual(state.velocity_mm_s, 50)
        self.assertEqual(state.acceleration_mm_s2, 500)

    def test_estimator_resets_after_measurement_gap(self):
        estimator = BallStateEstimator(max_gap_ms=100)
        estimator.update(1000, 0)
        state = estimator.update(1200, 20)
        self.assertEqual((state.velocity_mm_s, state.acceleration_mm_s2), (0, 0))

    def test_table_interpolates_and_clamps(self):
        table = LinearTable([[-50, -2], [0, 0], [50, 3]], name="slope_bias")
        self.assertEqual(table.at(-100), -2)
        self.assertEqual(table.at(25), 1.5)
        self.assertEqual(table.at(100), 3)

    def test_cascade_adds_slope_bias_and_limits_tilt(self):
        controller = RollerController(
            pid(100), pid(2_000), pid(20),
            LinearTable([[-100, -1], [0, 0], [100, 1]], name="slope_bias"),
            LinearTable([[-15, -150], [0, 0], [15, 150]], name="motor_curve"),
            target_min_mm=-100, target_max_mm=100, tube_angle_min_deg=-15,
            tube_angle_max_deg=15, motor_angle_min_deg=-120, motor_angle_max_deg=120)
        command = controller.step(20, BallState(1000, 0, 0, 0), 0, 0.02)
        self.assertTrue(command.enabled)
        self.assertGreater(command.target_tube_angle_deg, 0)
        self.assertGreater(command.target_motor_angle_deg, 0)
        self.assertLessEqual(abs(command.acceleration_command_mm_s2), 2_000)

    def test_tilt_sign_reverses_the_ball_correction_direction(self):
        common = dict(
            position_pid=pid(100), velocity_pid=pid(2_000), angle_pid=pid(20),
            slope_bias=LinearTable([[-100, 0], [100, 0]], name="slope_bias"),
            motor_by_tube_angle=LinearTable([[-15, -150], [0, 0], [15, 150]], name="motor_curve"),
            target_min_mm=-100, target_max_mm=100, tube_angle_min_deg=-15,
            tube_angle_max_deg=15, motor_angle_min_deg=-120, motor_angle_max_deg=120,
        )
        positive = RollerController(**common, tilt_sign=1)
        negative = RollerController(**common, tilt_sign=-1)
        ball = BallState(1000, 0, 0, 0)

        self.assertGreater(positive.step(20, ball, 0, 0.02).target_tube_angle_deg, 0)
        self.assertLess(negative.step(20, ball, 0, 0.02).target_tube_angle_deg, 0)

    def test_target_outside_tube_range_is_rejected(self):
        controller = RollerController(
            pid(10), pid(10), pid(10),
            LinearTable([[-10, 0], [10, 0]], name="slope_bias"),
            LinearTable([[-10, -10], [10, 10]], name="motor_curve"),
            target_min_mm=-10, target_max_mm=10, tube_angle_min_deg=-10,
            tube_angle_max_deg=10, motor_angle_min_deg=-10, motor_angle_max_deg=10)
        with self.assertRaisesRegex(ValueError, "outside"):
            controller.step(11, BallState(0, 0, 0, 0), 0, 0.1)

    def test_clamps_motor_output_to_soft_limits(self):
        controller = RollerController(
            pid(100), pid(2_000), pid(20),
            LinearTable([[-10, 10], [10, 10]], name="slope_bias"),
            LinearTable([[-10, -100], [10, 100]], name="motor_curve"),
            target_min_mm=-10, target_max_mm=10, tube_angle_min_deg=-10,
            tube_angle_max_deg=10, motor_angle_min_deg=-80, motor_angle_max_deg=80)
        command = controller.step(10, BallState(0, 0, 0, 0), 0, 0.1)
        self.assertEqual(command.target_motor_angle_deg, 80)


if __name__ == "__main__":
    unittest.main()

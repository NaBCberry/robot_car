import unittest

from robot_car.roller_control.y42_actuator import Y42Actuator


class FakeDriver:
    def __init__(self, interface, dry_run):
        self.interface = interface
        self.dry_run = dry_run
        self.opened = False
        self.closed = False

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True


class ReadingDriver(FakeDriver):
    def __init__(self, *args, responses):
        super().__init__(*args)
        self.responses = iter(responses)

    def receive(self, _timeout):
        return next(self.responses, None)


class Y42ActuatorTests(unittest.TestCase):
    def test_uses_absolute_trapezoid_commands_and_disables_on_close(self):
        calls = []
        driver = FakeDriver("can0", False)

        def build(arguments):
            calls.append(arguments)
            return arguments.address, b"payload"

        sent = []
        actuator = Y42Actuator(interface="can0", address=1,
                               driver_factory=lambda *_: driver, payload_builder=build,
                               payload_sender=lambda *args: sent.append(args))
        actuator.open()
        actuator.enable()
        actuator.move_absolute(-12.5, speed_rpm=20, acceleration_rpm_s=100,
                               deceleration_rpm_s=120)
        actuator.close()
        self.assertTrue(driver.opened)
        self.assertTrue(driver.closed)
        self.assertEqual([item.command for item in calls], ["enable", "trapezoid", "stop", "disable"])
        motion = calls[1]
        self.assertEqual((motion.direction, motion.position, motion.mode),
                         ("ccw", 12.5, "absolute-zero"))
        self.assertEqual(len(sent), 4)

    def test_clamps_position_before_building_can_payload(self):
        calls = []
        driver = FakeDriver("can0", False)

        def build(arguments):
            calls.append(arguments)
            return arguments.address, b"payload"

        actuator = Y42Actuator(interface="can0", address=1, soft_limit_min_deg=-10,
                               soft_limit_max_deg=10, driver_factory=lambda *_: driver,
                               payload_builder=build, payload_sender=lambda *_: None)
        position = actuator.move_absolute(20, speed_rpm=5, acceleration_rpm_s=30,
                                          deceleration_rpm_s=30)
        self.assertEqual(position, 10)
        self.assertEqual(calls[0].position, 10)

    def test_triggers_fixed_absolute_zero_home(self):
        calls = []
        actuator = Y42Actuator(
            interface="can0", address=1, driver_factory=lambda *_: FakeDriver("can0", False),
            payload_builder=lambda arguments: (calls.append(arguments) or (arguments.address, b"payload")),
            payload_sender=lambda *_: None,
        )

        actuator.home_absolute_zero()

        self.assertEqual((calls[0].command, calls[0].mode), ("home", 4))

    def test_moves_to_calibration_coordinate_from_current_position(self):
        calls = []
        actuator = Y42Actuator(
            interface="can0", address=1, driver_factory=lambda *_: FakeDriver("can0", False),
            payload_builder=lambda arguments: (calls.append(arguments) or (arguments.address, b"payload")),
            payload_sender=lambda *_: None,
        )

        target = actuator.move_to_coordinate(-575.6, current_angle_deg=0, speed_rpm=5,
                                             acceleration_rpm_s=30, deceleration_rpm_s=30)

        self.assertEqual(target, -575.6)
        self.assertEqual((calls[0].direction, calls[0].position, calls[0].mode),
                         ("ccw", 575.6, "relative-current"))

    def test_emm_uses_pulse_position_command(self):
        calls = []
        actuator = Y42Actuator(
            interface="can0", address=1, firmware="emm", pulses_per_revolution=3200,
            driver_factory=lambda *_: FakeDriver("can0", False),
            payload_builder=lambda arguments: (calls.append(arguments) or (arguments.address, b"payload")),
            payload_sender=lambda *_: None,
        )

        actuator.move_to_coordinate(-36, current_angle_deg=0, speed_rpm=5,
                                    acceleration_rpm_s=30, deceleration_rpm_s=30)

        self.assertEqual((calls[0].command, calls[0].direction, calls[0].position, calls[0].mode),
                         ("position", "ccw", 320, "relative-current"))

    def test_emm_absolute_position_uses_fixed_zero_coordinate(self):
        calls = []
        actuator = Y42Actuator(
            interface="can0", address=1, firmware="emm", pulses_per_revolution=3200,
            driver_factory=lambda *_: FakeDriver("can0", False),
            payload_builder=lambda arguments: (calls.append(arguments) or (arguments.address, b"payload")),
            payload_sender=lambda *_: None,
        )

        actuator.move_absolute(-36, speed_rpm=5, acceleration_rpm_s=30, deceleration_rpm_s=30)

        self.assertEqual((calls[0].command, calls[0].direction, calls[0].position, calls[0].mode),
                         ("position", "ccw", 320, "absolute-zero"))

    def test_emm_relative_target_uses_pulses(self):
        calls = []
        actuator = Y42Actuator(
            interface="can0", address=1, firmware="emm", pulses_per_revolution=3200,
            driver_factory=lambda *_: FakeDriver("can0", False),
            payload_builder=lambda arguments: (calls.append(arguments) or (arguments.address, b"payload")),
            payload_sender=lambda *_: None,
        )

        actuator.move_relative_target(-36, speed_rpm=5, acceleration_rpm_s=30,
                                      deceleration_rpm_s=30)

        self.assertEqual((calls[0].command, calls[0].direction, calls[0].position, calls[0].mode),
                         ("position", "ccw", 320, "relative-target"))

    def test_emm_converts_position_reading_to_degrees(self):
        driver = ReadingDriver("can0", False, responses=[
            (0x0100, bytes((0x36, 1, 0, 0, 0x10, 0x00, 0x6B)), True),
        ])
        actuator = Y42Actuator(interface="can0", address=1, firmware="emm",
                               driver_factory=lambda *_: driver,
                               payload_builder=lambda arguments: (arguments.address, b"payload"),
                               payload_sender=lambda *_: None)
        self.assertEqual(actuator.read_position_deg(), -22.5)

    def test_emm_converts_target_position_reading_to_degrees(self):
        driver = ReadingDriver("can0", False, responses=[
            (0x0100, bytes((0x33, 0, 0, 0, 0x10, 0x00, 0x6B)), True),
        ])
        actuator = Y42Actuator(interface="can0", address=1, firmware="emm",
                               driver_factory=lambda *_: driver,
                               payload_builder=lambda arguments: (arguments.address, b"payload"),
                               payload_sender=lambda *_: None)
        self.assertEqual(actuator.read_target_position_deg(), 22.5)

    def test_reads_multiturn_position_and_single_turn_encoder(self):
        driver = ReadingDriver("can0", False, responses=[
            (0x0100, bytes((0x36, 1, 0, 0, 0x18, 0x3A, 0x6B)), True),
            (0x0100, bytes((0x31, 0x80, 0x00, 0x6B)), True),
        ])
        actuator = Y42Actuator(interface="can0", address=1,
                               driver_factory=lambda *_: driver,
                               payload_builder=lambda arguments: (arguments.address, b"payload"),
                               payload_sender=lambda *_: None)
        self.assertEqual(actuator.read_position_deg(), -620.2)
        self.assertEqual(actuator.read_encoder_deg(), 180.0)

    def test_reads_absolute_home_status(self):
        driver = ReadingDriver("can0", False, responses=[
            (0x0100, bytes((0x3B, 0x03, 0x6B)), True),
        ])
        actuator = Y42Actuator(interface="can0", address=1,
                               driver_factory=lambda *_: driver,
                               payload_builder=lambda arguments: (arguments.address, b"payload"),
                               payload_sender=lambda *_: None)
        self.assertEqual(actuator.read_home_status(), 0x03)

    def test_reads_motor_status(self):
        driver = ReadingDriver("can0", False, responses=[
            (0x0100, bytes((0x3A, 0x01, 0x6B)), True),
        ])
        actuator = Y42Actuator(interface="can0", address=1,
                               driver_factory=lambda *_: driver,
                               payload_builder=lambda arguments: (arguments.address, b"payload"),
                               payload_sender=lambda *_: None)
        self.assertEqual(actuator.read_motor_status(), 0x01)

    def test_reports_rejected_confirmed_position_command(self):
        driver = ReadingDriver("can0", False, responses=[
            (0x0100, bytes((0xFD, 0xE2, 0x6B)), True),
        ])
        actuator = Y42Actuator(interface="can0", address=1,
                               driver_factory=lambda *_: driver,
                               payload_builder=lambda arguments: (arguments.address, bytes((0xFD, 0x6B))),
                               payload_sender=lambda *_: None)

        with self.assertRaisesRegex(RuntimeError, "rejected: status 0xE2"):
            actuator.move_to_coordinate(-10, current_angle_deg=0, speed_rpm=5,
                                        acceleration_rpm_s=30, deceleration_rpm_s=30, confirm=True)

    def test_accepts_completed_confirmed_position_command(self):
        driver = ReadingDriver("can0", False, responses=[
            (0x0100, bytes((0xFD, 0x9F, 0x6B)), True),
        ])
        actuator = Y42Actuator(interface="can0", address=1,
                               driver_factory=lambda *_: driver,
                               payload_builder=lambda arguments: (arguments.address, bytes((0xFD, 0x6B))),
                               payload_sender=lambda *_: None)

        actuator.move_to_coordinate(-10, current_angle_deg=0, speed_rpm=5,
                                    acceleration_rpm_s=30, deceleration_rpm_s=30, confirm=True)


if __name__ == "__main__":
    unittest.main()

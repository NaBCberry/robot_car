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


if __name__ == "__main__":
    unittest.main()

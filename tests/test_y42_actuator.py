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


if __name__ == "__main__":
    unittest.main()

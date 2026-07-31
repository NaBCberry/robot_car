import unittest

from robot_car.rollercontrold import RollerControlDaemon


class RollerControlDaemonTests(unittest.TestCase):
    def test_missing_or_invalid_feedforward_resets_to_zero(self):
        daemon = RollerControlDaemon.__new__(RollerControlDaemon)
        import threading
        daemon._feedforward_lock = threading.Lock()
        daemon._feedforward_mm_s2 = 12.5

        daemon.set_feedforward_mm_s2(None)
        self.assertEqual(daemon._feedforward(), 0.0)
        daemon.set_feedforward_mm_s2("invalid")
        self.assertEqual(daemon._feedforward(), 0.0)
        daemon.set_feedforward_mm_s2(42)
        self.assertEqual(daemon._feedforward(), 42.0)


if __name__ == "__main__":
    unittest.main()

import logging
import unittest

from robot_car.vehicle_tui import TuiLogHandler


class TuiLogHandlerTests(unittest.TestCase):
    def test_keeps_recent_lines_including_multiline_records(self):
        handler = TuiLogHandler(capacity=3)
        logger = logging.getLogger("test.vehicle_tui")
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        try:
            logger.info("first")
            logger.info("second\nthird")
            logger.info("fourth")
        finally:
            logger.removeHandler(handler)
        lines = handler.snapshot()
        self.assertEqual(len(lines), 3)
        self.assertTrue(lines[0].endswith("second"))
        self.assertEqual(lines[1], "third")
        self.assertTrue(lines[2].endswith("fourth"))


if __name__ == "__main__":
    unittest.main()

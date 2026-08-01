import logging
import unittest

from robot_car.vehicle_tui import TuiLogHandler, _wrap_terminal_line


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

    def test_wraps_ascii_and_double_width_text_without_losing_characters(self):
        self.assertEqual(_wrap_terminal_line("abcdefgh", 3), ["abc", "def", "gh"])
        self.assertEqual(_wrap_terminal_line("状态正常", 4), ["状态", "正常"])


if __name__ == "__main__":
    unittest.main()

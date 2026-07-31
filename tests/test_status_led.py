import unittest

from robot_car.observability.status_led import (BLUE, GREEN, OFF, RED, StatusLedController,
                                                  encode_ws2812)


class StatusLedTests(unittest.TestCase):
    def test_ws2812_frame_has_pixel_data_and_reset_tail(self):
        payload = encode_ws2812([(255, 0, 0)])
        self.assertEqual(len(payload), 33)
        self.assertEqual(payload[-24:], b"\x00" * 24)

    def test_healthy_state_maps_each_of_eight_indicators(self):
        controller = StatusLedController({"enabled": True, "count": 8}, writer=lambda _colors: None)
        colors = controller.render({
            "action": {"status": "IDLE", "ball_error_mm": 0},
            "gateway": {"received": 1, "decode_errors": 0},
            "recent_vision": True,
            "control_enabled": True,
            "balance_enabled": True,
            "fault": False,
        }, now_s=0)
        self.assertEqual(colors, [GREEN, GREEN, OFF, GREEN, BLUE, GREEN, OFF, GREEN])

    def test_fault_sets_the_safety_indicator_red(self):
        controller = StatusLedController({"enabled": True, "count": 8}, writer=lambda _colors: None)
        colors = controller.render({"action": {}, "gateway": {}, "fault": True}, now_s=0)
        self.assertEqual(colors[7], RED)


if __name__ == "__main__":
    unittest.main()

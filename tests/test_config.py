import copy
import unittest

from robot_car.config import SAFE_DEFAULTS, _validate_safe


class ConfigValidationTests(unittest.TestCase):
    def test_status_led_allows_full_scale_brightness(self):
        config = copy.deepcopy(SAFE_DEFAULTS)
        config["vehicle"]["status_led"]["brightness"] = 0.40
        _validate_safe(config)

    def test_status_led_rejects_brightness_above_full_scale(self):
        config = copy.deepcopy(SAFE_DEFAULTS)
        config["vehicle"]["status_led"]["brightness"] = 1.01
        with self.assertRaisesRegex(ValueError, "brightness"):
            _validate_safe(config)


if __name__ == "__main__":
    unittest.main()

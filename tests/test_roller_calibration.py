import tempfile
import unittest
from pathlib import Path

import yaml

from robot_car.web.roller_calibration import (build_roller_calibration,
                                               save_roller_calibration)


class RollerCalibrationTests(unittest.TestCase):
    def test_builds_pixel_scale_from_the_measured_tube_length(self):
        value = build_roller_calibration({
            "roi_xyxy": [100, 40, 600, 180],
            "center_x_px": 350,
            "tube_length_mm": 250,
            "axis_direction": -1,
        })
        self.assertEqual(value, {
            "roi_xyxy": [100.0, 40.0, 600.0, 180.0],
            "center_x_px": 350.0,
            "mm_per_pixel": 0.5,
            "axis_direction": -1,
        })

    def test_rejects_center_outside_the_selected_roi(self):
        with self.assertRaisesRegex(ValueError, "center point"):
            build_roller_calibration({
                "roi_xyxy": [100, 40, 600, 180],
                "center_x_px": 650,
                "tube_length_mm": 250,
            })

    def test_rejects_center_height_outside_the_selected_roi(self):
        with self.assertRaisesRegex(ValueError, "center point"):
            build_roller_calibration({
                "roi_xyxy": [100, 40, 600, 180],
                "center_x_px": 350,
                "center_y_px": 200,
                "tube_length_mm": 250,
            })

    def test_save_updates_only_the_roller_calibration_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.yaml"
            path.write_text("camera:\n  enabled: false\n  calibration:\n    image_to_capture_homography: []\n",
                            encoding="utf-8")
            saved = save_roller_calibration(path, {
                "roi_xyxy": [100, 40, 600, 180],
                "center_x_px": 350,
                "tube_length_mm": 250,
                "axis_direction": 1,
            })
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertEqual(document["camera"]["enabled"], False)
            self.assertEqual(document["camera"]["calibration"]["roller_balance"], saved)


if __name__ == "__main__":
    unittest.main()

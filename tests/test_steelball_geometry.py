import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from robot_car.camera.frame import CameraFrame
from robot_car.perception.steelball_adapter import SteelballAdapter
from robot_car.perception.steelball_geometry import ImageToCaptureProjector
from robot_car.perception.events import VisionEvent
from robot_car.visiond import VisionDaemon


class SteelballGeometryTests(unittest.TestCase):
    def test_preview_keeps_latest_event_between_inferences(self):
        daemon = VisionDaemon({
            "camera": {"enabled": False, "calibration": {"image_to_capture_homography": []}},
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "vision": {"plugins": []},
        })
        event = VisionEvent(1000, "steelball", "BALL_TARGET", 0.9, {}, 1, 150, 1280, 720, True)
        try:
            self.assertEqual(daemon._overlay_events([event], {"steelball"}, 1000), [event])
            self.assertEqual(daemon._overlay_events([], set(), 1040), [event])
            self.assertEqual(daemon._overlay_events([], {"steelball"}, 1080), [])
        finally:
            daemon.close()

    def test_adapter_uses_detection_runtime_for_det_model(self):
        created_configs = []

        class FakeConfig:
            def __init__(self, **kwargs):
                created_configs.append(kwargs)

        class FakeModel:
            def __init__(self, _config):
                self.predict_args = None

            def set_scheduling_params(self, **_kwargs):
                pass

            def predict(self, *args, **kwargs):
                self.predict_args = (args, kwargs)
                return [[10, 20, 30, 40]], [0.9], [0]

        with TemporaryDirectory() as directory:
            model_path = Path(directory) / "steelball_det.bin"
            model_path.touch()
            module = SimpleNamespace(YOLO26Config=FakeConfig, YOLO26Detect=FakeModel)
            adapter = SteelballAdapter("steelball", {
                "enabled": True, "config": {"model_path": str(model_path), "model_type": "det"},
            })
            with patch("robot_car.perception.steelball_adapter.importlib.import_module",
                       return_value=module) as importer:
                adapter.initialize()
            self.assertEqual(importer.call_args.args[0], "yolo26_det")
            self.assertEqual(created_configs[0]["classes_num"], 1)
            events = adapter.process(CameraFrame(1, 1, object(), 1280, 720))
            self.assertEqual(adapter.model.predict_args[1], {})
            self.assertEqual(events[0].event_type, "STEELBALL_DETECTED")

    def test_segmentation_adapter_disables_unneeded_masks(self):
        class FakeConfig:
            def __init__(self, **_kwargs):
                pass

        class FakeModel:
            def __init__(self, _config):
                self.predict_args = None

            def set_scheduling_params(self, **_kwargs):
                pass

            def predict(self, *args, **kwargs):
                self.predict_args = (args, kwargs)
                return [[10, 20, 30, 40]], [0.9], [0], []

        with TemporaryDirectory() as directory:
            model_path = Path(directory) / "steelball_seg.bin"
            model_path.touch()
            module = SimpleNamespace(YOLO26SegConfig=FakeConfig, YOLO26Seg=FakeModel)
            adapter = SteelballAdapter("steelball", {
                "enabled": True, "config": {"model_path": str(model_path), "model_type": "seg"},
            })
            with patch("robot_car.perception.steelball_adapter.importlib.import_module",
                       return_value=module) as importer:
                adapter.initialize()
            self.assertEqual(importer.call_args.args[0], "yolo26_seg")
            adapter.process(CameraFrame(1, 1, object(), 1280, 720))
            self.assertEqual(adapter.model.predict_args[1], {"return_masks": False})

    def test_visiond_injects_camera_calibration_into_steelball_plugin(self):
        homography = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        daemon = VisionDaemon({
            "camera": {"enabled": False, "calibration": {
                "image_to_capture_homography": homography,
            }},
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "vision": {"plugins": [{
                "name": "steelball", "enabled": False, "type": "steelball_segmentation",
                "config": {"image_to_capture_homography": []},
            }]},
        })
        try:
            adapter = daemon.scheduler.slots[0].plugin
            self.assertEqual(adapter.config["config"]["image_to_capture_homography"], homography)
            self.assertEqual(adapter.config["config"]["calibration_path"], "")
        finally:
            daemon.close()

    def test_visiond_reports_separate_camera_and_preview_rates(self):
        daemon = VisionDaemon({
            "camera": {"enabled": False, "calibration": {}},
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "vision": {"plugins": []},
        })
        try:
            daemon._update_rate("camera", 1000)
            daemon._update_rate("camera", 1100)
            daemon._update_rate("preview", 1000)
            daemon._update_rate("preview", 1050)
            daemon.latest_frame_width = 1280
            daemon.latest_frame_height = 720
            status = daemon.status()
            self.assertEqual(status["camera_fps"], 10.0)
            self.assertEqual(status["preview_fps"], 20.0)
            self.assertEqual((status["frame_width"], status["frame_height"]), (1280, 720))
        finally:
            daemon.close()

    def test_adapter_prefers_lowest_ball_over_higher_confidence(self):
        adapter = SteelballAdapter("steelball", {"config": {"target_class_id": 0}})
        selected = adapter._select_primary(
            [[100, 100, 140, 180], [300, 250, 360, 400]],
            [0.99, 0.60],
            [0, 0],
        )
        self.assertEqual(selected[0], [300, 250, 360, 400])

    def test_identity_homography_projects_capture_coordinates(self):
        projector = ImageToCaptureProjector([1, 0, 0, 0, 1, 0, 0, 0, 1])
        target = projector.project(300, 400)
        self.assertEqual(target.forward_mm, 300)
        self.assertEqual(target.lateral_mm, 400)
        self.assertEqual(target.range_mm, 500)
        self.assertAlmostEqual(target.bearing_mdeg, 53130, delta=1)

    def test_steelball_event_requires_calibration_for_target_coordinates(self):
        frame = CameraFrame(8, 1000, None, 1280, 720)
        adapter = SteelballAdapter("steelball", {"enabled": True, "config": {}})
        event = adapter._event_from_detection(frame, [100, 200, 200, 300], 0.9, 0)
        self.assertEqual(event.event_type, "STEELBALL_DETECTED")
        self.assertNotIn("range_mm", event.payload)

    def test_steelball_event_uses_capture_point_homography(self):
        frame = CameraFrame(8, 1000, None, 1280, 720)
        adapter = SteelballAdapter("steelball", {"enabled": True, "config": {}})
        adapter.projector = ImageToCaptureProjector([1, 0, 0, 0, 1, 0, 0, 0, 1])
        event = adapter._event_from_detection(frame, [100, 200, 200, 300], 0.9, 0)
        self.assertEqual(event.event_type, "BALL_TARGET")
        self.assertEqual(event.payload["forward_mm"], 150)
        self.assertEqual(event.payload["lateral_mm"], 300)
        self.assertEqual(event.payload["range_mm"], 335)


if __name__ == "__main__":
    unittest.main()

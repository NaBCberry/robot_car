import json
import unittest
from urllib.request import Request, urlopen

from robot_car.web.server import DebugServer, mjpeg_part


class MjpegPartTests(unittest.TestCase):
    def test_mjpeg_part_contains_one_complete_jpeg_frame(self):
        self.assertEqual(
            mjpeg_part(b"preview-bytes"),
            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 13\r\n\r\npreview-bytes\r\n",
        )


class DebugServerTests(unittest.TestCase):
    def setUp(self):
        self.saved_calibration = None

        def save_calibration(value):
            self.saved_calibration = value
            return {"roller_balance": value, "restart_required": True}

        try:
            self.server = DebugServer(
                "127.0.0.1", 0,
                lambda: {"healthy": True},
                lambda: {"events": []},
                lambda: {"frames_received": 1},
                lambda: b"preview-bytes",
                lambda sequence, _timeout: (1, b"preview-bytes") if sequence < 1 else (sequence, None),
                save_calibration,
            )
        except PermissionError:
            self.skipTest("sandbox disallows local TCP listeners")
        self.server.start()
        host, port = self.server.server.server_address[:2]
        self.base_url = f"http://{host}:{port}"

    def tearDown(self):
        self.server.close()

    def test_page_frame_json_and_calibration_endpoints(self):
        with urlopen(f"{self.base_url}/", timeout=1) as response:
            page = response.read().decode("utf-8")
            self.assertEqual(response.headers["Content-Type"], "text/html; charset=utf-8")
        self.assertIn("钢球视觉监控", page)
        self.assertIn('href="/calibration"', page)
        self.assertIn("相机帧率", page)
        self.assertIn("网页预览帧率", page)
        self.assertIn('src="/video_feed"', page)
        self.assertIn("/api/updates.json", page)

        with urlopen(f"{self.base_url}/api/frame.jpg", timeout=1) as response:
            self.assertEqual(response.headers["Content-Type"], "image/jpeg")
            self.assertEqual(response.read(), b"preview-bytes")

        with urlopen(f"{self.base_url}/video_feed", timeout=1) as response:
            self.assertEqual(response.headers["Content-Type"], "multipart/x-mixed-replace; boundary=frame")
            expected = mjpeg_part(b"preview-bytes")
            self.assertEqual(response.read(len(expected)), expected)

        with urlopen(f"{self.base_url}/api/results", timeout=1) as response:
            self.assertEqual(json.loads(response.read()), {"events": []})

        with urlopen(f"{self.base_url}/api/updates.json", timeout=1) as response:
            self.assertEqual(json.loads(response.read()),
                             {"status": {"healthy": True}, "results": {"events": []}})

        with urlopen(f"{self.base_url}/calibration", timeout=1) as response:
            calibration_page = response.read().decode("utf-8")
            self.assertIn("管槽钢球标定", calibration_page)
            self.assertIn("帧率读取中", calibration_page)
            self.assertIn("sourceSize", calibration_page)
            self.assertIn("scaleMarks", calibration_page)
            self.assertIn("index*10-120", calibration_page)
            self.assertIn("cursor-guide", calibration_page)

        request = Request(
            f"{self.base_url}/api/roller_balance/calibration",
            data=json.dumps({"roi_xyxy": [1, 2, 3, 4]}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urlopen(request, timeout=1) as response:
            self.assertEqual(json.loads(response.read())["restart_required"], True)
        self.assertEqual(self.saved_calibration, {"roi_xyxy": [1, 2, 3, 4]})


if __name__ == "__main__":
    unittest.main()

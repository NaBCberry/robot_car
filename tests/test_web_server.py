import json
import unittest
from urllib.request import urlopen

from robot_car.web.server import DebugServer, mjpeg_part


class MjpegPartTests(unittest.TestCase):
    def test_mjpeg_part_contains_one_complete_jpeg_frame(self):
        self.assertEqual(
            mjpeg_part(b"preview-bytes"),
            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 13\r\n\r\npreview-bytes\r\n",
        )


class DebugServerTests(unittest.TestCase):
    def setUp(self):
        try:
            self.server = DebugServer(
                "127.0.0.1", 0,
                lambda: {"healthy": True},
                lambda: {"events": []},
                lambda: {"frames_received": 1},
                lambda: b"preview-bytes",
                lambda sequence, _timeout: (1, b"preview-bytes") if sequence < 1 else (sequence, None),
            )
        except PermissionError:
            self.skipTest("sandbox disallows local TCP listeners")
        self.server.start()
        host, port = self.server.server.server_address[:2]
        self.base_url = f"http://{host}:{port}"

    def tearDown(self):
        self.server.close()

    def test_page_frame_and_json_endpoints_are_read_only(self):
        with urlopen(f"{self.base_url}/", timeout=1) as response:
            page = response.read().decode("utf-8")
            self.assertEqual(response.headers["Content-Type"], "text/html; charset=utf-8")
        self.assertIn("钢球识别", page)
        self.assertIn('src="/video_feed"', page)

        with urlopen(f"{self.base_url}/api/frame.jpg", timeout=1) as response:
            self.assertEqual(response.headers["Content-Type"], "image/jpeg")
            self.assertEqual(response.read(), b"preview-bytes")

        with urlopen(f"{self.base_url}/video_feed", timeout=1) as response:
            self.assertEqual(response.headers["Content-Type"], "multipart/x-mixed-replace; boundary=frame")
            expected = mjpeg_part(b"preview-bytes")
            self.assertEqual(response.read(len(expected)), expected)

        with urlopen(f"{self.base_url}/api/results", timeout=1) as response:
            self.assertEqual(json.loads(response.read()), {"events": []})


if __name__ == "__main__":
    unittest.main()

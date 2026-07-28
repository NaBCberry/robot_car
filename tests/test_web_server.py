import json
import unittest
from urllib.request import urlopen

from robot_car.web.server import DebugServer


class DebugServerTests(unittest.TestCase):
    def setUp(self):
        try:
            self.server = DebugServer(
                "127.0.0.1", 0,
                lambda: {"healthy": True},
                lambda: {"events": []},
                lambda: {"frames_received": 1},
                lambda: b"preview-bytes",
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
        self.assertIn("/api/frame.jpg", page)

        with urlopen(f"{self.base_url}/api/frame.jpg", timeout=1) as response:
            self.assertEqual(response.headers["Content-Type"], "image/jpeg")
            self.assertEqual(response.read(), b"preview-bytes")

        with urlopen(f"{self.base_url}/api/results", timeout=1) as response:
            self.assertEqual(json.loads(response.read()), {"events": []})


if __name__ == "__main__":
    unittest.main()

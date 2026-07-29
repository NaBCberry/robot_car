import importlib.util
import sys
import unittest
from pathlib import Path


def load_capture_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "capture_protocol_frames.py"
    spec = importlib.util.spec_from_file_location("capture_protocol_frames", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CaptureProtocolFramesTests(unittest.TestCase):
    def test_decodes_one_byte_length_balance_error_frame(self):
        capture = load_capture_module()
        frame = bytes.fromhex("A5 5A 02 01 0F 5D 06 05 08 00 78 00 4B 76 A2")
        message = capture.FrameDecoder().feed(frame)[0]
        self.assertEqual(capture.HEADER_STRUCT.size, 5)
        self.assertEqual(capture.MAX_PAYLOAD, 255)
        self.assertEqual(capture.unpack_motion(message.payload), {
            "mode": capture.MotionMode.BALANCE_ROLLER,
            "enabled": False,
            "flags": 8,
            "valid_for_ms": 120,
            "balance_valid": True,
            "error_mm": 75,
        })


if __name__ == "__main__":
    unittest.main()

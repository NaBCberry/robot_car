#!/usr/bin/env python3
"""Replay JSON-lines VisionEvent recordings to the local IPC socket."""

import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from robot_car.ipc.vision_socket import VisionEventPublisher
from robot_car.perception.events import VisionEvent


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording")
    parser.add_argument("--socket", default="/userdata/robot-car/runtime/vision-replay.sock")
    parser.add_argument("--interval-ms", type=int, default=100)
    args = parser.parse_args()
    publisher = VisionEventPublisher(args.socket)
    publisher.start()
    try:
        with open(args.recording, "r", encoding="utf-8") as stream:
            for line in stream:
                publisher.publish(VisionEvent.from_dict(json.loads(line)))
                time.sleep(args.interval_ms / 1000.0)
    finally:
        publisher.close()


if __name__ == "__main__":
    main()

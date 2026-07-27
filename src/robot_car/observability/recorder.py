"""JSON-lines event recorder writing only under the runtime data root."""

import json
from pathlib import Path
from typing import Any, Dict


class EventRecorder:
    def __init__(self, directory: Path, enabled: bool = False) -> None:
        self.enabled = enabled
        self.path = directory / "vision-events.jsonl"

    def record(self, item: Dict[str, Any]) -> None:
        if self.enabled:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(item, ensure_ascii=False) + "\n")

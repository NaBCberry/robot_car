"""High-level motion intent sent to the MSPM0 controller."""

from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional

from robot_car.protocol.messages import MotionMode

from .capture_target import CaptureTarget


@dataclass(frozen=True)
class MotionTarget:
    mode: MotionMode = MotionMode.IDLE
    enabled: bool = False
    valid_for_ms: int = 200
    capture_target: Optional[CaptureTarget] = None

    def safe(self) -> "MotionTarget":
        return MotionTarget(mode=MotionMode.DISABLED, enabled=False, valid_for_ms=self.valid_for_ms)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

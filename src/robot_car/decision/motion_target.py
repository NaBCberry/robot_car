"""High-level motion target sent to the MSPM0 controller."""

from dataclasses import asdict, dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class MotionTarget:
    mode: str = "IDLE"
    enable: bool = False
    target_speed_mm_s: int = 0
    target_steering_mdeg: int = 0
    speed_limit_mm_s: int = 0
    valid_for_ms: int = 200

    def safe(self) -> "MotionTarget":
        return MotionTarget(mode="IDLE", enable=False, valid_for_ms=self.valid_for_ms)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

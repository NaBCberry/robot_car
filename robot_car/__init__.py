"""Repository-local import shim for the src-layout package."""

from pathlib import Path


_source_package = Path(__file__).resolve().parent.parent / "src" / "robot_car"
__path__.append(str(_source_package))

__version__ = "0.1.0"

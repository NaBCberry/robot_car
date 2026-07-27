"""Process logging setup."""

import logging
from pathlib import Path


def configure_logging(name: str, level: str, log_dir: Path) -> logging.Logger:
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger()
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    if not logger.handlers:
        formatter = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
        console = logging.StreamHandler()
        console.setFormatter(formatter)
        file_handler = logging.FileHandler(log_dir / f"{name}.log", encoding="utf-8")
        file_handler.setFormatter(formatter)
        logger.addHandler(console)
        logger.addHandler(file_handler)
    return logging.getLogger(name)

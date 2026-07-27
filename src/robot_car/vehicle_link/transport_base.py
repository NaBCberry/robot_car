"""Transport abstraction."""

from abc import ABC, abstractmethod
from typing import Optional


class Transport(ABC):
    @abstractmethod
    def open(self) -> None:
        pass

    @abstractmethod
    def send(self, data: bytes) -> None:
        pass

    @abstractmethod
    def receive(self, timeout: float = 0.0) -> Optional[bytes]:
        pass

    @abstractmethod
    def close(self) -> None:
        pass

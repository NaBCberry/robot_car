"""Newline-delimited JSON fan-out over a local Unix Domain Socket."""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from pathlib import Path
from typing import List, Optional

from robot_car.perception.events import VisionEvent

from .schemas import event_envelope, parse_envelope


LOG = logging.getLogger(__name__)


class VisionEventPublisher:
    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self._server: Optional[socket.socket] = None
        self._clients: List[socket.socket] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            if not self.path.is_socket():
                raise RuntimeError(f"refusing to replace non-socket path: {self.path}")
            self.path.unlink()
        self._server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server.bind(str(self.path))
        os.chmod(self.path, 0o660)
        self._server.listen(8)
        self._server.settimeout(0.2)
        self._thread = threading.Thread(target=self._accept_loop, name="vision-ipc", daemon=True)
        self._thread.start()

    def _accept_loop(self) -> None:
        while not self._stop.is_set() and self._server is not None:
            try:
                client, _ = self._server.accept()
                with self._lock:
                    self._clients.append(client)
            except socket.timeout:
                continue
            except OSError:
                break

    def publish(self, event: VisionEvent) -> None:
        data = (json.dumps(event_envelope(event), separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
        with self._lock:
            clients = list(self._clients)
        failed = []
        for client in clients:
            try:
                client.sendall(data)
            except OSError:
                failed.append(client)
        if failed:
            with self._lock:
                for client in failed:
                    if client in self._clients:
                        self._clients.remove(client)
                    client.close()

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def close(self) -> None:
        self._stop.set()
        if self._server is not None:
            self._server.close()
            self._server = None
        if self._thread:
            self._thread.join(timeout=1.0)
        with self._lock:
            for client in self._clients:
                client.close()
            self._clients.clear()
        try:
            if self.path.is_socket():
                self.path.unlink()
        except FileNotFoundError:
            pass


class VisionEventSubscriber:
    def __init__(self, path: str, reconnect_delay: float = 0.1) -> None:
        self.path = path
        self.reconnect_delay = reconnect_delay
        self._socket: Optional[socket.socket] = None
        self._buffer = bytearray()

    def connect(self) -> bool:
        self.close()
        candidate = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            candidate.connect(self.path)
        except OSError:
            candidate.close()
            return False
        self._socket = candidate
        return True

    def receive(self, timeout: float = 0.1) -> Optional[VisionEvent]:
        if self._socket is None and not self.connect():
            time.sleep(min(timeout, self.reconnect_delay))
            return None
        self._socket.settimeout(timeout)
        while b"\n" not in self._buffer:
            try:
                data = self._socket.recv(65536)
            except socket.timeout:
                return None
            except OSError:
                self.close()
                return None
            if not data:
                self.close()
                return None
            self._buffer.extend(data)
        line, _, remainder = self._buffer.partition(b"\n")
        self._buffer = bytearray(remainder)
        value = json.loads(line.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("IPC message must be an object")
        return parse_envelope(value)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._buffer.clear()

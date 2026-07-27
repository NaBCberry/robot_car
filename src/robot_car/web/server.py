"""Small standard-library JSON debug server."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Optional


JsonProvider = Callable[[], Dict[str, Any]]


class DebugServer:
    def __init__(self, host: str, port: int, status: JsonProvider,
                 results: JsonProvider, metrics: JsonProvider) -> None:
        providers = {"/api/status": status, "/api/results": results, "/api/metrics": metrics}

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                provider = providers.get(self.path.split("?", 1)[0])
                if provider is None:
                    self.send_error(404)
                    return
                body = json.dumps(provider(), ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format: str, *args: object) -> None:
                del format, args

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self.server.serve_forever, name="vision-web", daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        if self.thread:
            self.thread.join(timeout=1.0)

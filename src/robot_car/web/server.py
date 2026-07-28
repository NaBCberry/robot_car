"""Read-only recognition page and JSON debug server."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Iterator, Optional, Tuple


JsonProvider = Callable[[], Dict[str, Any]]
FrameProvider = Callable[[], Optional[bytes]]
FrameWaiter = Callable[[int, float], Tuple[int, Optional[bytes]]]


INDEX_PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>钢球识别</title><style>
*{box-sizing:border-box}body{margin:0;background:#f2f5f7;color:#17212b;font:14px Arial,sans-serif}
header{height:52px;background:#173b4d;color:#fff;display:flex;align-items:center;padding:0 22px;gap:18px}
header strong{font-size:16px}#state{color:#b7e1d0}.layout{max-width:1440px;margin:0 auto;padding:18px;display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:18px}
.feed{background:#111b22;min-height:360px;display:flex;align-items:center;justify-content:center}.feed img{display:block;width:100%;height:auto;max-height:calc(100vh - 88px);object-fit:contain}
aside{background:#fff;border:1px solid #d8e0e5;padding:16px;align-self:start}.label{color:#60717f;font-size:12px;margin-top:14px}.value{font:600 18px Arial,sans-serif;margin-top:4px;overflow-wrap:anywhere}.muted{color:#7c8993}
@media(max-width:760px){.layout{grid-template-columns:1fr;padding:10px}.feed{min-height:220px}aside{display:grid;grid-template-columns:1fr 1fr;gap:0 18px}.label{margin-top:10px}}
</style></head><body><header><strong>钢球识别</strong><span id="state">连接中</span></header><main class="layout"><section class="feed"><img id="frame" src="/video_feed" alt="实时相机画面"></section><aside>
<div class="label">目标状态</div><div class="value" id="target">未检测到目标</div><div class="label">方位角</div><div class="value" id="bearing">-</div><div class="label">距离</div><div class="value" id="range">-</div><div class="label">置信度</div><div class="value" id="confidence">-</div><div class="label">帧号</div><div class="value" id="frame-id">-</div></aside></main><script>
const text=(id,value)=>document.getElementById(id).textContent=value;
async function refresh(){try{const [status,result]=await Promise.all([fetch('/api/status',{cache:'no-store'}).then(r=>r.json()),fetch('/api/results',{cache:'no-store'}).then(r=>r.json())]);const event=(result.events||[]).find(e=>e.event_type==='BALL_TARGET')||(result.events||[])[0];text('state',status.healthy?'视觉服务运行中':'视觉服务异常');if(event){const p=event.payload||{};text('target',event.event_type);text('bearing',p.bearing_mdeg===undefined?'-':(p.bearing_mdeg/1000).toFixed(1)+' deg');text('range',p.range_mm===undefined?'-':p.range_mm+' mm');text('confidence',(event.confidence*100).toFixed(1)+'%');text('frame-id',event.frame_id);}else{for(const id of ['target','bearing','range','confidence','frame-id'])text(id,id==='target'?'未检测到目标':'-');}}catch(_){text('state','无法连接视觉服务');}}
refresh();setInterval(refresh,500);
</script></body></html>"""


def mjpeg_part(frame: bytes) -> bytes:
    """Encode one JPEG as a multipart MJPEG payload."""
    return (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(frame)).encode("ascii") + b"\r\n\r\n" + frame + b"\r\n")


class DebugServer:
    def __init__(self, host: str, port: int, status: JsonProvider,
                 results: JsonProvider, metrics: JsonProvider, frame: FrameProvider,
                 wait_frame: FrameWaiter) -> None:
        providers = {"/api/status": status, "/api/results": results, "/api/metrics": metrics}

        def frame_stream() -> Iterator[bytes]:
            sequence = 0
            while True:
                sequence, image = wait_frame(sequence, 1.0)
                if image is not None:
                    yield mjpeg_part(image)

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                path = self.path.split("?", 1)[0]
                if path in {"/", "/index.html"}:
                    body = INDEX_PAGE.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path == "/api/frame.jpg":
                    body = frame()
                    if body is None:
                        self.send_error(503, "camera frame is not ready")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path == "/video_feed":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    try:
                        for body in frame_stream():
                            self.wfile.write(body)
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return
                provider = providers.get(path)
                if provider is None:
                    self.send_error(404)
                    return
                body = json.dumps(provider(), ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format: str, *args: object) -> None:
                del format, args

        class Server(ThreadingHTTPServer):
            allow_reuse_address = True
            daemon_threads = True

        self.server = Server((host, port), Handler)
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self.server.serve_forever, name="vision-web", daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        if self.thread:
            self.thread.join(timeout=1.0)

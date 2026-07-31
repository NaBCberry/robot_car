"""Recognition, roller calibration, and JSON debug server."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Iterator, Optional, Tuple


JsonProvider = Callable[[], Dict[str, Any]]
FrameProvider = Callable[[], Optional[bytes]]
FrameWaiter = Callable[[int, float], Tuple[int, Optional[bytes]]]
CalibrationSaver = Callable[[Dict[str, Any]], Dict[str, Any]]


INDEX_PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>钢球视觉监控</title><style>
*{box-sizing:border-box}body{margin:0;background:#f2f5f7;color:#17212b;font:14px Arial,sans-serif}
header{height:52px;background:#173b4d;color:#fff;display:flex;align-items:center;padding:0 22px;gap:18px}
header strong{font-size:16px}header a{color:#d7e7ef;text-decoration:none;padding:7px 2px;border-bottom:2px solid transparent}header a:hover{border-color:#80c6ad}#state{color:#b7e1d0}.layout{max-width:1440px;margin:0 auto;padding:18px;display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:18px}
.feed{background:#111b22;min-height:360px;display:flex;align-items:center;justify-content:center}.feed img{display:block;width:100%;height:auto;max-height:calc(100vh - 88px);object-fit:contain}
aside{background:#fff;border:1px solid #d8e0e5;padding:16px;align-self:start}.label{color:#60717f;font-size:12px;margin-top:14px}.value{font:600 18px Arial,sans-serif;margin-top:4px;overflow-wrap:anywhere}.muted{color:#7c8993}
@media(max-width:760px){.layout{grid-template-columns:1fr;padding:10px}.feed{min-height:220px}aside{display:grid;grid-template-columns:1fr 1fr;gap:0 18px}.label{margin-top:10px}}
</style></head><body><header><strong>钢球视觉监控</strong><a href="/">监控</a><a href="/calibration">手工标定</a><span id="state">连接中</span></header><main class="layout"><section class="feed"><img id="frame" src="/video_feed" alt="实时相机画面"></section><aside>
<div class="label">状态</div><div class="value" id="target">未检测到钢球</div><div class="label">相机帧率</div><div class="value" id="camera-fps">-</div><div class="label">网页预览帧率</div><div class="value" id="preview-fps">-</div><div class="label">中心距离偏差</div><div class="value" id="error">-</div><div class="label">置信度</div><div class="value" id="confidence">-</div><div class="label">帧号</div><div class="value" id="frame-id">-</div></aside></main><script>
const text=(id,value)=>document.getElementById(id).textContent=value;
function render(status,result){const events=result.events||[];const balance=events.find(e=>e.event_type==='BALL_BALANCE_STATE');const target=events.find(e=>e.event_type==='BALL_TARGET');text('state',status.healthy?'视觉服务运行中':'视觉服务异常');text('camera-fps',Number(status.camera_fps||0).toFixed(1)+' fps');text('preview-fps',Number(status.preview_fps||0).toFixed(1)+' fps');if(balance){const p=balance.payload||{};text('target','平衡状态有效');text('error',(p.error_mm??'-')+' mm');text('confidence',(balance.confidence*100).toFixed(1)+'%');text('frame-id',balance.frame_id);}else if(target){text('target','捕获目标有效');text('error','-');text('confidence',(target.confidence*100).toFixed(1)+'%');text('frame-id',target.frame_id);}else{for(const id of ['error','confidence','frame-id'])text(id,'-');text('target','未检测到钢球');}}
async function refresh(){try{const value=await fetch('/api/updates.json',{cache:'no-store'}).then(r=>r.json());render(value.status,value.results);const fps=Math.max(1,Number(value.status.preview_fps)||30);setTimeout(refresh,1000/fps)}catch(_){text('state','等待视觉数据');setTimeout(refresh,1000)}}refresh()
</script></body></html>"""


CALIBRATION_PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>管槽钢球标定</title><style>
*{box-sizing:border-box}body{margin:0;background:#f2f5f7;color:#17212b;font:14px Arial,sans-serif}
header{height:52px;background:#173b4d;color:#fff;display:flex;align-items:center;padding:0 22px;gap:18px}header strong{font-size:16px}header a{color:#d7e7ef;text-decoration:none;padding:7px 2px;border-bottom:2px solid transparent}header a.active{border-color:#80c6ad}
.layout{max-width:1440px;margin:0 auto;padding:18px;display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:18px}.feed{background:#111b22;min-height:360px;display:flex;align-items:center;justify-content:center}.stage{position:relative;width:100%;line-height:0}.stage img{display:block;width:100%;height:auto;max-height:calc(100vh - 88px);object-fit:contain}.stage canvas{position:absolute;inset:0;width:100%;height:100%;cursor:crosshair}
aside{background:#fff;border:1px solid #d8e0e5;padding:16px;align-self:start}.step{color:#60717f;margin:0 0 14px;line-height:1.5}.label{display:block;color:#60717f;font-size:12px;margin-top:14px}.input{width:100%;height:34px;margin-top:5px;border:1px solid #bac8d1;padding:6px 8px;font:14px Arial,sans-serif}.choice{display:flex;gap:12px;margin-top:8px}.action{width:100%;margin-top:18px;height:36px;background:#1e6b59;color:#fff;border:0;cursor:pointer;font:600 14px Arial,sans-serif}.action:disabled{background:#9faeb7;cursor:default}.reset{margin-top:10px;width:100%;height:32px;background:#fff;border:1px solid #aebdc6;cursor:pointer}.status{min-height:38px;margin-top:12px;line-height:1.45;color:#60717f}.status.error{color:#a42d24}.status.ok{color:#176146}.point{font-family:monospace;font-size:12px;color:#51636f;margin:6px 0}.cursor-guide{position:fixed;top:0;bottom:0;width:1px;background:#5ec9ef;box-shadow:0 0 3px #173b4d;pointer-events:none;z-index:10}
@media(max-width:760px){.layout{grid-template-columns:1fr;padding:10px}.feed{min-height:220px}}
</style></head><body><div class="cursor-guide" id="cursor-guide" hidden></div><header><strong>管槽钢球标定</strong><a href="/">监控</a><a class="active" href="/calibration">手工标定</a><span id="fps">帧率读取中</span></header><main class="layout"><section class="feed"><div class="stage"><img id="frame" src="/video_feed" alt="实时相机画面"><canvas id="canvas"></canvas></div></section><aside>
<p class="step" id="step">依次点击：管槽 ROI 左上角。</p><div class="point" id="points">尚未选点</div><label class="label">管槽有效长度（mm）<input class="input" id="length" type="number" min="20" max="2000" step="1" value="250"></label><span class="label">正方向</span><div class="choice"><label><input type="radio" name="direction" value="1" checked> 图像向右为正</label><label><input type="radio" name="direction" value="-1"> 图像向左为正</label></div><button class="action" id="save" disabled>保存标定</button><button class="reset" id="reset" type="button">重新选点</button><div class="status" id="status">标定只写入配置，不会启动电机。保存后需重启 visiond。</div></aside></main><script>
const frame=document.getElementById('frame'),canvas=document.getElementById('canvas'),ctx=canvas.getContext('2d'),cursorGuide=document.getElementById('cursor-guide');
const setupText=['点击：管槽 ROI 左上角。','点击：管槽 ROI 右下角。'];const setupKeys=['左上','右下'];const scaleMarks=Array.from({length:25},(_,index)=>index*10-120);let setupPoints=[],scalePoints=[],sourceSize={width:0,height:0};
function sync(){const rect=frame.getBoundingClientRect();canvas.width=Math.max(1,Math.round(rect.width));canvas.height=Math.max(1,Math.round(rect.height));draw()}
function showStatus(value,kind=''){const element=document.getElementById('status');element.textContent=value;element.className='status '+kind}
function labelFor(positionMm){return (positionMm/10).toFixed(0)+' cm'}
function update(){let message;if(setupPoints.length<setupText.length)message=setupText[setupPoints.length];else if(scalePoints.length<scaleMarks.length)message='点击刻度：'+labelFor(scaleMarks[scalePoints.length])+'。';else message='全部 25 个刻度已选定，确认后保存。';document.getElementById('step').textContent=message;const setupSummary=setupPoints.length?setupPoints.map((point,index)=>setupKeys[index]+' ('+point.x.toFixed(1)+', '+point.y.toFixed(1)+')').join('  '):'尚未选点';const scaleSummary=scalePoints.length?'已标定刻度 '+scalePoints.length+'/25（最后：'+labelFor(scalePoints[scalePoints.length-1].position_mm)+'）':'尚未标定刻度';document.getElementById('points').textContent=setupSummary+'；'+scaleSummary;document.getElementById('save').disabled=setupPoints.length!==2||scalePoints.length!==scaleMarks.length;if(setupPoints.length!==2||scalePoints.length===scaleMarks.length)cursorGuide.hidden=true;draw()}
function drawPoint(x,y,color,label){ctx.fillStyle=color;ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();ctx.fillText(label,x+6,y-6)}
function draw(){ctx.clearRect(0,0,canvas.width,canvas.height);if(!frame.naturalWidth)return;const sx=canvas.width/frame.naturalWidth,sy=canvas.height/frame.naturalHeight;ctx.lineWidth=2;ctx.font='12px Arial';setupPoints.forEach((point,index)=>drawPoint(point.x*sx,point.y*sy,'#ffd24a',setupKeys[index]));if(setupPoints.length===2){const [a,b]=setupPoints;ctx.strokeStyle='#55d5a2';ctx.strokeRect(a.x*sx,a.y*sy,(b.x-a.x)*sx,(b.y-a.y)*sy)}const zeroPoint=scalePoints.find(point=>point.position_mm===0);if(zeroPoint){ctx.strokeStyle='#ff9f43';ctx.beginPath();ctx.moveTo(zeroPoint.x*sx,0);ctx.lineTo(zeroPoint.x*sx,canvas.height);ctx.stroke()}scalePoints.forEach(point=>drawPoint(point.x*sx,point.y*sy,'#5ec9ef',labelFor(point.position_mm)));if(scalePoints.length&&scalePoints.length<scaleMarks.length){const point=scalePoints[scalePoints.length-1];ctx.fillStyle='#fff';ctx.fillText('下一个：'+labelFor(scaleMarks[scalePoints.length]),point.x*sx+8,point.y*sy+16)}}
canvas.addEventListener('mousemove',event=>{if(setupPoints.length!==2||scalePoints.length===scaleMarks.length)return;cursorGuide.style.left=Math.round(event.clientX)+'px';cursorGuide.hidden=false});canvas.addEventListener('mouseleave',()=>{cursorGuide.hidden=true});canvas.addEventListener('click',event=>{if(!frame.naturalWidth||scalePoints.length===scaleMarks.length)return;const rect=canvas.getBoundingClientRect(),point={x:(event.clientX-rect.left)*frame.naturalWidth/rect.width,y:(event.clientY-rect.top)*frame.naturalHeight/rect.height};if(setupPoints.length<2)setupPoints.push(point);else scalePoints.push({...point,position_mm:scaleMarks[scalePoints.length]});showStatus('');update()});document.getElementById('reset').addEventListener('click',()=>{setupPoints=[];scalePoints=[];showStatus('');update()});frame.addEventListener('load',sync);window.addEventListener('resize',sync);
document.getElementById('save').addEventListener('click',async()=>{const length=Number(document.getElementById('length').value);if(!Number.isFinite(length)){showStatus('请输入有效的管槽长度。','error');return}if(!frame.naturalWidth||!frame.naturalHeight||!sourceSize.width||!sourceSize.height){showStatus('尚未得到相机原始图像尺寸，请等待视频和帧率显示正常。','error');return}const sx=sourceSize.width/frame.naturalWidth,sy=sourceSize.height/frame.naturalHeight,[a,b]=setupPoints,zeroPoint=scalePoints.find(point=>point.position_mm===0);if(!zeroPoint){showStatus('缺少 0 cm 刻度。','error');return}const payload={roi_xyxy:[a.x*sx,a.y*sy,b.x*sx,b.y*sy],center_x_px:zeroPoint.x*sx,center_y_px:zeroPoint.y*sy,tube_length_mm:length,axis_direction:Number(document.querySelector('input[name="direction"]:checked').value),axis_points:scalePoints.map(point=>({position_mm:point.position_mm,pixel_x:point.x*sx}))};try{const response=await fetch('/api/roller_balance/calibration',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});const result=await response.json();if(!response.ok)throw new Error(result.error||'保存失败');showStatus('已保存 25 个刻度。请重启 visiond 后再启用平衡插件。','ok')}catch(error){showStatus(error.message,'error')}});
async function refreshFps(){try{const value=await fetch('/api/updates.json',{cache:'no-store'}).then(response=>response.json()),status=value.status;sourceSize={width:Number(status.frame_width||0),height:Number(status.frame_height||0)};document.getElementById('fps').textContent='相机 '+Number(status.camera_fps||0).toFixed(1)+' fps / 网页 '+Number(status.preview_fps||0).toFixed(1)+' fps';const fps=Math.max(1,Number(status.preview_fps)||30);setTimeout(refreshFps,1000/fps)}catch(_){document.getElementById('fps').textContent='帧率不可用';setTimeout(refreshFps,1000)}}refreshFps();sync();update();
</script></body></html>"""


def mjpeg_part(frame: bytes) -> bytes:
    """Encode one JPEG as a multipart MJPEG payload."""
    return (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(frame)).encode("ascii") + b"\r\n\r\n" + frame + b"\r\n")


class DebugServer:
    def __init__(self, host: str, port: int, status: JsonProvider,
                 results: JsonProvider, metrics: JsonProvider, frame: FrameProvider,
                 wait_frame: FrameWaiter, calibration_saver: Optional[CalibrationSaver] = None) -> None:
        providers = {"/api/status": status, "/api/results": results, "/api/metrics": metrics,
                     "/api/updates.json": lambda: {"status": status(), "results": results()}}

        def frame_stream() -> Iterator[bytes]:
            sequence = 0
            while True:
                sequence, image = wait_frame(sequence, 1.0)
                if image is not None:
                    yield mjpeg_part(image)

        class Handler(BaseHTTPRequestHandler):
            def send_json(self, status_code: int, value: Dict[str, Any]) -> None:
                body = json.dumps(value, ensure_ascii=False).encode("utf-8")
                self.send_response(status_code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

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
                if path == "/calibration":
                    if calibration_saver is None:
                        self.send_error(403, "web calibration is disabled")
                        return
                    body = CALIBRATION_PAGE.encode("utf-8")
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
                self.send_json(200, provider())

            def do_POST(self) -> None:
                if self.path.split("?", 1)[0] != "/api/roller_balance/calibration":
                    self.send_error(404)
                    return
                if calibration_saver is None:
                    self.send_error(403, "web calibration is disabled")
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 1 <= length <= 4_096:
                        raise ValueError("request body must be 1 to 4096 bytes")
                    value = json.loads(self.rfile.read(length).decode("utf-8"))
                    result = calibration_saver(value)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                    self.send_json(400, {"error": str(error)})
                    return
                except Exception:
                    self.send_json(500, {"error": "could not save calibration"})
                    return
                self.send_json(200, result)

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

# 已归档：最下方钢球 UART 联调

本页给出真实相机、YOLO26 DET 钢球模型和 MSPM0 UART 的联调入口。它不修改
`ultralytics_yolo26` 模型仓库，也不改写项目的配置文件。
该链路已归档，不属于当前滚珠平衡的两个主入口。

## 默认输出模式

先确认摄像头节点。例如实际节点为 `/dev/video0` 时，在项目根目录执行：

```bash
./scripts/archive/run_steelball_uart.sh --camera /dev/video0
```

脚本会检查相机与 `/dev/ttyS1` 均为字符设备，加载以下现有模型：

```text
/userdata/rdkstudio/projects/ultralytics_yolo26/model/steelball-yolo26n-det_bayese_640x640_nv12.bin
```

它只启用 `steelball` 视觉插件，使用 YOLO26 DET 对每帧检测。若画面有多个钢球，选择
边界框底边 `y2` 最大的一个；当 `y2` 相同时选择置信度更高的一个。该选择方式使画面中
最靠下的钢球成为唯一目标。

默认模型类型为 `det`。如需使用其他钢球检测模型，可通过参数覆盖：

```bash
./scripts/archive/run_steelball_uart.sh --camera /dev/video0 \
  --model-path /absolute/path/to/steelball_det_640x640_nv12.bin
```

`--model-path` 必须是与指定模型类型匹配的钢球模型。两种模型都会复用同一套相机、最低
钢球选择、坐标解算和 UART 发送链路。

识别框的底边中点 `(u, y2)` 经过临时单应矩阵得到相对于电磁铁捕获点的地面坐标：

```text
forward_mm = 1200 - 1.5 * y2
lateral_mm = 1.2 * u - 768
bearing_mdeg = atan2(lateral_mm, forward_mm) * 180000 / pi
range_mm = hypot(forward_mm, lateral_mm)
```

随后 `vehicled` 将这些值编码为 v2 `CMD_MOTION/CAPTURE_TARGET_POLAR`，通过
`/dev/ttyS1` 以 `115200` 波特率发送。默认配置强制消息的 `enabled=0`，但
`target_valid=1`、方位角与距离仍会发送，因此逻辑分析仪和 MSPM0 可以验证完整数据链，
却不得由该帧驱动车轮或电磁铁。输出模式不会等待 MSPM0 遥测，因此接收端仅连接逻辑
分析仪时也能观察到目标帧；传入 `--allow-motion` 后仍必须接收到正常遥测，否则会安全停车。

`Ctrl-C` 会停止两个守护进程；车辆通信进程退出时会再发送一个禁用目标。

## 实时识别页面

网页是否启动由 `config/base.yaml` 控制：

```yaml
web:
  enabled: true
  host: 127.0.0.1
  port: 8090
```

开启后，钢球启动脚本会在继续向 UART 发送极坐标的同时，启动 `visiond` 的只读页面。
在本机浏览器打开 `http://127.0.0.1:8090/`，可查看实时画面、识别框、方位角与距离。
将 `enabled` 改为 `false` 后，页面、HTTP 端口和 JPEG 编码均关闭，视觉识别与 UART
发送不受影响。

页面的视频区域使用 `GET /video_feed` 持续输出 MJPEG，不会每隔一段时间重新请求单张图片。
`web.preview_fps`、`web.preview_width` 和 `web.jpeg_quality` 分别限制网页预览的帧率、宽度和
编码质量；默认值为 25 FPS、640 像素和 75。它们只影响浏览器预览，不会改变模型输入、钢球
极坐标解算或 UART 输出。

## 心跳开关

`config/transport.yaml` 中的 `transport.heartbeat.enabled` 默认是 `true`。设为 `false`
后，`vehicled` 不再发送协议 `HEARTBEAT` 帧，但仍按原有频率发送 `CMD_MOTION`。只有在
MSPM0 已明确不依赖该心跳维持运行，或需要单独验证运动帧时才应关闭；若固件将心跳超时
视为停车条件，关闭后应预期车辆进入安全停车。

## 临时标定限制

临时参数保存在 `config/camera.yaml` 的
`camera.calibration.image_to_capture_homography`，针对 1280x720 图像，仅为本次联调
提供一个可观察的数值映射。它不是相机标定结果，不能保证真实距离、左右方向或电磁铁
捕获位置准确。

后续标定网页应采集多个已经测量的钢球地面位置 `(forward_mm, lateral_mm)` 与对应像素
底边中点，拟合并验证 `image_to_capture_homography`。网页应直接更新 `camera.yaml` 中
这 9 个数值，完成验证后才能作为真实运动依据。

## 允许实际运动

在车轮悬空、硬件急停有效、MSPM0 已验证超时停车且真实标定已替换之前，禁止让车辆运行。
脚本要求两个明确开关才会将 `enabled` 置为 `1`：

```bash
./scripts/archive/run_steelball_uart.sh \
  --camera /dev/video0 \
  --allow-motion \
  --allow-temporary-calibration
```

这两个开关只表示操作者确认使用临时标定进行联调，并不使临时参数变成可靠标定。MSPM0
仍必须自行执行方位/距离闭环、急停和目标超时停车；RDK 不发送左右轮 PWM、速度或转向量。

## 观测与故障检查

当网页开启时，脚本会在 `127.0.0.1:8090` 打开只读状态接口。可观察：

- `/api/status`：相机、模型插件与 IPC 的健康状态。
- `/api/results`：最近的 `BALL_TARGET`，其中包含 `bearing_mdeg` 与 `range_mm`。
- `/api/metrics`：帧和事件计数。

没有检测到钢球、投影结果位于电磁铁后方、目标过期或视觉流超时，均不会形成有效的捕获
目标。MSPM0 必须把缺失或过期的 `CMD_MOTION` 当作停车条件。

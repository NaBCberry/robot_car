# RDK X5 小车软件骨架

本项目在不修改 `ultralytics_yolo26`、`26TI-PaddleOCR` 和共享 `utils`
的前提下，把摄像头/视觉与车控通信拆为两个进程：

```text
camera -> visiond -> VisionEvent/Unix Socket -> vehicled -> protocol v1 -> MSPM0
                \-> read-only HTTP debug API
```

`visiond` 是唯一摄像头所有者；视觉插件只能处理它提供的同一帧。
`vehicled` 是唯一 MSPM0 通信出口，只接受标准化事件并发送模式、速度、
转向、限速和有效期，不发送左右电机 PWM。运行数据全部位于
`/userdata/robot-car`，不会写进 Git 项目。

## 安全默认值

- `camera.enabled: false` 且设备名为空，不会默认打开摄像头。
- `transport.enabled: false`，UART 设备与 CAN 通道/ID 均为空。
- `vehicle.control_enabled: false`，速度和限速均为零。
- 启动和正常停止都会通过已选传输发送一次禁用/零速目标；FakeTransport
  只在内存中记录。
- 配置错误、模型缺失、摄像头失败或真实链路超时不会产生有效运动目标。
- `visiond` 在相机帧持续到达时发布健康事件；事件流超过
  `vehicle.vision_timeout_ms` 未更新后，状态机会进入 `FAILSAFE`。
- systemd 文件仅为模板，本项目不安装服务、不启用自启动。

`transport.enabled` 控制是否允许打开真实链路；`vehicle.control_enabled`
独立控制是否允许目标的 `enable` 置位。真实车联调时两项都需要经过现场
安全检查后显式开启。

## 开发与模拟

在项目根目录运行（根目录的轻量导入 shim 使 `src` 布局可直接导入）：

```bash
cd /userdata/rdkstudio/projects/robot_car
python3 -m robot_car.vehicled --config-dir config --transport fake
python3 -m robot_car.visiond --config-dir config --simulate
```

推荐先启动 `vehicled`，再启动 `visiond`。模拟模式不打开摄像头和模型，
会生成需要三帧确认的 `SLOW_DOWN` 事件。停止两个进程可按 `Ctrl-C`。
默认调试 API 仅绑定环回地址：

- `GET http://127.0.0.1:8090/api/status`
- `GET http://127.0.0.1:8090/api/results`
- `GET http://127.0.0.1:8090/api/metrics`

网页接口仅用于观测，车控不解析 MJPEG，也不轮询 HTTP。运行脚本为
`scripts/run_vehicled.sh` 和 `scripts/run_visiond.sh`。

运行测试：

```bash
python3 -m unittest discover -s tests -v
```

## 配置真实摄像头与模型

1. 在 `config/camera.yaml` 明确填写板上确认过的 `device`，再设置
   `camera.enabled: true`。不要从示例猜测设备名。
2. 在 `config/vision.yaml` 分别填写 YOLO 的 `model_path`，以及 OCR 的
   `detection_model_path`、`recognition_model_path`。
3. 逐个设置插件 `enabled: true`，确认 BPU 核心、频率、优先级和最大处理
   时间适合当前板卡。
4. 先在 `vehicle.control_enabled: false` 下检查 `/api/status`、事件和负载。

YOLO adapter 复用现有 `YOLO26Detect.predict(frame)`；OCR adapter 复用现有
`PaddleOCR.predict(frame)`。两者都延迟导入板端 BPU 运行库，初始化失败会被
隔离并清晰记录，不会自行打开摄像头。

## 视觉插件扩展

所有插件实现 `VisionPlugin`：`initialize`、`process(frame)`、`health` 和
`close`，输出包含时间戳、帧号、来源、类型、置信度、TTL、结构化 payload
及图像尺寸的 `VisionEvent`。调度器对每个插件独立限频，线程池只保留相机
的最新帧；超时插件会跳帧，连续异常采用指数退避。

新增能力通常只需：

1. 在 `src/robot_car/perception` 新增 adapter。
2. 在 `plugin_registry.py` 注册新的 `type`。
3. 在 `config/vision.yaml` 添加插件项。
4. 在 `decision/rules.py` 添加标准事件的高层规则。
5. 添加插件单元测试。

`qr_code`、`color_marker`、`lane_vision`、`segmentation` 和
`pose_estimation` 已注册为默认关闭的占位插件；它们不加载不存在的模型。

## UART/CAN 联调

协议见 `src/robot_car/protocol/protocol_v1.md`。先保持车轮悬空、MSPM0
硬件急停有效，并按以下顺序联调：

1. MSPM0 实现 SLIP 分帧、CRC-16/CCITT、长度/版本检查和序列统计。
2. 实现 `CMD_MOTION`、`CMD_EVENT`、`HEARTBEAT`、`TELEMETRY`、`ACK`、
   `FAULT`，并为命令返回匹配 sequence 的 ACK。
3. MSPM0 对心跳和运动目标分别执行有效期超时，任一超时均自主安全停车；
   硬件 E-stop 与本地故障优先级最高。
4. UART 联调时安装板端已有策略允许的 `pyserial`，填写
   `transport.uart.device`，设置 `transport.type: uart` 和
   `transport.enabled: true`。波特率需与固件一致。
5. CAN 联调时安装 `python-can`，先确认收发器、电气终端、接口、bitrate、
   command/telemetry ID 和文档中的 7 字节分段，再填写 CAN 配置。
6. 先保持 `vehicle.control_enabled: false` 验证心跳、禁用帧、遥测、ACK、
   CRC 错误和断线停车；最后才在受控环境显式开启车控。

本仓库不猜测 `/dev/tty*`、CAN 接口或 CAN ID。`diagnose_hardware.sh` 只执行
只读枚举，不修改网络或硬件状态。可选依赖 `pyserial`、`python-can` 和板端
OpenCV 未被自动安装；基础 Python 依赖记录于 `requirements.txt`。

## 部署模板

`deploy/systemd` 中 vehicle 服务先于 vision 服务启动，并显式设置所需
`PYTHONPATH`、工作目录和运行目录。模板未写死 `User=root`；部署者应选择
非特权用户，再授予所需 `video`、`dialout` 或已配置 CAN 访问权限。模板
默认仍使用 FakeTransport，修改、安装和启用服务必须另行人工确认。

# RDK X5 小车软件骨架

本项目在不修改 `ultralytics_yolo26`、`26TI-PaddleOCR` 和共享 `utils`
的前提下，把摄像头/视觉与车控通信拆为两个进程：

```text
camera -> visiond -> VisionEvent/Unix Socket -> vehicled -> protocol v2 -> MSPM0
                \-> read-only HTTP debug API
```

`visiond` 是唯一摄像头所有者；视觉插件只能处理它提供的同一帧。
`vehicled` 是唯一 MSPM0 通信出口，只接受标准化事件并发送模式、有效期和可选的
极坐标目标，不发送左右电机 PWM、速度、转向或限速。运行数据全部位于
`/userdata/robot-car`，不会写进 Git 项目。

## 仓库目录与文件职责

```text
robot_car/
├── .gitignore                         # 忽略 Python 缓存和本地测试缓存
├── README.md                          # 项目架构、安全约束、运行和联调说明
├── requirements.txt                  # 基础 Python 依赖；硬件依赖仅作注释说明
├── robot_car/
│   └── __init__.py                    # 仓库根目录运行时的 src-layout 导入 shim
├── docs/
│   ├── vision_to_motion_flow.md       # OCR/通用视觉事件到 MSPM0 运动目标的流程图和时序图
│   ├── steelball_capture_control.md   # 钢球单一语义运动链路、协议分层、标定和安全语义
│   ├── steelball_uart_bringup.md      # 最下方钢球识别、临时标定和真实 UART 联调说明
│   └── roller_balance_direct_can.md   # ICM42688/Y42 直接 CAN 滚珠闭环、标定和安全流程
├── config/                            # 所有可部署参数，硬件路径不写死在代码中
│   ├── base.yaml                      # 运行数据根目录、日志级别、UDS 和调试 Web 配置
│   ├── camera.yaml                    # 摄像头启停、设备、图像参数及钢球捕获坐标标定
│   ├── vision.yaml                    # 视觉插件列表、频率、优先级、BPU 和确认策略
│   ├── vehicle.yaml                   # 状态机、车控总开关、心跳和超时参数
│   ├── transport.yaml                 # Fake/UART/CAN 类型、端口、波特率、CAN ID 和心跳开关
│   └── roller_control.yaml             # 独立 Y42 CAN 摆杆闭环的 IMU、PID、坡度和曲轴标定
├── deploy/
│   └── systemd/                       # 仅供人工部署的 systemd 模板，不自动安装
│       ├── robot-vehicle.service      # vehicled 服务模板，要求先启动
│       └── robot-vision.service       # visiond 服务模板，依赖 vehicle 服务
├── scripts/                           # 日常启动和只读诊断入口
│   ├── archive/
│   │   └── run_steelball_uart.sh       # 已归档：最下方钢球到 UART 的旧联调入口
│   ├── diagnose_hardware.sh           # 只读列举 Python、视频、串口和 CAN 候选资源
│   ├── send_protocol_frame.sh          # 显式确认后向指定 UART 发送一帧协议测试数据
│   ├── run_roller_balance_uart.sh      # 顶置滚珠位置偏差的安全 UART 输出入口
│   ├── run_roller_balance_can.sh       # 独立的 ICM42688/Y42 直接 CAN 控制入口，默认不使能
│   ├── calibrate_roller_control.sh     # 交互采集 Y42 软限位和曲轴-水管标定表
│   ├── probe_icm42688.sh                # 按配置只读探测 ICM42688 的 WHO_AM_I
│   ├── monitor_icm42688.sh              # 只读输出 IMU 原始值和俯仰角，不打开 CAN
│   ├── capture_protocol_frames.py       # 协议帧抓包和解析测试工具
│   └── capture_protocol_frames_windows.bat # Windows 抓包测试入口
├── src/
│   └── robot_car/                     # 应用主 Python 包
│       ├── __init__.py                # 包版本和包级说明
│       ├── config.py                  # YAML 合并、关键参数校验和运行目录创建
│       ├── visiond.py                 # 单摄像头所有者、插件调度、事件发布和调试 API
│       ├── vehicled.py                # 视觉订阅、状态机、心跳和 MSPM0 唯一通信出口
│       ├── rollercontrold.py           # 独立订阅滚珠事件并驱动 Y42 的闭环守护进程
│       ├── camera/                    # 摄像头采集层
│       │   ├── __init__.py            # camera 子包声明
│       │   ├── capture.py             # 唯一打开摄像头的实现，使用容量为 1 的最新帧队列
│       │   └── frame.py               # CameraFrame 帧号、时间戳、图像及尺寸结构
│       ├── perception/                # 与具体车控和通信解耦的视觉插件层
│       │   ├── __init__.py            # 导出 VisionEvent 和 VisionPlugin 公共接口
│       │   ├── plugin.py              # initialize/process/health/close 抽象插件协议
│       │   ├── plugin_registry.py     # 根据 vision.yaml 中的 type 创建插件
│       │   ├── events.py              # 跨插件统一 VisionEvent 数据结构和 TTL 判断
│       │   ├── scheduler.py           # 独立限频、并发执行、超时跳帧和故障退避
│       │   ├── stabilizer.py          # 连续多帧确认及中断后重新计数
│       │   ├── yolo_adapter.py        # 复用现有 YOLO26Detect，不打开摄像头
│       │   ├── ocr_adapter.py         # 复用现有 PaddleOCR，不打开摄像头
│       │   ├── steelball_adapter.py   # 复用 YOLO26 Seg/DET 检测钢球，并生成捕获目标事件
│       │   ├── steelball_geometry.py  # 图像到电磁铁捕获点相对坐标的单应性解算
│       │   └── placeholders.py        # QR、颜色、车道线、分割和姿态插件占位实现
│       ├── decision/                  # 只依赖标准视觉事件的高层决策层
│       │   ├── __init__.py            # decision 子包声明
│       │   ├── motion_target.py       # 模式、有效期和可选极坐标目标结构
│       │   ├── capture_target.py      # 钢球的相对角度/距离目标及目标失效安全值
│       │   ├── control_arbiter.py     # 在普通运动与 MCU 捕获伺服链路间互斥仲裁
│       │   ├── rules.py               # STOP/SLOW_DOWN/OCR 等事件到行为的规则映射
│       │   └── state_machine.py       # BOOT 到 FAULT 的状态迁移和失联安全逻辑
│       ├── ipc/                       # visiond 与 vehicled 的本机进程通信层
│       │   ├── __init__.py            # ipc 子包声明
│       │   ├── schemas.py             # IPC schema 版本、事件封装和输入校验
│       │   └── vision_socket.py       # UDS JSON-lines 发布、订阅、扇出和自动重连
│       ├── protocol/                  # RDK 与 MSPM0 共用的协议语义
│       │   ├── __init__.py            # 导出协议编解码公共 API
│       │   ├── messages.py            # 消息枚举及 motion、ACK、JSON payload 编解码
│       │   ├── framing.py             # A5 5A/长度分帧、CRC-16 和增量解码器
│       │   └── protocol_v2.md         # 固件与 Python 端共同遵循的字节级协议文档
│       ├── vehicle_link/              # 真实硬件与 FakeTransport 的统一通信层
│       │   ├── __init__.py            # vehicle_link 子包声明
│       │   ├── transport_base.py      # open/send/receive/close 抽象传输接口
│       │   ├── fake_transport.py      # 内存记录和遥测注入，用于无硬件测试
│       │   ├── uart_transport.py      # 显式配置后才加载 pyserial 和打开串口
│       │   ├── can_transport.py       # 显式配置后才加载 python-can，处理 CAN 分段
│       │   ├── gateway.py             # 序列号、协议帧、ACK、心跳和安全目标发送
│       │   ├── telemetry.py           # 最近一次 MSPM0 遥测的线程安全缓存
│       │   └── watchdog.py            # 基于单调时钟的链路接收超时判断
│       ├── web/                       # 与车控无依赖的只读调试服务
│       │   ├── __init__.py            # web 子包声明
│       │   ├── server.py              # 实时识别页面、MJPEG 视频流及只读 JSON API
│       │   └── overlay.py             # 在网页预览叠加识别框和极坐标，不参与决策
│       ├── observability/             # 日志、指标和可选事件记录
│           ├── __init__.py            # observability 子包声明
│           ├── logging.py             # 控制台和运行数据目录文件日志初始化
│           ├── metrics.py             # 线程安全的轻量计数器/指标快照
│           └── recorder.py            # 可选 JSON-lines 视觉事件记录器
│       └── roller_control/            # 与 UART/MSPM0 解耦的直接摆杆闭环模块
│           ├── icm42688.py            # SPI/I2C 身份校验、配置和原始 IMU 数据读取
│           ├── attitude.py            # 单轴互补滤波，输出摆杆实际俯仰角
│           ├── control.py             # 钢球状态估计、坡度补偿和级联 PID
│           ├── y42_actuator.py        # 复用 canstep 的 Y42 CAN 协议执行器
│           └── config.py              # 独立的滚珠闭环配置解析
├── tests/                             # 无摄像头和无 MSPM0 可运行的自动化测试
│   ├── test_protocol.py               # CRC、协议字段、ACK 及捕获目标安全门控
│   ├── test_state_machine.py          # 状态迁移、安全门控、超时和捕获伺服状态
│   ├── test_control_arbiter.py        # 两条控制链路互斥、目标过期及反馈语义
│   ├── test_steelball_geometry.py     # 钢球图像坐标到捕获点坐标的解算
│   ├── test_vehicled_capture.py       # 守护进程仅发送捕获链路，不发送有效普通运动
│   ├── test_vision_ipc.py             # UDS 连接、事件传递、断线和重连
│   └── test_web_server.py              # 实时识别页面、MJPEG 流与只读 API 路由
└── tools/
    └── replay_recording.py            # 将 JSON-lines 录像事件重放到独立 UDS
```

仓库外的 `/userdata/robot-car` 是运行数据目录：`logs/` 保存进程日志，
`recordings/` 保存可选录像或事件记录，`runtime/` 保存 Unix Socket、PID 等
瞬态文件，`calibration/` 保存相机和车辆标定数据。这些内容不进入 Git。

源码依赖方向保持单向：`visiond` 通过 `camera` 和 `perception` 生成事件，
经 `ipc` 交给 `vehicled`；`vehicled` 通过 `decision` 生成高层目标，再交给
`vehicle_link` 和 `protocol`。`decision` 不导入任何具体模型 adapter，视觉
插件也不能访问车辆传输层。

视觉到运动的逐步数据变化、文件路径、时序和安全分支见
[`docs/vision_to_motion_flow.md`](docs/vision_to_motion_flow.md)。
钢球捕获的专用控制过程、两条链路的比较及电磁铁无反馈处理见
[`docs/steelball_capture_control.md`](docs/steelball_capture_control.md)。
真实相机到 UART 的最下方钢球联调入口与临时标定限制见
[`docs/steelball_uart_bringup.md`](docs/steelball_uart_bringup.md)。

## 安全默认值

- `camera.enabled: false` 且设备名为空，不会默认打开摄像头。
- `transport.enabled: false`，UART 设备与 CAN 通道/ID 均为空。
- `transport.heartbeat.enabled: true` 默认周期发送协议心跳；关闭它只抑制 `HEARTBEAT`
  帧，不会停止 `CMD_MOTION`，因此必须与 MSPM0 的超时停车策略一致。
- `web.enabled` 位于 `config/base.yaml`，独立控制实时识别页面和只读 HTTP 服务；关闭后
  UART 与视觉决策仍正常运行，且不会生成网页 MJPEG 预览。
- `vehicle.control_enabled: false`，网关强制发送禁用运动模式。
- `vehicle.capture.enabled: false`；即使启用钢球插件，也不会下发有效捕获目标。
- `vehicle.capture.output_only: false` 是额外的极坐标输出保护开关。生产联调脚本会把
  它设为 `true`，使 UART 保留有效角度/距离字段但强制 `enabled=0`。
- `vehicle.capture.feedback.enabled: false`；未接入霍尔/电流/开关/视觉反馈时，只能
  上报 `CAPTURE_ATTEMPTED`，绝不把电磁铁动作描述为捕获成功。
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

当 `config/base.yaml` 中 `web.enabled: true` 时，访问 `http://127.0.0.1:8090/` 可查看
实时相机画面、钢球识别框、方位角和距离。

页面通过 `GET /video_feed` 持续接收 MJPEG；状态和识别结果每 500 ms 刷新一次。网页专用
编码默认限制为 25 FPS、最大宽度 640 像素、JPEG 质量 75，可在 `base.yaml` 的 `web` 下通过
`preview_fps`、`preview_width`、`jpeg_quality` 调整。网页接口仅用于观测，车控不解析 MJPEG，
也不轮询 HTTP。两个主入口会直接启动所需的 Python 守护进程，无需额外服务包装脚本。

## UART 协议单帧测试

`scripts/send_protocol_frame.sh` 默认从 `config/transport.yaml` 读取 UART 设备和波特率，
只打印 v2 帧，不会打开串口。加入 `--send` 才会实际写入 UART；控制类消息还必须显式
加入 `--unsafe-allow-control`。

例如，以下命令发送有效且允许捕获的 `CAPTURE_TARGET_POLAR`：目标 ID 为 `1`，方位角
`+32` 度、距离 `32 cm`、有效期 `200 ms`。

```bash
scripts/send_protocol_frame.sh --message-type 0x01 \
  --payload-hex '04 07 00 C8 00 01 00 00 7D 00 00 00 01 40 03 E8 00 00' \
  --unsafe-allow-control --send
```

省略 `--send` 可先核对逻辑分析仪应看到的字节。字段定义见
`src/robot_car/protocol/protocol_v2.md`。

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

钢球插件根据 `model_type` 复用相邻 `ultralytics_yolo26/runtime/python` 中的
`YOLO26Seg` 或 `YOLO26Detect` 和对应模型文件，只使用 `visiond` 提供的
`CameraFrame`；它不导入或运行 `steelball_web.py`，因此不会额外打开摄像头。要生成供 MCU
伺服使用的 `BALL_TARGET`，还必须填写并验证图像到电磁铁捕获点的标定矩阵。

## 已归档：最下方钢球 UART 联调

已确认相机设备后，下面的命令会打开真实相机和 `/dev/ttyS1`，复用
`ultralytics_yolo26` 的钢球 DET 模型，选取画面中边界框底边最低的钢球，并持续发送
`CAPTURE_TARGET_POLAR`。默认是输出模式：帧中仍有角度和距离，`enabled=0`，MSPM0
不得驱动电机。

```bash
cd /userdata/rdkstudio/projects/robot_car
./scripts/archive/run_steelball_uart.sh --camera /dev/video0
```

该入口已归档，不属于当前两个滚珠平衡主流程。`--camera` 必须替换为板上实际的视频设备。脚本会单独生成临时运行配置，不改写项目的
`config/transport.yaml`。临时标定和实际运动的双确认命令、输出字段及替换标定的方案见
[`docs/steelball_uart_bringup.md`](docs/steelball_uart_bringup.md)。

## 车载平衡滚珠

H 题使用固定在 25 cm 管槽正上方的相机，输出钢球相对中心的距离偏差，并以
`CMD_MOTION/BALANCE_ROLLER` 交给 MSPM0 本地摆杆 PID。它与电磁铁捕获链路相互独立，
标定、协议字段、启用顺序和网页观测方式见
[`docs/roller_balance.md`](docs/roller_balance.md)。
网页标定入口为 `http://<RDK-IP>:8090/calibration`，保存后重启 `visiond` 生效。
安全串口输出可直接执行 `./scripts/run_roller_balance_uart.sh`；它发送距离偏差但固定
`enabled=0`。

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

协议见 `src/robot_car/protocol/protocol_v2.md`。先保持车轮悬空、MSPM0
硬件急停有效，并按以下顺序联调：

1. MSPM0 实现 A5 5A 帧头扫描、长度读取、CRC-16/CCITT、版本检查和序列统计。
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

## 赛题动作调度与 TUI

统一动作入口为：

```bash
cd /userdata/rdkstudio/projects/robot_car
./scripts/run_vehicle_tui.sh --transport uart
```

界面输入动作编号并回车即可执行，`S` 或 `ESC` 停止，`Q` 安全退出。动作编号为：

| 编号 | 动作 |
|---:|---|
| 0 | 停止 |
| 1 | 顺时针巡线一圈回 A |
| 2 | 钢球 `0 -> +50mm -> -50mm` |
| 3 | 巡线到 B 并保持钢球中心 |
| 4 | 巡线一圈并保持钢球中心 |
| 5 | 巡线一圈并保持启动时钢球位置 |

下位机通过 `CMD_EVENT/ACTION_REQUEST` 请求动作时，RDK 会按协议帧序号去重，
先回复 ACK，再把请求送入与 TUI 相同的动作调度器。动作状态、当前阶段、最近下位机
动作号和钢球误差只刷新 TUI 的状态行，不会绕过安全看门狗。

滚珠直接 CAN 控制链的加速度前馈配置位于 `config/roller_control.yaml` 的
`control.feedforward`，默认关闭。当前铰链端 ICM42688 只能作为摆杆姿态/角速度来源；
要启用车体惯性前馈，应先接入经过校准的 M0 车体加速度遥测或把传感器固定到车体，
再设置 `enabled: true`、`gain` 和 `limit_mm_s2`。

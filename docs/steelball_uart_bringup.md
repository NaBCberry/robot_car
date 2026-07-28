# 最下方钢球 UART 生产联调

本页给出真实相机、YOLO26Seg 钢球模型和 MSPM0 UART 的联调入口。它不修改
`ultralytics_yolo26` 模型仓库，也不改写项目的 `config/transport.yaml`。

## 默认输出模式

先确认摄像头节点。例如实际节点为 `/dev/video0` 时，在项目根目录执行：

```bash
./scripts/run_steelball_uart.sh --camera /dev/video0
```

脚本会检查相机与 `/dev/ttyS1` 均为字符设备，加载以下现有模型：

```text
/userdata/rdkstudio/projects/ultralytics_yolo26/model/steelball_seg_bpu_bayese_640x640_nv12.bin
```

它只启用 `steelball` 视觉插件，使用 YOLO26Seg 对每帧检测。若画面有多个钢球，选择
边界框底边 `y2` 最大的一个；当 `y2` 相同时选择置信度更高的一个。该选择方式使画面中
最靠下的钢球成为唯一目标。

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
却不得由该帧驱动车轮或电磁铁。

`Ctrl-C` 会停止两个守护进程；车辆通信进程退出时会再发送一个禁用目标。

## 临时标定限制

临时参数保存在 `config/temporary_steelball_calibration.yaml`，针对 1280x720 图像，仅为
本次联调提供一个可观察的数值映射。它不是相机标定结果，不能保证真实距离、左右方向或
电磁铁捕获位置准确。

后续标定网页应采集多个已经测量的钢球地面位置 `(forward_mm, lateral_mm)` 与对应像素
底边中点，拟合并验证 `image_to_capture_homography`。网页产出一个包含这 9 个数值的 YAML
文件，替代临时文件路径后才能作为真实运动依据。

## 允许实际运动

在车轮悬空、硬件急停有效、MSPM0 已验证超时停车且真实标定已替换之前，禁止让车辆运行。
脚本要求两个明确开关才会将 `enabled` 置为 `1`：

```bash
./scripts/run_steelball_uart.sh \
  --camera /dev/video0 \
  --allow-motion \
  --allow-temporary-calibration
```

这两个开关只表示操作者确认使用临时标定进行联调，并不使临时参数变成可靠标定。MSPM0
仍必须自行执行方位/距离闭环、急停和目标超时停车；RDK 不发送左右轮 PWM、速度或转向量。

## 观测与故障检查

脚本会在 `127.0.0.1:8090` 打开只读状态接口。可观察：

- `/api/status`：相机、模型插件与 IPC 的健康状态。
- `/api/results`：最近的 `BALL_TARGET`，其中包含 `bearing_mdeg` 与 `range_mm`。
- `/api/metrics`：帧和事件计数。

没有检测到钢球、投影结果位于电磁铁后方、目标过期或视觉流超时，均不会形成有效的捕获
目标。MSPM0 必须把缺失或过期的 `CMD_MOTION` 当作停车条件。

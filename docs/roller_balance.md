# 车载平衡滚珠控制

本功能用于 H 题的 25 cm 凹槽摆杆：摄像头固定在管槽正上方，RDK 输出钢球相对中心 O 的
相对中心 O 的有符号距离偏差；MSPM0 保持摆杆电机与红外循迹的本地高频闭环。网页只用于观察。

## 数据链路

```mermaid
flowchart LR
    A[顶置相机] --> B[roller_balance_adapter.py]
    B --> C[BALL_BALANCE_STATE<br/>error_mm]
    C --> D[vehicled.py]
    D --> E[CMD_MOTION<br/>BALANCE_ROLLER]
    E --> F[MSPM0 摆杆 PID 与循迹]
```

## 网页标定

默认网页服务已启用手工标定。打开 `http://<RDK-IP>:8090/calibration`，或在监控页点击
“手工标定”，按顺序完成：

1. 点击管槽内壁的左上角。
2. 点击管槽内壁的右下角。
3. 点击物理中心 O 点，并填写管槽的有效长度（H 题为 `250 mm`）与正方向。
4. 点击保存。页面会原子更新 `config/camera.yaml` 的 `roller_balance` 配置；重启 `visiond`
   后才会使用新值。

管槽必须在画面中近似水平，左、右边界应与管槽轴线对应。当前一维计算使用图像 x 轴；若
相机画面中的管槽倾斜，应先调整相机安装方向，不能用旋转 ROI 代替。标定完成后建议将
`web.calibration_enabled` 设为 `false`，关闭配置写入接口。

## 配置字段

在 `config/camera.yaml` 的 `camera.calibration.roller_balance` 中填写：

| 字段 | 含义 |
|---|---|
| `roi_xyxy` | 覆盖管槽内壁的图像矩形 `[x1,y1,x2,y2]` |
| `center_x_px` | 中心 O 对应的钢球中心像素横坐标 |
| `mm_per_pixel` | 管槽轴方向的毫米/像素比例 |
| `axis_direction` | 图像向右映射为正方向时为 `1`，反向时为 `-1` |

应在管槽中心和至少两个已知刻度位置复核换算误差。不要使用现有电磁铁捕获单应矩阵代替
此标定；两者的坐标系和物理目标不同。

## 启用

1. 保持 `vehicle.control_enabled: false`，只开启 `roller_balance` 插件，确认网页中的
   ROI、中心线和距离偏差方向正确。
2. 保持 `vehicle.control_enabled: false`，设置 `vehicle.balance.enabled: true`、真实 UART
   配置和 `vehicle.balance.output_only: true`，验证 M0 或逻辑分析仪收到 6 字节平衡状态帧，
   且 `enabled=0,balance_valid=1`。仅输出模式可越过总控开关发送测量值，但绝不授权 MSPM0
   执行运动。
3. 车轮悬空、急停有效时关闭 `output_only` 并开启 `control_enabled`。M0 必须在视觉状态或
   心跳过期时停止积分并进入安全摆杆策略。

中心 O 固定为平衡目标，`error_mm=0` 表示钢球位于中心。

## UART 输出启动

默认现场配置使用 `/dev/ttyS1`、`115200` baud、`vehicle.balance.output_only: true`。
相机标定和 `roller_balance` 插件已准备好后，执行：

```bash
cd /userdata/rdkstudio/projects/robot_car
./scripts/run_roller_balance_uart.sh
```

该脚本同时启动 `visiond` 与 `vehicled`。识别到钢球后，`vehicled` 按
`vehicle.heartbeat_hz`（默认 20 Hz）发送 `CMD_MOTION/BALANCE_ROLLER`，其中只有
`error_mm` 是平衡控制量，且 `enabled=0`。MSPM0 必须先适配当前 v2 的 6 字节平衡 payload；
在完成悬空联调前，不要关闭 `output_only`。

### 先验证 UART 电气链路

在不启动相机和模型时，可直接发送一次 `error_mm=-32` 的安全平衡帧。逻辑分析仪应看到
一帧以 `A5 5A` 开始、payload 为 `05 08 00 78 FF E0` 的协议数据；其中
`enabled=0,balance_valid=1`，MSPM0 不得驱动电机。

```bash
cd /userdata/rdkstudio/projects/robot_car
./scripts/send_protocol_frame.sh --message-type 0x01 \
  --payload-hex '05 08 00 78 FF E0' \
  --unsafe-allow-control --send --repeat 20 --interval-ms 50
```

若此命令没有波形，请先在 RDK 板端检查 `/dev/ttyS1` 是否存在、UART3 引脚复用和 TX/RX/GND
连线；这与视觉识别链路无关。若此命令有波形而一键脚本没有，请查看两个服务是否仍在运行：
`pgrep -af 'robot_car.(visiond|vehicled)'`。

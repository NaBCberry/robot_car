# RDK-MSPM0 协议 v2

本文定义 RDK X5 与 MSPM0 之间的字节级通信协议。所有多字节整数均使用大端序。
v2 与 v1 **不兼容**：两端必须同时升级；收到版本号为 `1` 的帧必须拒绝，不能按
v2 重新解释。v2 不再存在 `CMD_CAPTURE_TARGET`，所有车辆运动请求统一使用
`CMD_MOTION (0x01)`。

UART 帧以固定两字节帧头 `0xA5 0x5A` 开始，**没有帧尾，也不使用字节转义**。接收端
读取帧头后的固定头，从 `payload_length` 得到剩余读取长度，再校验 CRC。payload 中出现
`0xA5 0x5A` 时按普通数据处理；CRC、长度错误或接收超时时重新扫描下一个帧头。

## 通用帧格式

| 字段 | 大小 | 说明 |
|---|---:|---|
| `frame_magic` | 2 字节 | 固定为 `0xA5 0x5A`，不参与 CRC |
| `protocol_version` | u8 | 固定为 `2` |
| `message_type` | u8 | 消息类型，见下表 |
| `sequence` | u16 | 发送方序列号，按 `65536` 取模递增 |
| `payload_length` | u8 | payload 长度，范围 `0`-`255` 字节 |
| `payload` | 可变 | 与消息类型对应的内容 |
| `crc` | u16 | `protocol_version` 至 payload 的 CRC-16/CCITT-FALSE；初值 `0xFFFF`，多项式 `0x1021` |

接收端必须拒绝 CRC、版本、长度或消息类型不合法的帧。需要确认的命令通过
`ACK` 中携带的原命令序列号匹配。

## 消息类型

| 值 | 名称 | payload |
|---:|---|---|
| `0x01` | `CMD_MOTION` | 下文定义的模式化运动意图 |
| `0x02` | `CMD_EVENT` | 包含 `event_type`、`payload`、`valid_for_ms` 的 UTF-8 JSON 对象 |
| `0x03` | `HEARTBEAT` | `sender_monotonic_ms:u32, valid_for_ms:u16` |
| `0x10` | `TELEMETRY` | UTF-8 JSON 对象 |
| `0x11` | `ACK` | `acknowledged_sequence:u16, status:u8`；`status=0` 表示接受 |
| `0x12` | `FAULT` | 包含 `code`、`severity`、`detail` 的 UTF-8 JSON 对象 |

`CMD_EVENT` 可用于低频业务事件，但不是车轮、PWM 或极坐标目标的承载方式。

### `ACTION_REQUEST`（下位机 -> RDK）

为保持 v2 兼容，下位机请求 RDK 执行赛题动作时使用 `CMD_EVENT`，其 UTF-8 JSON
格式如下：

```json
{"event_type":"ACTION_REQUEST","payload":{"action_id":3,"request_id":128,
"parameters":{"target_mm":50}},"valid_for_ms":1000}
```

`action_id` 和 `request_id` 均为无符号整数，分别占用 1 字节和 2 字节语义范围；
`parameters` 必须是 JSON 对象。RDK 按协议帧 `sequence` 去重，第一次收到时放入动作
队列并回复 `ACK status=0`，重复帧回复 `status=1` 且不得再次执行，字段非法时回复
`status=2`。动作是否完成通过后续状态遥测或 `ACTION_STATUS` 事件报告。

## `CMD_MOTION`

`CMD_MOTION` 表达 MSPM0 应执行的**高层运动模式**，不包含左右轮转速、转向角、
速度、限速或 PWM。MSPM0 负责每种模式的局部控制器、速度策略、编码器/IMU/灰度
闭环、急停和最终电机输出。

所有模式都有固定的公共头：

| 字段 | 大小 | 说明 |
|---|---:|---|
| `mode` | u8 | 运动模式 |
| `flags` | u8 | 标志位，见下表 |
| `valid_for_ms` | u16 | 本命令最多可被使用的时长，必须大于零 |

`flags` 定义如下：bit 0 是 `enabled`，为 `0` 时 MSPM0 必须安全停车；bit 1 是
`target_valid`，仅极坐标捕获模式可用；bit 2 是 `capture_armed`，仅极坐标捕获模式
可用；bit 3 是 `balance_valid`，仅滚珠平衡模式可用；bits 4-7 必须为零。其他模式
不能设置不属于自身的有效标志。

| `mode` | 名称 | 模式专属 payload | MSPM0 职责 |
|---:|---|---|---|
| `0` | `DISABLED` | 无，payload 共 4 字节 | 停止运动并禁用运动输出 |
| `1` | `IDLE` | 无，payload 共 4 字节 | 安全静止，保持可用 |
| `2` | `LINE_FOLLOW` | 无，payload 共 4 字节 | 使用 MSPM0 本地巡线和速度策略 |
| `3` | `VISION_ASSIST` | 无，payload 共 4 字节 | 使用 MSPM0 预置的视觉辅助策略 |
| `4` | `CAPTURE_TARGET_POLAR` | 下表字段，payload 共 18 字节 | 对齐、接近与捕获钢球 |
| `5` | `BALANCE_ROLLER` | 下表字段，payload 共 6 字节 | 本地摆杆平衡和本地循迹 |

因此 `LINE_FOLLOW` 的 payload 固定为四字节，绝不会包含速度、转向或限速字段。

### `CAPTURE_TARGET_POLAR` 专属字段

公共头后按下表追加 14 字节：

| 字段 | 大小 | 说明 |
|---|---:|---|
| `track_id` | u16 | RDK 目标跟踪器给出的目标标识 |
| `bearing_mdeg` | i32 | 钢球相对电磁铁捕获点前向轴的方位角，单位毫度 |
| `range_mm` | i32 | 钢球到电磁铁捕获点的地面平面距离，单位毫米 |
| `confidence_permille` | u16 | 视觉置信度，范围 `0`-`1000` |
| `measurement_age_ms` | u16 | 从测量到发送的时延，单位毫秒 |

零度是电磁铁捕获点的车体前向轴；`bearing_mdeg` 的正负方向必须与地面标定和
MSPM0 固件一致。`range_mm` 的参考点是电磁铁捕获点，不是摄像头或车体几何中心。

当 `target_valid=1` 时，上表字段必须是有效测量，且可按 `capture_armed` 决定是否
允许执行磁铁动作。当目标丢失、过期、置信度不足、捕获关闭或 RDK 车控总开关关闭时，
RDK 发送同一模式且 `enabled=0,target_valid=0` 的 18 字节安全帧，所有专属字段为零。
MSPM0 必须停止捕获运动。`capture_armed` 仅表示允许尝试捕获，不表示已经捕获成功。

MSPM0 必须拒绝未知模式、保留标志位、模式与 payload 长度不匹配、无效目标却携带
非零专属字段、过期测量或超量程目标。最新有效命令或心跳任一超时后，MSPM0 必须
安全停车；硬件急停优先级始终最高。

### `BALANCE_ROLLER` 专属字段

公共头后只追加 2 字节。相机固定在凹槽正上方，管槽中心 O 为零点；正方向由 RDK 标定
配置与 MSPM0 固件共同约定。

| 字段 | 大小 | 说明 |
|---|---:|---|
| `error_mm` | i16 | 钢球相对 O 点的有符号距离偏差，单位 mm |

当 `balance_valid=1` 时，MSPM0 只使用 `error_mm` 作为摆杆控制输入。速度和加速度不由
RDK 计算或发送。当钢球丢失、状态过期、平衡开关关闭或 RDK 车控总开关关闭时，RDK
发送同一模式且 `enabled=0,balance_valid=0` 的 6 字节安全帧，专属字段为零。MSPM0 不得对旧状态
积分；应执行本地安全中位或低增益保持策略。`BALANCE_ROLLER` 不携带车轮 PWM，MSPM0
仍独立执行红外循迹、急停和摆杆高频 PID。

## 反馈与遥测

建议 `TELEMETRY` 包含 `timestamp_ms`、`motion_sequence`、`speed_left_mm_s`、
`speed_right_mm_s`、`line_error`、`imu_yaw_mdeg`、`battery_mv`、`estop`、`faults`
和 `capture_state`。捕获反馈关闭时，MSPM0 只能上报 `CAPTURE_ATTEMPTED`，不能把
已发出磁铁动作误报为 `CAPTURED`。

若用于滚珠控制的车体惯性前馈，建议在同一 JSON 中增加：

```json
{"vehicle_motion":{"valid":true,"source":"m0_imu",
"ax_mm_s2":120,"ay_mm_s2":-8,"az_mm_s2":15,
"speed_mm_s":380,"yaw_rate_mdeg_s":420,"timestamp_ms":123456}}
```

RDK 必须校验 `valid`、时间戳和超时；数据无效或超过配置限幅时回退到纯反馈控制。
铰链端 ICM42688 的局部加速度不得直接标记为 `source=m0_imu`。

## CAN 映射

CAN 沿用相同编码帧和字节序。对 Classical CAN，每段数据的第一个字节为分段控制字，
其后最多承载 7 个协议帧字节：bit 7 标识首段，bit 6 标识尾段，bits 5-0 是从零开始的
分段索引。分段缺失或乱序时，接收端必须丢弃未完成帧。CAN ID、通道和波特率属于部署
配置，启用前必须与 MSPM0 固件确认一致。

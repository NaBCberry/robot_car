# RDK–MSPM0 协议 v1

本文定义 RDK X5 与 MSPM0 之间的字节级通信协议。所有多字节整数均使用
**大端序**（big-endian）。

UART 使用类 SLIP 的分帧方式：`0x7E` 同时作为一帧的开始和结束标记；帧内容中
出现的 `0x7E` 或 `0x7D` 必须转义为 `0x7D` 后接“原字节 XOR `0x20`”。

## 通用帧格式

| 字段 | 大小 | 说明 |
|---|---:|---|
| `protocol_version` | u8 | 协议版本，当前固定为 `1` |
| `message_type` | u8 | 消息类型，见下表 |
| `sequence` | u16 | 发送方序列号，按 `65536` 取模递增 |
| `payload_length` | u16 | payload 长度，范围为 `0`–`4096` 字节 |
| `payload` | 可变 | 与消息类型对应的内容 |
| `crc` | u16 | 对帧头和 payload 计算的 CRC-16/CCITT-FALSE；初值 `0xFFFF`，多项式 `0x1021` |

接收端必须拒绝 CRC、长度、版本或消息类型不合法的帧，并统计重复、过期和乱序的
序列号。需要确认的命令通过 `ACK` payload 中的原命令序列号进行匹配。

## 消息类型

| 值 | 名称 | payload |
|---:|---|---|
| `0x01` | `CMD_MOTION` | `mode:u8, enable:u8, speed_mm_s:i32, steering_mdeg:i32, speed_limit_mm_s:u32, valid_for_ms:u16` |
| `0x02` | `CMD_EVENT` | 包含 `event_type`、`payload`、`valid_for_ms` 的 UTF-8 JSON 对象 |
| `0x03` | `HEARTBEAT` | `sender_monotonic_ms:u32, valid_for_ms:u16` |
| `0x04` | `CMD_CAPTURE_TARGET` | 下文定义的固定长度二进制捕获目标 |
| `0x10` | `TELEMETRY` | UTF-8 JSON 对象，建议字段见下文 |
| `0x11` | `ACK` | `acknowledged_sequence:u16, status:u8`；`status=0` 表示已接受 |
| `0x12` | `FAULT` | 包含 `code`、`severity`、`detail` 的 UTF-8 JSON 对象 |

`CMD_MOTION.mode` 的值为：`0 DISABLED`、`1 IDLE`、`2 LINE_FOLLOW`、
`3 VISION_ASSIST`、`4 CAPTURE_SERVO`。无论其他字段为何值，`enable=0` 都表示
MSPM0 必须禁用 PWM，并将速度视为零。

每个运动命令均带有明确的有效期。最新有效运动目标或 RDK 心跳任一超时后，MSPM0
必须进入安全停车状态；硬件急停的优先级始终最高。

建议 `TELEMETRY` 使用以下键：`timestamp_ms`、`motion_sequence`、
`speed_left_mm_s`、`speed_right_mm_s`、`line_error`、`imu_yaw_mdeg`、
`battery_mv`、`estop`、`faults`。为兼容后续扩展，接收端应忽略未知 JSON 键。

## `CMD_CAPTURE_TARGET`

该消息用于 `MCU_TARGET_SERVO` 控制模式：RDK 上报钢球相对于**电磁铁捕获点**的
位置；MSPM0 负责本地接近、对齐、电机控制及电磁铁动作决策。它不是电机 PWM
命令。

| 字段 | 大小 | 说明 |
|---|---:|---|
| `flags` | u8 | bit 0：`target_valid`；bit 1：`capture_armed`；其余位必须为零 |
| `reserved` | u8 | 保留字节，必须为零 |
| `track_id` | u16 | RDK 目标跟踪器的目标标识 |
| `bearing_mdeg` | i32 | 目标相对捕获点前向轴的方位角，单位毫度；正负方向必须与标定及 MSPM0 固件一致 |
| `range_mm` | i32 | 目标到捕获点的地面平面距离，单位毫米 |
| `confidence_permille` | u16 | 视觉置信度，范围 `0`–`1000` |
| `measurement_age_ms` | u16 | 从测量时刻到发送时刻的时间差，单位毫秒 |
| `valid_for_ms` | u16 | MSPM0 最长允许使用该目标的时长，单位毫秒 |

MSPM0 必须丢弃无效、陈旧、超量程、低置信度或已过有效期的目标。当没有新鲜有效的
目标时，必须停止捕获过程中的车辆运动。`capture_armed` 仅表示 RDK 允许 MSPM0
尝试捕获，并不表示电磁铁必须报告捕获成功。

捕获结果遥测使用 `capture_state`，可取 `CAPTURED`、`CAPTURE_FAILED` 或
`CAPTURE_ATTEMPTED`。当捕获反馈关闭时，必须使用 `CAPTURE_ATTEMPTED`：它只表示
已发出电磁铁动作命令，实际是否吸住钢球未知。

## CAN 映射

CAN 传输沿用相同的编码帧语义和字节序。对于 Classical CAN，每个 CAN 数据字段的
第一个字节为分段控制字，其后最多承载 7 个协议帧字节：bit 7 标识首段，bit 6 标识
尾段，bits 5–0 为从零开始的分段索引。分段缺失或乱序时，接收端必须丢弃当前未完成
帧。

CAN ID、通道和波特率均为部署配置而非常量；启用 CAN 前，必须与 MSPM0 固件确认
这些参数一致。

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

`ACTION_REQUEST` 没有独立的 `message_type`，必须封装在 `CMD_EVENT (0x02)` 中。
发送方向固定为 **MSPM0 -> RDK X5**，用于请求 RDK 执行一个赛题动作。它只表达
“执行哪个动作及其参数”，不直接携带车轮速度、PWM 或电机角度。

#### 1. 外层帧和 JSON 结构

`CMD_EVENT.payload` 是 UTF-8 编码的 JSON 对象，结构固定如下：

```json
{
  "event_type": "ACTION_REQUEST",
  "payload": {
    "action_id": 6,
    "request_id": 128,
    "parameters": {"target_mm": 35, "timeout_ms": 30000}
  },
  "valid_for_ms": 1000
}
```

| JSON 路径 | 类型/范围 | 必填 | 说明 |
|---|---|:---:|---|
| `event_type` | 字符串，固定为 `ACTION_REQUEST` | 是 | 事件名称区分大小写，不能写成 `action_request` |
| `payload` | JSON object | 是 | 请求主体，不能为数组、字符串或 `null` |
| `payload.action_id` | 无符号整数 `0`-`255` | 是 | 赛题动作编号，见下表；协议层使用 u8 语义 |
| `payload.request_id` | 无符号整数 `0`-`65535` | 是 | 下位机生成的请求标识，用于日志和业务关联；可循环使用 |
| `payload.parameters` | JSON object | 是 | 动作参数；没有参数时发送 `{}`，不能省略或发送 `null` |
| `valid_for_ms` | 无符号整数 `1`-`65535` | 是 | 请求期望有效期，单位 ms；RDK 当前校验范围并用于事件语义 |

外层通用帧的 `sequence` 是 **RDK 去重依据**，与 JSON 内的 `request_id` 不同：

- `sequence` 由发送方每发一帧递增，回绕范围为 `0`-`65535`；
- `request_id` 由下位机的业务逻辑生成，只用于关联一次按键/任务请求；
- 重发同一请求时必须保持相同的 `sequence`，这样 RDK 才会回复重复状态而不再次执行；
- 如果使用新的 `sequence`，即使 `request_id` 相同，RDK 也会视为新的请求。

JSON 必须使用紧凑 UTF-8 编码，整个 `CMD_EVENT.payload`（包括 JSON 标点）不得超过
`255` 字节。禁止发送 `NaN`、`Infinity`、注释或额外的二进制尾部。

#### 2. 动作编号和参数

动作编号按试题编号定义，TUI 输入和 UART 请求使用相同数字：

| `action_id` | 对应题目 | `parameters` | RDK 行为 |
|---:|---:|---|---|
| `0` | 停止 | `{}` | 立即停止当前动作并发送安全运动状态 |
| `1` | 摆杆电机回零 | 可选 `timeout_ms`（默认 30000） | RDK 独占 CAN 控制 Y42 步进电机回到绝对坐标零点 |
| `2` | 题2 | 可选 `timeout_ms`，默认 20000 | 顺时针巡线一圈并在 A 点结束 |
| `3` | 题3 | 可选 `positive_mm`（默认 50）、`timeout_ms`（默认 5000） | 钢球从中心到正向位置，再回中心，最后到负向位置并稳定 |
| `4` | 题4 | 可选 `timeout_ms`（默认 8000） | M0 巡线到 B；RDK 直接 CAN 控制滚珠保持中心（`target_mm=0`） |
| `5` | 题5 | 可选 `timeout_ms`（默认 30000） | M0 巡线一圈；RDK 直接 CAN 控制滚珠保持中心（`target_mm=0`） |
| `6` | 题6 | **`target_mm` 必填**；可选 `timeout_ms`（默认 30000） | M0 巡线一圈；RDK 直接 CAN 控制滚珠保持启动时指定位置 |

`target_mm` 和 `positive_mm` 的单位均为 mm，中心 O 为 `0`，正负方向必须与
`camera.yaml` 和 MSPM0 的标定方向一致。`timeout_ms` 单位为 ms，必须为正整数。
动作 4 和动作 5 在 RDK 侧都使用滚珠中心平衡规则，区别仅在于巡线路径和完成检查点；
动作 6 才使用 `LINE_LAP_BALANCE_TARGET` 的指定位置规则。

动作 6 的完整请求示例：

```json
{"event_type":"ACTION_REQUEST","payload":{"action_id":6,"request_id":128,
"parameters":{"target_mm":35,"timeout_ms":30000}},"valid_for_ms":1000}
```

停止请求示例：

```json
{"event_type":"ACTION_REQUEST","payload":{"action_id":0,"request_id":129,
"parameters":{}},"valid_for_ms":1000}
```

#### 2.1 摆杆步进电机回零（`action_id=1`）

下位机需要让 RDK 控制摆杆步进电机回到机械绝对零点时，发送动作 `1`。下位机**不得**
把 Y42 CAN 帧透传到 UART，也不得自行把当前位置写成零点；绝对零点由电机已有的参数
固定定义。

```json
{"event_type":"ACTION_REQUEST","payload":{"action_id":1,"request_id":130,
"parameters":{"timeout_ms":30000}},"valid_for_ms":1000}
```

RDK 接受该动作后的职责如下：

1. 停止当前摆杆位置控制，确保 CAN 总线上的 Y42 只有一个控制者；
2. 按 [`canstep/docs/protocol.md`](../../../../canstep/docs/protocol.md) 发送 Y42 回零命令
   `A 9A 04 00 6B`，其中 `A` 是电机地址，`HomeMode=04` 表示“回到绝对位置坐标零点”，
   `Sync=00`；
3. 等待 Y42 对 `9A` 的接受响应以及完成响应 `9F`，必要时轮询 `3B` 回零状态；
4. 成功时报告 `ACTION_STATUS.status="COMPLETE"`、`reason="motor_home_complete"`；
   超时、拒绝或回零失败时报告 `status="FAILED"` 和对应原因，并停止后续摆杆动作。

`ACK status=0` 仅表示 RDK 已收到并排队该请求，**不代表电机已经回零**。下位机必须以
后续 `ACTION_STATUS` 为准；等待期间不得重复用新的 `sequence` 发送同一回零请求。若要
中断尚未完成的回零，发送 `action_id=0`；RDK 应执行安全停止，并按 Y42 协议使用
`A 9C 48 6B` 强制退出回零。

#### 3. RDK 接收和 ACK 时序

下位机应按以下顺序实现：

1. 发送带 `message_type=0x02` 的完整 v2 帧，并保存该帧的 `sequence`；
2. RDK 先由帧解码器校验帧头、版本、长度和 CRC，再校验 JSON 类型和字段范围；
3. RDK 根据 `sequence` 判断是否已经处理过；
4. RDK 对该帧返回 `ACK (0x11)`，ACK payload 为
   `acknowledged_sequence:u16`（大端）和 `status:u8`；
5. 只有 `status=0` 的首次请求才进入动作队列并执行；重复请求不会再次执行；
6. 动作开始、阶段变化、完成或失败通过 `ACTION_STATUS` 事件或 `TELEMETRY` 上报，
   不使用 ACK 表示动作完成。

ACK 状态定义：

| `status` | 名称 | 含义 | 下位机处理 |
|---:|---|---|---|
| `0` | `ACCEPTED` | 帧合法，首次收到，已放入动作队列 | 不再重发；等待状态事件 |
| `1` | `DUPLICATE` | `sequence` 已处理过 | 停止重试该序列，不重复执行 |
| `2` | `INVALID` | 帧已完整接收，但 JSON 或字段校验失败 | 修正请求后使用新的 `sequence` 重发 |

CRC、版本或帧长度错误的帧会被 RDK 丢弃并重新同步，通常不会产生 ACK。若在 ACK
超时前未收到响应，下位机可以重发**相同 sequence 的同一帧**；不得为同一次重试
随意修改 payload。RDK 的动作看门狗和急停逻辑仍然有效，收到 `ACCEPTED` 不代表
车辆已经完成动作。

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

题 4、5、6 由 RDK 直接控制摆杆时，MSPM0 可在同一 `TELEMETRY` JSON 顶层提供
`roller_feedforward_mm_s2`。该值必须已经换算到滚珠控制所需的管槽正方向，单位为
`mm/s^2`；正负方向必须与 `target_mm` 一致。

```json
{"roller_feedforward_mm_s2":120}
```

RDK 每次控制周期都使用最近一帧未超时遥测中的该值。字段缺失、`null`、非数值、非有限
数、遥测超时或链路中断时，前馈**立即视为 `0`**，不会延用上一次的值。MSPM0 应仅在有
有效车体加速度前馈时发送该字段；不具备前馈数据时可以省略它或显式发送 `0`。

若需要上报原始车体运动信息，可额外使用以下嵌套对象；它仅用于诊断，当前不会直接驱动
摆杆 PID：

```json
{"vehicle_motion":{"valid":true,"source":"m0_imu",
"ax_mm_s2":120,"ay_mm_s2":-8,"az_mm_s2":15,
"speed_mm_s":380,"yaw_rate_mdeg_s":420,"timestamp_ms":123456}}
```

铰链端 ICM42688 的局部加速度不得直接标记为 `source=m0_imu`。

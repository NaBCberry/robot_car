# RDK–MSPM0 Protocol v1

All multi-byte integers use **big-endian** byte order. UART uses a SLIP-like
delimiter: `0x7E` begins and ends a frame; bytes `0x7E` and `0x7D` inside the
frame become `0x7D` followed by `byte XOR 0x20`.

## Common frame

| Field | Size | Meaning |
|---|---:|---|
| protocol_version | u8 | `1` |
| message_type | u8 | table below |
| sequence | u16 | sender counter modulo 65536 |
| payload_length | u16 | 0–4096 bytes |
| payload | variable | message-specific |
| crc | u16 | CRC-16/CCITT-FALSE over header and payload, init `0xFFFF`, polynomial `0x1021` |

Receivers reject bad CRC/length/version/type. A receiver tracks duplicate,
old, and out-of-order sequence values. Commands requiring confirmation are
matched by the ACK payload's original sequence.

## Messages

| Value | Name | Payload |
|---:|---|---|
| `0x01` | CMD_MOTION | `mode:u8, enable:u8, speed_mm_s:i32, steering_mdeg:i32, speed_limit_mm_s:u32, valid_for_ms:u16` |
| `0x02` | CMD_EVENT | UTF-8 JSON object containing `event_type`, `payload`, `valid_for_ms` |
| `0x03` | HEARTBEAT | `sender_monotonic_ms:u32, valid_for_ms:u16` |
| `0x10` | TELEMETRY | UTF-8 JSON object; see below |
| `0x11` | ACK | `acknowledged_sequence:u16, status:u8` (`0` means accepted) |
| `0x12` | FAULT | UTF-8 JSON object containing fault `code`, `severity`, `detail` |

Motion mode values are `0 DISABLED`, `1 IDLE`, `2 LINE_FOLLOW`, and
`3 VISION_ASSIST`. `enable=0` always means PWM must be disabled and speed must
be treated as zero regardless of other fields. Every command has an explicit
validity window. MSPM0 must enter safe stop when the newest valid motion target
or RDK heartbeat expires; hardware E-stop always has priority.

Recommended TELEMETRY keys are `timestamp_ms`, `motion_sequence`,
`speed_left_mm_s`, `speed_right_mm_s`, `line_error`, `imu_yaw_mdeg`,
`battery_mv`, `estop`, and `faults`. Unknown JSON keys are ignored for forward
compatibility.

## CAN mapping

CAN carries the same encoded frame semantics and byte order. For Classical CAN,
each CAN data field starts with one segmentation byte and carries up to seven
frame bytes: bit 7 marks start, bit 6 marks end, and bits 5–0 are a zero-based
segment index. A missing/out-of-order segment discards the partial frame. CAN
IDs, channel, and bitrate are deployment configuration—not constants—and must
be agreed with the MSPM0 firmware before enabling CAN.

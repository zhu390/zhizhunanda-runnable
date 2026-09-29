# Jetson <-> RA6M5 正式视觉与比赛流程协议

> 对接基线：2026-07-17
>
> 传输：UART 115200 8N1
>
> 正式协议：`A5 5A + CRC16/MODBUS`
>
> 协议版本：`VER=0x01`

## 1. 使用范围与实现状态

本文是 Jetson Qt 端与 RA6M5 端的正式对接依据，覆盖全局心跳、Qt 流程控制、顶部 RGB-D 安全距离、三视图采集、视图选择、定靶启停、视觉误差、P000/P801、完成返回 HOME 和异常恢复。

比赛正式链路只使用本文的统一协议。旧 `FF ... FE`、`AA ... BB`、`CC ... DD` 帧仅供 PC 调试兼容，Jetson 正式程序不得发送或依赖旧帧。

当前 RA6M5 固件已经实现本文正式链路：`A5 5A` 帧解析、CRC16、独立心跳、`WORKFLOW_CTRL`、公共测距锁存、三视图单步驱动、定靶子状态机、原请求 `SEQ` 回传、重复控制帧抑制以及异常保持。正式协议默认关闭旧协议解析，比赛联调应只使用本文帧格式。

当前实现已经通过 Keil 编译和静态契约检查，尚需按第 10 章完成整机上板故障注入。Jetson 端不能因“帧可收发”跳过状态 ACK、业务完成状态或安全错误处理。

## 2. 串口参数与数据表示

| 项目 | 值 |
|---|---|
| 波特率 | `115200` |
| 数据位 | `8` |
| 校验位 | `None` |
| 停止位 | `1` |
| 流控 | `None` |
| 多字节整数 | 小端 |
| 最大 payload | `32 bytes` |
| 心跳发送周期 | `100~200 ms` |
| RA6M5 心跳超时 | `600 ms` |
| 视觉数据最长间隔 | 小于 `500 ms`，建议 `100~200 ms` |

`int16`、`uint16`、`uint32` 均按二进制补码/无符号整数直接传输，不发送 ASCII 十六进制字符串。

## 3. 统一帧格式

### 3.1 字节布局

```text
SOF0 SOF1 VER TYPE SEQ LEN PAYLOAD[LEN] CRC16_LO CRC16_HI
A5   5A   01  xx   xx  nn  ...          xx       xx
```

总帧长为 `8 + LEN` 字节。

| 字段 | 长度 | 说明 |
|---|---:|---|
| `SOF0` | 1 | 固定 `0xA5` |
| `SOF1` | 1 | 固定 `0x5A` |
| `VER` | 1 | 当前固定 `0x01` |
| `TYPE` | 1 | 消息类型 |
| `SEQ` | 1 | Jetson 请求/采样使用 `0x01~0xFF` 循环；`0x00` 保留给 RA6M5 异步故障 |
| `LEN` | 1 | payload 长度，范围 `0~32` |
| `PAYLOAD` | LEN | 按 TYPE 解释 |
| `CRC16` | 2 | CRC16/MODBUS，低字节先发 |

### 3.2 CRC16/MODBUS

- 初值：`0xFFFF`
- 多项式：`0xA001`
- 计算范围：从 `VER` 到最后一个 `PAYLOAD` 字节
- 不包含：`A5 5A` 和 CRC 两个字节
- 发送顺序：`CRC16_LO CRC16_HI`

Jetson Python 参考实现：

```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > 32:
        raise ValueError('payload too long')
    body = bytes([0x01, msg_type & 0xFF, seq & 0xFF, len(payload)]) + payload
    crc = crc16_modbus(body)
    return bytes([0xA5, 0x5A]) + body + crc.to_bytes(2, 'little')
```

### 3.3 接收校验顺序

1. 搜索连续帧头 `A5 5A`；
2. 检查 `VER=0x01`；
3. 检查 `LEN<=32`；
4. 等待完整的 `8+LEN` 字节；
5. 校验 CRC；
6. 检查 TYPE、payload 长度和业务参数；
7. 只有全部通过后才允许改变控制状态。

CRC错误、不完整帧、错误帧头直接丢弃，不产生机械臂动作。已通过 CRC 但 TYPE 或参数非法的帧回传错误。

### 3.4 SEQ、重发与重复命令

- Jetson 的 `HEARTBEAT`、`VISION_ERROR`、`SAFE_DISTANCE` 和全部控制帧必须共用一个线程安全的全局 SEQ 生成器。
- Jetson 每发送一个新帧都递增 `SEQ`，`0xFF` 之后回到 `0x01`，不得发送 `SEQ=0x00`。
- `SEQ=0x00` 只用于 RA6M5 主动上报、无法关联某条请求的异步故障。
- Jetson/PC 程序启动时应使用随机或持久化的非零初始 `SEQ`，不要每次重启都固定从 `0x01` 开始，避免与 RA6M5 最近控制命令缓存碰撞。
- 所有串口写入必须经过一个 TX 队列或全局互斥锁，禁止多个线程分别生成 SEQ 并交叠写帧。
- `HEARTBEAT`、`VISION_ERROR`、`SAFE_DISTANCE` 每个新采样使用新 `SEQ`。
- `WORKFLOW_CTRL`、`CAPTURE_CTRL`、`TARGET_CTRL` 属于控制命令。
- 控制命令等待 ACK `300 ms`，最多重发 `3` 次；重发时必须逐字节复用相同 `TYPE + SEQ + payload + CRC`。
- RA6M5 控制命令缓存有效期为 `3000 ms`。有效期内收到完全相同的重复控制命令时，只重发已有状态，不得再次启动复位或运动。
- 有效期内相同 `TYPE+SEQ` 携带不同 payload 时，RA6M5 回传 `SEQ_CONFLICT`，并保持原状态；缓存过期后按新命令重新做状态和安全检查。
- 控制命令对应的 `STATUS/ERROR` 帧头 `SEQ` 必须等于原请求 `SEQ`。
- `ALIGN_DONE`、`OUTPUT_ON/OFF` 等定靶会话状态沿用该次 `TARGET_CTRL START` 的 `SEQ`。

Jetson 不得通过固定延时猜测机械臂已经到位，必须等到对应状态回传后再发送下一条控制命令。

## 4. 消息类型

| TYPE | 方向 | 名称 | LEN | PAYLOAD |
|---:|---|---|---:|---|
| `0x01` | Jetson -> RA6M5 | `HEARTBEAT` | 4 | `uint32 tick_ms` |
| `0x02` | Jetson -> RA6M5 | `TARGET_CTRL` | 1 | `uint8 enable` |
| `0x03` | Jetson -> RA6M5 | `VISION_ERROR` | 5 | `int16 dcx + int16 dcy + uint8 valid` |
| `0x04` | Jetson -> RA6M5 | `CAPTURE_CTRL` | 2 | `uint8 action + uint8 point_id` |
| `0x05` | Jetson -> RA6M5 | `SAFE_DISTANCE` | 3 | `uint16 distance_mm + uint8 valid` |
| `0x06` | Jetson -> RA6M5 | `WORKFLOW_CTRL` | 1 | `uint8 action` |
| `0x81` | RA6M5 -> Jetson | `STATUS` | 3 | `uint8 event + uint8 value + uint8 error_code` |
| `0xFE` | RA6M5 -> Jetson | `ERROR` | 1 | `uint8 error_code` |

## 5. Jetson -> RA6M5 命令

### 5.1 HEARTBEAT，TYPE=0x01

```text
PAYLOAD: tick0 tick1 tick2 tick3
```

`tick_ms` 为 Jetson 本地单调毫秒计数，小端。Jetson 串口连接成功后全程每 `100~200 ms` 发送一次，不因 Qt 页面切换而停止。

心跳只表示通信在线，不能替代安全距离、视觉对准或 P000。

只有通过校验的 `HEARTBEAT` 可以刷新 RA6M5 的全局心跳计时；VISION_ERROR、SAFE_DISTANCE 和控制命令均不能代替心跳。

### 5.2 WORKFLOW_CTRL，TYPE=0x06

| action | 名称 | Qt 触发 | RA6M5 行为 |
|---:|---|---|---|
| `0x01` | `START_MEASURE` | 点击“开始” | 关激光、清理上一轮许可、soft_reset、前往公共测距位 |
| `0x02` | `FINISH_RETURN_HOME` | 点击“完成/返回” | 关定靶和激光，soft_reset，PASS 后回到 IDLE |
| `0x03` | `ABORT_HOLD` | 点击取消/安全停止 | 关激光并中止运动，原地保持，不自动 HOME |

`START_MEASURE` 不是启动定靶。RA6M5 到公共测距位后等待顶部 RGB-D 的 `SAFE_DISTANCE`。

### 5.3 SAFE_DISTANCE，TYPE=0x05

```text
PAYLOAD: distance_lo distance_hi valid
```

| 字段 | 说明 |
|---|---|
| `distance_mm` | 顶部 RGB-D 给出的实际距离，单位 mm |
| `valid=0x01` | 本帧测距有效 |
| `valid=0x00` | 未识别或测距无效，`distance_mm` 被忽略 |

发送规则：

1. 只在收到 `WORKFLOW MEASURE_POSITION_READY` 后开始发送；
2. 建议先在 Jetson 端做深度滤波，再每 `100~200 ms` 发送；
3. 持续发送到收到 `SAFE_DISTANCE=SAFE` 或 `UNSAFE`；
4. HOME、三视图移动和点痣阶段不要求持续发送；
5. RA6M5 使用固件中的标定阈值裁决，Jetson 不发送安全布尔结果。

当前固件要求连续 `3` 个不同 `SEQ` 的有效安全样本后锁存。正式安全距离阈值为 `150 mm`：`distance_mm>=150` 才能计入安全样本，低于该值按距离过近处理。

距离过近时采用分步撤离：RA6M5 每次沿 `+Y` 移动 `20 mm`，到位后回传 `WORKFLOW RETREAT_STEP_READY`；Jetson 收到该状态后必须重新采集实际深度，并使用新 `SEQ` 发送一帧新的 `SAFE_DISTANCE`。每步等待新测距最多 `2000 ms`，最多 `5` 步、总撤离上限 `100 mm`。累计移动量不能代替 RGB-D 实测距离；完成 5 步后仍低于 `150 mm` 时进入 `FAULT_HOLD`。撤离后即使距离恢复安全也不置位本轮安全锁存，只回传 `RETREAT_DONE_WAIT_RESTART`，等待用户用新 `SEQ` 再次点击“开始”。

一次测距锁存要求患者在本轮流程中保持稳定，并要求所有固化视图点按最不利距离完成安全标定。Jetson 检测到患者或场景明显变化时，应发送 `WORKFLOW_CTRL ABORT_HOLD`，并由用户重新点击 Qt“开始”完成测距。

### 5.4 CAPTURE_CTRL，TYPE=0x04

```text
PAYLOAD: action point_id
```

| action | point_id | 名称 | RA6M5 行为 |
|---:|---:|---|---|
| `0x01` | `1/2/3` | `GOTO_VIEW` | 移动到左/正/右固化拍摄点并保持 |
| `0x02` | `0` | `HOME` | 执行 soft_reset；用于采集开始前和三张图完成后 |
| `0x03` | `1/2/3` | `SELECT_VIEW` | 移动到医生选中的视图点，该点同时是定靶预开始点 |
| `0x04` | `0` | 旧调试保留 | 比赛正式流程禁止使用 |

`point_id` 定义：

| point_id | 视图 |
|---:|---|
| `1` | 左视图 |
| `2` | 正视图 |
| `3` | 右视图 |

Jetson 只下发编号，不下发坐标、关节角、`auto` 参数或电机命令。医生目前只选择一张视图，不在图像中点击具体痣点。

### 5.5 TARGET_CTRL，TYPE=0x02

| enable | 名称 | RA6M5 行为 |
|---:|---|---|
| `0x01` | `START` | 检查全部安全门，通过后进入 `TARGET_WAIT_DETECT` |
| `0x00` | `STOP_HOLD` | 立即关闭 P801、停止视觉伺服并保持当前位置，不自动 HOME |

点击 Qt“开始点痣”时先发送 `START`。Jetson 必须等到 `TARGET_CTRL_ON` 和 `TARGET_READY` 后才能开始发送视觉误差。

完成后回 HOME 不使用 `TARGET_CTRL STOP`，而使用 `WORKFLOW_CTRL FINISH_RETURN_HOME`。

### 5.6 VISION_ERROR，TYPE=0x03

```text
PAYLOAD: dcx_lo dcx_hi dcy_lo dcy_hi valid
```

定义：

```text
dcx = target_center_x - image_center_x
dcy = target_center_y - image_center_y
```

- `dcx>0`：目标在图像中心右侧；`dcx<0`：目标在左侧。
- `dcy>0`：目标在图像中心下方；`dcy<0`：目标在上方。
- `valid=1`：自动识别到可信目标，本帧误差有效。
- `valid=0`：未识别到目标或置信度不足，RA6M5 忽略 `dcx/dcy` 并按视觉丢失处理。

坐标映射方向由 RA6M5 标定参数承担，Jetson 不根据机械臂方向临时反转误差符号。

发送规则：

1. 只在收到 `TARGET_READY` 后发送；
2. 建议周期 `100~200 ms`，任意两帧间隔必须小于 `500 ms`；
3. 医生只选择视图，目标痣由 Jetson 自动识别和跟踪；
4. 没有可靠目标时必须发送 `valid=0`，不得用 `dcx=0,dcy=0,valid=1` 伪造对准；
5. 进入 P801 输出后仍继续发送，用于误差超限和视觉丢失安全监测。

## 6. RA6M5 -> Jetson 回传

### 6.1 STATUS，TYPE=0x81

```text
PAYLOAD: event value error_code
```

- `error_code=0x00` 表示该状态正常。
- 控制请求被拒绝时，RA6M5 回传对应 event、失败 value 和非零 `error_code`。
- Jetson 应同时解析 event、value 和 error_code，不能只判断 event。

状态事件：

| event | 名称 | value | 含义 |
|---:|---|---:|---|
| `0x01` | `TARGET_READY` | `0x01` | 已进入等待视觉状态，可发送 VISION_ERROR |
| `0x02` | `ALIGN_DONE` | `0x01` | 连续新帧对准且二次确认完成 |
| `0x03` | `OUTPUT` | `0x01/0x00` | P801 已打开/关闭 |
| `0x04` | `TARGET_CTRL` | `0x01/0x00` | 定靶状态机已启用/关闭或拒绝 |
| `0x05` | `HEARTBEAT_ALIVE` | `0x01` | 可选心跳响应 |
| `0x06` | `SAFE_DISTANCE` | `0x01/0x00` | 距离安全并锁存/距离过近 |
| `0x07` | `VISION_STATE` | `0x01/0x00` | 视觉重新有效/丢失或超时 |
| `0x10` | `CAPTURE_POINT_READY` | `1/2/3` | 左/正/右拍摄点已真实到位 |
| `0x11` | `CAPTURE_HOME_READY` | `0x01` | soft_reset final verify PASS |
| `0x12` | `SELECTED_VIEW_READY` | `1/2/3` | 选中视图点已到位 |
| `0x20` | `WORKFLOW` | 见下表 | 顶层比赛流程状态 |
| `0x21` | `COMMAND_ACK` | `0x01/0x00` | 控制命令已接受/拒绝，SEQ关联原请求 |

`WORKFLOW event=0x20` 的 value：

| value | 名称 | 含义 |
|---:|---|---|
| `0x00` | `IDLE` | 空闲，激光关闭 |
| `0x01` | `START_ACCEPTED` | 已接受 Qt“开始”并开始安全复位 |
| `0x02` | `MEASURE_POSITION_READY` | 已到公共测距位，可发送 SAFE_DISTANCE |
| `0x03` | `DISTANCE_SAFE_LATCHED` | 本轮安全距离许可已锁存 |
| `0x04` | `RETREAT_DONE_WAIT_RESTART` | 撤离完成，等待重新开始 |
| `0x05` | `RETURN_HOME_DONE` | 完成/返回的 soft_reset 已通过 |
| `0x06` | `ABORTED_HOLD` | 已关闭输出并中止运动，原地保持 |
| `0x07` | `FAULT_HOLD` | 故障锁存，等待人工重新开始 |
| `0x08` | `RETREAT_STEP_READY` | 一步 +Y 撤离已到位，Jetson 应发送一帧新的 SAFE_DISTANCE |

`COMMAND_ACK` 只用于 `WORKFLOW_CTRL`、`CAPTURE_CTRL` 和 `TARGET_CTRL`。接受命令时立即回 `value=1,error=0`，动作完成后再回对应业务状态；拒绝时回 `value=0,error=原因`，并且不得执行动作。

### 6.2 ERROR，TYPE=0xFE

`ERROR` 用于心跳超时、限位等异步故障。异步故障固定使用 `SEQ=0x00`；由控制命令触发的 ACK 和业务完成状态仍使用该命令原始 `SEQ`。预期内的请求拒绝优先通过 `STATUS.error_code` 返回。

| error_code | 名称 | 含义 |
|---:|---|---|
| `0x00` | `NONE` | 无错误 |
| `0x01` | `VERSION_MISMATCH` | 协议版本不支持 |
| `0x02` | `UNKNOWN_TYPE` | 未知消息类型 |
| `0x03` | `INVALID_PARAM` | payload 长度、action、point_id 或参数非法 |
| `0x04` | `POSE_INVALID` | 位姿无效，需要 soft_reset |
| `0x05` | `BUSY` | 当前状态正在执行其他动作 |
| `0x06` | `MOTION_FAILED` | IK、点位到达或运动确认失败 |
| `0x07` | `HEARTBEAT_TIMEOUT` | 超过 600 ms 未收到合法心跳 |
| `0x08` | `SAFETY_INPUT` | 限位、急停或硬件安全输入触发 |
| `0x09` | `SAFE_DISTANCE_TOO_CLOSE` | 实测距离低于标定阈值 |
| `0x0A` | `VISION_LOST` | 视觉无效或超过 500 ms 未更新 |
| `0x0B` | `SOFT_RESET_FAILED` | final verify FAIL |
| `0x0C` | `INVALID_STATE` | 当前流程状态不允许该命令 |
| `0x0D` | `SEQ_CONFLICT` | 相同 TYPE+SEQ 携带不同 payload |
| `0x0E` | `TARGET_GATE_DENIED` | 定靶安全门条件不完整 |
| `0x0F` | `MOTION_ABORTED` | 当前运动已被安全中止 |

### 6.3 状态中的错误用法

- 成功：`error_code=0x00`。
- 定靶启动被拒绝：`event=0x04, value=0x00, error_code=具体原因`。
- 距离过近：`event=0x06, value=0x00, error_code=0x09`。
- P000正常松开导致 OUTPUT_OFF：`event=0x03, value=0x00, error_code=0x00`。
- 视觉丢失导致 OUTPUT_OFF：先回 `OUTPUT_OFF, error_code=0x0A`，再回 `VISION_STATE=0`。
- 心跳超时、限位等异步故障：发送 `ERROR`，并进入 `WORKFLOW FAULT_HOLD`。

## 7. 比赛完整通信时序

以下每一条 `WORKFLOW_CTRL`、`CAPTURE_CTRL` 和 `TARGET_CTRL` 都必须先收到同 SEQ 的 `COMMAND_ACK`。为避免时序图重复，除首次 START 外，其余 ACK 不再逐行展开。

### 7.1 链路建立与公共测距

```text
Jetson -> RA6M5: HEARTBEAT，之后每 100~200 ms 持续发送

用户点击 Qt“开始”
Jetson -> RA6M5: WORKFLOW_CTRL START_MEASURE
RA6M5 -> Jetson: STATUS COMMAND_ACK value=1
RA6M5 -> Jetson: STATUS WORKFLOW START_ACCEPTED

RA6M5: 关闭 P801，清除旧会话，执行 soft_reset
RA6M5: final verify PASS 后移动到公共测距位
RA6M5 -> Jetson: STATUS WORKFLOW MEASURE_POSITION_READY

Jetson: 顶部 RGB-D 取得稳定距离
Jetson -> RA6M5: SAFE_DISTANCE distance_mm valid=1
```

距离安全分支：

```text
RA6M5 -> Jetson: STATUS SAFE_DISTANCE value=1
RA6M5 -> Jetson: STATUS WORKFLOW DISTANCE_SAFE_LATCHED
Jetson: 允许用户进入“采集人脸图”
```

距离不安全分支：

```text
RA6M5 -> Jetson: STATUS SAFE_DISTANCE value=0 error=0x09
RA6M5: 立即关 P801，沿 +Y 撤离 20 mm
RA6M5 -> Jetson: STATUS WORKFLOW RETREAT_STEP_READY
Jetson -> RA6M5: 发送一帧新 SEQ 的 SAFE_DISTANCE
RA6M5: 若仍不安全则重复上述步骤，最多 5 步；每步等待新测距最多 2 s
RA6M5 -> Jetson: STATUS WORKFLOW RETREAT_DONE_WAIT_RESTART
Jetson: 停止推进流程，提示用户重新点击“开始”
```

重新点击“开始”时，RA6M5 必须再次执行 `soft_reset -> 公共测距位 -> 安全距离确认`。HOME 点没有有效深度数据不能单独触发故障。

### 7.2 三视图逐步采集

```text
用户点击 Qt“采集人脸图”
Jetson -> RA6M5: CAPTURE_CTRL HOME, point_id=0
RA6M5: soft_reset
RA6M5 -> Jetson: STATUS CAPTURE_HOME_READY

Jetson -> RA6M5: CAPTURE_CTRL GOTO_VIEW, point_id=1
RA6M5 -> Jetson: STATUS CAPTURE_POINT_READY, value=1
Jetson: 拍摄并保存左视图

Jetson -> RA6M5: CAPTURE_CTRL GOTO_VIEW, point_id=2
RA6M5 -> Jetson: STATUS CAPTURE_POINT_READY, value=2
Jetson: 拍摄并保存正视图

Jetson -> RA6M5: CAPTURE_CTRL GOTO_VIEW, point_id=3
RA6M5 -> Jetson: STATUS CAPTURE_POINT_READY, value=3
Jetson: 拍摄并保存右视图

Jetson -> RA6M5: CAPTURE_CTRL HOME, point_id=0
RA6M5: soft_reset
RA6M5 -> Jetson: STATUS CAPTURE_HOME_READY
Jetson: Qt 显示三张视图
```

每一张图必须在对应 `CAPTURE_POINT_READY` 后拍摄。RA6M5 不自动前往下一点；Jetson 拍完一张后才发送下一点。

### 7.3 医生选择视图

```text
医生在 Qt 中只选择左/正/右的一张视图
Jetson: 得到 point_id=1/2/3
Jetson -> RA6M5: CAPTURE_CTRL SELECT_VIEW, point_id
RA6M5: 移动到该视图固化点并保持，P801 始终关闭
RA6M5 -> Jetson: STATUS SELECTED_VIEW_READY, value=point_id
```

该位置就是定靶预开始点。医生当前不选择具体痣坐标，Jetson 后续在实时画面中自动识别和跟踪目标痣。

### 7.4 开始点痣、视觉对准与输出

```text
用户点击 Qt“开始点痣”
Jetson -> RA6M5: TARGET_CTRL START

安全门通过:
RA6M5 -> Jetson: STATUS TARGET_CTRL value=1
RA6M5 -> Jetson: STATUS TARGET_READY value=1

Jetson: 开始每 100~200 ms 发送 VISION_ERROR
RA6M5: 视觉伺服对准；先连续 3 个新帧满足 abs(dcx/dcy)<=5 px，再连续 2 个新帧完成二次确认
RA6M5 -> Jetson: STATUS ALIGN_DONE value=1

Jetson/Qt: 提示操作者按住 P000
操作者按住 P000，且全部输出条件仍成立
RA6M5: 冻结机械臂当前位置，打开 P801
RA6M5 -> Jetson: STATUS OUTPUT value=1
```

若安全门不满足：

```text
RA6M5 -> Jetson:
STATUS TARGET_CTRL value=0 error_code=具体原因

Jetson:
不发送 VISION_ERROR
Qt 显示错误并等待人工处理
```

### 7.5 输出期间的视觉监测

输出期间 Jetson 继续发送视觉误差，但 RA6M5 不在 P801 开启时移动机械臂。

- 误差超出最终阈值：RA6M5 立即关闭 P801，回传 `OUTPUT_OFF`，退回 `TARGET_ALIGN`，重新对准后再次等待 P000。
- `valid=0` 或超过 `500 ms` 没有新视觉帧：立即关闭 P801，回传 `OUTPUT_OFF + VISION_LOST`，原地进入 `TARGET_WAIT_DETECT`。
- 视觉恢复：Jetson 继续发送 `valid=1`，RA6M5 回传 `VISION_STATE=1` 后重新对准，不需要医生再次点击“开始点痣”。
- P000 松开：立即关闭 P801并回传 `OUTPUT_OFF`，机械臂保持当前位置，等待 Qt“完成/返回”。
- 单次 P801 连续输出最长 `10 s`；达到上限后按正常 HOLD 路径关闭输出，必须重新完成对准和按键许可才能再次输出。

### 7.6 完成、返回与取消

正常完成：

```text
用户松开 P000
RA6M5 -> Jetson: STATUS OUTPUT value=0
RA6M5: 原地保持

用户点击 Qt“完成/返回”
Jetson -> RA6M5: WORKFLOW_CTRL FINISH_RETURN_HOME
RA6M5: 关闭定靶和 P801，执行 soft_reset

soft reset final verify PASS:
RA6M5 -> Jetson: STATUS WORKFLOW RETURN_HOME_DONE
RA6M5: 进入 IDLE
```

安全取消：

```text
Jetson -> RA6M5: WORKFLOW_CTRL ABORT_HOLD
RA6M5: 关闭 P801，中止运动并原地保持
RA6M5 -> Jetson: STATUS WORKFLOW ABORTED_HOLD
```

`ABORT_HOLD` 和心跳超时都不自动回 HOME。必须在故障解除和心跳稳定后，由用户重新点击 Qt“开始”，RA6M5 才执行安全复位。

### 7.7 心跳超时

超过 `600 ms` 没有合法心跳时，RA6M5 必须立即：

1. 拉低 P801；
2. 中止当前运动并拒绝新运动；
3. 清除视觉有效标志、安全距离锁存、选中视图和发射许可；
4. 进入 `FAULT_HOLD`，保持当前位置，不自动 HOME；
5. 尝试回传 `ERROR HEARTBEAT_TIMEOUT` 和 `WORKFLOW FAULT_HOLD`。

心跳恢复后不能自动续跑。Jetson 必须等待用户重新点击 Qt“开始”。

## 8. 超时、重发与并发规则

### 8.1 控制命令

1. Jetson 发送 `WORKFLOW_CTRL`、`CAPTURE_CTRL` 或 `TARGET_CTRL` 后等待同 SEQ 的 `COMMAND_ACK`。
2. `300 ms` 内未收到 ACK，可使用完全相同的帧重发。
3. 最多重发 `3` 次；重发不能更换 SEQ。
4. 收到 `COMMAND_ACK value=1` 后停止重发，继续等待业务完成状态。
5. 收到 `COMMAND_ACK value=0` 后立即停止重发并处理 `error_code`。
6. 等待动作完成期间必须继续发送心跳。

建议业务完成超时：

| 动作 | Jetson 等待上限 |
|---|---:|
| 公共测距位到达 | `15 s` |
| soft_reset / HOME | `45 s` |
| 单个三视图点到达 | `15 s` |
| 选中视图点到达 | `15 s` |
| TARGET_READY | `2 s` |
| 视觉对准 | 不设固定短超时，由用户取消或视觉状态决定 |

业务超时后不要改 SEQ 快速重发运动命令。Jetson 应在 Qt 显示超时，并发送 `ABORT_HOLD` 或等待人工确认。

### 8.2 并发

- 同一时刻只允许一个机械臂控制命令处于执行中。
- 当前动作未完成时，新 GOTO、HOME、SELECT 或 START 请求回 `BUSY`。
- `ABORT_HOLD`、心跳超时和硬件安全输入拥有最高优先级，不能被 BUSY 拒绝。
- VISION_ERROR 只更新最新视觉样本，不排队执行历史误差。
- 重复 `SEQ` 的 VISION_ERROR 不计入新视觉帧；SAFE_DISTANCE 在本轮 3 帧锁存窗口内与任一已计入 SEQ 重复时不增加 streak。
- HEARTBEAT 不改变机械臂业务状态。

### 8.3 Qt 必须遵守

- 未收到公共测距位到达，不发送有效 SAFE_DISTANCE。
- 未锁存安全距离，不允许点击“采集人脸图”或“开始点痣”。
- 未收到当前拍摄点到达，不拍照、不发送下一点。
- 未收到选中视图到达，不允许发送 TARGET_CTRL START。
- 未收到 TARGET_READY，不发送 VISION_ERROR。
- 未收到 ALIGN_DONE，不提示按 P000。
- 收到任何非零 error_code，停止自动推进并显示明确原因。

## 9. 完整 HEX 示例

以下 CRC 均按第 3 章规则计算，可直接作为 Jetson 单元测试向量。示例中的 SEQ 只是测试值，正式程序应自行递增。

| 场景 | HEX |
|---|---|
| HEARTBEAT，seq=01，tick=1234 | `A5 5A 01 01 01 04 D2 04 00 00 19 AF` |
| WORKFLOW START，seq=10 | `A5 5A 01 06 10 01 01 D8 DD` |
| COMMAND_ACK，seq=10 | `A5 5A 01 81 10 03 21 01 00 85 CC` |
| WORKFLOW START_ACCEPTED，seq=10 | `A5 5A 01 81 10 03 20 01 00 D4 0C` |
| WORKFLOW MEASURE_READY，seq=10 | `A5 5A 01 81 10 03 20 02 00 D4 FC` |
| SAFE_DISTANCE 120 mm，seq=11 | `A5 5A 01 05 11 03 78 00 01 B6 0B` |
| STATUS SAFE，seq=11 | `A5 5A 01 81 11 03 06 01 00 08 07` |
| CAPTURE HOME，seq=12 | `A5 5A 01 04 12 02 02 00 55 D2` |
| CAPTURE HOME_READY，seq=12 | `A5 5A 01 81 12 03 11 01 00 FC 03` |
| CAPTURE GOTO 左视图，seq=13 | `A5 5A 01 04 13 02 01 01 95 1E` |
| 左视图 READY，seq=13 | `A5 5A 01 81 13 03 10 01 00 90 03` |
| SELECT 右视图，seq=20 | `A5 5A 01 04 20 02 03 03 1A FB` |
| 右视图 SELECTED_READY，seq=20 | `A5 5A 01 81 20 03 12 03 00 34 A7` |
| TARGET START，seq=21 | `A5 5A 01 02 21 01 01 88 22` |
| TARGET_CTRL_ON，seq=21 | `A5 5A 01 81 21 03 04 01 00 E9 C3` |
| TARGET_READY，seq=21 | `A5 5A 01 81 21 03 01 01 00 F9 C2` |
| VISION dcx=-7,dcy=-50，seq=22 | `A5 5A 01 03 22 05 F9 FF CE FF 01 08 ED` |
| ALIGN_DONE，target seq=21 | `A5 5A 01 81 21 03 02 01 00 09 C2` |
| OUTPUT_ON，target seq=21 | `A5 5A 01 81 21 03 03 01 00 58 02` |
| OUTPUT_OFF，视觉丢失 | `A5 5A 01 81 21 03 03 00 0A D9 95` |
| VISION_LOST | `A5 5A 01 81 21 03 07 00 0A 98 54` |
| WORKFLOW FINISH，seq=30 | `A5 5A 01 06 30 01 02 99 16` |
| RETURN_HOME_DONE，seq=30 | `A5 5A 01 81 30 03 20 05 00 57 0B` |

Jetson 端应使用 `build_frame()` 动态生成实际帧，不应把示例 HEX 当作固定命令反复发送，因为实际 SEQ 必须随新请求变化。

## 10. 安全不变量与对接检查

RA6M5 实现必须始终满足：

1. P801 只可能在 `TARGET_OUTPUT` 中打开。
2. P000 是按住许可，不是直接激光开关。
3. 输出必须同时满足心跳、pose、安全距离锁存、选中视图、视觉稳定、P000和无限位/急停。
4. P801 打开期间机械臂冻结，视觉帧只用于监测。
5. 误差超限、视觉超时、P000松开、心跳超时或限位触发立即关闭 P801。
6. 心跳超时后原地保持，不能自动回 HOME或自动恢复。
7. 所有协议 HOME 都调用 soft_reset；final verify FAIL 不得回 HOME_READY。
8. 新开始、完成、取消、故障或掉电必须清除安全距离锁存和选中视图。
9. 单次输出达到 `10 s` 必须自动关闭，串口 `laser_on` 和 `target_enable` 不得绕过正式安全门。
10. 公共测距只有连续 3 个不同非零 SEQ 且实测距离均不低于 `150 mm` 才能锁存。
11. 距离过近时只能逐步 `+Y 20 mm`，每步到位后重新测距；最多 5 步，累计 100 mm 后仍不安全必须进入 `FAULT_HOLD`。
12. ALIGN_DONE 需要 `3+2` 共 5 个新有效帧满足 `abs(dcx)<=5 && abs(dcy)<=5`；任一无效或超限帧清除对应稳定计数。

Jetson 端交付前检查：

- CRC测试向量全部通过；
- 串口后台接收能处理粘包、拆包和多帧连续到达；
- 心跳独立线程不会被相机或 Qt 主线程阻塞；
- 所有消息共用一个非零全局 SEQ 生成器，所有写串口操作经过单一 TX 队列或全局锁；
- 控制命令重发保持相同 TYPE、SEQ、payload 和 CRC；
- `RETREAT_STEP_READY` 后重新采集实际深度，不能用累计移动量推算安全距离；
- STATUS 按 event/value/error_code 三字段解析；
- 三视图严格逐点驱动；
- 医生只选择视图，不发送痣点像素坐标；
- 未识别目标时发送 `VISION_ERROR valid=0`；
- 视觉误差发送周期不超过 `200 ms`；
- OUTPUT_ON/OFF、VISION_LOST、FAULT_HOLD 和 RETURN_HOME_DONE 均能更新 Qt；
- 收到错误后停止自动推进，不连续发送新的运动命令。

## 附录 A：旧调试协议

以下格式只用于旧固件或显式将 `JETSON_LEGACY_PROTOCOL_ENABLE` 改为 `1` 后的实验调试，不用于 Jetson 比赛程序。当前比赛固件默认值为 `0`，会忽略这些旧帧；PC 正式联调应使用 `DEBUG/formal_workflow_demo.py`。

```text
旧视觉误差:
FF 05 03 dcx_lo dcx_hi dcy_lo dcy_hi checksum FE
checksum = (05 + 03 + DATA...) & 0xFF

旧状态机启停:
AA 01 01 BB
AA 01 00 BB

旧 RA6M5 状态:
CC func value DD
```

旧协议无统一长度、版本、SEQ和CRC16，无法覆盖第 7 章完整比赛流程。新旧协议不得在同一场正式比赛会话中混发。

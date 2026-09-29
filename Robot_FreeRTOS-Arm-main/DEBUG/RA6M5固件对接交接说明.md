# RA6M5 固件对接交接说明

> 交付日期：2026-07-16  
> 交付方向：Jetson / Qt 视觉端 -> RA6M5 固件端  
> 当前结论：Jetson 已发送合法的连续测距启动请求；真实 RA6M5 固件尚未支持 `WORKFLOW_CTRL action=0x04`，因此连续安全距离功能目前没有进入实机工作状态。

## 1. 文档用途与版本基线

这份文件可直接交给 RA6M5 固件开发人员，目的是说明：

1. Jetson 当前会发送什么；
2. RA6M5 必须增加什么；
3. 双方如何回传状态、处理超时并完成真机验收；
4. 为什么 2026-07-16 的实机测试没有发送出 `SAFE_DISTANCE`。

当前正式依据只有：

- `视觉协议(5).md`：唯一正式串口字节协议；
- `项目演示(3).md`：当前正式比赛和演示流程；
- 本文件：针对 RA6M5 本次修改的交接与验收清单。

`视觉协议(3).md`、`视觉协议(4).md`、`项目演示(2).md` 都是旧版本，只能追溯，禁止作为当前实现依据。若本文件与 `视觉协议(5).md` 的字节定义冲突，以 `视觉协议(5).md` 为准。

当前仓库只有 Jetson/Qt、Python 协议实现和 RA6M5 无硬件模拟器，没有 RA6M5 的 C/C++、Keil 或 Renesas 固件工程。因此本文描述的是固件接口和验收要求，不表示真实 RA6M5 已经实现。

## 2. 本次必须解决的问题

Qt 以连续模式启动时会发送：

```text
TYPE=0x06 WORKFLOW_CTRL
LEN=0x01
PAYLOAD=0x04 START_MEASURE_CONTINUOUS
```

2026-07-16 实机连续两次收到 RA6M5 拒绝：

```text
TX WORKFLOW_START_MEASURE_CONTINUOUS seq=59
A5 5A 01 06 3B 01 04 68 D6

RX
A5 5A 01 81 3B 03 21 00 03 E0 5B
COMMAND_ACK_REJECTED
event=0x21 value=0x00 error=0x03 INVALID_PARAM
```

第二次测试同样失败：

```text
TX WORKFLOW_START_MEASURE_CONTINUOUS seq=161
A5 5A 01 06 A1 01 04 48 F9

RX COMMAND_ACK_REJECTED
seq=161 event=0x21 value=0x00 error=0x03 INVALID_PARAM
```

Qt 随后分别发送了 `ABORT_HOLD`，RA6M5 均返回 `COMMAND_ACK_ACCEPTED` 和 `WORKFLOW ABORTED_HOLD`。这证明：

- 串口 `/dev/ttyUSB0` 正常；
- `115200` 和统一 `A5 5A` 协议正常；
- CRC、`TYPE=0x06`、SEQ 和状态解析正常；
- 当前阻塞点是 RA6M5 的 action 白名单或状态机没有实现 `action=0x04`。

RA6M5 不能只把 `0x04` 加进白名单后直接当作 `0x01` 使用。只有下面这些功能全部实现后，才可以对 `0x04` 回传 `COMMAND_ACK_ACCEPTED`：

- 首次安全距离锁存；
- 锁存后的整轮连续距离消费；
- 独立的距离 freshness 看门狗；
- 距离过近时立即关闭 P801 并安全撤离；
- 距离更新超时时执行同样的安全撤离；
- 撤离完成后等待人工重新开始，禁止自动恢复。

## 3. 双方职责边界

| 设备 | 负责 | 不负责 |
|---|---|---|
| Qt | 接收“开始、采集、选择视图、开始点痣、完成/返回”等操作并显示状态 | 不直接控制电机和激光 GPIO |
| Jetson | 采集 RGB-D、识别 `arm_tip` 和 `face`、计算毫米净距、判断样本是否新鲜、组帧、解析状态 | 不发送任意关节角、笛卡尔坐标、运动速度或激光开启命令 |
| RA6M5 | 执行固化点位、复位、运动、距离阈值裁决、安全撤离、状态机以及 P801 输出 | 不得因为单帧视觉数据或单独按下 P000 直接开启 P801 |

RA6M5 是运动和激光安全的最终裁决者。Jetson 发送的是实际毫米净距，不发送“安全/不安全”的布尔裁决。最终安全阈值必须保存在 RA6M5 固件中，并根据机械制动距离、最坏姿态、误差上界和安全裕量进行真机标定。

## 4. Jetson 当前距离数据的含义

### 4.1 距离来源

`SAFE_DISTANCE.distance_mm` 是顶部 Astra RGB-D 计算出的 `arm_tip` 与 `face` 两个可信表面之间的最小三维净距，单位为毫米。

Jetson 的处理原则：

- 同时识别 `arm_tip` 和 `face`；
- 使用 RGB 与 Depth 的软件时间配对；
- 检查检测框、中心区域、背景环和 Depth 有效比例；
- 从两个对象的可信前景点计算三维表面最小净距；
- 距离突然变小时立即采用更小的原始值；
- 滤波只允许延迟“距离变大”，不能掩盖突然靠近；
- RA6M5 收到的是保守的最终 `reported_mm`，不是画面像素距离。

以下情况 Jetson 会保留界面上的上一次数值作为“已过期”显示，但完全不发送新的 `SAFE_DISTANCE`：

- YOLO 结果超过 `500 ms`；
- RGB/Depth 采样时差超限；
- Depth 有效比例不足；
- 对象无法从背景中可靠确认；
- 只有已经发送过的重复样本；
- RGB 或 Depth 没有新帧。

因此 RA6M5 必须把“收不到新距离”视为一个安全条件，而不能无限使用最后一次距离。

### 4.2 当前发送节奏

| 项目 | 当前值 |
|---|---:|
| Jetson HEARTBEAT 周期 | `150 ms`，程序限制在 `100~200 ms` |
| Jetson SAFE_DISTANCE 最快发送周期 | `150 ms`，程序限制在 `100~200 ms` |
| Jetson 距离最大年龄 | `500 ms` |
| RGB-D 最大允许软件时差 | `150 ms` |
| RA6M5 全局心跳超时建议初值 | `600 ms` |
| RA6M5 连续距离 freshness 超时建议初值 | `600 ms` |
| 控制命令 ACK 等待 | `300 ms` |
| 控制命令重发 | 最多重发 `3` 次，整帧完全相同 |

`HEARTBEAT` 看门狗和 `SAFE_DISTANCE` freshness 看门狗是两个独立的安全条件。心跳正常不能证明距离仍然有效；持续收到距离也不能代替心跳。

### 4.3 2026-07-16 视觉实测

运行约 51 秒，RGB-D 诊断结果如下：

| 指标 | 结果 |
|---|---:|
| 总诊断样本 | `125` |
| 有效 | `124` |
| 无效 | `1` |
| 有效率 | `99.2%` |
| `YOLO结果过期` | `0` 次 |
| 有效净距 | `316.0~332.2 mm`，平均 `326.9 mm` |
| 原始净距 | `316.0~334.8 mm`，平均 `327.7 mm` |
| 有效 RGB/Depth 时差 | `5.9~120.3 ms`，平均 `21.4 ms` |
| RGB 结果年龄 | `85.0~346.1 ms`，平均 `136.2 ms` |
| Depth FPS | `29.4~35.2`，平均 `30.2` |

唯一一次无效是 RGB/Depth 时差 `214.6 ms`，Jetson 已按设计静默处理。

这些 `316~332 mm` 只是当时场景的观测距离，不是 RA6M5 安全阈值。协议没有规定 `128 mm` 为固定阈值。Python 模拟器中的 `100/110 mm` 也只是测试占位值，不能直接写入正式固件。

本次 `TX SAFE_DISTANCE=0` 不是视觉没有数据，而是 RA6M5 拒绝 `0x04`，流程从未进入 `MEASURE_POSITION_READY`。Qt 按安全状态门禁止了距离发送。

## 5. UART 与统一帧格式

### 5.1 串口参数

| 项目 | 值 |
|---|---|
| 波特率 | `115200` |
| 数据位 | `8` |
| 校验位 | `None` |
| 停止位 | `1` |
| 流控 | `None` |
| 多字节整数 | 小端 |
| 最大 payload | `32 bytes` |

不发送 ASCII 十六进制字符串，串口上传输的是原始二进制字节。

### 5.2 帧布局

```text
SOF0 SOF1 VER TYPE SEQ LEN PAYLOAD[LEN] CRC16_LO CRC16_HI
A5   5A   01  xx   xx  nn  ...          xx       xx
```

总长度为 `8 + LEN` 字节。

| 字段 | 长度 | 说明 |
|---|---:|---|
| SOF0 | 1 | 固定 `0xA5` |
| SOF1 | 1 | 固定 `0x5A` |
| VER | 1 | 固定 `0x01` |
| TYPE | 1 | 消息类型 |
| SEQ | 1 | `0x00~0xFF` 循环 |
| LEN | 1 | payload 长度，最大 32 |
| PAYLOAD | LEN | 按 TYPE 解释 |
| CRC16 | 2 | CRC16/MODBUS，低字节先发送 |

### 5.3 CRC16/MODBUS

- 初值：`0xFFFF`
- 多项式：`0xA001`
- 计算范围：从 `VER` 到最后一个 payload 字节
- 不包含：`A5 5A` 和两个 CRC 字节
- 发送顺序：低字节、高字节

C 语言参考：

```c
uint16_t protocol_crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;

    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
```

接收端必须依次检查帧头、版本、`LEN<=32`、完整长度、CRC、TYPE、payload 长度和业务参数。错误帧头、不完整帧和 CRC 错误直接丢弃，不允许改变任何运动或输出状态。CRC 正确但 TYPE/参数非法时才回传协议错误。

## 6. 完整消息类型

| TYPE | 方向 | 名称 | LEN | PAYLOAD |
|---:|---|---|---:|---|
| `0x01` | Jetson -> RA6M5 | HEARTBEAT | 4 | `uint32 tick_ms` |
| `0x02` | Jetson -> RA6M5 | TARGET_CTRL | 1 | `uint8 enable` |
| `0x03` | Jetson -> RA6M5 | VISION_ERROR | 5 | `int16 dcx + int16 dcy + uint8 valid` |
| `0x04` | Jetson -> RA6M5 | CAPTURE_CTRL | 2 | `uint8 action + uint8 point_id` |
| `0x05` | Jetson -> RA6M5 | SAFE_DISTANCE | 3 | `uint16 distance_mm + uint8 valid` |
| `0x06` | Jetson -> RA6M5 | WORKFLOW_CTRL | 1 | `uint8 action` |
| `0x81` | RA6M5 -> Jetson | STATUS | 3 | `uint8 event + uint8 value + uint8 error_code` |
| `0xFE` | RA6M5 -> Jetson | ERROR | 1 | `uint8 error_code` |

特别注意：

- 连续模式是 `TYPE=0x06` 的 `action=0x04`，不是 `TYPE=0x04`；
- `TYPE=0x04` 是三视图的 `CAPTURE_CTRL`；
- 安全距离是 `TYPE=0x05`，帧中的 `LEN=0x03` 只是 payload 长度。

## 7. 命令、状态与错误码

### 7.1 WORKFLOW_CTRL action

| action | 名称 | RA6M5 行为 |
|---:|---|---|
| `0x01` | START_MEASURE | 普通一次测距锁存，不启用锁存后的连续 watchdog |
| `0x02` | FINISH_RETURN_HOME | 关闭定靶和 P801，soft_reset，完成后返回 IDLE |
| `0x03` | ABORT_HOLD | 立即关 P801、中止运动、原地保持，不自动 HOME |
| `0x04` | START_MEASURE_CONTINUOUS | 本次必须新增：首次锁存后整轮持续监测距离 |

`ABORT_HOLD` 具有最高安全优先级，不能因为 `BUSY` 而拒绝。

### 7.2 SAFE_DISTANCE payload

```text
distance_lo distance_hi valid
```

| 字段 | 含义 |
|---|---|
| `distance_mm` | `uint16` 小端，实际毫米净距 |
| `valid=0x01` | 本帧距离有效 |
| `valid=0x00` | 兼容无效帧，忽略 distance，不能刷新 freshness |

正式 Qt 在深度无效或过期时通常选择完全不发送，而不是周期发送 `valid=0`。RA6M5 即使收到 `valid=0`，也不得把它当成视觉仍然新鲜。

### 7.3 STATUS event

`STATUS TYPE=0x81` 的 payload 固定为：

```text
event value error_code
```

| event | 名称 | value |
|---:|---|---|
| `0x01` | TARGET_READY | `0x01` |
| `0x02` | ALIGN_DONE | `0x01` |
| `0x03` | OUTPUT | `0x01` 开，`0x00` 关 |
| `0x04` | TARGET_CTRL | `0x01` 开，`0x00` 关/拒绝 |
| `0x05` | HEARTBEAT_ALIVE | `0x01`，可选回传 |
| `0x06` | SAFE_DISTANCE | `0x01` 安全锁存，`0x00` 不安全/超时 |
| `0x07` | VISION_STATE | `0x01` 恢复，`0x00` 丢失 |
| `0x10` | CAPTURE_POINT_READY | `1/2/3` 左/正/右到位 |
| `0x11` | CAPTURE_HOME_READY | `0x01` |
| `0x12` | SELECTED_VIEW_READY | `1/2/3` |
| `0x20` | WORKFLOW | 见下表 |
| `0x21` | COMMAND_ACK | `0x01` 接受，`0x00` 拒绝 |

WORKFLOW `event=0x20` 的 value：

| value | 名称 | 含义 |
|---:|---|---|
| `0x00` | IDLE | 空闲，P801 关闭 |
| `0x01` | START_ACCEPTED | 已接受开始并进入复位/移动阶段 |
| `0x02` | MEASURE_POSITION_READY | 公共测距位真实到位，可以接收距离 |
| `0x03` | DISTANCE_SAFE_LATCHED | 本轮安全距离已经锁存 |
| `0x04` | RETREAT_DONE_WAIT_RESTART | 撤离完成，等待重新开始 |
| `0x05` | RETURN_HOME_DONE | 返回 HOME 并 final verify PASS |
| `0x06` | ABORTED_HOLD | 已停止并原地保持 |
| `0x07` | FAULT_HOLD | 故障锁存，等待人工处理 |

### 7.4 error_code

| error | 名称 | 含义 |
|---:|---|---|
| `0x00` | NONE | 无错误 |
| `0x01` | VERSION_MISMATCH | 协议版本不支持 |
| `0x02` | UNKNOWN_TYPE | 未知消息 TYPE |
| `0x03` | INVALID_PARAM | 长度、action、point_id 或参数非法 |
| `0x04` | POSE_INVALID | 位姿无效 |
| `0x05` | BUSY | 正在执行其他动作 |
| `0x06` | MOTION_FAILED | IK、运动或到位确认失败 |
| `0x07` | HEARTBEAT_TIMEOUT | 全局心跳超时 |
| `0x08` | SAFETY_INPUT | 限位、急停或硬件安全输入 |
| `0x09` | SAFE_DISTANCE_TOO_CLOSE | 新距离低于固件标定阈值 |
| `0x0A` | VISION_LOST | 视觉无效或超过 500 ms 未更新 |
| `0x0B` | SOFT_RESET_FAILED | soft_reset final verify FAIL |
| `0x0C` | INVALID_STATE | 当前状态不允许此命令 |
| `0x0D` | SEQ_CONFLICT | 相同 TYPE+SEQ 携带不同 payload |
| `0x0E` | TARGET_GATE_DENIED | 定靶安全门不完整 |
| `0x0F` | MOTION_ABORTED | 当前运动已被安全中止 |
| `0x10` | SAFE_DISTANCE_TIMEOUT | 连续距离 freshness 超时 |

连续距离超时必须回：

```text
TYPE=0x81 STATUS
event=0x06 SAFE_DISTANCE
value=0x00
error_code=0x10 SAFE_DISTANCE_TIMEOUT
```

不要只发送 `TYPE=0xFE, error=0x10`。Jetson 当前协议按照正式规范在 `STATUS.error_code` 中处理连续距离超时。

## 8. SEQ、ACK、重发与幂等

### 8.1 Jetson 发送规则

- 每个新帧使用新 SEQ，`0xFF` 后回绕到 `0x00`；
- HEARTBEAT、VISION_ERROR、SAFE_DISTANCE 的每个新样本均使用新 SEQ；
- 控制命令只有 WORKFLOW_CTRL、CAPTURE_CTRL、TARGET_CTRL；
- 控制命令 `300 ms` 未收到 ACK 时，会用完全相同的 `TYPE+SEQ+payload+CRC` 重发；
- 最多重发 3 次，重发时绝不更改 SEQ 或 payload。

### 8.2 RA6M5 必须实现

- 接受控制命令：先用同一 SEQ 回 `COMMAND_ACK value=1,error=0`；
- 拒绝控制命令：同一 SEQ 回 `COMMAND_ACK value=0,error=原因`，且不得执行动作；
- 接受后的业务完成状态仍使用原控制请求 SEQ；
- 收到完全相同的重复控制帧，只重放已经缓存的响应，不得再次启动复位或运动；
- 相同 `TYPE+SEQ` 但 payload 不同，回 `SEQ_CONFLICT=0x0D` 并保持原状态；
- 同一时刻只允许一个机械臂控制动作，安全优先的 ABORT_HOLD 除外。

建议缓存最近 64 个控制请求。SAFE_DISTANCE 建议缓存当前会话最近至少 32 个 `SEQ+payload`：

- 完全相同的 SAFE_DISTANCE 重放：静默忽略，绝不能刷新 freshness；
- 相同 SAFE SEQ 但 payload 不同：回 `ERROR SEQ_CONFLICT`，绝不能刷新 freshness；
- 缓存必须允许 8 位 SEQ 正常回绕，不能永久禁止某个 SEQ。

## 9. 连续安全距离状态机

### 9.1 状态流程

```text
IDLE / WAIT_RESTART
    |
    | WORKFLOW_CTRL action=0x04
    v
START_ACCEPTED
    |
    | soft_reset PASS + 公共测距位真实到位
    v
MEASURE_POSITION_READY
    |
    | 连续 3 个全新、有效、达到阈值的距离
    v
DISTANCE_SAFE_LATCHED / ACTIVE_CONTINUOUS
    |                         |
    | 新距离低于阈值          | 600 ms 没有全新有效距离
    v                         v
SAFE_DISTANCE_TOO_CLOSE    SAFE_DISTANCE_TIMEOUT
    \                         /
     \                       /
      v                     v
       P801 OFF + 中止当前动作 + 固化安全撤离
                         |
                         | 撤离真正完成
                         v
              RETREAT_DONE_WAIT_RESTART
                         |
                         | 只能人工重新点击“开始”
                         v
                  重新 soft_reset 和测距
```

### 9.2 收到 START_MEASURE_CONTINUOUS

收到合法 `TYPE=0x06,LEN=1,payload=0x04` 后：

1. 检查当前状态、急停、限位和故障锁存；
2. 立即确保 P801 为低电平；
3. 清除旧的距离锁存、视觉许可、选中视图、P000 许可和旧 watchdog 时间；
4. 用 START 请求的 SEQ 回 `COMMAND_ACK_ACCEPTED`；
5. 用相同 SEQ 回 `WORKFLOW START_ACCEPTED`；
6. 执行 soft_reset；
7. 只有 soft_reset final verify PASS 后才移动到公共测距位；
8. 只有公共测距位真实到位后才回 `WORKFLOW MEASURE_POSITION_READY`；
9. 到位失败、IK 失败、限位或复位失败时不得伪报 READY，应关 P801 并进入 FAULT_HOLD。

Qt 只有收到 `MEASURE_POSITION_READY` 后才开始发送有效距离。

### 9.3 首次安全锁存

当前联调基线使用连续 3 个全新安全样本：

```text
valid == 1
distance_mm >= D_SAFE_MM
样本没有在本会话重复
相邻计入样本的间隔 < 600 ms
```

- 前两个安全样本只累计 streak，不回锁存状态；
- 第三个样本使 `distance_safe_latched=true`；
- 用第三个 SAFE 样本的 SEQ 回 `STATUS SAFE_DISTANCE value=1,error=0`；
- 用本轮 START 的 SEQ 回 `WORKFLOW DISTANCE_SAFE_LATCHED`；
- 收到 `valid=0` 时清除锁存前的 streak；
- 两个有效安全样本间隔达到 600 ms 时，旧 streak 作废，从当前样本重新计数；
- 任一全新有效样本低于阈值，立即进入安全撤离，不等待三帧。

距离 freshness watchdog 从完成首次安全锁存的那个有效样本开始。首次锁存前 P801 和自动运动许可均关闭，RA6M5 在公共测距位等待；锁存前长时间没有新样本只重置 streak，不使用旧会话 watchdog 触发 HOME 阶段误撤离。

### 9.4 锁存后的连续监测

锁存后，RA6M5 必须在本轮整个活动工作流中继续接收全新距离，包括：

- capture HOME；
- 左/正/右三视图移动和保持；
- SELECT_VIEW；
- TARGET 对准；
- P801 输出监测阶段。

每个合法、全新、`valid=1` 且不低于阈值的距离更新 `last_fresh_safe_ms`。为了避免串口洪泛，正常安全刷新不需要每帧重复回 `SAFE_DISTANCE_OK`。

只有以下帧可以刷新距离 freshness：

```text
CRC、版本、长度和参数全部合法
&& 当前为已锁存的连续会话
&& SAFE SEQ/payload 是全新的
&& valid == 1
&& distance_mm >= D_SAFE_MM
```

以下内容绝对不能刷新距离 freshness：

- HEARTBEAT；
- `valid=0`；
- 完全重复的 SAFE 帧；
- 相同 SEQ、不同 payload 的冲突帧；
- CRC/长度/版本错误帧；
- 其他控制命令；
- Jetson 因 YOLO/Depth 过期而保持的静默。

watchdog 必须由独立的周期安全任务使用单调毫秒时钟检查，即使串口没有任何新字节也必须按时触发。不能只在 UART RX 回调里检查。建议检查周期 `10~20 ms`，毫秒差值使用无符号回绕安全写法。

### 9.5 距离过近

首次锁存前或锁存后，只要收到一个全新、有效且满足以下条件的样本：

```text
distance_mm < D_SAFE_MM
```

RA6M5 必须立即 fail-closed：

1. 第一优先级拉低 P801；
2. 撤销当前发射、定靶和自动运动许可；
3. 安全中止当前运动；
4. 清除距离锁存、选中视图、P000 许可等会话状态；
5. 使用已经真机验证的固化 +Y 撤离轨迹；
6. 用触发样本的 SEQ 回 `STATUS SAFE_DISTANCE value=0,error=0x09`；
7. 只有撤离真正成功到位后，才用本轮 START SEQ 回 `WORKFLOW RETREAT_DONE_WAIT_RESTART`；
8. 等待人工重新点击“开始”，不得自动恢复原动作。

如果撤离过程中发生急停、限位、IK/运动失败，不得回 `RETREAT_DONE`。必须保持 P801 关闭，回对应错误并进入 `FAULT_HOLD`。

### 9.6 距离更新超时

锁存后满足：

```text
now_ms - last_fresh_safe_ms >= 600 ms
```

必须执行与距离过近相同的 fail-closed 撤离：

1. 立即关 P801；
2. 清许可并安全中止当前动作；
3. 执行同一条已验证撤离轨迹；
4. 用最后一个新鲜 SAFE 样本的 SEQ 回 `STATUS SAFE_DISTANCE value=0,error=0x10`；
5. 撤离真正完成后，用本轮 START SEQ 回 `WORKFLOW RETREAT_DONE_WAIT_RESTART`；
6. 禁止自动恢复。

即使 HEARTBEAT 一直正常，距离超时仍必须触发。

### 9.7 退出和清理

以下事件必须取消连续 watchdog 并清除距离锁存，旧计时器之后不得再产生 timeout：

- 新的 START；
- FINISH_RETURN_HOME；
- ABORT_HOLD；
- 全局 HEARTBEAT_TIMEOUT；
- 急停或限位；
- soft_reset、IK 或运动失败；
- 掉电或状态机重新初始化。

## 10. 全局心跳与激光安全

Jetson 每 `100~200 ms` 发送一个合法 HEARTBEAT。只有 CRC、版本和长度全部正确的 HEARTBEAT 才能刷新全局心跳时间。

超过 `600 ms` 没有合法心跳时，RA6M5 必须：

1. 立即拉低 P801；
2. 中止当前运动并禁止新的自动运动；
3. 清除视觉有效、距离锁存、选中视图和发射许可；
4. 取消距离 watchdog；
5. 进入 `FAULT_HOLD` 并保持当前位置，不自动 HOME；
6. 尝试回传 `ERROR HEARTBEAT_TIMEOUT`；
7. 通信恢复后仍等待人工重新开始，不能自动续跑。

P801 只允许在 TARGET_OUTPUT 状态打开，至少同时满足：

```text
workflow_active
&& target_enabled
&& heartbeat_alive
&& distance_safe_latched
&& selected_view_ready
&& pose_valid
&& vision_fresh
&& vision_in_tolerance
&& alignment_confirmed
&& P000_pressed
&& !limit_triggered
&& !estop_active
&& !fault_latched
```

距离过近、距离超时、心跳超时、视觉丢失、误差超限、P000 松开、急停或限位时，P801 必须先关闭，再处理保持或撤离动作。

## 11. 普通模式兼容要求

`START_MEASURE action=0x01` 必须保留旧的一次性语义：

- 到公共测距位后接收距离；
- 安全锁存后关闭距离接收窗口；
- 不启用锁存后的连续距离 watchdog；
- 锁存后再收到 SAFE_DISTANCE，应回 `INVALID_STATE`；
- 只有显式 `action=0x04` 才进入连续模式。

Jetson 在 `0x04` 被拒绝时不会偷偷降级为 `0x01`，而是停止启动并发送 ABORT_HOLD。这是有意的安全行为。

## 12. 其余比赛流程保持不变

连续距离锁存成功后，现有流程仍按以下命令执行：

| Qt/Jetson 动作 | Jetson 下发 | RA6M5 成功回传 |
|---|---|---|
| 点击“开始” | WORKFLOW 0x04 | ACK -> START_ACCEPTED -> MEASURE_POSITION_READY |
| 三个安全距离锁存 | SAFE_DISTANCE | SAFE_DISTANCE SAFE -> DISTANCE_SAFE_LATCHED |
| 采集开始 | CAPTURE HOME, point=0 | ACK -> CAPTURE_HOME_READY |
| 左/正/右采集 | CAPTURE GOTO, point=1/2/3 | ACK -> CAPTURE_POINT_READY |
| 三视图结束 | CAPTURE HOME, point=0 | ACK -> CAPTURE_HOME_READY |
| 医生选择视图 | CAPTURE SELECT, point=1/2/3 | ACK -> SELECTED_VIEW_READY |
| 开始点痣 | TARGET_CTRL enable=1 | ACK -> TARGET_CTRL_ON -> TARGET_READY |
| 视觉对准 | 周期 VISION_ERROR | ALIGN_DONE |
| P000 按住且全部门满足 | 无激光开命令 | OUTPUT_ON |
| P000 松开 | 无需 Qt 命令 | OUTPUT_OFF |
| 完成/返回 | WORKFLOW FINISH | ACK -> RETURN_HOME_DONE |
| 取消/安全停止 | WORKFLOW ABORT_HOLD | ACK -> ABORTED_HOLD |

所有 CAPTURE/TARGET/WORKFLOW 控制都必须先 ACK，机械臂真实到位或动作真正完成后才能发送业务完成状态。Jetson 不使用固定延时猜测到位。

## 13. 连续模式 HEX 联调向量

下面的 CRC 已由当前 Jetson 构帧函数计算，可直接作为串口测试向量。SEQ 只用于测试，正式运行会递增。

| 场景 | HEX |
|---|---|
| START_CONTINUOUS，seq=10 | `A5 5A 01 06 10 01 04 18 DE` |
| ACK_ACCEPT，seq=10 | `A5 5A 01 81 10 03 21 01 00 85 CC` |
| ACK_REJECT INVALID_PARAM，seq=10 | `A5 5A 01 81 10 03 21 00 03 C4 5D` |
| START_ACCEPTED，seq=10 | `A5 5A 01 81 10 03 20 01 00 D4 0C` |
| MEASURE_POSITION_READY，seq=10 | `A5 5A 01 81 10 03 20 02 00 D4 FC` |
| SAFE 120 mm，seq=11 | `A5 5A 01 05 11 03 78 00 01 B6 0B` |
| SAFE 120 mm，seq=12 | `A5 5A 01 05 12 03 78 00 01 F2 0B` |
| SAFE 120 mm，seq=13 | `A5 5A 01 05 13 03 78 00 01 CF CB` |
| SAFE_OK，第三帧 seq=13 | `A5 5A 01 81 13 03 06 01 00 71 C7` |
| DISTANCE_SAFE_LATCHED，START seq=10 | `A5 5A 01 81 10 03 20 03 00 D5 6C` |
| SAFE 90 mm，seq=14 | `A5 5A 01 05 14 03 5A 00 01 DA 01` |
| TOO_CLOSE，seq=14 | `A5 5A 01 81 14 03 06 00 09 05 91` |
| SAFE_TIMEOUT，独立超时测试中最后 SAFE seq=13 | `A5 5A 01 81 13 03 06 00 10 71 9B` |
| RETREAT_DONE，START seq=10 | `A5 5A 01 81 10 03 20 04 00 D7 5C` |
| ABORT_HOLD，seq=20 | `A5 5A 01 06 20 01 03 59 13` |
| ABORT ACK，seq=20 | `A5 5A 01 81 20 03 21 01 00 C5 C8` |
| ABORTED_HOLD，seq=20 | `A5 5A 01 81 20 03 20 06 00 96 38` |

注意：过近分支和超时分支是两个独立测试，不能在同一个会话里先发送 `90 mm` 触发撤离后再等待 timeout。如果 `D_SAFE_MM > 120`，上表三个 `120 mm` 测试样本将被正确判定为过近。做三帧锁存测试时，应发送 `D_SAFE_MM + 20 mm`，不能为了匹配示例而修改正式阈值。

## 14. RA6M5 必须通过的验收项目

### 14.1 协议和状态机

- [ ] `action=0x04` 不再返回 INVALID_PARAM；
- [ ] 回传顺序严格为 ACK -> START_ACCEPTED -> 真实到位后的 MEASURE_POSITION_READY；
- [ ] 上述三帧使用同一个 START SEQ；
- [ ] 完全相同的 0x04 重发不会重复 soft_reset 或重复运动；
- [ ] 相同 TYPE+SEQ、不同 payload 返回 SEQ_CONFLICT；
- [ ] 初始三帧安全样本：前两帧不锁存，第三帧返回 SAFE_OK 和 LATCHED；
- [ ] 相邻初始样本间隔达到 600 ms 时 streak 重置；
- [ ] `0x01` 普通模式仍保持一次锁存，不启用连续 watchdog；
- [ ] ABORT_HOLD 在 BUSY 时仍可立即执行；
- [ ] ABORT 后等待 10 秒不会再出现旧会话的 SAFE_DISTANCE_TIMEOUT。

### 14.2 距离 freshness

- [ ] 锁存后每 `100~200 ms` 发送安全新样本，持续运行不误撤离；
- [ ] 只发 HEARTBEAT 不能刷新距离 watchdog；
- [ ] `valid=0` 不能刷新距离 watchdog；
- [ ] 重放旧 SAFE 帧不能刷新距离 watchdog；
- [ ] 相同 SEQ、不同 payload 不能刷新距离 watchdog；
- [ ] 没有 UART 新字节时，独立周期任务仍会触发 timeout；
- [ ] `<600 ms` 不提前触发，达到 `600 ms` 加一个任务周期内必须触发；
- [ ] timeout 回 `STATUS SAFE_DISTANCE value=0,error=0x10`；
- [ ] timeout 后撤离完成才回 RETREAT_DONE_WAIT_RESTART。

### 14.3 真机安全

- [ ] 在 CAPTURE HOME、三个视图、SELECT、TARGET_ALIGN、P801 ON 各阶段验证连续距离；
- [ ] 任一阶段发送 `D_SAFE_MM-1 mm`，单帧立即关 P801、停止当前动作并撤离；
- [ ] 不能为了让演示通过而关闭 watchdog；
- [ ] 验证固化“+Y”在真实坐标系中的物理方向确实远离人脸；
- [ ] 标定撤离目标、速度、加速度、到位容差和运动超时；
- [ ] 验证最坏通信、调度、识别延迟和机械制动距离；
- [ ] 撤离遇到限位、急停或运动失败时不得回 RETREAT_DONE；
- [ ] 以上故障必须保持 P801 低并进入 FAULT_HOLD；
- [ ] 撤离完成后不自动续跑，必须人工重新开始；
- [ ] P801 在任何撤离运动开始前已经拉低。

## 15. RA6M5 侧需要确认并回填

请固件开发人员完成后把以下项目回填给 Jetson 端：

| 项目 | RA6M5 回填 |
|---|---|
| 固件版本/提交号 | 待填写 |
| `action=0x04` 已支持 | 是 / 否 |
| `D_SAFE_MM` 正式标定值 | 待填写，不得照抄 100/110 mm 测试值 |
| 初始锁存连续样本数 | 建议 3，待确认 |
| 距离 freshness 阈值 | 初始 600 ms，待最坏工况确认 |
| watchdog 检查周期 | 待填写 |
| 固化撤离目标和坐标系 | 待填写 |
| 撤离速度/加速度 | 待填写 |
| 撤离到位容差/超时 | 待填写 |
| P801 关闭最坏延迟 | 待实测 |
| 从过近检测到运动停止的最坏延迟 | 待实测 |
| 全阶段连续深度验收 | PASS / FAIL |
| 急停/限位/撤离失败验收 | PASS / FAIL |

如果最终需要修改 `600 ms`、锁存帧数或状态码，必须由 Jetson 与 RA6M5 双方同时更新协议、实现和测试，不能由单方静默修改。

## 16. Jetson 联调入口

当前实机启动关键参数：

```bash
cd /home/jetson/jixiebi/arm_mole_qt

DISPLAY=:0 bash run_qt_app.sh \
  --camera-layout astra-rgbd \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 60 \
  --ui-fps 15 --flip none --rotate none \
  --astra-always-on 1 \
  --astra-rgb-enabled 1 \
  --astra-rgb-camera /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
  --astra-rgb-width 640 --astra-rgb-height 480 --astra-rgb-fps 30 \
  --astra-rgb-flip none \
  --astra-yolo-device 0 --astra-yolo-imgsz 416 --astra-yolo-interval 0.10 \
  --astra-depth-enabled 1 \
  --rgbd-distance-log --rgbd-distance-log-interval 0.25 \
  --rgbd-hardware-pair-verified \
  --continuous-safe-distance \
  --serial-port /dev/ttyUSB0 --baudrate 115200 --serial-protocol unified
```

无硬件协议模拟器自检：

```bash
cd /home/jetson/jixiebi
python3 ra6m5_sim_check.py
```

模拟串口服务：

```bash
cd /home/jetson/jixiebi
python3 ra6m5_sim_check.py --serve --verbose
```

Python 模拟器通过只表示 Jetson 端期待的协议状态机已经通过测试，不等于 RA6M5 真机安全功能已经验收。

## 17. 仓库内参考文件

- `视觉协议(5).md`：完整正式协议；
- `项目演示(3).md`：完整正式流程；
- `docs/WORKFLOW_PLAN.md`：Qt 与 RA6M5 的状态门；
- `docs/CAMERA_CONFIG.md`：可信 RGB-D 净距和无效静默规则；
- `arm_mole_qt/README.md`：Qt 启动和联调说明；
- `jetson_nano_vision/jetson_vision/vision_protocol.py`：CRC、常量、打包和解析参考；
- `jetson_nano_vision/jetson_vision/serial_sender.py`：SEQ、ACK、重发和发送状态门；
- `ra6m5_sim_check.py`：无硬件参考状态机；
- `tests/test_ra6m5_sim_check.py`：RA6M5 模拟状态机测试；
- `tests/test_continuous_safety_protocol.py`：连续安全协议测试；
- `tests/test_qt_safe_distance.py`：Qt 距离新鲜度和静默发送测试。

## 18. 不属于本次 RA6M5 阻塞的问题

日志中的：

```text
OpenNI2 camera don't support Watchdog function!
```

是 Astra 相机 SDK 自身的设备 watchdog 能力提示，不是 RA6M5 的连续距离 freshness watchdog。

关闭 Qt 时出现的 `libusb cancel/timeout` 是 Depth Pipeline 停止阶段的日志，也不是当前串口或 RA6M5 状态机故障。

## 19. RA6M5 端对照核查补充（2026-07-16）

> 核查基线：RA6M5 工程根目录的 `视觉协议.md`、`Middle/jetson_vision.h/.c`、`APP/robot_workflow.h/.c`。  
> 本节用于记录双方文档和当前固件之间的差异。若本节与前述第 1~18 章冲突，当前联调以本节和 RA6M5 工程根目录的 `视觉协议.md` 为准。

### 19.1 核查结论

本次失败不是 RA6M5 丢帧，也不是 CRC、波特率或状态回传异常。Jetson 发送的帧已经被 RA6M5 完整解析，但其中：

```text
TYPE   = 0x06 WORKFLOW_CTRL
ACTION = 0x04
```

不属于当前正式协议。当前 RA6M5 固件只接受：

| action | 名称 | 当前含义 |
|---:|---|---|
| `0x01` | `START_MEASURE` | Qt“开始”：soft_reset 后前往公共测距位 |
| `0x02` | `FINISH_RETURN_HOME` | 完成并返回 HOME |
| `0x03` | `ABORT_HOLD` | 关闭输出、中止运动并原地保持 |

因此 RA6M5 对 `action=0x04` 回传：

```text
COMMAND_ACK value=0x00 error=0x03 INVALID_PARAM
```

属于符合当前正式协议的行为，不能把它判定为“固件未实现已有合法命令”。`START_MEASURE_CONTINUOUS=0x04` 是 Jetson 端提出的新增协议扩展，目前没有写入 RA6M5 正式协议，也没有经过双方确认和固件实现。

特别注意以下三个 `0x04` 不是同一个字段：

| 所在字段 | 当前含义 |
|---|---|
| `CAPTURE_CTRL action=0x04` | 旧调试保留，比赛流程禁止使用 |
| `WORKFLOW STATUS value=0x04` | `RETREAT_DONE_WAIT_RESTART` |
| `WORKFLOW_CTRL action=0x04` | 当前未定义，会返回 `INVALID_PARAM` |

### 19.2 交接说明与当前正式实现的差异

| 项目 | 交接说明第 1~18 章描述 | 当前正式协议/固件 | 核查处理 |
|---|---|---|---|
| 正式依据 | 以 Jetson 仓库的 `视觉协议(5).md` 为准 | 本工程已有 RA6M5 C/Keil 工程，正式依据为工程根目录 `视觉协议.md` | 联调前必须先统一版本，不能单方新增动作码 |
| Qt“开始” | `WORKFLOW action=0x04` | `WORKFLOW action=0x01` | Jetson 当前必须改发 `0x01` |
| 连续距离监测 | 锁存后贯穿 HOME、三视图、定靶和输出 | 当前仅在公共测距位完成一次三帧安全锁存；锁存后不要求继续发送距离 | 连续监测属于未来扩展，不是当前阻塞缺陷 |
| 距离 freshness watchdog | 锁存后 `600 ms` 无新距离即撤离 | 当前没有锁存后的距离 freshness watchdog | 不得把 `0x10` 超时行为当作已实现能力 |
| `error=0x10` | `SAFE_DISTANCE_TIMEOUT` | 当前错误码只定义到 `0x0F` | 如未来采用，必须同步修改协议、Jetson、RA6M5 和测试向量 |
| 首次锁存样本间隔 | 相邻样本必须小于 `600 ms` | 当前要求连续 3 个新 `SEQ` 的有效安全样本，但未实现该 `600 ms` 间隔判定 | 该时间条件属于新增需求 |
| 锁存后收到 SAFE_DISTANCE | 持续消费并刷新 watchdog | 当前在 `FLOW_SAFE_READY` 及后续流程中忽略该数据 | Jetson 当前收到锁存状态后应停止发送 SAFE_DISTANCE |
| Jetson SEQ | 写成 `0x00~0xFF` 循环 | Jetson 只能使用 `0x01~0xFF`；`0x00` 保留给 RA6M5 异步故障 | Jetson 构帧器必须跳过 `0x00` |
| 异步距离超时 SEQ | 使用最后一帧 SAFE_DISTANCE 的 SEQ | 当前规则是无法关联请求的异步故障使用 `SEQ=0x00` | 若新增超时，应由双方先明确回传 TYPE、event 和 SEQ |
| 控制命令缓存 | 建议缓存 64 条 | 当前 RA6M5 缓存最近 8 条控制命令 | 64 条是优化建议，不是当前协议契约 |
| SAFE_DISTANCE 去重 | 建议缓存至少 32 条 | 当前 RA6M5 只抑制与最近一帧相同的 SAFE SEQ | 扩展连续模式前需要重新设计去重窗口和 SEQ 回绕 |
| 撤离步长 | 文中未锁定，要求固化 +Y 撤离 | `视觉协议.md` 写 `10 mm/步、最多 5 步`，当前代码实际为 `15 mm/步、最多 5 步` | 这是当前真实不一致项，上板前必须统一；现固件实际最大退让为 `75 mm` |
| 激光引脚 | P801 | 当前固件 `LASER_PIN=BSP_IO_PORT_08_PIN_01` | 一致 |

### 19.3 当前可以直接联调的启动流程

Jetson 当前应关闭 `--continuous-safe-distance` 模式，并按正式一次测距流程执行：

1. 串口打开后，先每 `100~200 ms` 连续发送合法 `HEARTBEAT`。
2. 用户点击 Qt“开始”时发送 `WORKFLOW_CTRL action=0x01`，使用新的非零 `SEQ`。
3. 等待同一 `SEQ` 的 `COMMAND_ACK value=1,error=0`。
4. 等待同一 `SEQ` 的 `WORKFLOW START_ACCEPTED`。
5. 等待同一 `SEQ` 的 `WORKFLOW MEASURE_POSITION_READY`，不得用固定延时猜测到位。
6. 发送 3 个不同非零 `SEQ` 的有效安全距离样本。
7. 等待 `SAFE_DISTANCE value=1` 和 `WORKFLOW DISTANCE_SAFE_LATCHED`。
8. 停止发送 SAFE_DISTANCE，继续三视图、选择视图和定靶流程；HEARTBEAT 全程不得停止。

例如 `SEQ=0x10` 的当前正式启动帧为：

```text
A5 5A 01 06 10 01 01 D8 DD
```

字段解释：

```text
A5 5A       帧头
01          VER
06          WORKFLOW_CTRL
10          SEQ
01          LEN
01          START_MEASURE
D8 DD       CRC16/MODBUS，小端
```

Jetson 当前不能发送：

```text
A5 5A 01 06 <SEQ> 01 04 <CRC_LO> <CRC_HI>
```

也不能在 `0x04` 被拒绝后自动换一个新 SEQ 重发相同启动意图。收到拒绝后应停止自动推进、显示 `INVALID_PARAM`；发送 `ABORT_HOLD` 只负责安全保持，不会启动比赛流程。

### 19.4 串口发送还需修复的独立问题

RA6M5 联调日志还出现过：

```text
crc failed type=0x01 rx=0x5AA5
```

`0x5AA5` 对应下一帧的帧头字节 `A5 5A` 被上一帧当成 CRC，说明 Jetson 发送流中存在帧截断、帧交叠或长度与实际写入不一致。这个问题和 `action=0x04` 被拒绝是两个独立问题：前者是发送并发/完整性问题，后者是协议动作码错误。

Jetson 端必须满足：

- HEARTBEAT、WORKFLOW、CAPTURE、TARGET、VISION 和 SAFE_DISTANCE 共用同一个串口发送队列或全局 TX 互斥锁；
- 一帧必须组装成完整 `bytes` 后由单一发送入口一次写出；
- 禁止多个线程分别直接调用串口 `write()`；
- PC 模拟端和真实 Jetson 不得同时驱动 RA6M5 的同一 Jetson UART；
- 关闭或重新打开串口前，停止所有发送线程并清空待发送队列。

### 19.5 连续安全距离方案的定位

交接说明提出的“锁存后持续距离监测、距离过近/超时立即关闭 P801 并撤离”具有安全价值，但它会改变当前已经确认的比赛控制契约，不能通过仅给 action 白名单增加 `0x04` 完成。若后续决定采用，至少需要共同确认并一次性更新：

1. `WORKFLOW_CTRL action=0x04` 的正式定义和兼容策略；
2. 距离 freshness 起止状态、超时值和检查周期；
3. 异步超时的 TYPE、event、error 和 `SEQ=0x00` 规则；
4. SAFE_DISTANCE 去重窗口、SEQ 回绕和冲突处理；
5. 距离过近时是否中止正在执行的 HOME/CAPTURE/TARGET 动作；
6. 撤离步长统一为 `10 mm` 还是当前代码的 `15 mm`；
7. 连续深度失效时的真机撤离验收和最坏响应时间。

在以上内容完成双方评审、代码实现和上板故障注入前，`START_MEASURE_CONTINUOUS=0x04` 只能标记为“协议变更提案”，不能作为当前比赛正式命令，也不能要求 RA6M5 对它回传 ACK 接受。

### 19.6 双方当前行动项

**Jetson / Qt 端：**

- 将 Qt“开始”恢复为 `WORKFLOW_CTRL action=0x01`；
- 当前启动参数移除 `--continuous-safe-distance`；
- SEQ 只使用 `0x01~0xFF`；
- 串口发送统一经过单写线程或 TX 锁；
- 收到 `COMMAND_ACK value=0` 时停止自动流程并显示具体错误。

**RA6M5 端：**

- 暂不把 `action=0x04` 加入白名单；
- 按当前 `0x01` 流程完成首次真机联调；
- 上板确认三帧距离锁存、三视图、定靶、P000/P801 和返回 HOME；
- 尽快统一撤离步长：文档 `10 mm` 与代码 `15 mm` 只能保留一个值；
- 若双方正式决定采用连续监测，再单独制定协议变更和固件实施计划。

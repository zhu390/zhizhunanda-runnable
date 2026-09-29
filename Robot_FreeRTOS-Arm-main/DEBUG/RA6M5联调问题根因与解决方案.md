# RA6M5 联调问题根因与解决方案

> 日期：2026-07-17  
> 分析范围：`Robot_FreeRTOS - Arm` 当前代码、正式 `视觉协议.md`、Jetson 队友提供的 `RA6M5当前问题与联调处理清单.md`，以及 2026-07-17 上位机实机日志。  
> 本文只给出修改和联调方案，本次没有修改固件代码。  
> 所有视觉闭环和状态机测试必须先物理断开激光，确认 P801 始终为低电平。

## 1. 结论摘要

当前状态机主体已经能够运行，不应推倒重写。2026-07-17 的 RA6M5 日志已经出现：

```text
[JETSON_RX] workflow_ctrl ...
[WORKFLOW] IDLE -> HOMING
[WORKFLOW] MEASURE_POSITION -> WAIT_SAFE_DISTANCE
[WORKFLOW] WAIT_SAFE_DISTANCE -> SAFE_READY
[JETSON_RX] capture_ctrl ...
[WORKFLOW] CAPTURE_HOME -> SELECTED_VIEW
[JETSON_RX] target_ctrl=1
[WORKFLOW] SELECTED_VIEW -> TARGET_ACTIVE
```

这证明该轮测试中 Jetson -> RA6M5 的 UART、统一帧解析和顶层工作流并非完全失效。当前无法完成点痣流程，主要由以下问题叠加造成：

| 优先级 | 问题 | 判断 |
|---|---|---|
| P0 | Jetson UART 接收流约每 `6.7 s` 固定出现一次 CRC 错误 | 已确认错误周期受 DTC 缓冲区长度影响，根因是普通模式接收完成后的重新挂载空窗；最终方案应改为无空窗环形缓冲区 |
| P0 | 视觉闭环没有收敛，`dcy` 从约 `49` 增大到 `72` | 当前 `dcy` 实际映射到机械臂 `Z`，映射符号或增益需单轴标定，不能称为机械臂 Y 轴问题 |
| P0 | Jetson 大部分时间发送 `valid=0` | RA6M5 会按安全设计立即退回 `WAIT_DETECT`，因此不可能产生 `ALIGN_DONE`、P003 或 P801 许可 |
| P0 | 最新一次 Jetson 端称“只有 TX、没有 RA6M5 RX” | 目前缺少同一时刻的双端原始日志；需要分开验证 RA6M5 P112 TX 和 Jetson RX，不能用 COM16 日志代替 |
| P1 | 安全距离阈值和撤离参数没有按最终意图统一 | 当前阈值 `100 mm`、撤离 `15 mm/步 x 5 = 75 mm`；最终约定应为阈值 `150 mm`、撤离 `20 mm/步 x 5 = 100 mm` |
| P1 | 对准共需连续 7 帧，比赛现场恢复速度偏慢 | 7 帧不是本轮无法发射的主因，但可在方向、目标锁定和 `valid` 稳定后调整为 `3+2` 共 5 帧 |
| P1 | SAFE_DISTANCE 三帧去重不完整 | 当前只排除与上一帧相同的 SEQ，未严格保证三个不同非零 SEQ |
| P1 | SEQ 生成和控制命令缓存缺少完整跨端约束 | Qt 重启、并发写串口或 8 位 SEQ 回绕时可能重放旧状态或产生 `SEQ_CONFLICT` |

当前收益最高的处理顺序是：

```text
先把 SCI2 RX 改为环形缓冲区，消除固定周期丢字节
-> 确认 RA6M5 -> Jetson 回传物理链路
-> 单轴标定 dcx/dcy 方向
-> 解决 Jetson valid=0 和目标稳定锁
-> 同步安全距离、撤离和 SEQ 规则
-> 验证 3+2 帧 ALIGN_DONE/P003
-> 最后验证 P000/P503/P801
```

## 2. UART 问题的真实边界

### 2.1 COM16 与 Jetson UART 不是同一条链路

- COM16 是 RA6M5 调试日志口，只能由一个 PC 程序独占打开。
- Jetson 正式二进制协议使用 RA6M5 `SCI2`：
  - `P112 = SCI2_TXD`，连接 Jetson/USB 串口的 RX；
  - `P113 = SCI2_RXD`，连接 Jetson/USB 串口的 TX；
  - 两端必须共地，电平必须为 3.3 V TTL。
- 当前 COM16 监听曾出现 `PermissionError` 和 `ClearCommError failed`，原因是多个 PC 程序争用 COM16。这不能证明 SCI2 链路故障。

联调时必须保持：

```text
COM16：只允许一个 RA6M5 日志程序打开
SCI2：只允许一个 Jetson/PC 模拟程序发送和接收正式协议
```

### 2.2 已确认的固定周期 CRC 根因

本轮日志在视觉连续发送阶段记录到以下 CRC 错误间隔：

```text
6.700, 6.690, 6.701, 6.693, 6.671, 6.691 ... 秒
```

间隔高度稳定，并且联调已经确认 CRC 错误周期会随 DTC 缓冲区长度变化，因此不再需要做 `512/2048` 字节确认实验。当前 `jetson_vision.c` 的接收方式是：

1. `R_SCI_UART_Read()` 启动一次 1024 字节 DTC 普通模式接收；
2. 视觉任务每 10 ms 查询剩余长度；
3. 缓冲区收满后处理末尾数据；
4. 再次调用 `R_SCI_UART_Read()` 重新挂载 1024 字节接收。

在 DTC 收满到任务重新挂载之间，最长存在约 10 ms 空窗。此时到达的串口字节可能转为 `UART_EVENT_RX_CHAR`，但当前回调明确忽略了该事件，因此会固定丢失一部分帧。日志中 CRC 高字节经常变成下一帧头 `0xA5`，与该现象一致。增大 DTC 缓冲区只能延长故障出现周期，不能消除重新挂载空窗。

### 2.3 确定的最终修复

比赛前采用最简单且可验证的方案：取消 Jetson UART RX 的 1024 字节 DTC 普通模式轮询，改为 `UART_EVENT_RX_CHAR + 环形缓冲区`，或使用具备同等无空窗特性的循环接收机制。本项目优先采用前者。

建议结构：

```text
robot_jetson_callback ISR
    -> 收到一个字节
    -> 写入 512/1024 字节 RX ring
    -> 只更新 write_index/overflow_count，不打印日志

vision_service_thread 每 10 ms
    -> 从 read_index 追到 write_index
    -> 逐字节调用协议 parser
```

RA6M5 当前实际协议流量远低于 115200 满载，按字节中断的开销可控，而且工程中的调试 UART 已经使用相同思路。环形缓冲区满时必须丢弃新字节、增加 `rx_overflow_count`，不能覆盖尚未消费的数据。

同时增强 parser 的重新同步：CRC 失败后从收到的数据中重新搜索连续 `A5 5A`，不要把下一帧的 `A5` 当成上一帧 CRC 后直接丢掉整帧。

## 3. RA6M5 回传为什么需要单独验证

当前代码收到控制命令后确实会执行：

```text
command_ack()
-> jetson_send_status()
-> R_SCI_UART_Write()
-> 等待 UART_EVENT_TX_COMPLETE，超时 20 ms
```

因此“Jetson 没看到 RX”不能直接推断为状态机没有发送。应按以下方式定位：

1. RA6M5 在发送每个业务帧时记录 `seq/event/value/error/tx_result`；
2. 用逻辑分析仪或示波器观察 P112；
3. 如果 RA6M5 打印 `tx_result=OK` 且 P112 有波形，但 Jetson 无 `RX RAW`，检查 P112 -> Jetson RX、共地、USB 串口 RX 和 Jetson 接收线程；
4. 如果 P112 没有波形，检查 SCI2 TX 引脚复用、`R_SCI_UART_Write()` 返回值和 TX complete 回调；
5. Jetson 收到原始字节但解析失败时，再检查 `A5 5A`、LEN、CRC 范围和 CRC 小端顺序。

当前 `command_ack()` 等调用方忽略了 `jetson_send_status()` 的返回值。建议增加发送统计和失败上下文，但不要在控制任务中无限重试：

```text
tx_ok_count
tx_start_fail_count
tx_complete_timeout_count
last_tx_seq/event/error
```

Jetson 控制命令本身已有 300 ms、最多 3 次的同帧重发机制；RA6M5 收到重发后应重放缓存 ACK，而不是重复运动。

## 4. 视觉闭环不收敛的原因和处理

### 4.1 当前真正生效的参数

当前 `TARGET_USE_VISUAL_SERVO=1`，所以实际使用的是速度式视觉伺服参数：

```c
TARGET_VS_KX_MM_S_PER_PX      = 0.35f
TARGET_VS_KZ_MM_S_PER_PX      = 0.35f
TARGET_VS_MAX_SPEED_MM_S      = 8.0f
TARGET_VS_FINE_MAX_SPEED_MM_S = 3.0f
TARGET_VS_CMD_TIMEOUT_MS      = 250u
```

`TARGET_KX_MM_PER_PX`、`TARGET_KY_MM_PER_PX`、`TARGET_MAX_STEP_MM` 和 `TARGET_ALIGN_PERIOD_MS` 属于 `TARGET_USE_VISUAL_SERVO=0` 时的旧小步 AUTO 分支，当前修改它们不会改善实际跟随。

当前映射为：

```text
dcx -> 机械臂 X 速度 vx
dcy -> 机械臂 Z 速度 vz
机械臂 Y 速度固定为 0
```

因此队友清单中的“Y 轴视觉闭环发散”应改称“图像纵向误差 dcy 到机械臂 Z 轴的映射未标定”。

### 4.2 当前日志说明了什么

日志曾出现：

```text
dcy=49,  vz=+8.00
dcy=72,  vz=+8.00
```

误差没有下降，说明至少存在以下一种情况：

1. `dcy -> Z` 的符号相反；
2. `8 mm/s` 饱和速度对当前相机视场和机械结构过快；
3. X/Z 运动存在明显图像耦合；
4. Jetson 在运动后切换或丢失了目标，前后 `dcy` 不是同一个稳定目标的误差。

只凭这两段数据不能直接永久反转参数，必须先完成单轴实验。

### 4.3 单轴标定步骤

物理断开激光，并把视觉速度上限临时降到 `2 mm/s`：

1. 将目标放在画面下方，保持 `dcx≈0、dcy>30`；
2. 只执行一个短时间 `+Z` 修正，然后 HOLD；
3. 比较同一目标的 `abs(dcy_before)` 和 `abs(dcy_after)`；
4. 若误差增大，则把 `TARGET_VS_KZ_MM_S_PER_PX` 改为负值；
5. 对 `dcx>30` 使用同样方法确定 X 轴符号；
6. 符号正确后再逐步调增益，不得同时改符号、增益和速度上限。

建议首轮标定值：

```text
abs(KX/KZ) = 0.05~0.10 mm/s/px
粗调最大速度 = 2 mm/s
精调最大速度 = 0.8~1.0 mm/s
```

确认误差持续收敛后，再将粗调速度逐步提高到 `3~4 mm/s`。比赛前没有必要恢复到当前 `8 mm/s`，稳定优先于对准速度。

### 4.4 增加发散保护

完成方向标定后，RA6M5 应记录每次有效新视觉帧的主误差：

```text
error = max(abs(dcx), abs(dcy))
```

同一目标连续两到三次修正后，如果误差相对修正前增大超过 `5 px` 或 `20%`，执行：

```text
P801 OFF
视觉速度清零
robot_target -> HOLD/FAULT
使用现有 MOTION_FAILED(0x06) 回传
打印 before/after、vx/vz 和触发轴
```

暂不新增协议错误码，避免比赛前再次修改双方协议。

## 5. 为什么现在无法到达 ALIGN_DONE 和激光状态

### 5.1 当前固件实际需要 3+4 共 7 帧

当前 RA6M5 的对准条件不是“出现三帧有效数据”这么简单，而是：

```text
abs(dcx) <= 5
&& abs(dcy) <= 5
&& 连续 3 个新 valid=1 帧进入 CONFIRM
&& 再连续 4 个新 valid=1 帧完成二次确认
```

也就是至少需要连续 `7` 个处于阈值内的新有效帧。进入 `CONFIRM` 时计数会重新开始，因此前 3 帧不能同时计入后 4 帧。任意一帧 `valid=0`、误差超限或视觉超时，都会清除稳定计数，并执行：

```text
停止视觉伺服
清除视觉有效状态
ALIGN/CONFIRM/OUTPUT -> WAIT_DETECT
回传 VISION_LOST
关闭 P801
```

### 5.2 7 帧偏保守，但不是本轮失败的根因

本轮日志只有短暂的 `valid=1`，随后长时间 `valid=0`，而且有效帧误差仍为几十像素。即使把要求从 7 帧降到 3 帧，这些数据仍然不能满足 `abs(dcx)<=5 && abs(dcy)<=5`，所以当前无法产生：

```text
ALIGN_DONE
P003 ready LED ON
P000 发射许可
P503 output LED ON
P801 HIGH
```

因此必须先解决 Jetson 稳定目标锁、`valid=0` 和 X/Z 映射方向，再调整帧数。目标不可靠时继续发送 `valid=0` 是正确的安全行为，不能用 `dcx=0,dcy=0,valid=1` 伪造对准。

### 5.3 比赛版建议改为 3+2 共 5 帧

在视觉方向和目标锁定确认正常后，建议只把二次确认从 4 帧降为 2 帧：

```c
TARGET_ALIGN_STABLE_COUNT   = 3u
TARGET_CONFIRM_STABLE_COUNT = 2u
```

这样仍保留“初次对准 + 二次确认”两道门槛，但总要求从 7 帧降为 5 帧。Jetson 每 `100~200 ms` 发送一帧时，理论确认时间约为 `0.5~1.0 s`，更适合比赛演示。

以下安全规则保持不变：

1. 只统计新收到且 `valid=1` 的视觉帧；
2. `abs(dcx)>5` 或 `abs(dcy)>5` 时立即清零稳定计数；
3. `valid=0`、视觉超时、心跳超时或故障时立即关闭 P801；
4. 在完成单轴标定前，不通过扩大 `±5 px` 阈值掩盖方向错误；
5. 比赛前暂不把总门槛降到 5 帧以下。

## 6. 安全距离、撤离和 SEQ 的最终联合约定

### 6.1 安全距离和撤离行程是两个独立参数

此前文档把“安全距离阈值”和“撤离步长”混在一起。最终意图是把安全距离从 `10 cm` 提高到 `15 cm`，不是把单次撤离步长改为 `15 mm`。

| 参数 | 当前固件 | 最终约定 | 含义 |
|---|---:|---:|---|
| 安全距离阈值 | `100 mm` | `150 mm` | RGB-D 实测距离低于该值时判定不安全 |
| 单次 +Y 撤离步长 | `15 mm` | `20 mm` | 每次完成后必须重新测距 |
| 最大撤离次数 | `5` | `5` | 单轮安全恢复最多执行 5 次 |
| 最大累计撤离行程 | `75 mm` | `100 mm` | `20 mm x 5`，即最多撤离 10 cm |

后续代码目标参数为：

```c
ROBOT_WORKFLOW_SAFE_DISTANCE_MM     = 150u
ROBOT_WORKFLOW_RETREAT_STEP_MM      = 20.0f
ROBOT_WORKFLOW_RETREAT_MAX_STEPS    = 5u
```

`150 mm` 是必须达到的实际安全距离，`100 mm` 是机械臂本轮最多允许执行的撤离行程，两者不要求相等。即使机械臂已累计撤离 100 mm，只要 RGB-D 实测仍小于 150 mm，也不能判定安全。

### 6.2 撤离必须逐步测量，不能一次盲走 100 mm

安全恢复流程固定为：

```text
收到有效距离且 distance_mm < 150
-> RA6M5 沿 +Y 撤离 20 mm
-> 等待该次运动真实完成
-> RA6M5 回传 RETREAT_STEP_READY
-> Jetson 重新拍摄并发送新的 SAFE_DISTANCE
-> 若 distance_mm >= 150：停止撤离并保持，等待用户重新点击 Qt“开始”
-> 若仍不安全且次数 < 5：再撤离 20 mm
-> 若完成 5 步后仍小于 150 mm：进入故障/HOLD，禁止继续自动撤离和激光输出
```

Jetson 必须发送 RGB-D 的实际毫米值，不能把机械臂移动量直接加到上一帧距离上。RA6M5 是 `150 mm` 安全门槛的最终判定方。心跳超时或其他异步故障时仍按既定方案立即停止并保持当前位置，不自动回 HOME。

该参数变更后必须在同一次联调版本中同步更新：

- RA6M5 `robot_workflow.h`；
- 正式 `视觉协议.md`；
- `控制文档.md`；
- Jetson/Qt 的安全距离提示、参数校验和验收脚本。

本文只记录决策，本次不修改固件代码。

### 6.3 SAFE_DISTANCE 三帧必须使用不同非零 SEQ

正式要求为三个不同的非零 SEQ。当前代码只拒绝“与上一帧相同”的 SAFE SEQ，例如：

```text
SEQ=10 -> SEQ=11 -> SEQ=10
```

当前仍可能累计为三帧。RA6M5 应在本轮测距上下文保存已计入 streak 的三个 SEQ：

1. `SEQ=0`：拒绝计数；
2. 与本轮已计入的任一 SEQ 重复：忽略，不增加 streak；
3. `valid=0` 或距离小于 `150 mm`：清空 streak 和对应 SEQ；
4. 新 `WORKFLOW START`、ABORT 或故障：清空本轮 SEQ；
5. 第三个不同非零安全样本才锁存 `DISTANCE_SAFE_LATCHED`。

“相邻有效样本超过 600 ms 后清零 streak”仍属于待双方批准的增强项，比赛前不单方加入正式协议。

### 6.4 Jetson 与 RA6M5 的 SEQ 联合规则

比赛前不增加 `session_id/epoch` 等新协议字段，采用以下最小联合方案：

**Jetson / Qt 端：**

1. HEARTBEAT、VISION、SAFE_DISTANCE 和所有控制帧共用一个线程安全的全局 SEQ 生成器；
2. SEQ 范围只允许 `0x01~0xFF`，`0xFF` 后回绕到 `0x01`，不得发送 `0x00`；
3. 程序启动时随机选择一个非零初始 SEQ，降低重启后与旧缓存碰撞的概率；
4. 每个新的测量、心跳或控制请求必须分配新 SEQ；
5. 只有控制帧 ACK 超时重试时，才允许原样重发同一帧；重发的 TYPE、SEQ、payload 和 CRC 必须完全一致；
6. 控制 ACK 等待 `300 ms`，最多重试 3 次；
7. 所有串口写入都经过单一 TX 队列或全局互斥锁，禁止多个线程分别维护 SEQ 并交叠写串口。

**RA6M5 端：**

1. 对控制命令以 `TYPE + SEQ` 查缓存，并比较 payload；
2. `TYPE + SEQ + payload` 完全一致且仍在缓存有效期内时，判定为重发，只重放原 ACK/结果，不重复执行运动；
3. 相同 `TYPE + SEQ` 但 payload 不同时，回传 `SEQ_CONFLICT`，不执行命令；
4. 控制命令缓存增加 `WORKFLOW_COMMAND_CACHE_TTL_MS = 3000u`，超过 3 秒的条目按新命令处理；
5. 与某个控制请求对应的 ACK 和业务完成状态沿用该请求的 SEQ；
6. RA6M5 主动产生、无法关联到具体控制请求的异步故障使用 `SEQ=0x00`；
7. SAFE_DISTANCE 按 6.3 保存本轮全部已计入 SEQ，不能只比较上一帧。

这样既能让 Jetson 可靠重试，又能防止 RA6M5 重复运动，并降低 Qt 重启和 8 位 SEQ 回绕导致的旧状态重放风险。
## 7. 建议增加的 RA6M5 联调日志

不要打印每个原始字节，也不要永久打印每一帧心跳。建议每秒输出一次汇总，并在状态变化或错误时输出明细：

```text
[JETSON_UART] rx_bytes=... frames_ok=... crc_fail=... overflow=...
               tx_ok=... tx_start_fail=... tx_timeout=...
               last_type=... last_seq=... last_len=...

[WORKFLOW] old=... new=... reason=...
[RX] type=... seq=... len=... crc=OK
[ACK] seq=... accepted=... error=... tx=...
[SAFE] seq=... mm=... valid=... streak=... duplicate=...
[VISION] seq=... valid=... dcx=... dcy=...
[SERVO] vx=... vz=... error_before=... error_after=...
[LASER_GATE] permitted=... denied_reason=...
```

其中 `denied_reason` 至少应区分：未 ALIGN、P000 未按、视觉失效、心跳失效、距离未锁存、pose invalid、运动未停止、限位/故障。

## 8. 双方问题归属和执行清单

### 8.1 问题归属

| 问题 | RA6M5 责任 | Jetson / Qt 责任 | 联调完成标准 |
|---|---|---|---|
| 固定周期 CRC 错误 | SCI2 RX 改为无空窗环形缓冲区，增加 overflow/CRC 统计 | 所有发送经单一 TX 队列，完整帧一次写入 | 连续运行 5 分钟无固定周期 CRC 增长、无 overflow |
| RA6M5 回传不可见 | 记录 TX 返回值并用 P112 波形确认发送 | 确认 RX 设备、接线、共地和解析线程 | 同一 SEQ 的 ACK 和业务状态可在 Jetson 原始日志中看到 |
| 视觉长期 `valid=0` | 保持 fail-closed，不绕过安全门 | 稳定锁定同一真实目标，运动后持续跟踪 | 微调期间连续输出真实 `valid=1`，丢失时正确发 `valid=0` |
| 视觉误差不收敛 | 完成 X/Z 方向、增益和速度上限标定 | 保证前后帧属于同一目标并提供原始误差日志 | 每次短修正后主误差总体下降，无连续发散 |
| 安全距离和撤离参数 | 使用 `150 mm` 门槛、`20 mm/步`、最多 5 步 | 发送实际毫米值，每次 `RETREAT_STEP_READY` 后重新测距 | 不安全时逐步撤离；安全后停止；100 mm 后仍不安全则 HOLD |
| 对准确认帧数 | 实施 `3+2` 共 5 帧，并保留所有 fail-closed 条件 | 以 `100~200 ms` 周期发送新有效帧 | 连续 5 帧满足 `±5 px` 后收到 ALIGN_DONE |
| SEQ 生成、重试和回绕 | 3 秒命令缓存、重复 ACK 重放、冲突检测、异步故障用 0 | 全局非零 SEQ 生成器、随机初值、统一 TX 队列 | 重试不重复运动，Qt 重启和回绕不重放旧业务状态 |
| SAFE_DISTANCE 去重 | 保存本轮全部三个非零 SEQ | 每次新测量分配新 SEQ | 重复帧不增加 streak，三个不同安全样本才锁存 |

### 8.2 RA6M5 端

- [ ] 删除“修改 DTC 缓冲区长度继续实验”的步骤，直接将 SCI2 RX 改为 `UART_EVENT_RX_CHAR + ring buffer` 或等效无空窗方案；
- [ ] 环形缓冲区满时丢弃新字节并增加 `rx_overflow_count`，不得覆盖未消费数据；
- [ ] parser 在 CRC 失败后重新搜索 `A5 5A` 帧头；
- [ ] 增加 RX/CRC/overflow/TX 统计和带 TYPE/SEQ 的 ACK、业务状态日志；
- [ ] 用 P112 波形确认每次 ACK/状态确实发送；
- [ ] 将安全距离门槛改为 `150 mm`，撤离改为 `20 mm/步`、最多 5 步；
- [ ] 每步撤离完成后回传 `RETREAT_STEP_READY` 并等待新测距，禁止连续盲走 100 mm；
- [ ] 完成 5 步后仍小于 150 mm 时进入 HOLD/FAULT，保持 P801 关闭；
- [ ] 将视觉确认参数改为 `3+2` 共 5 帧，不改变 `±5 px` 门槛；
- [ ] 严格实现三个不同非零 SAFE_DISTANCE SEQ；
- [ ] 给控制命令缓存增加 `3000 ms` TTL，并按 TYPE、SEQ 和 payload 区分重发与冲突；
- [ ] ACK/业务状态沿用请求 SEQ，异步故障使用 `SEQ=0x00`；
- [ ] 物理断开激光，完成 X、Z 两个方向的单轴符号标定；
- [ ] 降低视觉速度上限，确认 `abs(dcx/dcy)` 总体收敛后再提速；
- [ ] 保持 `valid=0` fail-closed、P000 和激光唯一出口逻辑不变。

### 8.3 Jetson / Qt 端

- [ ] HEARTBEAT、VISION、SAFE_DISTANCE 和控制帧共用一个线程安全的全局 SEQ 生成器；
- [ ] 使用 `0x01~0xFF`，随机非零初值，`0xFF` 后回到 `0x01`，不得主动发送 `SEQ=0x00`；
- [ ] 所有帧只经过一个 TX 队列/全局写锁，一帧组装完整后一次 `write()`；
- [ ] 每个新测量和新请求分配新 SEQ；只有控制 ACK 超时重试才原样复用整帧和 SEQ；
- [ ] 控制帧按 300 ms 超时、最多 3 次重试，重试时 TYPE、SEQ、payload 和 CRC 不变；
- [ ] 输出 `RX RAW`、CRC 结果、TYPE、SEQ、event/value/error；
- [ ] 确认实际打开的是与 RA6M5 SCI2 相连的设备，且 TX/RX 交叉、共地；
- [ ] RGB-D 发送实际 `distance_mm`；Qt 显示和验收标准统一使用 `150 mm` 安全门槛；
- [ ] 收到 `RETREAT_STEP_READY` 后重新拍摄，再发送新的 SAFE_DISTANCE 和新 SEQ；
- [ ] 不根据“已经移动 100 mm”自行推断安全；仍以 RGB-D 实测是否达到 150 mm 为准；
- [ ] 当前正式开始命令保持 `WORKFLOW_CTRL action=0x01`，不启用未批准的 `0x04` 连续距离方案；
- [ ] 解决目标身份稳定锁问题，使同一黑痣在运动后持续输出真实 `valid=1`；
- [ ] `valid=0` 时不得伪造零误差。

### 8.4 推荐实施顺序

```text
双方先冻结并同步视觉协议/控制文档中的参数和 SEQ 规则
-> RA6M5 完成环形缓冲区、150/20x5、5帧确认、SEQ缓存修复
-> Jetson 完成全局 SEQ、单 TX 队列、150 mm 提示和逐步重测
-> 断开激光进行 5 分钟纯通信验收
-> 断开激光验证安全距离与逐步撤离
-> 断开激光完成 X/Z 视觉标定和 ALIGN_DONE
-> 最后接回激光，验证 P000/P003/P503/P801
```

任何一端没有完成本节对应项时，不进入下一阶段，避免把通信、视觉和运动问题混在一次测试里。

## 9. 无激光验收顺序

### A. 纯通信，5 分钟

1. 只发送 `150~200 ms` 心跳；
2. 周期发送不会引起运动的控制请求并检查同 SEQ ACK；
3. RA6M5 `crc_fail` 不得再按固定周期增长；
4. RX overflow、TX timeout 必须为 0；
5. 重发同一控制帧只重放 ACK，不重复执行动作；
6. COM16 和 SCI2 分别只有一个程序占用。

### B. 安全距离与撤离

**已安全路径：**

```text
START_MEASURE
-> ACK / START_ACCEPTED
-> MEASURE_POSITION_READY
-> 三个不同非零 SEQ，且 distance_mm >= 150
-> DISTANCE_SAFE_LATCHED
```

**不安全路径：**

```text
distance_mm < 150
-> 只撤离 +Y 20 mm
-> RETREAT_STEP_READY
-> Jetson 重新测距并使用新 SEQ 发送
-> 达到 150 mm 后停止撤离并保持
```

验收时还必须验证：完成 5 步、累计 100 mm 后仍小于 150 mm，RA6M5 进入 HOLD/FAULT，不继续移动、不允许 P801 输出。恢复安全后等待用户重新点击 Qt“开始”，之后先回 HOME 再进入公共测距位。

### C. 正式工作流到选中视图

```text
DISTANCE_SAFE_LATCHED
-> HOME / GOTO 1 / GOTO 2 / GOTO 3 / HOME
-> 医生选择一张视图
-> SELECTED_VIEW_READY
```

每个动作必须以业务完成状态推进，禁止固定延时猜测。选中的三视图拍摄点也是后续点痣状态机的预开始点。

### D. 单轴视觉标定

- P801 物理断开；
- X 轴一次短修正后 `abs(dcx)` 下降；
- Z 轴一次短修正后 `abs(dcy)` 下降；
- `valid=0` 时速度在 250 ms 内归零；
- 连续误差增大时进入 HOLD。

### E. ALIGN_DONE 与指示灯

1. 连续发送同一真实目标的有效视觉帧，每个新帧使用新非零 SEQ；
2. 实施 `3+2` 参数后，至少连续 5 个新帧满足 `abs(dcx)<=5 && abs(dcy)<=5`；
3. Jetson 收到 `ALIGN_DONE`；
4. P003 点亮；
5. 未按 P000 时 P503 和 P801 始终关闭。

注意：修改前固件仍按 `3+4` 共 7 帧判断，不能用旧固件执行 5 帧验收。

### F. 最终安全输出

完成 A~E 后才允许接回激光输出：

1. 按住 P000，P503 和 P801 开启并回传 OUTPUT_ON；
2. 松开 P000，P003、P503 和 P801 都关闭并回传 OUTPUT_OFF；
3. 分别注入 `valid=0`、视觉超时、心跳超时、误差超限和限位，P801 必须先关闭；
4. P000 松开后机械臂保持当前位置，等待 Qt“完成/返回”；
5. 点击“完成/返回”，soft_reset final verify PASS 后回到 IDLE。

## 10. 当前不建议做的事情

- 不要推倒现有顶层状态机；主流程已经能推进到 `TARGET_ACTIVE`。
- 不要把 150 mm 安全门槛误写成 150 mm 单步撤离，也不要把累计撤离 100 mm 自动当成已经安全。
- 不要连续盲走 100 mm；必须每撤离 20 mm 后重新测距。
- 不要用增大对准阈值掩盖方向错误或目标不稳定。
- 不要仅靠把 7 帧降为 5 帧解决当前几十像素误差和 `valid=0`。
- 不要修改当前未生效的 `TARGET_KX_MM_PER_PX/TARGET_KY_MM_PER_PX` 来调速度式视觉伺服。
- 不要把 `valid=0` 当成零误差。
- 不要继续用增大 DTC 缓冲区长度规避固定周期丢字节；该方法只会延后故障。
- 不要让 Jetson 的不同线程分别生成 SEQ 或直接并发写串口。
- 不要让 Jetson 主动使用 `SEQ=0x00`；该值保留给 RA6M5 异步故障。
- 不要在双方未确认前加入 `WORKFLOW action=0x04`、连续 SAFE_DISTANCE watchdog 或新错误码。
- 不要同时使用多个程序打开 COM16，也不要让 PC 模拟器和真实 Jetson 同时驱动 SCI2。

完成本文 P0 项以及第 8 节双方清单后，现有状态机具备继续整机联调的基础。在这些项完成之前，单纯修改视觉阈值或反复按 P000 都不会稳定进入激光输出状态。
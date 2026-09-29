# RA6M5 当前问题与联调处理清单

> 日期：2026-07-17  
> 交付方向：Jetson / Qt 视觉端 -> RA6M5 固件端  
> 当前状态：视觉端软件测试通过，但完整无激光硬件流程尚未验收通过。  
> 安全要求：本文所有运动验证必须关闭并物理隔离激光输出；在 `ALIGN_DONE` 前禁止测试 P000/P801。

## 1. 文档目的与正式依据

本文只记录本轮实机联调仍然存在的问题、责任边界、建议修改和验收方法，可直接交给 RA6M5 固件开发人员。

当前正式依据：

1. `视觉协议(5).md`；
2. `项目演示(3).md`；
3. `RA6M5固件对接交接说明.md` 顶部重要修订及第 19 章；
4. RA6M5 工程内与上述文件双方确认一致的正式协议。

`视觉协议(3).md`、`视觉协议(4).md`、`项目演示(2).md` 以及历史连续测距提案不能作为当前实现依据。任何新增动作码、状态码或安全语义必须由双方确认后再实现。

当前 Jetson 仓库不包含 RA6M5 的 C/C++、Keil 或 Renesas 固件源码，因此本文对 RA6M5 行为的判断来自双方协议、实机串口记录和上板现象；固件内部原因及修改完成情况必须由 RA6M5 团队结合源码和板端日志确认。

## 2. 当前结论

| 优先级 | 问题 | 当前影响 | 主要处理方 |
|---|---|---|---|
| P0 | 当前 Jetson 只有 TX，未收到 RA6M5 的 RX/COMMAND_ACK/业务 READY | 无法进入任何正式工作流 | 双方联查，RA6M5 先确认 UART 原始收字节 |
| P0 | Y 轴视觉闭环发散 | 目标会越调越偏并移出画面 | RA6M5 修正图像误差到物理轴的映射/增益 |
| P1 | RGB-D 有效测距不连续 | 测距阶段可能长时间等不到 3 个可靠样本 | Jetson 增加分阶段诊断并修正投影/检测框稳定性 |
| P1 | 8 位 SEQ 缓存跨重启/回绕没有会话边界 | 可能出现 `SEQ_CONFLICT` 或旧状态重放 | RA6M5 增加 session epoch 或短 TTL |
| P1 | 末端目标偶发暂时丢失 | 会发送 `VISION_ERROR valid=0`，对准暂时停止 | Jetson 继续做整机回归；RA6M5 必须安全恢复而非误判对准 |
| 待评审增强 | 3 个安全距离样本没有最大时间间隔 | 零散有效样本可能跨较长静默间隔后仍被累计；但这不是现行正式协议违规 | 双方评审后再决定是否新增 streak 超时清零 |

当前最先处理的顺序是：

```text
UART RX/ACK
-> MEASURE_POSITION_READY 及后续各业务 READY
-> Y/X 单轴方向验证
-> SAFE_DISTANCE 连续性验证
-> 完整无激光流程
-> 最后才允许评审手动 P000/P801
```

## 3. 问题一：当前没有收到 RA6M5 回包

### 3.1 当前实机证据

Jetson 系统侧已经确认：

```text
/dev/ttyUSB0
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
USB ID 1a86:7523 CH340
```

Qt 持续生成并写出统一协议心跳：

```text
TX HEARTBEAT ... A5 5A 01 01 ...
```

但是当前日志没有出现：

```text
RX RAW
COMMAND_ACK_ACCEPTED
WORKFLOW ... READY
TARGET_CTRL_ON
TARGET_READY
```

因此，`TX` 只能证明 Jetson 调用了串口写入，不能证明 RA6M5 UART 已经收到数据。

历史实机曾成功完成：

```text
TARGET_CTRL START
-> COMMAND_ACK_ACCEPTED
-> TARGET_CTRL_ON
-> TARGET_READY
-> VISION_ERROR valid=1
-> VISION_VALID
```

说明 CH340、115200 波特率和统一协议曾经正常；当前更像链路、端口、接线或固件接收状态的间歇性回归，而不是协议从未匹配。

### 3.2 RA6M5 端必须先增加的诊断

请在不驱动电机、不打开激光的情况下打印：

```text
uart_rx_raw_byte_count
last_rx_byte
frame_header_ok_count
frame_crc_ok_count
frame_crc_error_count
decoded_type
decoded_seq
decoded_len
command_ack_sent
current_workflow_state
```

按以下分支定位：

1. `uart_rx_raw_byte_count=0`：检查 Jetson TX -> RA6M5 RX、RA6M5 TX -> Jetson RX、共地、供电、正确 UART 引脚、115200-8N1、接收中断/DMA和接收任务；
2. 有原始字节但没有完整帧：检查帧头查找、粘包/拆包、`LEN`、超时和环形缓冲区；
3. 有完整帧但 CRC 错误：检查 CRC16/MODBUS 范围和低字节先发送；
4. 帧解析正确但没有 ACK：检查业务分发和 ACK 发送任务；
5. 已有 ACK 但没有 READY：检查急停、限位、fault、状态门、运动命令和真实到位判断。

### 3.3 通信验收标准

在不运动的协议检查阶段，RA6M5 至少应能稳定收到心跳，并在收到合法控制帧后立即返回同一 `SEQ` 的 `COMMAND_ACK`。Jetson 必须能看到对应 `RX RAW`；仅看 RA6M5 或 Jetson 单侧的“已发送”日志不算通过。

## 4. 问题二：Y 轴视觉闭环发散

### 4.1 实机证据

之前无激光对准测试中，Jetson 连续发送的有效误差出现：

```text
dcy: 49 -> 63 -> 91 -> 92
```

机械臂修正后 `abs(dcy)` 持续增大，最终目标移出画面。这是闭环方向错误、增益过大或轴耦合造成的发散。

### 4.2 Jetson 坐标定义已经确认

```text
dcx = target_center_x - image_center_x
dcy = target_center_y - image_center_y
```

含义：

```text
dcx > 0：目标在画面右侧
dcy > 0：目标在画面下方
```

Jetson 按 `int16 dcx + int16 dcy + uint8 valid`、小端 `<hhB` 原样打包，没有在发送前反号。图像误差到电机方向、关节方向或笛卡尔方向的映射属于 RA6M5 标定责任，不能通过 Jetson 临时翻转 `dcy` 掩盖。

### 4.3 RA6M5 修改与验证要求

1. 物理隔离激光；
2. 将目标摆到 `dcx≈0、dcy>0`；
3. RA6M5 只执行一次很小的 Y 轴修正，随后立即 HOLD；
4. 比较修正前后误差，必须满足：

```text
abs(dcy_after) < abs(dcy_before)
```

5. 如果误差增大，修正 RA6M5 的垂直映射符号；
6. 如果方向正确但来回振荡，降低步长/增益，并检查 X/Y 轴耦合；
7. Y 轴通过后，用同样方法单独验证 X 轴；
8. 单轴通过前禁止恢复连续视觉伺服。

建议 RA6M5 增加独立发散保护：连续两次有效修正后主误差仍增大，立即关闭 P801、停止运动并进入 HOLD，同时回传明确错误状态。

## 5. 待双方评审的安全增强：SAFE_DISTANCE 三样本时间连续性

### 5.1 当前正式行为与 600 ms 语义

当前协议要求 RA6M5 在收到 3 个使用不同非零 `SEQ` 的有效 `SAFE_DISTANCE` 样本后锁存安全距离。Jetson 对以下情况采用静默处理：

- YOLO 结果过期；
- RGB/Depth 不同步；
- 检测框 Depth 点不足；
- 检测框有效 Depth 比例不足；
- 没有新的 RGB-D 样本；
- 重复样本。

静默的含义是：

```text
不发送 SAFE_DISTANCE
不发送旧距离
不使用 valid=0 伪装刷新
```

界面可保留上一次值，但会明确显示“已过期/未发送”。

必须特别区分：

- 当前正式工作流只使用 `WORKFLOW_CTRL action=0x01` 的一次测距锁存；
- 当前正式的 `600 ms` 是 RA6M5 全局 HEARTBEAT 超时语义；
- Jetson 的 `500 ms` 是本地 RGB-D 样本最大年龄，不是 RA6M5 距离看门狗；
- `action=0x04`、锁存后持续距离消费、SAFE_DISTANCE freshness `600 ms`、相邻锁存样本 `<600 ms` 和 `SAFE_DISTANCE_TIMEOUT=0x10` 均属于未来协议提案，不是现行正式要求。

### 5.2 待评审风险

当前正式协议没有“相邻有效样本必须小于某个有限间隔”的条件。如果固件只记录有效样本数量而不记录间隔，可能出现：

```text
有效样本1
-> 长时间无效/静默
-> 有效样本2
-> 长时间无效/静默
-> 有效样本3
-> 仍然锁存安全
```

这三个样本不代表患者和机械臂处于同一个连续、稳定的测距场景。

这是安全增强建议，不应在双方批准协议变更之前认定为 RA6M5 当前协议缺陷，也不能作为现行流程的拒收条件。当前正式流程中，锁存后如 Jetson 检测到患者或场景明显变化，应发送 `ABORT_HOLD`，再由用户重新开始一次测距。

### 5.3 双方批准后的建议实现

如果双方评审并批准该增强，建议增加：

```text
if (now_ms - last_valid_safe_distance_ms > 600) {
    safe_distance_streak = 0;
}
```

同时要求：

1. 只统计 CRC 正确、状态允许、`valid=1`、不同非零 `SEQ` 的新样本；
2. 重复 `SEQ` 不增加 streak；
3. 超过最大间隔立即清零 streak；
4. 工作流中止、重新开始或场景重置时清零 streak；
5. 满足 3 个连续有效样本后才能回传安全裁决；
6. 最大间隔的正式数值、错误码和验收方法必须由双方共同确认，不能由任一端单独写入当前运行命令。

## 6. Jetson 当前 RGB-D 问题，供 RA6M5 联调理解

当前实时日志经常在以下状态之间切换：

```text
VALID reported_mm≈186 mm
INVALID reason=检测框Depth点不足
```

全局 Depth 仍约为 30 FPS、有效率约 78%~82%，而且同一帧通常同时存在 `arm` 和 `face` 检测。因此无效的主要原因不是整路 Depth 断流，而是某个检测框投影到 Depth 后，经中心区域、背景外环和前景表面归属筛选，剩余可靠点不足 8 个。

Jetson 端下一步会增加以下诊断，再根据数据修正投影或检测稳定性：

```text
arm/face 检测框坐标、面积、置信度
投影后原始 Depth 点数
中心区域点数
背景外环点数
前景筛选后点数
框裁剪比例
RGB/Depth 时间差
RGB 检测框到 Depth 的投影叠图
```

在拿到这些数据前，不建议直接降低 8 点门限、20% 有效率或 15 mm 前景分离值，因为放宽门限可能把背景误当成机械臂/人脸表面并报告过大的安全净距。

RA6M5 端在当前正式协议下必须遵守：没有新 `SAFE_DISTANCE` 时不能自行复制最后一次距离形成新样本，重复 `SEQ` 也不能增加三样本计数。是否因静默时间过长而清零已收到的部分 streak，属于第 5 章待双方批准的增强。

## 7. 问题三：8 位 SEQ 缓存缺少跨会话边界

Jetson 的 `SEQ` 使用 `0x01~0xFF` 并跳过 `0x00`。HEARTBEAT、VISION 和控制帧共享 8 位 SEQ，因此运行时间足够长时一定回绕。

Jetson 当前会在同一进程生命周期内避开近期使用过的同类型控制键，但 Qt 重启后无法知道 RA6M5 仍缓存了哪些旧 `(TYPE, SEQ)`。

如果 RA6M5 控制请求缓存保留时间过长，可能出现：

- `SEQ_CONFLICT`；
- 把本轮新命令误认成旧重试；
- 重放旧成功状态，但没有执行本轮真实运动。

建议 RA6M5 采用以下任一方案：

1. 增加双方会话 epoch/session id；或
2. 控制请求缓存使用与 ACK 重试窗口匹配的短 TTL，过期后删除；或
3. 在明确的链路/工作流重新初始化边界清空旧缓存。

验收时必须包含：SEQ 回绕、Qt 重启、相同 TYPE/SEQ 再次出现、旧 ACK 延迟到达等测试。

## 8. 末端视觉状态，供 RA6M5 正确处理 VISION_ERROR

Jetson 端已加入完整脸基线、单眼/局部脸拒绝、眼眉区域排除和目标稳定锁定。当前静止实测：

| 项目 | 结果 |
|---|---:|
| 完整脸有效帧 | 31 |
| 完整脸中找到真黑痣 | 31/31 |
| 被拒绝的局部脸帧 | 4/4 |
| 输出稳定候选的帧 | 22 |

当前剩余问题是机械臂运动或局部脸后，重新累计稳定目标约需 `0.45~0.9 s`。此时 Jetson 会按协议发送：

```text
VISION_ERROR dcx=0 dcy=0 valid=0
```

RA6M5 必须忽略其中的 `dcx/dcy`，关闭 P801、停止本次视觉修正并进入可恢复的视觉丢失状态；重新收到可靠的 `valid=1` 后才能继续。`valid=0` 绝不能当作 `dcx=0,dcy=0` 已对准。

开始点痣的正确顺序是：

```text
SELECTED_VIEW_READY
-> 用户点击 Qt“开始点痣”
-> Jetson 立即发送 TARGET_CTRL START
-> RA6M5 返回 COMMAND_ACK、TARGET_CTRL_ON、TARGET_READY
-> Jetson 每 100~200 ms 发送 VISION_ERROR
-> RA6M5 自动对准
-> 误差连续满足阈值后返回 ALIGN_DONE
-> 最后才允许人工按住 P000，由 RA6M5 裁决是否开启 P801
```

目标在点击“开始点痣”之前不需要已经居中；居中本身就是 RA6M5 在 `TARGET_READY` 后根据 `dcx/dcy` 完成的工作。

## 9. RA6M5 建议打印的联调日志

为了双方同时监听时能够准确定位，建议每个业务帧至少打印：

```text
RX type=... seq=... len=... crc_ok=...
ACK seq=... accepted/rejected error=...
STATE old=... new=... reason=...
SAFE_DISTANCE seq=... distance=... streak=... delta_ms=...
VISION_ERROR seq=... dcx=... dcy=... valid=...
VISION_STEP axis=... command=... before_error=... after_error=...
P801 requested=... actual=... gate_reason=...
HOLD reason=...
```

日志必须区分“收到帧”“通过解析”“接受业务命令”“开始运动”“真实到位”五个阶段，不能只打印一个笼统的 READY。

## 10. 完整无激光验收步骤

### 阶段 A：UART 链路

- Qt 独占 CH340；
- RA6M5 能看到连续 HEARTBEAT 原始字节；
- Jetson 能看到 RA6M5 `RX RAW`；
- 合法控制命令能收到同 SEQ 的 ACK；
- 连续运行至少 5 分钟无 write timeout、CRC 风暴或异常断链。

### 阶段 B：安全距离

- 进入 `MEASURE_POSITION_READY` 后才允许消费 `SAFE_DISTANCE`；
- 三个有效样本使用不同非零 SEQ；
- 无效/静默期间不复用最后值；
- 安全锁存后停止消费本轮 SAFE_DISTANCE，符合当前一次测距正式流程。

测试时同时记录三个有效样本的实际间隔，作为是否批准第 5 章增强的评审数据；在协议变更正式批准前，不把“相邻样本超过 600 ms”作为现行验收失败项。

### 阶段 C：三视图

- 严格按左、正、右三个 `CAPTURE_POINT_READY` 分别拍摄；
- 每张图只在对应到位状态之后采集；
- HOME 后再显示三视图；
- 选择视图后收到正确 point_id 的 `SELECTED_VIEW_READY`。

### 阶段 D：单轴视觉方向

- 激光物理断开；
- Y 轴一次小步后 `abs(dcy)` 下降；
- X 轴一次小步后 `abs(dcx)` 下降；
- 连续误差增大时自动 HOLD；
- `valid=0` 时不运动并关闭 P801。

### 阶段 E：完整无激光流程

连续执行至少 3 次：

```text
START_MEASURE
-> SAFE_DISTANCE 裁决
-> HOME
-> 左/正/右三视图
-> 选择一张视图
-> SELECTED_VIEW_READY
-> TARGET_CTRL START
-> TARGET_READY
-> VISION_ERROR valid=1/0 正确切换
-> ALIGN_DONE
-> FINISH_RETURN_HOME 或 ABORT_HOLD
```

三次均需确认：

- 目标 `track_id` 和坐标始终对应真实黑痣；
- 没有把 `valid=0` 当成已对准；
- `abs(dcx/dcy)` 总体收敛；
- 没有旧 ACK/READY 重放；
- P801 全程保持关闭。

只有完整无激光流程连续通过后，才能单独评审“ALIGN_DONE 后人工按住 P000”的 P801 开启测试。

## 11. 安全动作条件

不同异常必须进入协议规定的安全状态，不能把所有情况混成同一个 READY 或笼统 HOLD：

| 条件 | RA6M5 必须执行的动作 |
|---|---|
| 串口 I/O 故障或 HEARTBEAT 超时 | 立即关闭 P801、停止运动并进入安全保持状态 |
| `VISION_ERROR valid=0` 或超过 `500 ms` 没有新视觉帧 | 立即关闭 P801、停止视觉修正、回传 `VISION_LOST`，原地进入 `TARGET_WAIT_DETECT`；视觉恢复后重新对准 |
| 连续视觉修正后误差明显增大 | 建议立即关闭 P801、停止运动并 HOLD，回传明确的发散原因 |
| SAFE_DISTANCE 状态不允许、SEQ 重复或场景重置 | 不增加三样本计数，不锁存；场景变化时执行 `ABORT_HOLD` 并重新测距 |
| 急停、限位、驱动故障或状态机不一致 | 立即关闭 P801并执行相应硬件故障安全处理 |
| 未收到 `ALIGN_DONE` 前请求激光 | 拒绝开启 P801，并保留可诊断的门控原因 |
| 控制命令 SEQ 冲突或无法确认是否属于本轮 | 不执行新运动，回传明确错误；状态不确定时进入安全保持 |

## 12. 双方待办清单

### RA6M5

- [ ] 打印 UART 原始收字节和分层解析计数；
- [ ] 恢复稳定的 COMMAND_ACK 和对应业务 READY 回包；
- [ ] 修正并单轴验证 Y/X 图像误差映射；
- [ ] 增加闭环发散自动 HOLD；
- [ ] 与 Jetson 共同评审 SAFE_DISTANCE streak 最大样本间隔提案；未批准前不作为现行协议缺陷；
- [ ] 控制缓存增加 session/TTL 边界；
- [ ] 完成三次完整无激光验收。

### Jetson / Qt

- [x] `dcx/dcy` 使用正式图像坐标定义并按小端发送；
- [x] 目标无效时发送 `VISION_ERROR valid=0`；
- [x] RGB-D 无效/过期/重复样本不重发旧 SAFE_DISTANCE；
- [x] 局部脸和眼眉区域 fail-closed；
- [ ] 增加 RGB-D 分阶段点数和投影叠图诊断；
- [ ] 记录三次整机测试的目标数量、track_id、首次锁定时间、最长无效间隔及 ALIGN_DONE；
- [ ] 与 RA6M5 共同完成完整无激光验收。

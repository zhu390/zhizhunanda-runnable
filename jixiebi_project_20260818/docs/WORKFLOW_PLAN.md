# 机械臂点痣正式流程方案

> 当前业务基线：根目录 `项目演示(3).md`
>
> Jetson 侧协议镜像：根目录 `视觉协议(5).md`；跨端联调须与 RA6M5 工程根目录 `视觉协议.md` 一致
>
> 适用对象：仿真人脸模型演示，不面向真人治疗

## 1. 系统边界

### Qt / Jetson

Qt 是比赛业务主控和唯一串口所有者，负责：

- 接收用户的开始、采集、选择视图、开始点痣、完成和取消操作；
- 维护流程门控，并按 RA6M5 的 ACK、业务状态和错误推进；
- 接收顶部 Astra RGB-D 距离和末端单目视觉结果；
- 保存左、正、右三张视图；
- 在选中视图点的实时画面中自动检测、跟踪目标并计算 `dcx/dcy`；
- 使用单一后台串口会话发送心跳、控制命令、距离和视觉误差。

Jetson 不下发任意笛卡尔坐标、关节角、电机速度、文本动作配方或激光开启命令。

### RA6M5

RA6M5 是运动和 P801 的最终安全裁决者，负责：

- 固化公共测距位和左/正/右视图点；
- 执行 `soft_reset`、到位验证、视觉伺服和原地保持；
- 校验帧、处理重复控制命令并按请求 `SEQ` 回传；
- 裁决安全距离、心跳、pose、P000、限位、急停和故障状态；
- 只有全部许可同时成立时才能拉高 P801。

### 顶部 Astra RGB-D

Astra 用于公共测距和三视图画面：

- RGB 中识别 `arm_tip` 和 `face`；
- 将检测框映射到 Depth，计算末端与人脸的 3D 间距；
- 给 Qt 提供距离值、有效性和预览画面；
- 不独立打开 RA6M5 串口，也不自行决定流程已经安全。

手动 ROI 只用于测距调试和标定复核，不作为正式自动输入。

### 末端单目

末端单目在机械臂到达医生选中的视图点后，基于实时画面自动识别目标：

- 输出实时目标中心和 `dcx/dcy`；
- 未识别到可靠目标时输出无效状态；
- 不复用三视图静态图片中的目标坐标；
- 不独立打开正式串口。

## 2. 单一串口架构

```text
Astra RGB-D 距离 ─┐
                   ├─> Qt 状态机 ─> 单一 SerialSender ─> RA6M5
末端单目目标 ──────┘                    ↑
                                        └─ 后台接收和心跳
```

正式运行中只有 Qt 主进程可以打开 `/dev/ttyUSB*`。多个进程同时写一个串口会破坏帧顺序、SEQ/ACK 关联、重发去重和心跳判断，因此 `shuangmu`、单目命令行工具与协议 CLI 只能在 Qt 未运行时做隔离测试。

## 3. 正式状态流

```text
IDLE
  │ 点击“开始”
  ▼
START / WAIT_MEASURE_POSITION
  │ MEASURE_POSITION_READY
  ▼
WAIT_SAFE_DISTANCE
  ├─ UNSAFE ─> RETREAT_DONE_WAIT_RESTART
  └─ SAFE + DISTANCE_SAFE_LATCHED
          │ 点击“采集人脸图”
          ▼
CAPTURE_HOME_BEFORE
  -> CAPTURE_VIEW_1
  -> CAPTURE_VIEW_2
  -> CAPTURE_VIEW_3
  -> CAPTURE_HOME_AFTER
          │ 医生选择左/正/右视图
          ▼
WAIT_SELECTED_VIEW_READY
          │ 点击“开始点痣”
          ▼
TARGET_WAIT_READY
  -> TARGET_WAIT_DETECT
  -> TARGET_ALIGN
  -> ALIGN_DONE / 等待 P000
  -> TARGET_OUTPUT 或 TARGET_HOLD
          │ 点击“完成/返回”
          ▼
RETURN_HOME -> IDLE
```

任意阶段都可以收到 `ABORT_HOLD`、通信超时或安全错误并进入原地保持/故障保持。故障恢复不能跳回中间阶段，必须重新从“开始”和公共测距进入新一轮。

## 4. 操作与通信时序

### 4.1 链路建立

串口连接后独立后台任务每 `100~200 ms` 发送 `HEARTBEAT`。心跳不代表距离安全、视觉对准或 P000 许可；只有合法心跳能刷新 RA6M5 的 `600 ms` 全局超时。

### 4.2 “开始”与公共测距

1. 点击“开始”时 Qt 发送 `WORKFLOW_CTRL START_MEASURE (action=0x01)`；当前正式启动不得使用 `--continuous-safe-distance`。
2. RA6M5 先回同 `SEQ` 的 `COMMAND_ACK`，再回 `START_ACCEPTED`。
3. RA6M5 执行 `soft_reset`，PASS 后前往公共测距位。
4. 收到 `MEASURE_POSITION_READY` 后，Qt 才允许发送 Astra 的 `SAFE_DISTANCE`。
5. Qt 清空到位前的检测状态，只接受到位后的新 RGB 与新 Depth 样本；每 `100~200 ms` 最多消费一次新采样。过期、不同步、质量不足或重复样本时不发送，不复用上一距离，短暂无效也不重置保守滤波器。
6. 当前 RA6M5 使用 3 个不同非零 `SEQ` 的有效安全样本完成锁存；`SAFE` 后还要等待 `DISTANCE_SAFE_LATCHED`，`UNSAFE` 后等待撤离完成并由用户重新开始。

收到 `SAFE`/`UNSAFE` 裁决后，Qt 停止发送 `SAFE_DISTANCE`；HOME、三视图和点痣阶段也不再发送。锁存后连续刷新、`WORKFLOW_CTRL action=0x04` 和 `SAFE_DISTANCE_TIMEOUT (0x10)` 都属于未来协议变更提案，当前 RA6M5 未实现，不能用于正式流程。

### 4.3 三视图采集

只有安全距离已锁存时，“采集人脸图”才可用：

| 顺序 | 命令 | 完成状态 | Qt 动作 |
|---:|---|---|---|
| 1 | `CAPTURE_CTRL HOME, 0` | `CAPTURE_HOME_READY` | 进入采集画面 |
| 2 | `CAPTURE_CTRL GOTO_VIEW, 1` | `CAPTURE_POINT_READY, 1` | 保存左视图 |
| 3 | `CAPTURE_CTRL GOTO_VIEW, 2` | `CAPTURE_POINT_READY, 2` | 保存正视图 |
| 4 | `CAPTURE_CTRL GOTO_VIEW, 3` | `CAPTURE_POINT_READY, 3` | 保存右视图 |
| 5 | `CAPTURE_CTRL HOME, 0` | `CAPTURE_HOME_READY` | 展示三张图 |

每个控制命令都先等同 `SEQ` 的 ACK，再等业务完成状态。RA6M5 不自动前往下一点；Qt 只有成功保存当前图片后才发送下一条命令。任何错误都中止采集，禁止用固定延时假定到位。

### 4.4 选择视图

医生只选择左、正、右三张视图中的一张，不在静态图片上选择具体痣。Qt 发送 `CAPTURE_CTRL SELECT_VIEW, point_id`，收到 `SELECTED_VIEW_READY` 后才启动该位置的实时单目目标检测。

三视图图片用于选择机械视角，不是定靶坐标来源。保存图片时的坐标、候选编号或检测结果不得直接发送给 RA6M5。

### 4.5 开始点痣与实时视觉

1. 选中视图已到位、距离锁存仍有效且没有故障时，用户点击“开始点痣”。
2. Qt 发送 `TARGET_CTRL START` 并等待 ACK。
3. 收到 `TARGET_CTRL_ON` 和 `TARGET_READY` 后，Qt 才开始每 `100~200 ms` 发送 `VISION_ERROR`。
4. 自动检测到可靠目标时，将首个连续稳定目标锁为本次定靶身份；后续必须同时通过痣半径位移门、尺度门、近邻歧义门和已排除候选检查，才能发送实时 `dcx/dcy, valid=1`。
5. 原目标未检测到、身份不匹配或置信度不足时发送 `valid=0`；不得切换到另一候选、发送旧坐标或用零误差冒充已对准。
6. 收到 `ALIGN_DONE` 后，Qt 提示操作者按住 P000。
7. P801 输出期间仍继续发送视觉状态；误差超限、视觉丢失、心跳失效或其他联锁失效时，RA6M5 立即关闭输出。

医生选择视图后由 Jetson 自动追踪目标。当前正式流程不支持医生在图上指定某一颗痣，也不支持顺序处理多个静态候选。

### 4.6 完成与取消

- “完成/返回”发送 `WORKFLOW_CTRL FINISH_RETURN_HOME`。RA6M5 关定靶和 P801，执行 `soft_reset`；只有 final verify PASS 后回 `RETURN_HOME_DONE` 并进入 IDLE。
- “取消/安全停止”发送 `WORKFLOW_CTRL ABORT_HOLD`。RA6M5 立即关 P801、中止运动并原地保持，不自动 HOME。
- 硬件急停独立于上述两个软件动作，Qt 只能显示说明和状态，不能替代它。

## 5. 正式协议门控

正式消息类型：

```text
Jetson -> RA6M5
  HEARTBEAT       TYPE=0x01
  TARGET_CTRL     TYPE=0x02
  VISION_ERROR    TYPE=0x03
  CAPTURE_CTRL    TYPE=0x04
  SAFE_DISTANCE   TYPE=0x05
  WORKFLOW_CTRL   TYPE=0x06

RA6M5 -> Jetson
  STATUS          TYPE=0x81
  ERROR           TYPE=0xFE
```

`WORKFLOW_CTRL`、`CAPTURE_CTRL` 和 `TARGET_CTRL` 的规则：

- 新命令分配新 `SEQ`；
- Jetson 只分配 `0x01~0xFF`，回绕时跳过为 RA6M5 异步故障保留的 `0x00`；
- `300 ms` 未收到 ACK 时重发完全相同的原始字节，最多三次；
- RA6M5 对完全相同的重复命令只重发已有状态，不重复运动；
- 相同 `TYPE+SEQ` 但 payload 不同返回 `SEQ_CONFLICT`；
- ACK 和业务完成状态都使用原请求 `SEQ`；
- 只有 `COMMAND_ACK value=1,error=0` 后才接受后续业务状态；
- `value=0`、非零 `error_code`、`ERROR` 或超时都不能当作对应正常完成状态推进；定靶阶段 `VISION_LOST (0x0A)` 是流程内可恢复状态，只允许关输出、原地等待并继续视觉上报，不能推进到对准或输出。

旧 `FF/AA/CC` 帧没有完整版本、CRC、SEQ 和业务握手，禁止用于正式流程。

## 6. Qt 按钮启用原则

| 控件 | 启用条件 |
|---|---|
| “开始” | 串口可用、没有待完成控制命令；故障处理后允许从头重启 |
| “采集人脸图” | 当前工作流有效、距离已锁存、没有故障或运动中的控制命令 |
| 视图选择 | 三张图完成且采集后 HOME_READY、距离锁存有效 |
| “开始点痣” | `SELECTED_VIEW_READY`、距离锁存有效、没有故障或待完成控制命令 |
| “完成/返回” | 工作流活动且串口可用 |
| “取消/安全停止” | 工作流活动或自动动作正在进行 |

按钮变为可点击不等于 RA6M5 必须执行；RA6M5 仍需再次检查全部安全门并可拒绝请求。

## 7. 数据有效性规则

- 深度距离只在 `MEASURE_POSITION_READY` 后使用；收到裁决后停止发送。
- 深度、RGB 检测框或时间戳无效时不得生成有效距离。距离滤波不得把当前下降后的净距向上拖延。
- 正式距离默认 fail-closed；只有 V4L RGB/SDK Depth 的设备配对、严格分辨率、畸变、外参、最坏采集时序及缺深/遮挡通过真机验收后，才能启用 `--rgbd-hardware-pair-verified`。
- RGB 与 Depth 每个物理样本最多产生一次 `valid=1`；框整体或中心区域有效深度比例不足时必须无效。
- 静态三视图不能提供定靶视觉误差。
- `VISION_ERROR` 只在本次 `TARGET_CTRL START` 的 `TARGET_READY` 后发送。
- 没有新视觉帧时发送 `valid=0` 或由接收端判定超时，绝不延长旧结果寿命。
- 状态必须匹配当前请求/定靶会话 `SEQ`，旧会话迟到帧不得改变当前界面状态。

## 8. 验收清单

1. Qt 单独占用串口，页面切换时心跳仍连续，合法心跳最大间隔小于 `500 ms`。
2. 未收到 `MEASURE_POSITION_READY` 时，不发送有效距离；收到 SAFE/UNSAFE 后停止发送。
3. 三视图严格按两次 HOME 和三个点位 READY 顺序执行，每张照片只保存一次。
4. 医生只选择视图；选中视图到位后实时单目重新检测目标。
5. 未收到 `TARGET_READY` 时无视觉帧；目标丢失时可观测到 `valid=0`。
6. 输出期间视觉丢失、误差超限、P000 松开、心跳丢失、限位或急停均能立即关闭 P801。
7. ACK 丢失时保持原 SEQ/原字节重发；重复命令不会重复运动。
8. “完成/返回”最终回 HOME；“取消/安全停止”原地保持，两者行为可区分。
9. `soft_reset final verify FAIL`、错误状态和超时不会被 Qt 当成成功到位。
10. 患者或场景明显变化后清除本轮许可，并要求重新从公共测距开始。
11. 控制业务超时会请求 `ABORT_HOLD`；安全停止未确认时停止心跳并禁止未知状态下重新开始。
12. 多候选排序改变不会切换治疗目标；锁定目标丢失时只能看到 `valid=0`。

## 9. 当前实现与固件依赖

Jetson 可以先按正式协议实现界面、解析、后台心跳、ACK 重发和状态门控。RA6M5 侧仍须确认 `WORKFLOW_CTRL`、测距锁存、顶层安全状态机、按请求 SEQ 回传、重复命令去重、视觉丢失保持、完成与取消分离等接口已经完成并上板验证。

在这些固件依赖未确认前，只能做模拟器或台架联调，不能把文档完成视为整机安全闭环已经完成。

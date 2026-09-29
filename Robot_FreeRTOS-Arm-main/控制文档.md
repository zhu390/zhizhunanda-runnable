 

# Robot_FreeRTOS-Arm 机械臂控制文档

> 本文档以当前工作区代码为事实来源，记录机械臂控制架构、已完成改动、剩余任务和验证状态。
> 更新时间：2026-07-05

---

## 1. 工程范围

当前主要启动路径包括机械臂控制任务、P801 激光初始化和视觉定靶线程：

```text
robot_thread_entry
    -> LED_Init
    -> BSP_Laser_Init
    -> BSP_UART_Init
    -> BSP_CAN_Init
    -> robot_init
        -> robot_control_task
        -> robot_cmd_service
vision_service_thread_entry
    -> jetson_vision_init
    -> robot_target_init
    -> 10 ms 周期运行 jetson_vision_process + robot_target_step
```

机械臂运动学控制 J1~J5，J6 由夹爪模块独立控制。`vision_service_thread` 当前负责 Jetson 视觉误差解析和激光定靶状态机，不再驱动旧视觉采摘流程。

工程中已移除旧 `K230_cmd` 和 `robot-take` 编译项，改为 `jetson_vision` 和 `robot_target`。底盘、DHT11、MQTT、W800 等外围模块仍保留，因此 4.1 工程裁剪仍是部分完成。

---

## 2. 当前控制架构

### 2.1 事件与任务

UART/MQTT 命令进入 `g_robot.cmd_queue`，由 `robot_cmd_service` 解析后发送控制事件；`robot_control_task` 串行处理机械臂事件。

主要事件包括：

- 单关节相对/绝对旋转
- 自动笛卡尔运动
- 时间函数轨迹
- soft reset / hard reset
- remote 连续控制
- J1~J5 批量角度读取

AUTO 在当前位姿无效时会被拒绝。反馈读取异常但仍可继续时，系统使用 `ROBOT_STATUS_POSE_DEGRADED` 标记退化状态。

### 2.2 AUTO 运动流程

```text
目标位移
    -> 10 ms 正弦速度 S 曲线笛卡尔插值
    -> 每个路径点进行逆运动学求解
    -> IK 分支锁定选择连续解
    -> 每 10 ms 更新一个关节目标节点
    -> 每个路径点执行一次前馈 + P 控制
    -> 终点保持 10 个控制周期，前馈清零并由 P 项收敛
    -> J1~J5 非阻塞发送速度命令
    -> 结束后批量停止 J1~J5
```

路径和逆解结果使用静态缓冲区：

```c
#define ROBOT_MAX_PATH_SIZE (800)

static struct position s_path_buf[ROBOT_MAX_PATH_SIZE];
static float s_result_buf[ROBOT_MAX_PATH_SIZE * ROBOT_MAX_JOINT_NUM];
```

路径最多保存 800 点；S 曲线达到容量上限时会把最后一点强制替换为目标点。AUTO 和时间函数运动不再在运行时分配路径/逆解内存。

### 2.3 前馈 + P 控制器

控制周期为 10 ms（100 Hz），控制器结构为：

```c
error = shortest_angle_diff(current_angle, target_angle);
velocity = ROBOT_JOINT_FF_GAIN[joint] * feedforward
         + ROBOT_JOINT_KP[joint] * error;
velocity = clamp(velocity, -200.0f, 200.0f);
```

各关节 Kp（单位 `1/s`）：

```c
{0.65f, 3.00f, 3.00f, 2.00f, 2.30f, 10.0f}
```

控制器无积分项和微分项，没有原 PID 的积分饱和。速度前馈使用独立系数 `ROBOT_JOINT_FF_GAIN[]` 做逐关节缩放，参数见第 6 章。

前馈时间基准按目标更新周期计算：

- AUTO：S 曲线每 10 ms 生成一个目标点，相邻 IK 节点角差除以 10 ms，再按 `ROBOT_JOINT_FF_GAIN[]` 做逐关节缩放。
- 时间函数：同样按 10 ms 生成目标点，但路径由时间函数直接给出，不经过 S 曲线规划器；速度前馈同样进入 `robot_pid_one_period()` 并经过 `ROBOT_JOINT_FF_GAIN[]` 缩放。
- remote：相邻目标角差除以 50 ms；首次更新前馈为 0，避免进入模式时产生冲击；当前也复用 `robot_pid_one_period()`，因此前馈同样经过 `ROBOT_JOINT_FF_GAIN[]` 缩放。
- 角差通过 `robot_angle_diff()` 归一化到 `[-180, 180]`，避免跨越 0/360 度时方向错误。

`ave_error` 使用实际控制采样次数作为分母，表示各关节的平均绝对跟踪误差，不再按路径点数错误放大。

### 2.4 CAN 反馈与控制输出

每个控制周期采用并行反馈读取：

1. 清除 `motor_rx_flag` 和残留 RX 信号量。
2. 连续发送 J1~J5 位置读取请求。
3. CAN ISR 按电机地址将 `0x36` 回包写入独立缓存并置位。
4. `BSP_CAN_WaitAllMotors()` 统一等待，当前批量等待上限为 6 ms。
5. 解析已收到的回包并更新关节角度。

速度控制使用 `robot_joint_velocity_nowait()`：发送 `0xF6` 后立即返回，不逐关节等待应答。CAN 总线仍由互斥锁保护，避免与夹爪、单关节读取等操作互相干扰。

停止操作使用 `robot_joint_stop_all()`：连续发送所有 stop 命令，再通过 `motor_stop_flag` 统一等待 `0xFE` 回包。

`BSP_CAN_DrainRx()` 当前负责清空 RX 信号量，避免残留广播帧或旧通知消耗下一次请求的应答窗口。

### 2.5 复位与位姿有效性

soft reset 当前流程：

1. 读取并映射各关节当前角度。
2. 发送位置模式回 HOME 命令。
3. 根据初始角度误差估算行进时间，并增加 300 ms 余量。
4. 每 40 ms 轮询反馈，最少等待 320 ms，最多等待 3 s。
5. 校验 HOME；失败时执行最多 3 个 refine pass。
6. 每个关节每个 pass 最多微调 3 轮，容差为 1.0 度。
7. 最终校验通过后设置 HOME 位姿；失败则清除 `POSE_VALID` 并进入退化状态。

`robot_angle_map()` 对距离 min/max 不超过 0.1 度的读数直接夹紧到边界，避免边界噪声被错误卷绕到另一圈。soft reset 映射还提供额外 1 度边界容差。

当前 J2/J3/J5 的软件限位相对 HOME 留有 2 度余量：J2 下限 88 度、J3 下限 -92 度、J5 上限 92 度。

### 2.6 IK 连续性

逆运动学已经实现分支锁定：

- 使用加权平方角度距离选择最接近当前姿态的有效解。
- 权重为 `{20, 15, 12, 5, 3, 1}`。
- 对非上一帧分支乘以 `IK_SWITCH_PENALTY = 10`，降低路径中途跳解概率。
- 新 AUTO、时间函数、remote 和复位完成后调用 `robot_kinematics_reset_branch_lock()`，首帧重新从实际姿态选择最近解。
- J6 不参与整臂轨迹驱动，AUTO/时间函数保持进入运动时的 J6 角度。

当前只处理了精确奇异点的基础分支，尚未实现 4.7 的奇异区连续阻尼。

---

## 3. 关节参数


| 关节 |   HOME |  软件限位 | 减速比 | 正方向 | 回零方向 |
| ------ | -------: | ----------: | -------: | -------- | ---------- |
| J1   |  90 度 |  0~360 度 |  50.00 | CCW    | 负方向   |
| J2   |  90 度 | 88~180 度 |  99.99 | CW     | 负方向   |
| J3   | -90 度 | -92~90 度 |  50.89 | CW     | 负方向   |
| J4   |   0 度 | -90~90 度 |  51.00 | CW     | 负方向   |
| J5   |  90 度 |   0~92 度 |  26.85 | CCW    | 正方向   |
| J6   |   0 度 |  0~360 度 |  51.00 | CW     | 负方向   |

J4 使用 `JOINT_LIMIT_6`，J6 使用 `JOINT_LIMIT_4`，这是当前硬件接线映射，不按名称交换。

---

## 4. 核心控制实现

本章只记录当前代码已经采用的控制方法。


| 编号 | 模块                 | 当前状态       | 核心作用                                                               |
| ------ | ---------------------- | ---------------- | ------------------------------------------------------------------------ |
| 4.1  | 工程裁剪             | 部分完成       | K230/robot-take 旧采摘链路已替换为 Jetson/robot_target，外围模块仍保留 |
| 4.2  | 初始化保护           | 已完成         | FSP 外设 open 失败会被正确识别                                         |
| 4.3  | 前馈 + P 控制        | 已完成         | 用目标角速度前馈和比例误差共同生成关节速度                             |
| 4.4  | 100 Hz 控制周期      | 已完成         | AUTO 路径点和控制周期按 10 ms 对齐                                     |
| 4.5  | CAN 并行反馈         | 已完成         | J1~J5 批量读取反馈，速度命令非阻塞发送                                 |
| 4.6  | IK 分支锁定          | 已完成         | 优先选择接近当前姿态的逆解，降低跳解概率                               |
| 4.8  | S 曲线轨迹规划       | 已完成         | 末端笛卡尔目标按正弦速度剖面生成平滑路径                               |
| 4.9  | 到位确认             | 已完成         | 单关节和 AUTO 结束后都读取真实角度确认结果                             |
| 4.10 | Jetson 视觉通信      | 已完成         | 解析视觉误差帧和状态机启停帧，回传 RA6M5 状态                          |
| 4.11 | 激光定靶状态机       | 已完成         | 视觉闭环对准、P000 安全许可、P801 激光输出                             |
| 4.12 | 末端精度与分辨率提升 | 已完成当前实现 | 两段式视觉步长、AUTO 防排队、输出前二次确认                            |

### 4.3 前馈 + P 控制

**是什么**：每 10 ms 读取一次关节角度，用目标角速度前馈和当前角度误差生成速度命令。

**好处**：前馈负责跟上轨迹速度，P 项负责修正实际误差；相比统一 Kp 的纯 P 控制，AUTO 跟随误差更容易按关节单独调节。

**核心方法**：相邻 IK 节点角差除以 10 ms 得到 `feedforward`，再乘以每个关节自己的 `ROBOT_JOINT_FF_GAIN[]`。

**主要代码**：`APP/robot.c -> robot_pid_one_period()`

```c
float error = robot_angle_diff(g_robot.joints[j].current_angle, target_angle[j]);
float v = ROBOT_JOINT_FF_GAIN[j] * feedforward[j]
        + ROBOT_JOINT_KP[j] * error;
if (v >  ROBOT_FF_OUTPUT_LIMIT) v =  ROBOT_FF_OUTPUT_LIMIT;
if (v < -ROBOT_FF_OUTPUT_LIMIT) v = -ROBOT_FF_OUTPUT_LIMIT;
robot_joint_velocity_nowait(j, dir, fabsf(v), ROBOT_JOINT_DEFAULT_ACCELERATION);
```

### 4.5 CAN 并行反馈

**是什么**：每个控制周期批量读取 J1~J5 的位置反馈，而不是一个关节读完再读下一个。

**好处**：反馈等待时间更短，控制周期更稳定，减少串行 CAN 等待造成的轨迹滞后。

**核心方法**：连续发送多个读取请求，由 CAN ISR 按电机地址分发回包，再统一等待 `motor_rx_flag`。

**主要代码**：`USER_BSP/bsp_can.c -> BSP_CAN_WaitAllMotors()`，`APP/robot.c -> robot_update_all_angles()`

### 4.6 IK 分支锁定

**是什么**：逆解存在多个可行姿态时，优先选择最接近当前关节角的解。

**好处**：减少 AUTO 轨迹中途突然换分支造成的关节跳变。

**核心方法**：对候选 IK 解计算加权角度距离，并对切换分支增加惩罚。

**主要代码**：`Middle/robot_kinematics.c -> robot_kinematics_get_optimal_result()`

### 4.8 S 曲线轨迹规划

**是什么**：AUTO 不再用固定间距线性插补，而是按正弦速度剖面生成笛卡尔路径点。

**好处**：起步和停止更平滑，避免速度突变带来的机械抖动；长路径有加速、匀速、减速，短路径自动降低峰值速度。

**核心方法**：以 `ROBOT_PID_PERIOD = 10 ms` 为时间步长积分速度，生成变距路径点；最后一个点强制设为精确目标。

**主要代码**：`APP/robot.c -> robot_path_interpolation_scurve()`

```c
const float dt = ROBOT_PID_PERIOD / 1000.0f;
v = (SCURVE_VMAX * 0.5f) *
    (sinf(SCURVE_OMEGA * t - 3.14159265f * 0.5f) + 1.0f);
s += v * dt;
path[n].x = start.x + ux * s;
path[n].y = start.y + uy * s;
path[n].z = start.z + uz * s;
```

当前关键参数：`SCURVE_VMAX = 180 mm/s`，`SCURVE_AMAX = 300 mm/s²`，`ROBOT_MAX_PATH_SIZE = 800`。

### 4.9 位置模式到位确认

**是什么**：位置命令执行完后，不直接相信命令目标，而是读取真实关节角度确认是否到位。

**好处**：避免 `cur_pos` 被写成一个机械臂实际没到达的位置，减少连续 AUTO 时误差累计。

**核心方法**：单关节命令用 `robot_joint_wait_target()`；AUTO/time_func 用 `robot_auto_final_confirm()` 批量确认最终 IK 角度。

**主要代码**：`APP/robot.c -> robot_joint_wait_target()`、`robot_auto_final_confirm()`、`robot_auto_move_interpolation()`

```c
if (robot_auto_final_confirm(result, path_size, ROBOT_JOINT_POS_CONFIRM_TOL_DEG)) {
    g_robot.cur_pos.x = target_pos->x;
    g_robot.cur_pos.y = target_pos->y;
    g_robot.cur_pos.z = target_pos->z;
    ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_VALID);
} else {
    ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_VALID);
    ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_POSE_DEGRADED);
}
```

当前位置确认参数：`3.0°` 容差、连续 `2` 次命中、AUTO 终点确认超时 `800 ms`。

轨迹结束前还保留固定末端稳定段：`ROBOT_PID_SETTLE_PERIODS=10`，按 `10 ms` 控制周期共 `100ms`。这是一段固定收敛窗口，不是自适应到位超时；期间前馈清零，只用 P 项保持终点目标。

### 4.10 Jetson 视觉通信

**是什么**：RA6M5 通过 Jetson UART 接收视觉误差和定靶状态机启停命令，并向 Jetson 回传状态。

**好处**：Jetson 只需要发送像素误差和启停指令，RA6M5 负责机械臂运动、安全许可和激光输出。

**当前实现**：同一个 UART 已具备 `A5 5A + CRC16/MODBUS` 统一协议解析基础，同时保留旧 `FF/AA/CC` 帧供 PC 调试兼容。比赛正式链路只使用统一协议，旧帧不参与第 9 章流程。

**主要代码**：`Middle/jetson_vision.c/.h`

```text
Jetson -> RA6M5 视觉误差：FF 05 03 dcx_lo dcx_hi dcy_lo dcy_hi checksum FE
Jetson -> RA6M5 启动状态机：AA 01 01 BB
Jetson -> RA6M5 关闭状态机：AA 01 00 BB
RA6M5 -> Jetson 状态回传：CC func value DD
```

常用回传：`CC 04 01 DD` 状态机已启用，`CC 01 01 DD` 预定位完成，`CC 02 01 DD` 对准完成，`CC 03 01/00 DD` 输出开/关。

**比赛正式协议**：视觉误差、状态机启停、流程控制、心跳和状态回传统一使用一种二进制帧格式，不再让 Jetson 端维护 `FF/AA/CC` 三套帧头帧尾。

推荐帧格式：

```text
SOF0 SOF1 VER TYPE SEQ LEN PAYLOAD... CRC16_LO CRC16_HI
A5   5A   01  xx   xx  nn  ...        xx       xx
```

字段含义：


| 字段        | 含义                                                      |
| ------------- | ----------------------------------------------------------- |
| `SOF0/SOF1` | 固定帧头`A5 5A`，用于丢字节后重新同步                     |
| `VER`       | 协议版本，第一版固定`0x01`                                |
| `TYPE`      | 消息类型，区分心跳、视觉误差、启停控制和状态回传          |
| `SEQ`       | 序号，Jetson 每发一帧递增，RA6M5 回传时带回，方便判断丢帧 |
| `LEN`       | `PAYLOAD` 字节数                                          |
| `PAYLOAD`   | 不同消息类型的数据                                        |
| `CRC16`     | 从`VER` 到 `PAYLOAD` 的 CRC16，错误帧直接丢弃             |

以下消息类型表是早期草案，不作为 Jetson 对接表；正式值以第 9 章和 `视觉协议.md` 为准：


|   TYPE | 方向            | 名称         | PAYLOAD                                            |
| -------: | ----------------- | -------------- | ---------------------------------------------------- |
| `0x01` | Jetson -> RA6M5 | HEARTBEAT    | `uint32 jetson_tick_ms`                            |
| `0x02` | Jetson -> RA6M5 | TARGET_CTRL  | `uint8 enable`，`1` 启动，`0` 关闭                 |
| `0x03` | Jetson -> RA6M5 | VISION_ERROR | `int16 dcx` + `int16 dcy` + `uint8 valid`          |
| `0x81` | RA6M5 -> Jetson | STATUS       | `uint8 event` + `uint8 value` + `uint8 error_code` |
| `0xFE` | RA6M5 -> Jetson | ERROR        | `uint8 error_code`                                 |

`STATUS.event` 当前沿用现有语义：`1` 预定位完成，`2` 对准完成，`3` 输出开关，`4` 状态机启停结果，`5` 心跳响应。项目演示流程需要扩展时，保留 `0x10` 表示三视图拍摄点位到达，`0x11` 表示三视图拍摄完成，`0x12` 表示已进入 Jetson 选中的定靶预开始点。

心跳建议由 Jetson 每 `100~200 ms` 发送一次。RA6M5 超过 `600 ms` 没收到心跳时，判定 Jetson 通信失效，执行：停止视觉伺服、关闭 P801、退出或冻结定靶状态机，并回传 `ERROR`。心跳不能替代 P000 发射许可，它只用于确认上位视觉链路仍然在线。

文本分隔协议如 `502,task,#` 便于人工调试，但正式 Jetson 通信用二进制长度 + CRC 更合适：解析边界清晰、抗串口噪声更强、不会因为逗号/换行丢失导致字段错位。上位机调试界面可以继续显示解析后的文本，不要求串口本身改成文本协议。

迁移方式建议分两步：第一步在 `jetson_vision.c` 中兼容新旧协议；第二步 Jetson 稳定切到新协议后，再删除旧 `FF/AA/CC` 解析。

#### 4.10.1 项目一键演示流程规划（历史草案）

本节保留用于说明三视图功能的来源。比赛最终控制流程、消息语义和安全状态机以第 9、10 章及 `视觉协议.md` 为准；特别是第一次 Qt“开始”只进入公共测距位，医生只选择一张视图，不点击具体痣点。

**目标**：把人脸三视图拍摄、Jetson 选择疑似黑痣图像、机械臂移动到对应预开始点、视觉伺服定靶和 P801 输出串成一个完整比赛演示流程。

**原则**：Jetson 只下发点位序号和视觉误差，不直接下发任意机械臂坐标。机械臂侧只执行已保存、已验证的左/正/右三组拍摄点位配方，降低误发坐标导致自碰撞或越界的风险。

项目演示状态链路：

```text
PROJECT_IDLE
  -> CAPTURE_LEFT
  -> CAPTURE_FRONT
  -> CAPTURE_RIGHT
  -> RETURN_HOME
  -> WAIT_JETSON_SELECT
  -> MOVE_SELECTED_PRESTART
  -> TARGET_ENABLE
  -> TARGET_READY
  -> TARGET_ALIGN
  -> TARGET_CONFIRM
  -> TARGET_OUTPUT
  -> TARGET_DONE / TARGET_RECOVER
```

三视图拍摄阶段复用上位机“拍摄点位”页的三组配方：

```text
左视图：soft_reset -> auto 45 -130 -15  -> abs_rotate 0 85  -> read_all
正视图：soft_reset -> auto 0 -130 -15   -> abs_rotate 0 90  -> read_all
右视图：soft_reset -> auto -45 -130 -15 -> abs_rotate 0 105 -> read_all
```

流程定义：


| 阶段         | 动作                                                                                           | 通过条件                              |
| -------------- | ------------------------------------------------------------------------------------------------ | --------------------------------------- |
| 三视图拍摄   | 依次执行左/正/右拍摄点位配方                                                                   | 每个点位到位后 Jetson QT 保存对应图像 |
| 回 HOME      | 三张图拍完后执行`soft_reset`                                                                   | `soft reset final verify PASS`        |
| Jetson 选择  | Jetson QT 同屏显示左/正/右三张图，医生只选择其中一张视图；目标痣由 Jetson 自动识别和跟踪       | Jetson 得到`point_id=1/2/3`           |
| 进入预开始点 | Jetson 下发`point_id`，机械臂移动到对应左/正/右预开始点                                        | 回传`TARGET_PRESTART_READY`           |
| 定靶输出     | 复用当前一键定靶：心跳、`TARGET_CTRL enable`、READY、视觉误差、ALIGN_DONE、P000、OUTPUT_ON/OFF | P000 松开或视觉超时后 P801 关闭       |

推荐协议复用 `TYPE=0x04 CAPTURE_CTRL`：


| action | point_id | 含义                                                |
| -------: | ---------: | ----------------------------------------------------- |
| `0x01` |  `1/2/3` | 移动到左/正/右拍摄点位                              |
| `0x02` |      `0` | 三视图拍摄完成，机械臂回 HOME                       |
| `0x03` |  `1/2/3` | Jetson 选择黑痣所在视图，机械臂进入对应定靶预开始点 |

推荐状态回传：


|  event |   value | 含义                                  |
| -------: | --------: | --------------------------------------- |
| `0x10` | `1/2/3` | 已到达左/正/右拍摄点位，Jetson 可拍照 |
| `0x11` |     `1` | 三视图拍摄完成，已回 HOME             |
| `0x12` | `1/2/3` | 已进入 Jetson 选中的定靶预开始点      |

安全约束：

- 三视图拍摄和回 HOME 阶段必须保持 P801 关闭。
- `point_id` 只允许 `1/2/3`，非法值直接丢弃并回传 `ERROR`。
- 进入定靶预开始点后，激光仍必须满足现有条件：状态机启用、视觉稳定、P000 按住、无心跳超时、无限位。
- `TARGET_CTRL disable`、心跳超时、限位或 P000 松开时，仍按现有规则关闭 P801 并退出输出。

### 4.11 激光定靶状态机

**是什么**：一个 10 ms 周期运行的安全状态机，负责预定位、视觉对准、输出确认和激光开关。

**好处**：激光不会被单帧视觉误差或串口误触发直接打开；必须视觉稳定、状态机启用、P000 按住后才允许 P801 输出。

**核心方法**：状态按 `INIT -> PRE_POSITION -> WAIT_DETECT -> ALIGN -> CONFIRM -> OUTPUT -> DONE/RECOVER` 流转。`PRE_POSITION` 只负责粗定位到视觉可接管区域，AUTO 结束后由 `robot_auto_result_t` 回传 `OK/FAILED/ABORTED`；只有 `OK` 才进入 `WAIT_DETECT`。`WAIT_DETECT` 等待视觉误差期间每 `300 ms` 周期回传 READY，避免单帧状态回传丢失导致 Jetson 或上位机误判未就绪。最后对准由视觉伺服继续闭环完成，非输出状态持续关激光。

**主要代码**：`APP/robot_target.c/.h`，`src/vision_service_thread_entry.c`

```c
obs.fire_button = BSP_Laser_FireKey_IsPressed();
robot_target_step(&obs);
```

```c
if ((s_target.confirm_stable_count >= TARGET_CONFIRM_STABLE_COUNT) && fire_allowed) {
    jetson_send_status_u8(RA6_TO_JETSON_OUTPUT, 1u);
    enter_state(TARGET_OUTPUT, now);
}
```

P801 为激光输出，高电平开；P000 为发射许可 KEY，默认按下低电平。P003 为“视觉已确认、可以按 P000”指示灯，P503 为“P801 已实际输出”确认灯，两者均为低电平点亮。P003 只在顶层状态为 `FLOW_TARGET_ACTIVE` 且收到 `ALIGN_DONE` 后点亮；P503 只跟随 `robot_workflow` 的真实 `laser_active` 状态。视觉丢失、误差超限、P000 松开、状态机退出或故障时相应指示灯熄灭。

### 4.12 末端精度与分辨率提升

**是什么**：视觉精瞄阶段采用粗/精两段步长，并在输出前增加二次确认。

**好处**：误差大时移动不拖沓，接近目标后小步逼近；输出前再确认多帧视觉数据，降低误触发概率。

**核心方法**：`TARGET_ALIGN` 根据像素误差切换步长；发送小步 AUTO 前检查 `robot_is_auto_busy()`；进入 `TARGET_CONFIRM` 后继续累计新视觉帧，达到阈值才允许输出。

**主要代码**：`APP/robot_target.c/.h`，`APP/robot.c -> robot_is_auto_busy()`

```c
float err_px = fmaxf(fabsf((float)s_target.dcx), fabsf((float)s_target.dcy));
float step_limit = (err_px <= TARGET_ALIGN_TOL_PX_COARSE) ?
                   TARGET_MAX_STEP_MM_FINE : TARGET_MAX_STEP_MM;
float dx = clampf_local(s_target.dcx * TARGET_KX_MM_PER_PX,
                        -step_limit, step_limit);
float dz = clampf_local(s_target.dcy * TARGET_KY_MM_PER_PX,
                        -step_limit, step_limit);
```

```c
if (robot_is_auto_busy()) {
    break;
}
```

当前参数：粗步长 `2.5 mm`，精步长 `0.8 mm`，粗/精切换阈值 `15 px`，最终对准阈值 `5 px`，对准确认 `3` 帧，输出前二次确认 `2` 帧，总计 `3+2=5` 个新有效帧。

## 6. 关键参数速查


| 参数              |                                 当前值 | 含义                          |
| ------------------- | ---------------------------------------: | ------------------------------- |
| 控制周期          |                                `10 ms` | 100 Hz 控制                   |
| AUTO 最大路径点数 |                                  `800` | 约 8 s 轨迹缓存上限           |
| S 曲线最大速度    |                             `180 mm/s` | `SCURVE_VMAX`                 |
| S 曲线最大加速度  |                           `300 mm/s²` | `SCURVE_AMAX`                 |
| 前馈 + P 输出限幅 |                           `±200 °/s` | `ROBOT_FF_OUTPUT_LIMIT`       |
| Kp                | `{0.65, 3.00, 3.00, 2.00, 2.30, 10.0}` | J1~J6 比例增益                |
| FF_GAIN           |    `{1.0, 0.65, 0.55, 1.0, 0.60, 1.0}` | J1~J6 前馈缩放                |
| 末端稳定周期      |                                   `10` | 前馈清零后约 100 ms P 收敛    |
| 位置确认容差      |                                `3.0°` | 关节到位确认                  |
| AUTO 终点确认超时 |                               `800 ms` | `robot_auto_final_confirm()`  |
| 定靶预定位        |                       `<0, -130, -15>` | `TARGET_PRE_X/Y/Z`            |
| 视觉修正比例      |                 `X=0.38, Z=0.35 mm/px` | 像素误差到机械臂小步          |
| 视觉粗步长        |                               `2.5 mm` | 大误差阶段                    |
| 视觉精步长        |                               `0.8 mm` | 小误差阶段                    |
| 粗/精切换阈值     |                                `15 px` | `TARGET_ALIGN_TOL_PX_COARSE`  |
| 最终对准阈值      |                                 `5 px` | `TARGET_ALIGN_TOL_PX`         |
| 对准确认          |                                 `3` 帧 | `TARGET_ALIGN_STABLE_COUNT`   |
| 输出前确认        |                                 `2` 帧 | `TARGET_CONFIRM_STABLE_COUNT` |
| 视觉有效期        |                               `500 ms` | 超时关闭输出                  |
| 激光输出          |                                 `P801` | 高电平开                      |
| 发射许可          |                                 `P000` | 按下低电平                    |
| 可按键提示灯      |                                 `P003` | 低电平亮，收到 ALIGN_DONE 后亮 |
| 输出确认灯        |                                 `P503` | 低电平亮，仅 P801 实际输出时亮 |

---

## 7. 最小回归检查

1. `soft_reset`：最终出现 `soft reset final verify PASS`。
2. `read_all`：J1~J5 批量反馈正常。
3. `auto 15 0 0`、`auto 0 -70 0`：S 曲线运动能结束，终点确认通过。
4. 连续 AUTO：上一条未结束时不会堆积新的视觉小步 AUTO。
5. `abs_rotate <joint> <angle>`：单关节命令能等待真实角度到位。
6. Jetson 使用正式 `A5 5A + CRC16` 协议发送 `TARGET_CTRL enable=1`：安全条件满足时回传 `STATUS TARGET_CTRL_ON`。
7. 连续视觉误差进入阈值：回传 `STATUS ALIGN_DONE`。
8. 按住 P000 且二次确认完成：回传 `STATUS OUTPUT_ON`，P801 输出。
9. 松开 P000、视觉超时、误差超限、限位触发或状态机关闭：P801 拉低并回传 `STATUS OUTPUT_OFF`。
10. 收到 `ALIGN_DONE` 后 P003 点亮；P801 实际打开后 P503 点亮；退出输出或异常时对应 LED 熄灭。

## 8. 控制流程

我们是通过jeston nx和RA6M5之间互相通信来实现的，jeston端是QT界面的展示来的，可以通过触碰来下发串口数据给RA6M5。

采用正式的A5 5A + CRC16链路

现在的Jeston端QT界面演示和状态机的演示其实有点没匹配好，导致现在情况不太乐观来的

我现在来说明一下我们项目的全流程展示，你去要通过我的说明并且提出一些问题来完善我们jeston和ra6m5的通信协议。
第一步，在jeston端QT界面点击“开始”，机械臂向前延伸到达预定位置，（此预定位置和开启状态机，机械臂到的预定位置是同一个位置，但是这里并没有开启激光定靶状态机）。

第二步，Jeston端QT界面点击“采集人脸图”，JESTON端QT界面会实现一个界面的跳转，到RGB-D的深度图，随后机械臂自动复位，回到HOME。（这里的关键在于JESTON端什么时候让RA6M5控制机械臂复位到HOME点）。RA6M5给Jeston端一个返回值，Jeston端确认后下发采集三视图的指令给RA6M5，先开始采集左视图，采集完毕后RA6M5回传给JESton，jeston收到后下发给RA6M5采集中视图的指令，RA6M5到位后回传给Jeston，jeston采集完后下发采集右视图的指令给RA6M5,，机械臂到位后，RA6M5回传给jeston，jeston采集右视图，采集完毕，jeston下发指令给RA6M5，机械臂回到Home点，Jeston端的QT界面会自动跳转显示刚刚采集的三视图。

第三步，Jeston端QT界面显示三视图，医生可以根据三视图界面选择其中一个视角的画面，例如点击右视图，机械臂会先回到HOME点，然后运动到采集右视图的位置，在Jeston端QT界面点击“开始点痣”，机械臂会以当前选择的三视图位置为预开始位置，后面根据Jeston端提供的黑痣视觉误差来瞄准定位，激光发射，完成点痣操作，这一部分就是点痣的状态机是一样的，和PC端的机械臂上位机，“一键点痣”相同。

补充：jeston端的心跳是全局作用量来的，安全距离是在第一步的时候，（在jeston端QT界面点击“开始”，机械臂向前延伸到达预定位置）通过顶部的RGB-D深度相机采集的，安全距离的效果是jeston端发送的实际距离小于安全距离的时候，机械臂的Y轴方向后退，但是Jeston端没有发生安全距离的时候，不能影响到整个的状态机工作，既深度相机没有采集到安全距离，因为机械臂在HOME点深度相机无法采集到有效的安全距离。

## 9. 比赛正式控制流程与通信细节

本章是对第 8 章业务描述的正式化说明，规定 Qt、Jetson 与 RA6M5 在比赛流程中各自负责什么、发送什么，以及收到什么回传后才能进入下一步。正式字节编码、CRC、消息类型和错误码见 `视觉协议.md`。

### 9.1 控制权与安全职责


| 设备    | 主要职责                                                                            | 明确禁止                                           |
| --------- | ------------------------------------------------------------------------------------- | ---------------------------------------------------- |
| Qt 界面 | 接收医生的“开始”“采集人脸图”“选择视图”“开始点痣”“完成/返回”操作并显示状态 | 不直接控制电机或激光 GPIO                          |
| Jetson  | 维护界面流程、采集 RGB-D/三视图、自动识别目标痣、计算`dcx/dcy`、组帧和解析回传      | 不下发任意笛卡尔坐标、关节角、电机速度或激光开命令 |
| RA6M5   | 执行固化点位、复位、视觉伺服、状态机、安全判断和 P801 输出                          | 不因单帧视觉数据或单独按下 P000 直接打开激光       |

RA6M5 是运动和激光安全的最终裁决者。Jetson 请求“开始”只表示请求，RA6M5 必须检查当前状态后决定接受或拒绝。

### 9.2 正式通信约定

- 比赛链路只使用 `A5 5A + VER + TYPE + SEQ + LEN + PAYLOAD + CRC16/MODBUS`。
- 当前固件默认 `JETSON_LEGACY_PROTOCOL_ENABLE=0`；旧 `FF/AA/CC` 帧和 UART `target_enable/laser_on` 均不能启动正式比赛流程或绕过激光安全门。
- Jetson 打开串口后每 `100~200 ms` 发送一次 `HEARTBEAT`；RA6M5 心跳超时阈值为 `600 ms`。
- 多字节字段使用小端；`dcx/dcy` 为有符号 `int16`，距离为单位 `mm` 的 `uint16`。
- Jetson 的心跳、视觉、测距和控制帧共用一个线程安全的全局 `SEQ` 生成器；新帧在 `0x01~0xFF` 循环，程序启动使用随机非零初值，`0x00` 保留给 RA6M5 异步故障。
- Jetson 所有写串口操作必须经过单一 TX 队列或全局锁。控制 ACK 等待 `300 ms`、最多重试 3 次，重试必须保留完全相同的 TYPE、SEQ、payload 和 CRC。
- RA6M5 控制命令缓存有效期为 `3000 ms`；有效期内完全相同的重发只重放缓存状态，不重复执行运动，相同 TYPE+SEQ 但 payload 不同则回传 `SEQ_CONFLICT`。
- `WORKFLOW_CTRL`、`CAPTURE_CTRL`、`TARGET_CTRL` 被接受或拒绝后，RA6M5 先用同一 `SEQ` 回传 `COMMAND_ACK`；动作完成后再回传对应业务状态。
- 控制命令的完成状态使用对应请求的 `SEQ`。Jetson 必须按状态回传推进流程，不能只靠固定延时。
- 心跳、限位等非某条控制命令触发的异步故障固定使用 `SEQ=0x00`；本轮 workflow 状态使用本轮 `START_MEASURE` 的 `SEQ`。
- `point_id=1/2/3` 分别表示左视图、正视图、右视图。点位坐标和关节配方只保存在 RA6M5。
- 所有协议触发的回 HOME 都执行 `soft_reset`；只有 `soft reset final verify PASS` 才能回传 HOME 完成。
- P801 在非 `TARGET_OUTPUT` 状态必须保持低电平。

### 9.3 上电与链路建立

1. RA6M5 上电初始化 P801 为低电平，比赛流程处于 `IDLE`。
2. Jetson 打开串口并开始周期发送 `HEARTBEAT`。
3. Qt 在心跳有效前禁用“开始点痣”；普通界面可显示“等待机械臂连接”。
4. 心跳有效只证明通信在线，不代表距离安全、视觉对准或允许发射。

### 9.4 第一步：Qt“开始”与公共测距

第一次“开始”只用于让顶部 RGB-D 在公共位置测量安全距离，不能启动定靶状态机。

当前公共测距位固定为 `(X,Y,Z)=(0,-130,-15) mm`。


| 顺序 | 发送方 -> 接收方 | 消息/操作                                | 接收方行为与回传                                                                 |
| -----: | ------------------ | ------------------------------------------ | ---------------------------------------------------------------------------------- |
|    1 | Qt -> Jetson     | 用户点击“开始”                         | Jetson 发送`WORKFLOW_CTRL action=START_MEASURE`                                  |
|    2 | Jetson -> RA6M5  | `TYPE=0x06, action=0x01`                 | RA6M5 先回`COMMAND_ACK`，再关闭 P801、清除旧许可并回传 `WORKFLOW START_ACCEPTED` |
|    3 | RA6M5 内部       | 执行`soft_reset`                         | PASS 后移动到固化的公共测距位；FAIL 则进入`FAULT_HOLD`                           |
|    4 | RA6M5 -> Jetson  | `STATUS WORKFLOW MEASURE_POSITION_READY` | Jetson 才允许顶部 RGB-D 生成有效距离                                             |
|    5 | Jetson -> RA6M5  | `SAFE_DISTANCE distance_mm valid=1`      | RA6M5 使用固件中的标定阈值做安全裁决                                             |

安全距离采用“测距阶段确认、整轮流程锁存”，不要求点痣阶段持续实时发送。RA6M5 当前正式判定阈值为 `150 mm`，只有连续 3 个不同非零 `SEQ` 的有效样本均达到该阈值才锁存：

- 距离合格：RA6M5 回传 `STATUS SAFE_DISTANCE=SAFE` 和 `WORKFLOW DISTANCE_SAFE_LATCHED`，置位本轮 `distance_safe_latched=true`。
- 距离不合格：RA6M5 回传 `STATUS SAFE_DISTANCE=UNSAFE, error_code=SAFE_DISTANCE_TOO_CLOSE`，关闭 P801，并沿 `+Y` 每次撤离 `20 mm`。
- 每一步撤离真实到位后回传 `WORKFLOW RETREAT_STEP_READY`；Jetson 收到后才发送一个新 `SEQ` 的 `SAFE_DISTANCE`。RA6M5 最多等待 `2000 ms`，超时进入 `FAULT_HOLD`。
- 若新距离仍低于 `150 mm`，则继续下一步；最多 `5` 步、累计 `100 mm`。达到上限仍不安全时进入 `FAULT_HOLD`，不能用累计移动量代替 RGB-D 实测距离。
- 距离恢复安全后不直接恢复流程，也不置位 `distance_safe_latched`。RA6M5 原地等待并回传 `WORKFLOW RETREAT_DONE_WAIT_RESTART`，医生必须重新点击 Qt“开始”。
- 重新开始：RA6M5 先 `soft_reset` 回 HOME，再进入公共测距位并重新测量。
- HOME 或移动阶段没有有效深度数据不是故障；只有处于 `WAIT_SAFE_DISTANCE` 时才消费该测距结果。
- 新流程开始、流程完成/取消、心跳超时、限位/急停、复位失败或掉电都会清除距离安全锁存。
- 一次锁存成立的前提是患者位置在本轮流程中保持稳定，且全部固化视图点已按最不利距离完成标定。患者明显移动或场景改变时，Jetson 必须请求 `ABORT_HOLD` 并重新从公共测距开始。

### 9.5 第二步：Qt“采集人脸图”与三视图

只有 `distance_safe_latched=true` 时才允许开始三视图流程。三张图由 Jetson 逐点驱动，RA6M5 到位后不会自动前往下一点。


| 顺序 | Jetson -> RA6M5                        | RA6M5 -> Jetson                       | Jetson 动作              |
| -----: | ---------------------------------------- | --------------------------------------- | -------------------------- |
|    1 | `CAPTURE_CTRL action=HOME, point_id=0` | `STATUS CAPTURE_HOME_READY`           | Qt 切换到 RGB-D/采集界面 |
|    2 | `CAPTURE_CTRL action=GOTO, point_id=1` | `STATUS CAPTURE_POINT_READY, value=1` | 本地保存左视图           |
|    3 | `CAPTURE_CTRL action=GOTO, point_id=2` | `STATUS CAPTURE_POINT_READY, value=2` | 本地保存正视图           |
|    4 | `CAPTURE_CTRL action=GOTO, point_id=3` | `STATUS CAPTURE_POINT_READY, value=3` | 本地保存右视图           |
|    5 | `CAPTURE_CTRL action=HOME, point_id=0` | `STATUS CAPTURE_HOME_READY`           | 显示三张已保存图像       |

`CAPTURE_POINT_READY` 只能在该点位的完整动作成功结束后发送。任一步出现 IK、到位超时、限位或 `soft_reset` 失败，RA6M5 必须回传 `ERROR` 并停止流程，Jetson 不得拍照或发送下一点。

### 9.6 第三步：医生选择视图

当前比赛版本中，医生只选择左、正、右三张视图中的一张，不在图像上点击具体痣的位置。

1. 医生点击一张视图，Jetson 得到 `point_id=1/2/3`。
2. Jetson 发送 `CAPTURE_CTRL action=SELECT, point_id`。
3. RA6M5 确认当前流程、心跳和距离锁存有效，必要时先完成 HOME，再移动到该视图对应的固化点位。
4. 到位后 RA6M5 回传 `STATUS SELECTED_VIEW_READY, value=point_id`。
5. 机械臂保持在该位置，P801 保持关闭，定靶状态机尚未启用。
6. 该选中视图点就是点痣状态机的预开始点，不再执行第二次预定位。
7. Jetson 在选中视图对应的实时画面中自动识别并跟踪目标痣；未识别到目标时不得伪造 `dcx=0,dcy=0`。

### 9.7 Qt“开始点痣”与视觉对准

点击“开始点痣”后的握手顺序固定如下。表中的业务状态都在同 `SEQ` 的 `COMMAND_ACK` 之后发送：


| 顺序 | 发送方 -> 接收方 | 消息                                   | 进入下一步的条件                                         |
| -----: | ------------------ | ---------------------------------------- | ---------------------------------------------------------- |
|    1 | Jetson -> RA6M5  | `TARGET_CTRL enable=1`                 | 仅表示请求启动                                           |
|    2 | RA6M5 -> Jetson  | `STATUS COMMAND_ACK value=1`           | 请求已被接受；拒绝时 value=0 并携带错误码                |
|    3 | RA6M5 -> Jetson  | `STATUS TARGET_CTRL_ON`                | RA6M5 已通过心跳、pose、距离锁存、选中视图和安全输入检查 |
|    4 | RA6M5 -> Jetson  | `STATUS TARGET_READY`                  | 定靶子状态机已进入`WAIT_DETECT`                          |
|    5 | Jetson -> RA6M5  | 周期发送`VISION_ERROR dcx,dcy,valid=1` | 只在收到`TARGET_READY` 后开始发送                        |
|    6 | RA6M5 -> Jetson  | `STATUS ALIGN_DONE`                    | 先连续 3 帧、再连续 2 帧满足 `abs(dcx/dcy)<=5 px`         |
|    7 | Qt/Jetson        | 提示操作者按住 P000                    | P000 只是发射许可，不直接控制 P801                       |

启动请求至少需要同时满足：

```text
heartbeat_alive
&& pose_valid
&& distance_safe_latched
&& selected_view_ready
&& !motion_busy
&& !limit_triggered
&& !estop_active
&& !fault_latched
```

任何条件不满足时，RA6M5 保持 P801 关闭，回传 `STATUS TARGET_CTRL_OFF` 并在 `error_code` 中说明原因。Jetson 收到启动失败后停止发送视觉误差，在 Qt 显示原因并等待人工处理。

### 9.8 激光输出、视觉丢失与结束

P801 只允许在 `TARGET_OUTPUT` 中打开，完整许可条件为：

```text
workflow_active
&& target_enabled
&& heartbeat_alive
&& distance_safe_latched
&& pose_valid
&& vision_fresh
&& vision_in_tolerance
&& alignment_confirmed
&& P000_pressed
&& !limit_triggered
&& !estop_active
&& !fault_latched
```

- 进入输出：RA6M5 停止机械臂视觉修正并冻结当前位置，P801 拉高，回传 `STATUS OUTPUT_ON`。
- 输出期间：Jetson 继续发送视觉误差，仅用于安全监测，机械臂不在激光开启时运动。
- 误差重新超限：RA6M5 立即关闭 P801，回传 `OUTPUT_OFF`，退回 `TARGET_ALIGN` 重新对准。
- 视觉超过 `500 ms` 未更新或 `valid=0`：立即关闭 P801 并回传 `VISION_LOST`；机械臂原地保持，退回 `WAIT_DETECT`，视觉恢复后可继续，无需再次点击“开始点痣”。
- P000 松开：立即关闭 P801 并回传 `OUTPUT_OFF`，进入 `TARGET_HOLD`。机械臂保持当前位置，等待 Qt“完成/返回”。
- 单次 P801 输出有独立硬上限 `TARGET_OUTPUT_MAX_MS=10000 ms`；到时自动关闭并进入 `TARGET_HOLD`，避免按键或上位机异常造成持续输出。
- Qt“完成/返回”：Jetson 发送 `WORKFLOW_CTRL action=FINISH_RETURN_HOME`。RA6M5 关闭定靶状态机和 P801，执行 `soft_reset`；只有 final verify PASS 后才回传 `WORKFLOW RETURN_HOME_DONE` 并进入 `IDLE`。

### 9.9 心跳与异常恢复

心跳是整个比赛流程的全局安全条件。统一协议启用后，超过 `600 ms` 没有收到合法心跳时，RA6M5 必须：

1. 立即拉低 P801；
2. 中止当前运动并禁止新的自动运动；
3. 清除视觉有效标志、距离锁存、选中视图和发射许可；
4. 进入 `FAULT_HOLD` 并保持当前位置，不自动回 HOME；
5. 尝试回传 `ERROR HEARTBEAT_TIMEOUT`；
6. 通信恢复后仍保持故障等待，不能自动续跑；
7. 等待用户重新点击 Qt“开始”，再执行 `soft_reset -> 公共测距位`。

上述异步 `ERROR` 使用 `SEQ=0x00`。心跳恢复本身不会清除 `FAULT_HOLD`、安全距离锁存或运动中止代际，只有新的 Qt“开始”请求可以触发安全复位。

限位、急停、pose invalid、IK/运动失败及复位失败使用相同的“先关激光、再停止运动、锁存故障、等待人工重新开始”原则。CRC错误、长度错误和不完整帧只丢弃，不改变任何控制状态。

### 9.10 Qt 操作与协议映射速查


| Qt 操作/自动动作    | Jetson 下发                             | RA6M5 成功回传                                  |
| --------------------- | ----------------------------------------- | ------------------------------------------------- |
| 串口连接            | 周期`HEARTBEAT`                         | 可选`HEARTBEAT_ALIVE`                           |
| 点击“开始”        | `WORKFLOW_CTRL START_MEASURE`           | `START_ACCEPTED`，随后 `MEASURE_POSITION_READY` |
| 顶部 RGB-D 得到距离 | `SAFE_DISTANCE`                         | `DISTANCE_SAFE_LATCHED` 或距离过近错误          |
| 点击“采集人脸图”  | `CAPTURE_CTRL HOME`                     | `CAPTURE_HOME_READY`                            |
| 采集左/正/右        | 依次`CAPTURE_CTRL GOTO, point_id=1/2/3` | 对应`CAPTURE_POINT_READY`                       |
| 三视图采集结束      | `CAPTURE_CTRL HOME`                     | `CAPTURE_HOME_READY`                            |
| 医生选择一张视图    | `CAPTURE_CTRL SELECT, point_id`         | `SELECTED_VIEW_READY`                           |
| 点击“开始点痣”    | `TARGET_CTRL enable=1`                  | `TARGET_CTRL_ON`，随后 `TARGET_READY`           |
| 自动识别目标        | 周期`VISION_ERROR`                      | 稳定后`ALIGN_DONE`                              |
| 按住 P000           | 不发送激光开命令                        | 满足全部条件后`OUTPUT_ON`                       |
| 松开 P000           | 无需 Qt 命令                            | `OUTPUT_OFF`，机械臂保持当前位置                |
| 点击“完成/返回”   | `WORKFLOW_CTRL FINISH_RETURN_HOME`      | `RETURN_HOME_DONE`                              |
| 点击取消/安全停止   | `WORKFLOW_CTRL ABORT_HOLD`              | `ABORTED_HOLD`，不自动 HOME                     |

## 10. 新比赛安全状态机（当前代码实现）

### 10.1 文档状态

本章描述当前代码实现，事实来源为 `robot_workflow.c/.h`、`robot_capture.c/.h`、`robot_target.c/.h`、`jetson_vision.c/.h`、`robot.c` 和视觉服务线程。代码已通过 Keil `Target_1` 编译及静态契约检查；整机串口联调、限位/心跳故障注入和激光 GPIO 时序仍需上板验收，不能只以“协议帧能收发”判断完成。

### 10.2 两级状态机结构

采用“顶层比赛流程 + 定靶子状态机”两级结构，避免 `robot_capture`、`robot_target` 和 Qt 命令分别维护互相冲突的状态。

顶层状态：

```text
FLOW_IDLE
  -> FLOW_HOMING
  -> FLOW_MEASURE_POSITION
  -> FLOW_WAIT_SAFE_DISTANCE
  -> FLOW_SAFE_READY
  -> FLOW_CAPTURE
  -> FLOW_CAPTURE_HOME
  -> FLOW_SELECTED_VIEW
  -> FLOW_TARGET_ACTIVE
  -> FLOW_TARGET_HOLD
  -> FLOW_RETURN_HOME
  -> FLOW_IDLE

任意状态 -> FLOW_RETREAT_WAIT_RESTART
任意状态 -> FLOW_FAULT_HOLD
```

定靶子状态：

```text
TARGET_INIT
  -> TARGET_WAIT_DETECT
  -> TARGET_ALIGN
  -> TARGET_CONFIRM
  -> TARGET_OUTPUT
  -> TARGET_HOLD

视觉丢失: TARGET_ALIGN/CONFIRM/OUTPUT -> TARGET_WAIT_DETECT
误差超限: TARGET_CONFIRM/OUTPUT -> TARGET_ALIGN
```

选中的三视图点已经是定靶预开始点，因此新流程不再从 `TARGET_INIT` 自动发送一次 `TARGET_PRE_POSITION` AUTO，避免重复移动。

### 10.3 顶层状态职责


| 状态                        | 允许输入                 | RA6M5 行为                   | 退出条件               |
| ----------------------------- | -------------------------- | ------------------------------ | ------------------------ |
| `FLOW_IDLE`                 | `WORKFLOW START`         | 激光关闭，清理上一轮状态     | 接受开始请求           |
| `FLOW_HOMING`               | 心跳、停止请求           | 执行`soft_reset`             | final verify PASS      |
| `FLOW_MEASURE_POSITION`     | 心跳、停止请求           | 移动到公共测距位             | 运动真实到位           |
| `FLOW_WAIT_SAFE_DISTANCE`   | `SAFE_DISTANCE`          | 校验并累计有效测距           | 安全锁存或触发撤离     |
| `FLOW_SAFE_READY`           | 采集 HOME 请求           | 等待 Qt“采集人脸图”        | 接受采集请求           |
| `FLOW_CAPTURE`              | 当前点完成后的下一点请求 | 单次执行一个固化视图点       | 三张图逐点完成         |
| `FLOW_CAPTURE_HOME`         | 选择视图请求             | `soft_reset` 后等待选择      | 收到合法`point_id`     |
| `FLOW_SELECTED_VIEW`        | `TARGET_CTRL START`      | 在选中点保持，激光关闭       | 安全门通过并启用定靶   |
| `FLOW_TARGET_ACTIVE`        | 视觉帧、P000、停止/完成  | 运行定靶子状态机             | P000松开或完成/异常    |
| `FLOW_TARGET_HOLD`          | 完成、取消、重新对准请求 | 激光关闭，机械臂保持         | 完成返回或重新对准     |
| `FLOW_RETURN_HOME`          | 心跳                     | 关闭定靶并`soft_reset`       | final verify PASS      |
| `FLOW_RETREAT_WAIT_RESTART` | 新`WORKFLOW START`       | 安全撤离后原地等待           | 人工重新开始           |
| `FLOW_FAULT_HOLD`           | 新`WORKFLOW START`       | 激光关闭、运动停止、故障锁存 | 故障解除并人工重新开始 |

### 10.4 状态上下文

顶层状态机至少保存以下上下文，所有变量只能由状态机任务统一修改：

```c
typedef struct
{
    workflow_state_t state;
    uint8_t selected_view_id;
    bool distance_safe_latched;
    bool target_enabled;
    bool fault_latched;
    uint8_t active_command_type;
    uint8_t active_command_seq;
    uint32_t last_heartbeat_ms;
} robot_workflow_t;
```

`distance_safe_latched` 只允许在公共测距位由合格测距置位；任何新流程、完成、取消或故障都会清零。`selected_view_id` 只允许由合法 `CAPTURE_CTRL SELECT` 更新。

### 10.5 事件处理优先级

每个 `10 ms` 服务周期按固定优先级处理，低优先级事件不能覆盖高优先级安全动作：

1. 限位、急停、硬件故障；
2. 心跳超时；
3. `ABORT_HOLD`、完成返回和状态机关闭；
4. 安全距离结果；
5. 当前运动的 `OK/FAILED/ABORTED` 结果；
6. 视觉超时、视觉误差和 P000；
7. 普通流程推进命令。

所有打开激光的代码必须集中在一个安全出口中：先计算完整许可条件，再在临界区内确认 `workflow state` 未被并发改变，最后才能调用 `BSP_Laser_On()`。其他分支只能调用 `BSP_Laser_Off()`。

### 10.6 模块边界


| 模块                                | 当前职责                                                                     |
| ------------------------------------- | ------------------------------------------------------------------------------ |
| `Middle/jetson_vision.c/.h`         | SCI2 RX_CHAR环形接收、统一协议解析、CRC与重同步、保存命令`SEQ`、发送关联状态；不直接控制机械臂 |
| `APP/robot_workflow.c/.h`           | 顶层比赛状态机，唯一持有流程状态、安全锁存、选中视图和激光开启许可           |
| `APP/robot_capture.c/.h`            | 只执行单个固化 HOME/视图配方并返回结果，不自行决定下一视图                   |
| `APP/robot_target.c/.h`             | 只实现等待视觉、对准、确认、输出和保持子状态，不负责比赛开始或三视图流程     |
| `src/vision_service_thread_entry.c` | 将协议事件投递给`robot_workflow`，不复制业务判断                             |
| `APP/robot.c`                       | 提供可查询结果的 AUTO、`soft_reset` 和真正的运动中止接口                     |
| `USER_BSP/bsp_laser.c/.h`           | P801默认关闭、P000读取；不包含业务状态                                       |
| `USER_BSP/bsp_led.c/.h`             | P003/P503低电平指示输出；具体点亮条件由`robot_workflow`统一决定               |

### 10.7 当前实现要点


| 已实现项       | 代码行为                                                                                       |
| ---------------- | ------------------------------------------------------------------------------------------------ |
| SCI2 接收       | 不再使用有限 DTC 缓冲反复重挂；`UART_EVENT_RX_CHAR` 写入 1024 字节环形缓冲，任务逐字节解析并在 CRC 失败后重搜 `A5 5A` |
| 顶层 workflow  | `TYPE=0x06` 驱动完整比赛流程，视觉线程只向 `robot_workflow_step()` 投递事件                    |
| 命令关联与去重 | 控制缓存有效期 `3000 ms`；期内相同 `TYPE+SEQ+payload` 只重发状态，不重复运动，不同 payload 回 `SEQ_CONFLICT` |
| 公共测距锁存   | 仅在公共测距位累计 3 个不同非零`SEQ` 且均 `>=150 mm` 的安全样本；锁存后本轮使用                 |
| 分步安全撤离   | 距离过近时`+Y 20 mm/步`，每步等待真实新测距 `2 s`，最多 5 步/100 mm；安全后仍等待人工重启      |
| 三视图单步执行 | 严格接受`HOME -> GOTO 1 -> GOTO 2 -> GOTO 3 -> HOME -> SELECT`，每次只执行一个请求             |
| 定靶起点       | `SELECT` 到位位置直接进入定靶，不重复执行 `TARGET_PRE_POSITION`                                |
| 激光唯一出口   | 业务代码中只有`robot_workflow.c` 调用 `BSP_Laser_On()`；UART `laser_on/target_enable` 默认拒绝 |
| 异常中止       | 运动事件带 generation；中止后旧队列事件失效，心跳/限位/运动失败进入保持状态                    |
| 输出保护       | P000释放后再按下才允许输出；视觉超时/超限、P000松开或`10 s` 上限立即关闭                       |
| 状态灯观察     | `ALIGN_DONE`后P003亮；P801真实输出时P503亮；丢失许可、退出或故障时熄灭                        |

### 10.8 当前超时与参数速查


| 参数                 |                     当前值 |
| ---------------------- | ---------------------------: |
| 公共测距位           |          `(0,-130,-15) mm` |
| 安全距离正式阈值     |                    `150 mm` |
| 安全样本数           |           `3` 个不同 `SEQ` |
| 撤离步长/上限        | `20 mm/步`，最多 `5` 步/`100 mm` |
| 每步新测距等待       |                  `2000 ms` |
| AUTO/单拍摄点超时    |                 `15000 ms` |
| soft_reset/HOME 超时 |                 `45000 ms` |
| 心跳超时             |                   `600 ms` |
| 视觉有效期           |                   `500 ms` |
| 单次激光输出上限     |                 `10000 ms` |

### 10.9 必须通过的安全验收

1. 未收到有效心跳，任何流程命令都不能打开 P801。
2. 未完成公共测距或距离不安全，`TARGET_CTRL START` 必须被拒绝。
3. 未选择视图或选中视图未到位，不能启动定靶。
4. 未收到 `TARGET_READY` 时发送视觉误差，不能越级进入输出。
5. P000单独按下不能开激光；全部条件满足后才允许输出。
6. 输出时视觉超时、误差超限、P000松开、限位或心跳超时，P801必须立即拉低。
7. 心跳恢复后不能自动续跑，必须由用户重新点击“开始”。
8. 所有 HOME 回传都必须以 `soft reset final verify PASS` 为依据。
9. 重发同一 `SEQ` 的控制命令不得造成重复运动。
10. 任一错误路径都不得残留 `distance_safe_latched`、发射许可或激光高电平。

PC 模拟 Jetson 联调使用 `DEBUG/formal_workflow_demo.py`。该脚本只发送正式 `A5 5A + CRC16` 帧，持续维持心跳，并按 ACK/业务完成状态逐步执行测距、三视图、视图选择、定靶、P000 和最终 HOME；旧 `target_state_demo.py` 仅适用于旧协议固件。

### 10.10 已知联调约束

- 当前视觉线程尚未接入独立急停 GPIO，`estop_active` 观察量默认是 `false`；现阶段真实硬件安全输入是关节限位、P000、心跳和软件故障。若比赛要求独立急停，必须先分配 GPIO 并接入该观察量。
- UART/MQTT 的 `auto`、关节旋转、`soft_reset` 等手动调试命令仍可直接进入底层运动队列。正式 workflow 运行时不得从 COM7 或 MQTT 下发这些命令，比赛现场应关闭相关上位机自动发送；后续建议将手动运动限制为仅 `FLOW_IDLE` 可用。
- 当前正式安全距离阈值为 `150 mm`。上板仍需用真实结构验证全部固化视图的最不利距离；若标定要求改变阈值，必须同步修改 RA6M5、`视觉协议.md`、Qt 提示和双方验收标准。

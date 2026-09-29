# 机械臂点痣装置 Qt 界面

这是 Jetson Nano B01 上的正式离线操作界面。比赛流程以根目录 [`项目演示(3).md`](../项目演示(3).md) 为准；[`视觉协议(5).md`](../视觉协议(5).md) 是 Jetson 侧协议镜像，跨端联调还必须与 RA6M5 工程根目录 `视觉协议.md` 保持一致。

## 职责

Qt 主进程负责：

- 同时接收顶部 Astra RGB-D 和末端单目视觉结果，并在正式流程中持续计算 arm-face 三维净距；
- 维护比赛业务状态，按 RA6M5 回传推进按钮和页面；
- 独占 RA6M5 串口，后台发送心跳、接收状态并处理控制命令重发；
- 显示三张固化视图，让医生选择其中一张；
- 在选中视图的实时画面中自动识别和跟踪目标；
- 显示错误、对准、P000 提示和 P801 输出状态。

Qt 不直接控制电机或激光 GPIO，不下发任意坐标、关节角或激光开启命令。RA6M5 是运动与 P801 的最终裁决者。

## 安装

Jetson Nano 上优先使用系统包：

```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-opencv python3-numpy python3-serial
```

也可以在合适的虚拟环境中安装：

```bash
python3 -m pip install -r requirements.txt
```

## 运行

无串口预览：

```bash
python3 -m app.main
```

正式统一协议：

```bash
cd /home/jetson/jixiebi/arm_mole_qt

DISPLAY=:0 bash run_qt_app.sh \
  --camera-layout astra-rgbd \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 30 --ui-fps 20 --flip none --rotate none \
  --camera-brightness 0 --camera-gain 64 --camera-backlight-compensation 0 \
  --facial-face-device 0 --facial-face-imgsz 416 --facial-face-refresh-frames 6 \
  --facial-stable-frames 4 --facial-stable-seconds 0.35 \
  --astra-always-on 1 --astra-rgb-enabled 1 \
  --astra-rgb-camera /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
  --astra-rgb-width 640 --astra-rgb-height 480 --astra-rgb-fps 30 --astra-rgb-flip none \
  --astra-rgb-calibration /home/jetson/jixiebi/calibration/astra_rgb/rgb_calib_params.npz \
  --astra-yolo-device 0 --astra-yolo-imgsz 416 --astra-yolo-interval 0.10 \
  --astra-depth-enabled 1 --astra-depth-distortion-mode raw \
  --rgbd-distance-log --rgbd-distance-log-interval 0.25 \
  --rgbd-continuous-stop-mm 120 --rgbd-continuous-guard-interval 0.20 \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 115200 --serial-protocol unified
```

`run_full_arm.sh` 已内置当前 `640x480 / 17.5 mm` 曲面假人候选偏移 `--laser-offset-x-px 78 --laser-offset-y-px -14`，直接运行脚本即可通过 Qt 的激光瞄准参考像素检查。若重新标定，可在脚本末尾追加新的 `--laser-offset-x-px <实测dx> --laser-offset-y-px <实测dy>` 覆盖内置值。安全标定步骤见 [`docs/LASER_AIM_CALIBRATION.md`](../docs/LASER_AIM_CALIBRATION.md)。

上面的默认命令故意不包含 `--rgbd-hardware-pair-verified`。它不是普通启动开关：只有真机确认 V4L RGB 与 SDK Depth 的设备身份、严格标定分辨率、畸变/外参、最坏时序、末端缺深与遮挡均通过验收后，才可在命令末尾追加；未添加时正式距离保持无效且不发送 `SAFE_DISTANCE`，不会误把未经验证的映射当成安全许可。

末端 YHTek 的正式联动采集档位暂回到 `640x480@30 FPS MJPG`。`960x540@30` 真机完整算法速度也能接近，但画面是 16:9，末端视觉显示框会明显变宽，且需要重新标定激光参考像素；在黑痣识别继续优化前不作为正式默认值。识别优先亮度基线为 `brightness=0 / gain=64 / backlight=0`，Qt 每次打开末端摄像头都会重放这三个参数。较亮的 `8 / 72 / 1` 在当前人脸模型上会使皮肤层次变浅，因此不再作为默认值。终端 `[END_CAMERA]` 日志会区分请求值、驱动回报值和启动实测值。

如果后续再次切换到 `960x540` 或其他分辨率，旧分辨率下测得的相机内参和激光 X/Y 像素偏移不能直接沿用。正式点痣必须使用当前运行分辨率重新测量激光瞄准参考像素。

当前默认启用增强候选过滤：眼睛/眉毛响应区不会生成黑痣候选，三视图每张图只显示最可信候选。若现场发现增强版误伤真实黑痣，可临时回退：

```bash
DISPLAY=:0 bash run_full_arm.sh \
  --facial-enhanced-filters 0 \
  --facial-triptych-top-candidate-only 0
```

当前正式启动不得添加 `--continuous-safe-distance`。程序会直接阻止这个旧参数；它原本会发送协议未定义的 `START_MEASURE_CONTINUOUS (0x04)`，真实 RA6M5 会以 `INVALID_PARAM` 拒绝。现在使用的是独立的本地 RGB-D 联锁：初始测距阶段仍只按正式协议发送 `SAFE_DISTANCE`，RA6M5 原有 `150 mm` 判定、锁存和撤离逻辑不变；锁存后 Qt 以 `--rgbd-continuous-guard-interval` 的节拍继续计算经过验证的 arm-face 三维表面净距，但不再发送 `SAFE_DISTANCE`。当有效净距严格小于 `--rgbd-continuous-stop-mm 120` 时，Qt 只发送现有的 `WORKFLOW_CTRL ABORT_HOLD`，且每个会话最多触发一次。

`PARTIAL_FACE` 与上述 `120 mm` 门保持独立：它防止把眼睛、眉毛等局部框当成完整人脸和目标身份，不能由深度放行。人脸定位器会在一个小 Haar/YOLO 框被可信整头皮肤框包住时改用整头框，以修复假人眼睛被误判为人脸的问题。没有新鲜且有效的 RGB-D 样本时不会伪造距离；终端通过 `[RGBD_DISTANCE]` 和 `[RGBD_GUARD]` 显示测距与联锁结果。

使用 `--camera-layout astra-rgbd` 时，`--astra-always-on 1` 默认生效：Qt 启动后立即打开 Astra RGB 与 Depth，并在页面切换时保持两个数据流，关闭 Qt 时统一释放。不要再同时运行独立 Orbbec Depth 脚本，否则会争抢同一设备；如需恢复按页面启停，可显式使用 `--astra-always-on 0`。

收到 `TARGET_READY` 后，串口后台线程独立处理 `VISION_ERROR`，不依赖 Qt 界面定时器。每个全新相机结果最多发送一次；两个新结果之间串口保持静默，不复用旧坐标，也不再因 `150 ms` 串口节拍快于识别帧率而人为插入 `valid=0`。已经发送过有效目标后，单帧 `RAW_CANDIDATE_0/STABILITY_PENDING/TARGET_LOCK_LOST` 等可恢复跟踪抖动先进入最长 `0.35 s` 的有界静默：期间绝不复用旧坐标，同一目标以新鲜帧复锁后直接恢复；超过窗口仍未恢复才只发送一次 `valid=0`。`NO_FACE/PARTIAL_FACE`、瞄准未标定和摄像头故障等非瞬态错误仍立即发送一次 `valid=0`。结果刚超过 `0.5 s` freshness 上限时也不发送旧坐标，由串口线程再保留最长 `0.25 s` 的调度宽限；无新帧时 RA6M5 的独立视觉 watchdog 仍是外层停机保护。人脸候选稳定器会分配会话内 `track_id`，定靶身份锁优先续跟最初锁住的同一 ID。完整初锁仍要求 `4 帧 + 0.35 s`；已完整确认的同一 ID 只在中断不超过 `0.45 s` 时允许用 `2` 个新鲜匹配帧快速复锁。超时、尺度/歧义门失败或疑似换痣都禁止快速复锁。终端 `[VISION_GATE]` 会打印跟踪 ID、稳定帧计数、结果年龄、失效类别和是否实际发布。

排查 `arm-face 净距` 抖动或间歇显示无效时，可添加 `--rgbd-distance-log`。终端会在状态或失效原因变化时立即打印，并在持续有效时默认每 `0.5 s` 输出最终保守净距、原始净距、滤波值、RGB/Depth 软件采样时差、点云有效比例和检测状态；可用 `--rgbd-distance-log-interval 0.25` 调整周期。该日志只观察现有结果，不会绕过安全门或改变串口有效性。

末端人脸定位与 Astra YOLO 可先验证下列低延迟参数：

```bash
--ui-fps 20 \
--facial-face-device 0 \
--facial-face-imgsz 416 \
--facial-face-refresh-frames 6 \
--facial-stable-frames 4 \
--facial-stable-seconds 0.35 \
--astra-yolo-device 0 \
--astra-yolo-imgsz 416 \
--astra-yolo-interval 0.10
```

Jetson 实测当前模型的纯 GPU 推理约从 `32.8 ms (640)` 降到 `26.2 ms (416)`。`416` 可能降低较小 `arm_tip` 的检出率，必须用演示的最远距离、最差光照和遮挡姿态重新验收；若丢检则恢复 `640`。

界面演示模式：

```bash
python3 -m app.main --demo
```

`--demo` 使用模拟画面和模拟状态，只用于界面检查。正式 Qt 运行期间，不得同时启动其他带 `--serial-port /dev/ttyUSB0` 的视觉脚本或协议 CLI。

手工协议按钮属于隔离联调功能，只有显式使用 `--protocol-debug-controls` 时才显示；比赛正式操作不使用这些按钮。

## 按钮与状态门

| 按钮 | 正式含义 | 必须等待的关键回传 |
|---|---|---|
| “开始” | `WORKFLOW_CTRL START_MEASURE`，开始新一轮并前往公共测距位 | `COMMAND_ACK`、`START_ACCEPTED`、`MEASURE_POSITION_READY` |
| “采集人脸图” | 执行三视图采集 | 安全距离已锁存；随后逐步等待 HOME/POINT READY |
| 点击左/正/右视图 | `CAPTURE_CTRL SELECT_VIEW` | `SELECTED_VIEW_READY` |
| “开始点痣” | `TARGET_CTRL START` | `TARGET_CTRL_ON`、`TARGET_READY` |
| “完成/返回” | `WORKFLOW_CTRL FINISH_RETURN_HOME` | `RETURN_HOME_DONE` |
| “取消/安全停止” | `WORKFLOW_CTRL ABORT_HOLD` | `ABORTED_HOLD`，原地保持 |
| “硬件急停说明” | 说明独立急停位置和处理方法 | 不发送软件急停替代命令 |

医生在三视图页只选择视图，不点击或选择具体痣。静态采集图中的旧坐标不得用于定靶；机械臂到 `SELECTED_VIEW_READY` 后，末端单目必须在实时画面中重新识别。

## 正式时序

1. 串口连接后，后台全程每 `100~200 ms` 发送 `HEARTBEAT`。
2. 收到 `MEASURE_POSITION_READY` 后，Qt 才把 Astra 的有效距离作为 `SAFE_DISTANCE` 发送；收到 `SAFE` 或 `UNSAFE` 后停止，锁存后不再发送距离。
3. 距离只接受到位后的新 RGB/Depth 样本；机械臂和人脸表面都必须能从各自外圈背景中确认，可信表面采用最小三维净距。距离下降立即按更小原始值生效；深度稀疏、YOLO 过期、时序不同步或对象归属不明时保留界面旧数值作为过期提示，但不发送业务帧。
4. 首个 `valid=1` 测距帧发送时就锁住对应人脸场景，并按 SAFE 样本 `SEQ` 保存快照；SAFE 回包和流程锁存必须匹配该快照。锁存后的场景变化由 Jetson 场景守卫监测，不是继续发送 `SAFE_DISTANCE`；明显变化会自动 `ABORT_HOLD`。随后执行 `HOME -> GOTO 1 -> GOTO 2 -> GOTO 3 -> HOME`，每张照片只能在对应 `CAPTURE_POINT_READY` 后保存。
5. 选择视图并收到 `SELECTED_VIEW_READY` 后，才允许点击“开始点痣”。
6. 点击“开始点痣”不要求目标已经居中，但必须已配置实测激光瞄准参考像素；否则不发送 `TARGET_CTRL START`。收到 `TARGET_READY` 后，首个连续稳定目标被锁定为本次定靶身份；半径位移、尺度、歧义和已排除候选共同约束续跟。只发送全新视觉结果；短时同 ID 复锁期间有界静默，超过门限或疑似换痣时才发送一次 `valid=0`。
7. P801 输出期间仍持续更新视觉状态。收到 `ALIGN_DONE` 时只提示按住 P000，不由 Qt 直接开激光。

每条控制命令都必须先匹配同 `SEQ` 的 `COMMAND_ACK`，再匹配业务完成状态。Jetson 新帧的 `SEQ` 只使用 `0x01~0xFF`，回绕时跳过为 RA6M5 异步故障保留的 `0x00`。Qt 不以固定延时推断机械臂到位。控制命令在 `300 ms` 未收到 ACK 时使用完全相同的字节重发，最多三次。除定靶阶段可恢复的 `VISION_LOST (0x0A)` 外，非零 `error_code`、错误帧或超时会停止自动流程并请求安全处理；若 `ABORT_HOLD` 也未获确认，Qt 会关闭串口心跳并禁止在未知状态下重新开始。

## 桌面图标

```bash
bash install_desktop_launcher.sh
```

脚本会创建 `机械臂点痣装置.desktop`。

## 安全说明

- “完成/返回”和“取消/安全停止”不是同一个动作：前者安全回 HOME，后者关输出后原地保持。
- 心跳丢失、限位、急停、复位失败、IK/运动错误或患者明显移动后，不得自动续跑；排除问题后重新点击“开始”。
- 软件按钮不能替代硬件急停、P000、限位和 RA6M5 故障联锁。
- 旧 `FF/AA/CC` 帧不属于正式 Qt 链路。

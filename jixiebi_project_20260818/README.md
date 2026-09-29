# 机械臂点痣项目总览

本仓库用于在仿真人脸模型上演示“公共测距、三视图采集、视图选择、末端单目自动定靶和 P000/P801 输出联锁”。当前阶段不面向真人治疗。

## 文档基线

- [`视觉协议(5).md`](视觉协议(5).md) 是 Jetson 侧的当前协议镜像；跨端联调以 RA6M5 工程根目录 `视觉协议.md` 与该镜像共同确认的一致版本为准。
- [`项目演示(3).md`](项目演示(3).md) 是当前正式比赛操作流程。
- `视觉协议(3).md`、`视觉协议(4).md` 和 `项目演示(2).md` 只用于追溯旧设计，不得作为实现或联调依据。
- 旧 `FF ... FE`、`AA ... BB`、`CC ... DD` 帧只允许隔离的 PC 兼容调试；Jetson 正式程序只使用 `A5 5A + VER + TYPE + SEQ + LEN + PAYLOAD + CRC16/MODBUS`。

当前正式比赛流程使用 `WORKFLOW_CTRL action=0x01` 的一次测距锁存。`action=0x04` 连续测距和 `SAFE_DISTANCE_TIMEOUT (0x10)` 仅为未来协议变更提案，不是当前 RA6M5 固件能力。文档定义不等于固件已经上板验证，比赛前必须完成双端联调和安全测试。

## 工程目录

```text
jixiebi/
  arm_mole_qt/          # 正式 Jetson 操作界面和流程状态机
  jetson_nano_vision/   # 单目识别、统一协议、串口发送与联调工具
  shuangmu/             # Astra RGB-D 采集、识别和测距实验
  calibration/          # 单目及 Astra RGB 标定图片和参数
  models/               # YOLO 等模型
  datasets/             # 训练数据
  pyorbbecsdk/          # Orbbec 第三方 SDK
  docs/                 # 摄像头配置、工程组织和流程说明
```

正式运行的关键文件：

- `arm_mole_qt/app/main.py`：Qt 界面、业务状态机、两路相机结果汇总。
- `arm_mole_qt/app/arm_controller.py`：Qt 到 RA6M5 的控制封装。
- `jetson_nano_vision/jetson_vision/serial_sender.py`：唯一串口会话、后台心跳、接收、控制命令重发。
- `jetson_nano_vision/jetson_vision/vision_protocol.py`：正式统一协议的打包、CRC 和状态解析。
- `jetson_nano_vision/jetson_vision/rgbd_distance.py`：RGB 检测框到 Depth 点云距离的计算。
- `jetson_nano_vision/jetson_vision/scene_guard.py`：安全锁存后的患者/场景稳定性守卫。
- `jetson_nano_vision/jetson_vision/target_lock.py`：定靶会话内的单一目标身份锁，禁止候选排序变化导致换痣。
- `jetson_nano_vision/jetson_vision/vision/facial_candidate.py`：末端单目候选目标检测。

## 正式串口所有权

正式比赛时，RA6M5 串口只能由 Qt 主进程打开一次：

```text
Astra RGB-D ─┐
             ├─> Qt 主进程 ─> 单一 SerialSender ─> RA6M5
末端单目 ────┘
```

`shuangmu/*.py`、`mono_mole_target.py`、`mole_preview.py` 和协议 CLI 都是独立视觉或串口联调工具。正式 Qt 运行时不得再让这些进程打开同一个 `/dev/ttyUSB*`，否则会造成帧交叉、SEQ/ACK 归属错误和心跳竞争。

## 正式操作流程

流程不能靠固定延时跳步。每条 `WORKFLOW_CTRL`、`CAPTURE_CTRL`、`TARGET_CTRL` 都要先收到同 `SEQ` 的 `COMMAND_ACK`，再等待同 `SEQ` 的业务完成状态。

1. Qt 连接串口后，后台全程每 `100~200 ms` 发送 `HEARTBEAT`。
2. 点击“开始”，发送 `WORKFLOW_CTRL START_MEASURE (action=0x01)`。收到 `MEASURE_POSITION_READY` 后才发送顶部 RGB-D 的 `SAFE_DISTANCE`。
3. 每个有效距离必须来自测距位到达后的新 RGB 与新 Depth 样本；过期、不同步或重复样本不发送，不复用旧值。RA6M5 当前使用 3 个不同非零 `SEQ` 的有效安全样本完成锁存；收到 `SAFE`/`UNSAFE` 裁决后立即停止发送，只有 `DISTANCE_SAFE_LATCHED` 成立才允许采集。
4. 点击“采集人脸图”，严格执行 `HOME -> GOTO 1 -> GOTO 2 -> GOTO 3 -> HOME`；每个点真实到位后才保存对应照片和发送下一条命令。
5. 医生只选择左、正、右三张视图之一，不点击或指定某一颗痣。收到 `SELECTED_VIEW_READY` 后，Jetson 在该位置的实时末端单目画面中自动识别目标。
6. 点击“开始点痣”，发送 `TARGET_CTRL START`。只有依次收到 `TARGET_CTRL_ON` 和 `TARGET_READY` 后，才每 `100~200 ms` 发送 `VISION_ERROR`。
7. 首个连续稳定目标在本次定靶会话内被锁定；后续同时使用痣半径位移门、尺度门、近邻歧义门和“已排除的其他痣”记录续跟。完整初锁仍需 `4 帧 + 0.35 s`；已经完整确认的同一 `track_id` 发生不超过 `0.45 s` 的短中断时，只允许凭 `2` 个新鲜、脸部相对位置与尺度均匹配的帧快速复锁。已有效会话内的单帧跟踪抖动最多静默 `0.35 s`，不复用旧坐标；恢复失败、疑似换痣或非瞬态安全错误时只发送一次 `valid=0`。漏检造成上游 `track_id` 重建时，Qt 默认只在最后有效锁定后的 `5 s` 内允许尝试保守重绑定，而且必须满足唯一候选、靠近旧锚、尺度一致、不是已排除候选并再次连续确认。禁止伪造 `dcx=0,dcy=0,valid=1`。P801 输出期间仍继续发送实时视觉状态。
8. 收到 `ALIGN_DONE` 后提示操作者按住硬件 P000；是否允许 P801 输出由 RA6M5 最终裁决。
9. 点击“完成/返回”，发送 `WORKFLOW_CTRL FINISH_RETURN_HOME`，等待 `RETURN_HOME_DONE`。
10. 点击“取消/安全停止”，发送 `WORKFLOW_CTRL ABORT_HOLD`，关输出并原地保持，不自动 HOME。硬件急停仍是独立安全输入。

非零 `error_code` 不能当作对应正常状态推进。唯一的流程内可恢复例外是定靶阶段 `VISION_LOST (0x0A)`：Qt 保持周期发送视觉状态，RA6M5 关闭 P801 并原地等待，视觉重新有效后可继续对准。其他错误、独立 `ERROR`、控制超时、心跳丢失、场景明显变化或患者移动都会停止自动推进。首个有效 SAFE 样本发送时即按其 `SEQ` 绑定人脸快照并开始场景守卫；变化会主动发送 `ABORT_HOLD`，并要求重新从“开始”和公共测距进入新一轮。

## 相机配置

末端单目摄像头：

```text
设备：Generic USB Camera YHTek
固定路径：/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0
推荐：640x480 / MJPG / 30 FPS
标定：正式点痣必须使用当前运行分辨率重新测量激光参考像素
```

Astra RGB：

```text
设备：Sonix USB 2.0 Camera SN0001
固定路径：/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
推荐：640x480 / MJPG / 30 FPS
标定：calibration/astra_rgb/rgb_calib_params.npz
Depth：通过 pyorbbecsdk Pipeline 读取
```

设备和详细测试命令见 `docs/CAMERA_CONFIG.md`。

## 末端视觉横向扫描（独立诊断）

横向扫描不放入 Qt，也不打开 RA6M5 串口。下面的命令只独占末端摄像头，
使用扫描专用的人脸参考框平滑和跨短暂运动模糊的候选段融合；退出时只打印
脸框归一化后的候选摘要，不会生成 `dcx/dcy`、`valid` 或任何机械臂/激光命令：

```bash
cd /home/jetson/jixiebi/jetson_nano_vision

DISPLAY=:0 /usr/bin/python3 -m jetson_vision.facial_candidate_preview \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 30 --flip none --face-device 0 \
  --stable-frames 4 --stable-radius 6 --view algorithm --background black \
  --scan-mode --scan-reference-alpha 0.10 \
  --record-dir /home/jetson/jixiebi/diagnostics/manual_face_scan
```

机械臂横向扫描完成后在预览窗口按 `Q`。只有输出
`[SCAN_RESULT] UNIQUE ...` 才表示多视角结果支持唯一候选；`INVALID` 表示候选不足
或仍有歧义，不能把扫描期间的旧像素坐标直接用于定靶。开始正式定靶前仍必须在
机械臂静止后，用当前末端相机的新鲜帧重新取得目标像素坐标。

## 运行入口

正式 Qt（只有完成下述 RGB-D 真机验收后才能添加 `--rgbd-hardware-pair-verified`）：

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
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 115200 --serial-protocol unified
```

上面的默认命令故意不包含 `--rgbd-hardware-pair-verified`。此时 Qt 可以预览和联调协议，但正式深度源始终标记为无效且不发送 `SAFE_DISTANCE`。验收必须确认 V4L RGB 与 SDK Depth 来自同一台 Astra、SDK 内外参与实际 `640x480` 严格匹配，并覆盖畸变、最坏采集延迟、末端缺深、遮挡、框边缘和安全阈值；全部验收通过后才可在命令末尾追加该参数，不能为了让流程通过而直接添加。

当前正式联动先使用 `640x480@30 FPS MJPG`。`run_full_arm.sh` 已内置当前曲面假人候选偏移 `--laser-offset-x-px 78 --laser-offset-y-px -14`；若重新标定，可在启动脚本末尾追加新的 `--laser-offset-x-px/--laser-offset-y-px` 覆盖。`960x540@30 FPS` 的原始取流也能约 `30 FPS`，但它是 16:9 宽屏，会让末端视觉显示框明显变宽，并且需要重新标定激光参考像素；在黑痣识别稳定性继续优化前，`960x540` 只作为高清测试档。启动终端的 `[END_CAMERA]` 会同时打印请求模式、驱动回报和启动实测帧率；右侧“画面 FPS”是检测/界面处理帧率，不等于传感器采集帧率。

末端分辨率变更后，相机内参和激光参考像素必须按当前分辨率重新标定。Astra RGB/Depth 仍保持 `640x480`，本次变更不修改双目深度标定。

当前正式启动命令不得添加 `--continuous-safe-distance`：真实 RA6M5 会把对应的未定义 `action=0x04` 以 `INVALID_PARAM` 拒绝。人脸靠近后的连续撤离属于未来提案，必须等双方共同更新协议、Jetson、RA6M5 和真机验收后再启用；`error=0x10` 当前也不是正式错误码。

无硬件界面演示：

```bash
cd arm_mole_qt
python3 -m app.main --demo
```

协议打包与解析自检：

```bash
cd jetson_nano_vision
python3 -m jetson_vision.protocol_self_test
```

RA6M5 伪串口联调（不访问真实机械臂、相机或激光）：

```bash
# 简单方式：一条命令启动模拟串口和模拟 Qt
cd /home/jetson/jixiebi
DISPLAY=:0 /usr/bin/python3 ra6m5_sim_check.py --qt-demo
```

需要观察每一帧协议时，也可以使用下面的双终端方式：

```bash
# 终端 1：保持运行，并记下输出的 /dev/pts/N
cd /home/jetson/jixiebi
/usr/bin/python3 ra6m5_sim_check.py --serve --verbose
```

```bash
# 终端 2：把 /dev/pts/N 替换成终端 1 的实际输出
cd /home/jetson/jixiebi/arm_mole_qt
DISPLAY=:0 bash run_qt_app.sh \
  --demo --detector-mode spot \
  --serial-port /dev/pts/N --baudrate 115200 --serial-protocol unified \
  --serial-safe-distance-source fixed \
  --serial-safe-distance-mm 120 --serial-safe-distance-valid 1
```

模拟器终端支持 `status`、`p000 on`、`p000 off` 和 `quit`。固定距离仅用于这条无硬件演示命令，不能连接真实串口使用；停止终端 1 后该 `/dev/pts/N` 会立即失效。

Astra YOLO + Depth 独立预览（不连接正式串口）：

```bash
cd shuangmu
DISPLAY=:0 PYTHONPATH=../pyorbbecsdk/install/lib python3 yolo_rgb_depth_distance.py
```

协议 CLI 只用于 Qt 未运行时的台架联调。执行任何运动命令前，应确认现场、限位、急停和固件状态均安全；完整命令以工具的 `--help` 为准。

## 阅读顺序

1. `项目演示(3).md`：比赛业务流程。
2. `视觉协议(5).md`：字节编码、状态、SEQ、ACK、重发和异常恢复。
3. `arm_mole_qt/README.md`：正式界面及按钮语义。
4. `docs/WORKFLOW_PLAN.md`：模块边界和实施状态。
5. `docs/CAMERA_CONFIG.md`：相机身份、标定和独立测试。
6. `jetson_nano_vision/README.md`、`shuangmu/README.md`：视觉子模块与调试工具。

## 安全边界

- 本工程当前只面向仿真人脸模型和低风险演示验证。
- Jetson 不直接发送机械臂坐标、关节角、电机速度或“打开激光”命令。
- RA6M5 是运动和 P801 的最终安全裁决者；P000、急停、限位、心跳和故障锁存不能由软件界面替代。
- 所有协议触发的 HOME 都必须经过 `soft_reset final verify PASS`，失败时保持故障，不得伪报到位。
- 关闭 Qt 或控制业务超时时，Qt 优先发送 `ABORT_HOLD`；若未获确认则停止串口心跳，由 RA6M5 看门狗进入安全保持。
- 串口连续读或写失败达到 3 次时，通信线程产生终止事件并停止心跳；Qt 不继续沿用旧的 `OUTPUT_ON` 或流程状态。

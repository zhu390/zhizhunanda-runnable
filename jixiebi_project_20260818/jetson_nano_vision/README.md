# Jetson Nano Vision

这个工程提供摄像头打开、标定、去畸变、黑痣检测、RGB-D 距离计算以及 Jetson 与 RA6M5 的统一协议模块。比赛正式业务由 `arm_mole_qt` 汇总视觉结果并独占串口；本目录中的单目命令行程序和协议 CLI 主要用于算法与台架联调。

正式通信只以根目录 `视觉协议(5).md` 为准，正式操作只以 `项目演示(3).md` 为准。`视觉协议(3).md`、`视觉协议(4).md` 和旧 `FF/AA/CC` 帧都是历史兼容资料，不得用于 Jetson 比赛程序。

原来的树莓派按键、OLED、UART7、心电图控制、云台控制都没有带进来，后面可以按 Jetson Nano 的新项目需求重新接。

## 目录

```text
jetson_nano_vision/
  jetson_vision/
    camera_utils.py          # V4L2/Picamera2 相机打开工具
    camera_test.py           # 摄像头测试
    snapshot.py              # 抓帧保存（拍人脸样本喂给离线调参用）
    reference_capture.py     # 采集无黑痣多帧中值基准图
    calib_capture.py         # 采集棋盘格标定图
    calib_solve.py           # 求解相机内参和畸变参数
    undistort.py             # 运行时去畸变
    detect_preview.py        # 相机 + 去畸变 + 矩形检测预览
    mole_preview.py          # 相机 + 去畸变 + 黑痣检测 + 串口发送
    reference_mole_preview.py # 基准差分黑痣检测与多帧稳定性预览
    mono_mole_target.py      # 末端单目黑痣定靶，输出 u/v、dcx/dcy 和协议帧
    mole_image_test.py       # 单张图片黑痣检测离线调参
    serial_sender.py         # RA6M5 单串口会话、心跳、接收、ACK/重发
    vision_protocol.py       # 正式 A5 5A 协议打包、CRC 与状态解析
    rgbd_distance.py         # 带前景归属证明的 RGB/Depth 最小三维净距
    scene_guard.py           # 测距样本起持续监测患者/场景变化
    target_lock.py           # 单一定靶身份锁与换痣防护
    vision/facial_candidate.py # 末端单目人脸区域候选目标检测
    vision/rect_detect.py    # 矩形靶检测
    vision/mole_detect.py    # 黑痣检测（肤色掩膜 + black-hat + 形状过滤）
    vision/reference_mole_detect.py # 无黑痣基准配准和新增暗斑检测
```

## 安装依赖

Jetson Nano 通常建议优先使用系统自带 OpenCV。如果 `python3 -c "import cv2"` 能正常运行，可以只装 numpy：

```bash
python3 -m pip install numpy
```

如果你的环境没有 OpenCV，再尝试：

```bash
python3 -m pip install -r requirements.txt
```

如果需要串口下发：

```bash
python3 -m pip install pyserial
```

## 换新摄像头后的步骤

新摄像头一般是 USB UVC，`camera_utils.py` 直接支持，但**焦距、畸变都变了**，必须重新做标定。流程：

1. 验证相机能开：

   ```bash
   python3 -m jetson_vision.camera_test --camera 0 --display 1 --flip none
   ```

2. 重新采集棋盘格标定图（15-25 张，覆盖画面四个角和中心、不同角度）：

   ```bash
   python3 -m jetson_vision.calib_capture --camera 0 --flip none --pattern 6x9 --out calib_images
   ```

3. 求解新的内参畸变：

   ```bash
   python3 -m jetson_vision.calib_solve --images calib_images --pattern 6x9 --square-mm 20 --out ../calibration/mono_end_camera/mono_calib_params.npz
   ```

当前仓库中新的单目摄像头标定结果已经放在：

```text
../calibration/mono_end_camera/images/
../calibration/mono_end_camera/mono_calib_params.npz
```

当前旧参数文件已经清理，运行时请显式指定新参数：

```bash
python3 -m jetson_vision.mole_system --camera 0 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --reference reference/clean_face.png
```

## 测试摄像头

USB/V4L2 摄像头：

```bash
python3 -m jetson_vision.camera_test --camera 0 --display 1 --flip none
```

Jetson Nano CSI 摄像头：

```bash
python3 -m jetson_vision.camera_test --camera csi --display 1 --flip none
```

无窗口只看帧率：

```bash
python3 -m jetson_vision.camera_test --camera 0 --display 0 --flip none
```

## 黑痣检测（单目方案）

### 末端单目定靶输出

以下命令用于最终点痣前的单目精定位：识别黑痣，显示 `u/v` 和视觉误差 `dcx/dcy`，同时在终端打印当前视觉协议帧。

```bash
python3 -m jetson_vision.mono_mole_target \
    --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
    --width 640 --height 480 --fps 60 --flip none --rotate none --detector dark \
    --wait-arm-stable \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --reference reference/clean_face.png
```

隔离串口联调（必须先退出正式 Qt，不能与机械动作比赛流程并行）：

```bash
python3 -m jetson_vision.mono_mole_target \
    --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
    --width 640 --height 480 --fps 60 --flip none --rotate none --detector dark \
    --wait-arm-stable --start-on-ready \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --reference reference/clean_face.png \
    --serial-port /dev/ttyUSB0 --baudrate 115200 \
    --isolated-serial-debug \
    --serial-protocol unified
```

这个独立工具不能代替 Qt 的 `WORKFLOW_CTRL -> SAFE_DISTANCE -> CAPTURE_CTRL -> TARGET_CTRL` 顶层状态机。正式运行时，顶部 RGB-D 和末端单目都只向 Qt 提供数据，由 Qt 的单一 `SerialSender` 发送。

稳定目标输出示例：

```text
STABLE_TARGET u=335 v=248 raw_u=334 raw_v=249 dcx=15 dcy=8 frame=A5 5A ...
```

`dcx/dcy` 按画面中心计算，正式帧定义见 `视觉协议(5).md`。旧 `FF 05 03 ... FE` 只能通过 `--serial-protocol legacy` 做隔离的 PC 兼容测试。
默认启用 2D 卡尔曼滤波，`u/v` 是滤波后的稳定坐标，`raw_u/raw_v` 是滤波前的稳定窗口坐标。默认按黑痣中心约 `3 px` 抖动处理，目标突然跳变超过 `15 px` 时自动重置。

末端单目保持摄像头原始方向，统一使用 `--flip none --rotate none`。贴纸黑痣优先用 `--detector dark`，不依赖无黑痣基准图。

默认加上 `--wait-arm-stable`：程序启动后只预览，不识别、不发坐标。机械臂稳定停下后，按 `D` 或空格开始识别；按 `P` / `H` 可暂停识别。正式 Qt 链路只有在本次定靶会话先后收到 `TARGET_CTRL_ON`、`TARGET_READY` 后才开始发视觉帧；命令行的 `--start-on-ready` 仅用于隔离联调。

### 一体化运行（推荐）

以下命令在同一窗口中完成去畸变、基准重采、配准、肤色 ROI、差分检测、形状过滤、多帧确认和坐标显示：

```bash
python3 -m jetson_vision.mole_system --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --reference reference/clean_face.png

# 隔离联调时下发稳定目标误差（正式 Qt 必须已退出）
python3 -m jetson_vision.mole_system --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --reference reference/clean_face.png \
    --serial-port /dev/ttyUSB0 --baudrate 115200
```

按键：

- `R`：相机位置改变后，重新采集无黑痣基准。按键前必须取下全部黑痣。
- `D`：贴好黑痣后进入检测。
- `P`：返回普通预览。
- `N`：切换对齐图、ROI、暗差分图和二值图。
- `S`：保存当前带检测标注的结果图。
- `Q` / Esc：退出。

按 `R` 后默认倒计时 3 秒，再采集 30 帧中值基准，便于操作者移开手。相机明显移动而未重采基准时，程序返回 `ALIGN_FAILED`，不输出坐标。

注意：重采基准只能恢复视觉检测。相机相对机械臂的位置改变后，视觉坐标到机械臂坐标的外参映射仍需重新标定。

固定摄像头和固定人脸模型时，优先使用无黑痣基准差分方案：

```bash
# 采集 30 帧中值基准图，显示模式下按 SPACE 开始
python3 -m jetson_vision.reference_capture --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz --frames 30 --display 1

# 实时检测相对基准新增的黑痣，按 N 切换调试视图
python3 -m jetson_vision.reference_mole_preview --camera 1 \
    --width 640 --height 480 --fps 30 --flip none \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --reference reference/clean_face.png --display 1
```

该流程会先对当前画面与基准图做小幅配准。配准失败、目标不稳定或没有新增暗斑时，不输出稳定目标。

以下肤色掩膜 + black-hat 方案保留为不依赖基准图的辅助检测器。

检测流程：肤色掩膜（HSV+YCrCb）→ black-hat 提局部暗斑 → 阈值 + 形状过滤 → 按显著性排序。正式流程把实时检测结果交给 Qt，不由检测器单独占用串口。

### 第一步：先用单图离线调参

直接在摄像头上调参很折腾。先抓一张实拍图离线调好参数：

```bash
# 单帧抓拍
python3 -m jetson_vision.snapshot --camera 0 --out face.jpg --flip none

# 或开窗按空格连拍多张
python3 -m jetson_vision.snapshot --camera 0 --out face --display 1 --flip none

# 在样本上拖滑动条调参
python3 -m jetson_vision.mole_image_test --image face.jpg
```

`mole_image_test` 滑动条说明：

- **min_area / max_area**：黑痣面积像素范围。摄像头距离不同会差很多，先按当前距离的痣实际占多少像素估个范围，再用滑动条收紧。
- **circ x100 / solid x100**：圆度和实心度，越接近 1 越像痣。眉毛细线圆度低，会被过滤。
- **aspect x10**：长宽比上限 ×10。`22` 表示长宽比 ≤ 2.2。
- **bh_ksize**：black-hat 核大小，应比痣的直径**大**一点（25-35 一般够）。
- **contrast**：痣相对周围肤色的灰度差阈值。光照亮就调高，暗就调低。
- **skin_dilate**：肤色掩膜 erode 大小，越大越严格地避开五官（也越容易把脸边缘的痣排除）。
- **skin_mask**：0 / 1 切换，调参时切到 0 看不带肤色限制的原始结果。

按键：
- `n` 在 `[overlay / skin / blackhat / binary]` 之间循环切换视图，看中间结果方便定位是哪一步把痣过滤掉了。
- `s` 把当前参数保存为 `mole_params.json`。
- `q` / Esc 退出。

### 第二步：把调好的参数应用到摄像头实时检测

```bash
# 加载离线调好的参数
python3 -m jetson_vision.mole_preview --camera 0 --flip none \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --params-json mole_params.json

# 隔离串口联调（正式 Qt 必须已退出）
python3 -m jetson_vision.mole_preview --camera 0 --flip none \
    --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
    --params-json mole_params.json \
    --serial-port /dev/ttyUSB0 --baudrate 115200

# 现场再微调（带 trackbar，按 s 保存到同一个 json）
python3 -m jetson_vision.mole_preview --camera 0 --flip none \
    --params-json mole_params.json --tune
```

串口帧格式见 `vision_protocol.py` 和根目录 `视觉协议(5).md`。正式演示统一使用 `A5 5A + CRC16/MODBUS` 协议，消息包括：

```text
HEARTBEAT       TYPE=0x01  uint32 tick_ms
TARGET_CTRL     TYPE=0x02  uint8 enable
VISION_ERROR    TYPE=0x03  int16 dcx + int16 dcy + uint8 valid
CAPTURE_CTRL    TYPE=0x04  uint8 action + uint8 point_id
SAFE_DISTANCE   TYPE=0x05  uint16 distance_mm + uint8 valid
WORKFLOW_CTRL   TYPE=0x06  uint8 action
```

正式定靶使用 `dcx = target_x - laser_aim_reference_x`、`dcy = target_y - laser_aim_reference_y`，均按 `int16_t` 小端发送。`laser_aim_reference_x/y` 必须在实际 `15~20 mm` 工作距离下实测，不能直接使用图像中心。未检测到可靠目标时发送 `valid=0`，不能复用旧坐标；两个全新相机结果之间也不得因串口周期更快而人为插入无效帧。

正式状态门控：

- `SAFE_DISTANCE` 只在收到 `WORKFLOW MEASURE_POSITION_READY` 后发送，收到 SAFE/UNSAFE 后停止；
- `VISION_ERROR` 只在收到本次目标会话的 `TARGET_READY` 后发送，P801 输出期间仍继续；
- `WORKFLOW_CTRL`、`CAPTURE_CTRL`、`TARGET_CTRL` 必须先匹配同 `SEQ` 的 `COMMAND_ACK`，再等业务完成状态；
- 超时重发使用相同 `TYPE + SEQ + payload` 的原始字节，最多三次；
- 非零 `error_code`、`ERROR` 或旧会话状态都不能当作正常完成状态推进；`VISION_LOST (0x0A)` 只进入关输出、原地等待恢复的分支。

RA6M5 回传 `STATUS TYPE=0x81` 或 `ERROR TYPE=0xFE`。旧 `FF/AA/CC` 只保留历史兼容，不属于正式链路。

## 测试矩形检测和去畸变

未启用标定：

```bash
python3 -m jetson_vision.detect_preview --camera 0 --display 1 --flip none
```

启用标定参数：

```bash
python3 -m jetson_vision.detect_preview --camera 0 --display 1 --flip none --calib-params ../calibration/mono_end_camera/mono_calib_params.npz
```

如果是 CSI 摄像头，把 `--camera 0` 换成 `--camera csi`。多路 CSI 可用 `--camera csi:1`。

## 重新采集标定图

如果上 Jetson 后实际画面分辨率、镜头或安装有变化，重新采集：

```bash
python3 -m jetson_vision.calib_capture --camera 0 --flip none --pattern 6x9 --out calib_images
python3 -m jetson_vision.calib_solve --images calib_images --pattern 6x9 --square-mm 20 --out ../calibration/mono_end_camera/mono_calib_params.npz
```

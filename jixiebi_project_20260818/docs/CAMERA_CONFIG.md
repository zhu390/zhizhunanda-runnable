# 摄像头配置记录

## 1. 机械臂末端单目摄像头

当前已经确认：

```text
用途：机械臂末端单目黑痣精定位
设备名称：Generic USB Camera YHTek
固定路径：/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0
临时 video 节点：/dev/video0
推荐格式：MJPG
推荐分辨率：640x480
当前稳定帧率：30 FPS（连续读取实测约30 FPS）
完整识别实测：约18.0 FPS（隔离运行末端算法）
标定参数：正式点痣必须使用当前运行分辨率重新测量激光参考像素
识别优先亮度：brightness=0 / gain=64 / backlight_compensation=0
```

不要在正式命令里依赖 `/dev/video0`。插拔摄像头后 `/dev/video0` 可能变化，应优先使用 `/dev/v4l/by-id/...` 固定路径。

## 2. 支持的主要帧率

`v4l2-ctl --list-formats-ext` 的设备能力表宣称支持：

```text
MJPG 640x480:
  400 FPS
  200 FPS
  120 FPS
  60 FPS

MJPG 1280x720:
  200 FPS
  120 FPS
  60 FPS
  30 FPS

MJPG 1280x1024:
  200 FPS
  120 FPS
  60 FPS
  30 FPS
```

能力表不是实际性能证明。真机连续阻塞读取和完整算法实测结果如下：

```text
640x480@30（请求60也一样）：原始约30 FPS，完整算法约18.0 FPS
960x540@30：                 原始约30 FPS，完整算法约17.6 FPS
1280x720@60：                原始约30 FPS，完整算法约7.1 FPS
```

因此正式联动暂时回到 `640x480@30`。`960x540@30` 比 `640x480` 多约 `69%` 像素，算法速度下降不明显，但它是 16:9 宽屏，会让末端视觉显示框变宽，并且必须重新标定激光瞄准参考像素；在黑痣识别稳定性继续优化前，`960x540` 只作为高清测试档。`1280x720` 只保留为画质/静态采集测试，不用于实时定靶。更改分辨率后必须重新标定相机内参和激光瞄准参考像素；Astra RGB/Depth 仍保持 `640x480`。

Qt 每次打开末端摄像头都会重放当前现场确认的亮度参数。独立预览前也可手工设置：

```bash
v4l2-ctl \
  -d /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --set-ctrl=brightness=0,gain=64,backlight_compensation=0
```

自动曝光和自动白平衡仍保持开启，因此这三个值提供识别优先的亮度基线，但不会锁死明暗。较亮的 `8 / 72 / 1` 会让当前人脸画面偏白、候选轮廓变差，不再作为默认值。强激光和太阳不得直射传感器。

## 3. 推荐测试命令

正式末端档位预览：

```bash
cd ~/jixiebi/jetson_nano_vision

python3 -m jetson_vision.camera_test \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 30 --display 1
```

单目摄像头 60 FPS 能力表/驱动回报对照（不作为正式档位）：

```bash
python3 -m jetson_vision.camera_test \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 60 --display 1
```

单目摄像头 120 FPS 压力测试：

```bash
python3 -m jetson_vision.camera_test \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 120 --display 1
```

SSH 无窗口取流测试：

```bash
python3 -m jetson_vision.camera_test \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 30 --display 0
```

## 4. 单目黑痣定位命令

末端单目定靶：识别黑痣，显示 `u/v`、`dcx/dcy`，并打印当前视觉协议帧：

```bash
cd ~/jixiebi/jetson_nano_vision

python3 -m jetson_vision.mono_mole_target \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 60 \
  --flip none --rotate none --detector dark \
  --wait-arm-stable \
  --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
  --reference reference/clean_face.png
```

如果需要隔离验证 `A5 5A + CRC16` 的视觉帧（必须先退出正式 Qt，并确认不会触发机械动作）：

```bash
python3 -m jetson_vision.mono_mole_target \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 60 \
  --flip none --rotate none --detector dark \
  --wait-arm-stable --start-on-ready \
  --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
  --reference reference/clean_face.png \
  --serial-port /dev/ttyUSB0 --baudrate 115200 \
  --isolated-serial-debug \
  --serial-protocol unified
```

正式比赛中 Qt 主进程独占 RA6M5 串口，单目和 Astra 都只向 Qt 提供视觉数据。`SAFE_DISTANCE` 只能在收到 `WORKFLOW MEASURE_POSITION_READY` 后由 Qt 发送，并在收到 SAFE/UNSAFE 裁决后停止；锁存后的 HOME、三视图和定靶阶段不再发送。当前正式启动只使用 `START_MEASURE (action=0x01)`，不得添加尚未获 RA6M5 正式协议支持的连续模式参数。不能让独立单目或 Astra 脚本全程发送。

旧 `FF/AA/CC` 兼容调试链路可加：

```bash
--serial-protocol legacy
```

一体化基准采集 + 检测工具：

```bash
cd ~/jixiebi/jetson_nano_vision

python3 -m jetson_vision.mole_system \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 60 \
  --flip none \
  --calib-params ../calibration/mono_end_camera/mono_calib_params.npz \
  --reference reference/clean_face.png
```

`mono_mole_target` 只负责最终定靶：加载已经采好的无黑痣基准图，检测新增黑痣，稳定后输出：

```text
STABLE_TARGET u=... v=... raw_u=... raw_v=... dcx=... dcy=... frame=A5 5A ...
```

其中 `u/v` 是图像像素坐标。调试预览可显示相对图像中心的原始误差，但正式定靶必须使用 `dcx/dcy = u/v - 实测激光瞄准参考像素`，正式帧定义见根目录 `视觉协议(5).md`。
默认启用 2D 卡尔曼滤波，`u/v` 是滤波后的稳定坐标，`raw_u/raw_v` 是稳定窗口给出的原始坐标。当前默认参数按单目黑痣中心抖动约 `3 px` 处理；若目标突然跳变超过 `15 px`，滤波器会自动重置，避免拖拽到错误位置。

末端单目保持摄像头原始方向，统一使用 `--flip none --rotate none`。当前贴纸黑痣使用 `--detector dark` 更稳定；后续若回到无黑痣基准差分流程，再改回 `--detector auto` 或 `--detector reference`。

`--wait-arm-stable` 会让程序启动后只显示预览，不做黑痣识别，也不输出坐标帧。机械臂稳定停下后，按 `D` 或空格开始识别；按 `P` / `H` 可以重新暂停识别。正式 Qt 链路必须先收到本次目标会话的 `TARGET_CTRL_ON` 和 `TARGET_READY` 才发送视觉误差；独立工具的 `--start-on-ready` 只用于隔离台架联调。

第一版建议默认使用 `60 FPS`。如果同时开启 Astra RGB + Depth 后出现掉帧、延迟或打不开摄像头，先降回 `30 FPS`。

## 5. 当前分工

```text
机械臂末端单目摄像头：
  Generic USB Camera YHTek
  /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0
  用于最终黑痣精定位

Astra Pro Plus / 双目 RGB：
  Sonix USB 2.0 Camera SN0001
  /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
  临时 video 节点：/dev/video2
  推荐格式：MJPG
  推荐分辨率：640x480
  推荐帧率：30 FPS
  RGB 标定参数：calibration/astra_rgb/rgb_calib_params.npz
  用于全脸观察、三点拍照和安全距离判断
```

## 6. 双目 / Astra RGB 测试命令

`v4l2-ctl --list-formats-ext` 已确认该 RGB 设备支持：

```text
MJPG:
  1920x1080 30 FPS
  2048x1536 30 FPS
  1280x960  30 FPS
  1280x720  30 FPS
  640x480   30 FPS
  320x240   30 FPS

YUYV:
  1920x1080 3 FPS
  2048x1536 3 FPS
  1280x960  10 FPS
  1280x720  10 FPS
  640x480   30 FPS
  320x240   30 FPS
```

第一版建议使用 `MJPG 640x480 30 FPS`。Astra RGB 主要用于全脸观察和三点拍照，不需要高帧率；同时使用 MJPG 可以降低 USB 带宽压力。

```bash
cd ~/jixiebi/jetson_nano_vision

python3 -m jetson_vision.camera_test \
  --camera /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
  --width 640 --height 480 --fps 30 --display 1
```

该 RGB 视角用于全脸观察和三点拍照。Astra RGB 彩色镜头标定图在 `calibration/astra_rgb/images/`，参数在 `calibration/astra_rgb/rgb_calib_params.npz`。Astra 的 Depth 数据不通过 `/dev/video2` 读取，而是通过 `pyorbbecsdk` 的 `Pipeline` 读取，Depth 内参运行时从 SDK 读取。

## 7. Astra Depth 测试命令

Orbbec Python SDK 已按当前系统的 Python 3.10 重新编译，当前有效模块为：

```text
pyorbbecsdk/install/lib/pyorbbecsdk.cpython-310-aarch64-linux-gnu.so
```

当前状态：

```text
Astra Pro Plus 设备识别：已通过
Python 3.10 pyorbbecsdk 导入：已通过
udev 权限规则：已安装
Depth 伪彩图显示：已通过
YOLO arm_tip / face 识别：已训练并接入
YOLO + Depth 距离计算：已接入
```

这里的“已接入”只代表数据链和算法能运行，不代表已经取得正式安全许可。Qt 默认将正式距离保持为无效且不发送 `SAFE_DISTANCE`；只有完成以下真机验收后才可在正式命令中添加 `--rgbd-hardware-pair-verified`：

- V4L 固定路径与 SDK Depth 确认来自同一台 Astra，断开/更换设备会被识别；
- SDK RGB/Depth 内外参的标定分辨率与实际流严格一致，RGB 畸变和投影误差已实测；
- 独立 V4L 与 SDK 采集的最坏缓存/时序差已测量，并纳入安全裕量；
- 覆盖金属末端缺深、背景冒充、遮挡、检测框边缘和稀疏深度；
- RA6M5 阈值使用最不利姿态、误差上界和机械制动裕量完成标定。

`--rgbd-allow-approximate-mapping` 仅在 `--demo` 或 `--protocol-debug-controls` 下生效，正式统一协议不能用近似映射绕过外参。

当前 Astra 的 SDK 无法读取硬件去畸变开关，所以默认 `--astra-depth-distortion-mode auto` 会得到 `unknown` 并按安全规则拒绝正式测距。真机诊断使用已经与 SDK 点云交叉核对过的原始 Brown 深度几何，同时加载实际 V4L RGB 标定：

```bash
--astra-rgb-calibration /home/jetson/jixiebi/calibration/astra_rgb/rgb_calib_params.npz \
--astra-depth-distortion-mode raw
```

这两个参数只解决畸变模型和内参来源，不能代替设备身份、重投影误差、实尺净距、遮挡/缺深和机械制动裕量验收。

Depth 点击测试：

```bash
cd ~/jixiebi/shuangmu

PYTHONPATH=../pyorbbecsdk/install/lib python3 orbbec_depth_click_3d.py
```

如果出现深度伪彩窗口，左键点击画面后，终端应打印该点的 `X / Y / Z` 毫米坐标。

YOLO 训练图片采集：

```bash
cd ~/jixiebi/shuangmu

DISPLAY=:0 python3 yolo_rgb_capture.py
```

按 `S` 保存单张图片，按 `A` 开关自动保存，按 `Q` 退出。自动保存示例：

```bash
DISPLAY=:0 python3 yolo_rgb_capture.py --auto-interval 0.8
```

图片保存到 `datasets/arm_face_yolo/raw/images/`，后续在电脑上标注两个类别：`arm_tip` 和 `face`。

YOLO 自动识别 + 深度测距：

```bash
cd ~/jixiebi/shuangmu

DISPLAY=:0 PYTHONPATH=../pyorbbecsdk/install/lib python3 yolo_rgb_depth_distance.py
```

当前默认模型路径：

```text
models/arm_face_yolo/best.pt
```

这个脚本会在 Astra RGB 图中检测 `arm_tip` 和 `face`，再映射到深度图 ROI 计算距离。它现在是纯显示/测距实验工具，串口发送已禁用，不能产生正式 `SAFE_DISTANCE`。若 RGB 检测框和深度 ROI 有固定偏差，可在实验中用下面参数观察：

```bash
DISPLAY=:0 PYTHONPATH=../pyorbbecsdk/install/lib python3 yolo_rgb_depth_distance.py \
  --arm-depth-offset-x 0 --arm-depth-offset-y 0 \
  --face-depth-offset-x 0 --face-depth-offset-y 0
```

机械臂末端 / 人脸手动 ROI 测距：

```bash
cd ~/jixiebi/shuangmu

PYTHONPATH=../pyorbbecsdk/install/lib python3 depth_roi_distance.py
```

注意：`depth_roi_distance.py` 不会真正识别机械臂末端或人脸。它只使用 `depth_roi_rois.json` 里保存的旧框，或当前手动拖出的框来计算距离。机械臂复位、人脸模型移动或相机角度变化后，旧框不会跟着目标走。

自动跟踪 ROI 测试（实验功能，当前不建议用于正式安全判断）：

```bash
cd ~/jixiebi/shuangmu

PYTHONPATH=../pyorbbecsdk/install/lib python3 depth_roi_distance.py \
  --auto-track-roi
```

操作方式：

```text
1：选择机械臂末端 ROI
2：选择人脸模型 ROI
鼠标左键拖框：设置当前 ROI
T：打开 / 关闭自动跟踪 ROI
S：手动保存 ROI
C：清空 ROI
Q / Esc：退出
```

两个 ROI 都选好后会自动保存到 `shuangmu/depth_roi_rois.json`。该文件只用于手动 ROI 调试，不参与当前 YOLO 正式测距流程。

`--auto-track-roi` 会以上一次保存的 ROI 为初始位置，在附近按深度连通区域自动更新框。当前它容易受背景深度、相机角度和机械臂末端形状影响，只作为实验观察使用；正式自动识别已经改为 YOLO。

独立实验脚本仍可显示 1D 卡尔曼值、`distance_p05` 和 `min`，这些字段只用于观察。正式 Qt 不使用实验脚本的 5% 分位作为安全量：机械臂与人脸两个检测框都必须由中心表面和外圈背景证明对象归属，然后保留所有可信前景点并取两表面的最小三维净距。这样细小但真实的末端凸出不会被分位数忽略，框内只有后方背景时也不会误报有效距离。

正式滤波是单向保守的：距离下降立即采用当前更小的原始净距，滤波只允许延迟“距离变大”，不能掩盖突发靠近。安装 SciPy 时直接用 KD-Tree 计算两朵完整点云的精确最近距离，不再扣除会被单个离群点放大的抽样覆盖半径；无 SciPy 的抽样回退若任一覆盖半径超过 `30 mm`，整帧直接无效。框整体/中心深度不足、外圈背景不足、对象无法从背景中确认、RGB/Depth 过期或不同步，以及质量门未通过的帧，都不会提交给距离滤波器。Qt 可保留上次数值作为已过期显示，但不会发送新的 `SAFE_DISTANCE`。

如果终端持续出现 `Pipeline source frameset queue fulled`，说明测距计算压力偏大，可以降低采样点数或测距频率：

```bash
PYTHONPATH=../pyorbbecsdk/install/lib python3 depth_roi_distance.py \
  --max-points 300 \
  --distance-interval 0.5
```

如果后续报 USB 权限或设备打不开，先安装 Orbbec udev 规则，然后拔插 Astra：

```bash
cd ~/jixiebi/pyorbbecsdk
sudo bash scripts/install_udev_rules.sh
```

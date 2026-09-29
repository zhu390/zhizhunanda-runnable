# Qt 界面技术说明文档

本文档说明 `arm_mole_qt` 桌面界面的代码结构、运行方式、界面模块、按钮流程、摄像头线程、基准图采集、识别结果显示和 RA6M5 串口交互逻辑。

## 1. 界面定位

`arm_mole_qt` 是运行在 Jetson Nano 上的离线 Qt 桌面界面，用于把视觉识别、机械臂伸展、目标预定位和点痣操作集中到一个操作界面中。

界面不依赖网络，也不依赖微信小程序。它直接调用本地 `jetson_nano_vision` 工程中的摄像头、视觉识别和串口协议代码。

## 2. 入口和运行命令

入口文件：

```text
arm_mole_qt/app/main.py
```

推荐运行命令：

```bash
cd ~/jixiebi/arm_mole_qt
python3 -u -m app.main --camera 0 --width 320 --height 240 --fps 30 --flip none \
    --detector-mode reference \
    --serial-port /dev/ttyUSB0 --baudrate 115200
```

也可以使用脚本：

```bash
cd ~/jixiebi/arm_mole_qt
bash run_qt_app.sh --camera 0 --width 320 --height 240 --fps 30 --flip none \
    --detector-mode reference \
    --serial-port /dev/ttyUSB0 --baudrate 115200
```

演示模式：

```bash
cd ~/jixiebi/arm_mole_qt
python3 -u -m app.main --demo
```

## 3. 依赖

Jetson 上推荐使用系统包安装：

```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-opencv python3-numpy python3-serial
```

Python 依赖文件：

```text
arm_mole_qt/requirements.txt
```

主要依赖：

| 依赖 | 用途 |
| --- | --- |
| `PyQt5` | Qt 桌面界面 |
| `opencv-python` / `python3-opencv` | 摄像头读取、图像处理、画面叠加 |
| `numpy` | 基准图中值计算、图像数组处理 |
| `pyserial` | 串口通信 |

## 4. 代码结构

```text
arm_mole_qt/
  app/
    __init__.py
    main.py              # Qt 主界面、摄像头线程、视觉识别流程
    arm_controller.py    # RA6M5 串口控制封装
  README.md
  run_qt_app.sh
  requirements.txt
```

Qt 界面会把 `jetson_nano_vision` 加入 `sys.path`：

```python
ROOT = Path(__file__).resolve().parents[2]
VISION_ROOT = ROOT / "jetson_nano_vision"
if VISION_ROOT.exists():
    sys.path.insert(0, str(VISION_ROOT))
```

因此 Qt 可以直接复用这些视觉模块：

```text
jetson_nano_vision/jetson_vision/camera_utils.py
jetson_nano_vision/jetson_vision/vision/mole_detect.py
jetson_nano_vision/jetson_vision/vision/reference_mole_detect.py
jetson_nano_vision/jetson_vision/serial_sender.py
jetson_nano_vision/jetson_vision/vision_protocol.py
```

## 5. 主类说明

### 5.1 `DetectionTarget`

目标检测结果的数据结构：

```python
@dataclass
class DetectionTarget:
    x: int
    y: int
    radius: int
    area: float
    contrast: float
    circularity: float
```

字段含义：

| 字段 | 说明 |
| --- | --- |
| `x` / `y` | 目标中心像素坐标 |
| `radius` | 目标近似半径 |
| `area` | 目标轮廓面积 |
| `contrast` | 目标对比度或平均变暗程度 |
| `circularity` | 圆度 |

### 5.2 `DetectionResult`

每帧识别结果的数据结构：

```python
@dataclass
class DetectionResult:
    detected: bool
    targets: List[DetectionTarget]
    frame_width: int
    frame_height: int
    fps: float
    mode: str
    message: str = ""
```

它通过 Qt 信号从摄像头线程传给主界面。

### 5.3 `VideoWorker`

`VideoWorker` 继承 `QThread`，负责：

- 打开摄像头。
- 持续读取画面。
- 执行图像翻转。
- 执行普通黑点检测或基准图差分检测。
- 采集无黑痣基准图。
- 画 OpenCV 叠加层。
- 把 `QImage` 和检测结果发送给主界面。

主要信号：

| 信号 | 作用 |
| --- | --- |
| `frame_ready` | 把当前显示帧发给主界面 |
| `result_ready` | 把检测结果发给主界面 |
| `status_changed` | 把运行状态文本发给主界面 |
| `reference_changed` | 通知主界面基准图状态变化 |

### 5.4 `MainWindow`

`MainWindow` 是主窗口，负责：

- 构建界面布局。
- 管理按钮状态。
- 接收并显示摄像头画面。
- 显示目标坐标、视觉误差、半径、对比度和 FPS。
- 管理 RA6M5 等待 `READY` 的流程。
- 调用 `ArmController` 发送串口命令。

### 5.5 `ArmController`

位置：

```text
arm_mole_qt/app/arm_controller.py
```

负责：

- 打开 RA6M5 串口。
- 发送定靶状态机启动帧 `AA 01 01 BB`。
- 发送定靶状态机关闭帧 `AA 01 00 BB`。
- 发送视觉误差帧。
- 发送激光输出帧。
- 轮询 RA6M5 状态帧。

## 6. 界面布局

界面由三部分组成：

```text
顶部状态栏
  - 标题
  - 状态提示
  - 当前时间

中间主区域
  - 左侧摄像头画面
  - 右侧运行流程 / 识别结果 / 最近记录

底部按钮区
  - 采集基准图
  - 开始识别
  - 单次识别
  - 停止识别
  - 预定位
  - 点痣操作
  - 急停
  - 退出
```

## 7. 按钮功能

| 按钮 | 功能 |
| --- | --- |
| `采集基准图` | 无黑痣状态下采集多帧中值基准图 |
| `开始识别` | 请求机械臂伸展，收到 `READY` 后连续识别 |
| `单次识别` | 对当前画面执行一次识别 |
| `停止识别` | 停止识别并请求关闭 RA6M5 定靶状态机 |
| `预定位` | 发送当前目标视觉误差帧，不触发激光 |
| `点痣操作` | 发送视觉误差帧，再请求激光输出 |
| `急停` | 当前协议未定义软件急停，仅提示使用硬件急停 |
| `退出` | 关闭 Qt 程序 |

## 8. 基准图按钮流程

点击 `采集基准图` 后：

```text
_capture_reference()
  -> 如果 RA6M5 已连接
       -> arm.start_target_state_machine()
       -> 发送 AA 01 01 BB
       -> 设置 _ready_after_action = "reference"
       -> 等待 RA6M5 READY
  -> 如果 RA6M5 未连接
       -> 直接 _start_reference_capture_after_ready()
```

收到 `READY` 后：

```text
_start_reference_capture_after_ready()
  -> worker.request_reference_capture()
  -> reference_state 显示“采集中”
```

摄像头线程中执行：

```text
_consume_reference_capture(frame)
  -> 丢弃 reference_warmup 帧
  -> 收集 reference_frames 帧
  -> np.median(stack, axis=0)
  -> 保存 clean_face.png
  -> _load_reference_detector()
```

默认基准图路径：

```text
jetson_nano_vision/reference/clean_face.png
```

## 9. 开始识别流程

点击 `开始识别` 后：

```text
_start_detection()
  -> 如果 RA6M5 已连接
       -> 发送 AA 01 01 BB
       -> 设置 _ready_after_action = "detect"
       -> 等待 READY
  -> 如果 RA6M5 未连接
       -> worker.start_detection()
```

收到 RA6M5 `READY` 后：

```text
_start_detection_after_ready()
  -> worker.start_detection()
  -> workflow_state = "连续识别"
```

摄像头线程中：

```text
_consume_detection_mode()
  -> MODE_DETECTING
  -> _detect(frame)
```

## 10. 检测模式

启动参数：

```bash
--detector-mode auto
--detector-mode spot
--detector-mode reference
```

| 模式 | 说明 |
| --- | --- |
| `auto` | 有基准图时使用基准差分，没有基准图时退回普通黑点检测 |
| `spot` | 只使用普通黑点检测 |
| `reference` | 强制使用基准差分；没有基准图时提示先采集基准图 |

Qt 中推荐使用：

```bash
--detector-mode reference
```

这样不会在缺少基准图时悄悄退回普通检测，避免眼睛、眉毛、底座被误识别。

## 11. 识别结果显示

右侧 `识别结果` 显示：

| 字段 | 来源 |
| --- | --- |
| `状态` | `DetectionResult.message` 或是否发现目标 |
| `目标数` | `len(result.targets)` |
| `像素坐标` | 首个目标的 `x / y` |
| `视觉误差` | `dcx / dcy` 或带激光偏置后的误差 |
| `激光偏置` | `ArmController.laser_offset_text()` |
| `半径` | `DetectionTarget.radius` |
| `对比度` | `DetectionTarget.contrast` |
| `画面 FPS` | 摄像头线程统计值 |
| `模式` | `预览` / `单次识别` / `连续识别` / `采集基准图` |

视觉误差计算：

```text
dcx = target_x - frame_width / 2
dcy = target_y - frame_height / 2
```

如果配置了激光偏置：

```text
dcx = dcx - laser_offset_x_px
dcy = dcy - laser_offset_y_px
```

## 12. 按钮互锁

界面通过这些状态变量防止流程冲突：

```python
self._waiting_for_arm_ready
self._detection_running
self._reference_capturing
self._ready_after_action
```

按钮刷新逻辑在：

```python
def _refresh_controls(self) -> None:
```

规则：

- 等待 `READY` 时不能再次开始识别或采基准图。
- 连续识别时不能再次开始识别或采基准图。
- 采集基准图时不能执行单次识别。
- 没有目标时禁用 `预定位` 和 `点痣操作`。
- `预定位` 和 `点痣操作` 需要 RA6M5 串口已连接。

## 13. RA6M5 状态轮询

主界面用 `QTimer` 每 `50 ms` 轮询 RA6M5：

```python
self.arm_timer = QTimer(self)
self.arm_timer.timeout.connect(self._poll_arm_status)
self.arm_timer.start(50)
```

状态处理函数：

```python
def _poll_arm_status(self) -> None:
```

关键状态：

| 状态 | Qt 行为 |
| --- | --- |
| `READY` | 根据 `_ready_after_action` 决定开始识别或采集基准图 |
| `TARGET_CTRL_ON` | 显示机械臂伸展中 |
| `TARGET_CTRL_OFF` | 中止等待 |
| `ERROR` | 停止识别流程 |

## 14. Qt 与串口协议关系

Qt 本身不直接拼帧，而是调用：

```text
arm_mole_qt/app/arm_controller.py
```

`ArmController` 再调用：

```text
jetson_nano_vision/jetson_vision/serial_sender.py
jetson_nano_vision/jetson_vision/vision_protocol.py
```

常用帧：

| 功能 | 帧 |
| --- | --- |
| 开启定靶状态机 | `AA 01 01 BB` |
| 关闭定靶状态机 | `AA 01 00 BB` |
| 激光输出开启 | `AA 03 01 BB` |
| 视觉误差帧 | `[FF][05][03][dcx][dcy][checksum][FE]` |

## 15. 主要启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--camera` | `0` | 摄像头来源 |
| `--width` | `640` | 采集宽度 |
| `--height` | `480` | 采集高度 |
| `--fps` | `30` | 采集帧率 |
| `--flip` | `none` | 图像翻转 |
| `--detector-mode` | `auto` | 检测模式 |
| `--reference` | 空 | 基准图路径 |
| `--reference-frames` | `30` | 采集基准图帧数 |
| `--reference-warmup` | `30` | 采基准图前丢弃帧数 |
| `--ref-min-darkening` | `18` | 基准差分变暗阈值 |
| `--ref-min-mean-darkening` | `18.0` | 平均变暗阈值 |
| `--ref-min-confidence` | `0.80` | 最小置信度 |
| `--serial-port` | 空 | RA6M5 串口 |
| `--baudrate` | `115200` | 串口波特率 |
| `--arm-ready-timeout-ms` | `30000` | 等待 READY 超时 |
| `--fullscreen` | 关闭 | 启动后全屏 |

## 16. 推荐操作顺序

1. 启动 Qt：

   ```bash
   cd ~/jixiebi/arm_mole_qt
   python3 -u -m app.main --camera 0 --width 320 --height 240 --fps 30 --flip none \
       --detector-mode reference \
       --serial-port /dev/ttyUSB0 --baudrate 115200
   ```

2. 不贴黑痣。
3. 点击 `采集基准图`。
4. 等待界面显示 `基准图已启用`。
5. 贴黑痣。
6. 点击 `开始识别`。
7. 目标稳定后先点 `预定位`。
8. 安全确认后再点 `点痣操作`。

## 17. 常见问题

### 17.1 Qt 界面没有“采集基准图”按钮

说明 Jetson 上运行的可能还是旧版代码。检查：

```bash
cd ~/jixiebi/arm_mole_qt
grep -n "采集基准图" app/main.py
```

如果没有输出，需要把新版 `arm_mole_qt/app/main.py` 同步到 Jetson。

### 17.2 提示“请先采集无黑痣基准图”

说明当前使用的是：

```bash
--detector-mode reference
```

但还没有可用的 `clean_face.png`。先点击 `采集基准图`。

### 17.3 提示“基准图尺寸不一致”

采基准图时的分辨率和当前运行分辨率不一致。必须保持一致：

```bash
--width 320 --height 240 --flip none
```

### 17.4 点击开始识别后没反应

优先检查：

- 串口是否正确：`/dev/ttyUSB0` 是否存在。
- RA6M5 是否回传 `READY`。
- 右侧 `等待 READY` 是否一直计时。
- 是否超时显示 `等待 READY 超时`。

### 17.5 预定位和点痣按钮是灰色

原因通常是：

- RA6M5 串口未连接。
- 当前没有识别目标。
- 正在等待 `READY`。
- 正在采集基准图。

## 18. 后续可改进点

1. 给 Qt 增加 ROI 设置控件，允许现场限制检测区域。
2. 给 Qt 增加基准图预览按钮，确认 `clean_face.png` 是否正确。
3. 给 Qt 增加调参面板，在线修改 `ref-min-darkening`、`ref-min-confidence`。
4. 增加串口日志导出，方便排查 RA6M5 状态。
5. 增加识别稳定性窗口，避免单帧误识别直接进入预定位。


# shuangmu 双目 / Astra RGB-D 调试区说明

这个目录只放双目视觉和 Astra RGB-D 的采集、标定、模型训练和独立测距脚本。比赛正式流程由 `arm_mole_qt` 统一调用视觉结果并独占 RA6M5 串口；本目录脚本不是另一套比赛主控。正式业务和协议分别见根目录 `项目演示(3).md`、`视觉协议(5).md`。

## 正式流程中的职责

- Astra RGB 检测 `arm_tip` 和 `face`，Depth 计算两者 3D 间距。
- 距离结果只提供给 Qt；Qt 只有收到 RA6M5 的 `MEASURE_POSITION_READY` 后才发送有效 `SAFE_DISTANCE`，收到 SAFE/UNSAFE 后停止发送。
- 正式 Qt 运行时，本目录脚本不得另外打开同一个 `/dev/ttyUSB*`。
- `depth_roi_distance.py` 只用于手工测距、标定和问题定位，不作为正式安全输入。
- 文档或脚本中保留的串口参数仅用于 Qt 完全退出后的隔离台架调试，不代表正式运行架构。

## 标定文件位置

机械臂末端单目标定放在：

```text
calibration/mono_end_camera/
```

Astra RGB 彩色镜头标定放在：

```text
calibration/astra_rgb/images/
calibration/astra_rgb/rgb_calib_params.npz
```

Astra Depth 的内参主要由 Orbbec SDK 在运行时读取。如果后续导出固定参数，放到：

```text
calibration/astra_depth/
```

## Astra RGB 标定

采集棋盘格图片：

```bash
cd ~/jixiebi/shuangmu

DISPLAY=:0 python3 rgb_calib_capture.py
```

默认保存到：

```text
calibration/astra_rgb/images/
```

解算 RGB 彩色镜头内参：

```bash
python3 rgb_calib_solve.py
```

默认输出到：

```text
calibration/astra_rgb/rgb_calib_params.npz
```

当前结果：`640x480`，RMS 重投影误差约 `0.2850 px`，质量优秀。

## RGB-D / Astra 实验

常用脚本：

```text
yolo_rgb_depth_distance.py    # 独立测距入口：YOLO识别 arm_tip/face + Depth测距
yolo_rgb_capture.py           # YOLO训练图片采集
depth_roi_distance.py         # 手动ROI测距调试
orbbec_depth_click_3d.py      # 点击深度图并打印3D坐标
rgb_preview.py                # RGB预览
rgb_snapshot.py               # RGB单张截图
rgb_calib_capture.py          # Astra RGB棋盘格采集
rgb_calib_solve.py            # Astra RGB内参解算
```

当前保留的模型和训练数据：

```text
models/arm_face_yolo/best.pt
datasets/arm_face_yolo/
```

YOLO 训练图片采集：

```bash
cd ~/jixiebi/shuangmu

DISPLAY=:0 python3 yolo_rgb_capture.py
```

窗口打开后，按 `S` 保存当前 Astra RGB 图片；按 `A` 可开启 / 关闭自动保存，自动保存需要启动时加 `--auto-interval`，例如每 0.8 秒保存一张：

```bash
DISPLAY=:0 python3 yolo_rgb_capture.py --auto-interval 0.8
```

图片默认保存到：

```text
datasets/arm_face_yolo/raw/images/
```

后续拷到电脑标注，类别建议固定为 `0=arm_tip`、`1=face`。

YOLO 自动识别 + 深度测距：

```bash
cd ~/jixiebi/shuangmu

DISPLAY=:0 PYTHONPATH=../pyorbbecsdk/install/lib python3 yolo_rgb_depth_distance.py
```

脚本默认加载：

```text
models/arm_face_yolo/best.pt
```

它会在 Astra RGB 画面中检测 `arm_tip` 和 `face`，再把检测框映射到深度图 ROI 计算两者距离。由于 Astra RGB 和 Depth 不是完全同轴，如果深度图上的框偏了，可以用下面参数微调：

`yolo_rgb_depth_distance.py` 的串口发送路径已经禁用；即使传入兼容保留的 `--serial-port` 参数，程序也会报错退出。需要检查 `SAFE_DISTANCE` 编码时，请在 Qt 完全退出后使用正式协议 CLI 或 `ra6m5_sim_check.py`，不要把 RGB-D 可视化进程变成第二个串口所有者。

```bash
DISPLAY=:0 PYTHONPATH=../pyorbbecsdk/install/lib python3 yolo_rgb_depth_distance.py \
  --arm-depth-offset-x 0 --arm-depth-offset-y 0 \
  --face-depth-offset-x 0 --face-depth-offset-y 0
```

手动 ROI 测距测试：

```bash
cd ~/jixiebi/shuangmu
PYTHONPATH=../pyorbbecsdk/install/lib python3 depth_roi_distance.py
```

这个脚本不会识别机械臂末端或人脸，只会读取上一次保存的框，或者使用你当前手动拖出来的框来计算距离。机械臂复位、人脸模型移动或相机角度变化后，需要重新拖框。

如果要让框跟着机械臂末端和人脸附近的深度区域自动微调：

```bash
PYTHONPATH=../pyorbbecsdk/install/lib python3 depth_roi_distance.py \
  --auto-track-roi
```

操作：按 `1` 后拖框选择机械臂末端，按 `2` 后拖框选择人脸模型；两个框都有以后终端会持续打印距离，窗口左上角也会显示毫米值。

框选会自动保存到：

```text
depth_roi_rois.json
```

下次启动手动 ROI 脚本会自动加载该文件，不需要重复框选。按 `T` 可打开或关闭自动跟踪，按 `S` 可手动保存，按 `C` 清空当前窗口里的框，按 `Q` 退出。如果相机、人脸模型或机械臂末端位置变了，重新拖框即可覆盖保存。

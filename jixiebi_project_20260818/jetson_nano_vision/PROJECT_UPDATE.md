# Jetson Nano 新工程更新说明

## 本次更新目标

从旧的树莓派视觉工程中，单独拆出一个面向 Jetson Nano 的新工程，只保留与摄像头、相机标定、去畸变和基础视觉检测相关的可复用内容。

新工程路径：

```text
jetson_nano_vision/
```

## 已迁移内容

已从旧工程复制并整理：

```text
jetson_nano_vision/
  jetson_vision/
    camera_utils.py                # 相机打开、翻转、V4L2/CSI/GStreamer 支持
    camera_test.py                 # 摄像头测试脚本
    calib_capture.py               # 棋盘格标定图采集脚本
    calib_solve.py                 # 相机内参和畸变求解脚本
    undistort.py                   # 运行时去畸变工具
    detect_preview.py              # 相机 + 去畸变 + 矩形检测预览
    vision/
      rect_detect.py               # 矩形目标检测
```

## 未迁移内容

以下旧项目内容没有带入新工程：

- 树莓派按键逻辑
- OLED 显示逻辑
- UART7 云台通信协议
- ECG 心电图绘制控制
- 旧的 QGimbal 控制层和串口 stub
- 参考代码中的海康 SDK / Jetson Orin 旧实现

这样做是为了让 Jetson Nano 新项目保持干净，后续控制协议和业务逻辑可以重新设计。

## Jetson Nano 相机支持

`camera_utils.py` 已在新工程中增加 Jetson 友好的打开方式：

```bash
# USB / V4L2 摄像头
python3 -m jetson_vision.camera_test --camera 0 --display 1 --flip -1

# Jetson Nano CSI 摄像头
python3 -m jetson_vision.camera_test --camera csi --display 1 --flip -1

# 多路 CSI 示例
python3 -m jetson_vision.camera_test --camera csi:1 --display 1 --flip -1
```

如果需要，也可以直接把 GStreamer pipeline 作为 `--camera` 参数传入。

## 标定参数状态

旧标定图片已经清理。当前新单目摄像头的标定图和参数保存在：

```text
calibration/mono_end_camera/images/
calibration/mono_end_camera/mono_calib_params.npz
```

运行 `jetson_nano_vision` 时建议直接指定这个新参数文件：

```bash
python3 -m jetson_vision.mole_system --camera 0 --calib-params ../calibration/mono_end_camera/mono_calib_params.npz
```

如果上 Jetson 后摄像头、镜头、分辨率或安装状态发生变化，建议重新采集标定图：

```bash
python3 -m jetson_vision.calib_capture --camera 0 --flip -1 --pattern 6x9 --out calib_images
python3 -m jetson_vision.calib_solve --images calib_images --pattern 6x9 --square-mm 20 --out ../calibration/mono_end_camera/mono_calib_params.npz
```

CSI 摄像头时把 `--camera 0` 换成 `--camera csi`。

## 检测预览

不使用标定参数：

```bash
python3 -m jetson_vision.detect_preview --camera 0 --display 1 --flip -1
```

使用标定参数去畸变：

```bash
python3 -m jetson_vision.detect_preview --camera 0 --display 1 --flip -1 --calib-params ../calibration/mono_end_camera/mono_calib_params.npz
```

CSI 摄像头：

```bash
python3 -m jetson_vision.detect_preview --camera csi --display 1 --flip -1 --calib-params ../calibration/mono_end_camera/mono_calib_params.npz
```

## 已验证

已在当前开发机上完成：

- 新工程文件结构检查
- 新单目 26 张标定图解算确认，RMS 约 0.2854 px
- 所有新工程 Python 文件语法检查通过

未在当前机器验证：

- Jetson Nano 实机 CSI 摄像头打开
- Jetson Nano 实机 GStreamer pipeline
- 实际相机画面下的标定误差和矩形检测效果

这些需要上板后根据真实摄像头和光照环境再确认。

## 后续建议

1. 先在 Jetson Nano 上跑通 `camera_test.py`。
2. 再生成或确认 `../calibration/mono_end_camera/mono_calib_params.npz`。
3. 用 `detect_preview.py` 验证去畸变和矩形检测。
4. 确认视觉输入稳定后，再开始接入新项目的控制逻辑。

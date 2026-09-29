# 标定文件分类

这个目录只放相机标定相关数据，不放运行脚本。

## 1. 机械臂末端单目摄像头

```text
mono_end_camera/
  images/                         # 旧640x480单目标定图，26张
  images_backup_same_as_current/  # 与 images 完全相同的备份副本
  mono_calib_params.npz           # 旧640x480单目标定参数
```

对应设备：

```text
Generic USB Camera YHTek
/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0
```

用途：黑痣精定位、输出 `u/v` 和 `dcx/dcy`。

当前参数：

```text
image_size = 640x480
RMS reprojection error = 0.2854 px
```

正式末端采集已经改为 `960x540@30 FPS`。上述文件保留用于追溯，不能直接作为
新分辨率的正式内参；必须用 `960x540` 重新采集棋盘格并求解后，再启用对应的
畸变校正。旧 `640x480` 激光参考像素同样不能换算或沿用。

## 2. Astra Pro Plus RGB 彩色镜头

```text
astra_rgb/
  images/                 # Astra RGB棋盘格标定图，23张
  rgb_calib_params.npz    # Astra RGB彩色镜头内参
```

对应设备：

```text
Sonix USB 2.0 Camera SN0001
/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
```

用途：Astra RGB 画面预览、全脸拍照、YOLO 识别 `arm_tip` 和 `face`。

当前参数：

```text
image_size = 640x480
RMS reprojection error = 0.2850 px
```

## 3. Astra Depth

```text
astra_depth/
```

Depth 内参目前运行时从 Orbbec SDK 读取，不靠这些棋盘格图片生成。

典型运行输出：

```text
depth intrinsics from SDK: Intrinsics(fx=577.969, fy=577.969, cx=320.202, cy=239.200)
```

后续如果需要固定保存 SDK 读到的深度内参、深度尺度或 RGB-Depth 对齐参数，就放到 `astra_depth/`。

## 4. 不是标定的文件

```text
datasets/arm_face_yolo/labels/
datasets/arm_face_yolo/raw/images/
```

这些是 YOLO 训练图片和标注，用于识别机械臂末端和人脸，不是相机标定参数。

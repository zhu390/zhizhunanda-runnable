# 激光瞄准参考像素标定

这项标定用于测量末端相机画面中的真实瞄准参考像素。正式视觉误差应以该参考像素为零点，不能从相机中心、机械尺寸、旧照片或黑痣位置猜测。

## 安全要求

- 治疗激光必须断电并物理隔离；不得为了标定绕过 RA6M5 联锁。
- 只允许使用低功率可见瞄准光或经过批准的维护标定光源。
- 只照射固定的哑光标定卡，禁止对准真人、动物、镜面或其他反光物体。
- Qt 必须退出，RA6M5 保持安全 HOLD，机械臂周围清空并确认急停可用。
- 本工具只打开 USB 相机，不发送串口命令，也不会开启任何激光或机械臂输出。

## 采集条件

使用正式末端相机、安装姿态和图像设置：

```text
设备：/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0
分辨率：640x480
帧率：30 FPS
方向：flip none / rotate none
```

用量块或可靠量具固定“工具工作基准面到哑光卡”的实际距离。分别在 `15 mm`、`17.5 mm`、`20 mm` 采集；命令中的距离必须是现场实测值，不能填估计值。

## 运行工具

例如实测距离为 `17.5 mm`：

```bash
cd /home/jetson/jixiebi/jetson_nano_vision

DISPLAY=:0 /usr/bin/python3 -m jetson_vision.laser_offset_tuner \
  --camera /dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0 \
  --width 640 --height 480 --fps 30 --flip none \
  --work-distance-mm 17.5
```

红色参考十字默认从图像中心开始，不会用 `27.81 mm` 或任意 `mm/px` 猜测偏移。

- 鼠标左键：点击真实可见瞄准光斑中心。
- `a/d`、`i/k`：水平或垂直微调 `1 px`。
- `A/D`、`I/K`：水平或垂直微调 `10 px`。
- `s` 或 `p`：打印工作距离、实际取流分辨率、参考像素和相对图像中心的偏移。
- `q` 或 `Esc`：退出。

输出示例仅说明字段格式，不代表本机实测值：

```text
work_distance_mm=17.50 resolution=640x480 reference_pixel=(u,v) offset_px=(dx,dy)
```

对同一距离和同一选中视图重复测量至少 20 次，记录参考像素中位数及 X/Y 标准差。左、正、右三个实际定靶姿态都要检查。若 `15 / 17.5 / 20 mm` 之间或三个姿态之间的参考点变化超过验收阈值，应分别保存参考点或建立经验证的插值，不得强行共用一个常数。

## 使用实测偏移启动

`run_full_arm.sh` 会把末尾参数透传给 Qt。完成标定并经过复核后，使用工具打印的实测偏移：

```bash
cd /home/jetson/jixiebi/arm_mole_qt

DISPLAY=:0 bash run_full_arm.sh \
  --laser-offset-x-px <实测dx> \
  --laser-offset-y-px <实测dy>
```

当前联调脚本内置的曲面假人候选值为：

```text
resolution=640x480 reference_pixel=(398,226) offset_px=(78,-14)
```

该值来自 `/home/jetson/jixiebi/calibration/mono_end_camera/laser_aim_reference_candidate.json`。

不得把示例占位符替换为图像中心、当前黑痣坐标或旧日志中的误差。更换相机、镜头、分辨率、安装位置、工作距离或图像方向后必须重新标定。

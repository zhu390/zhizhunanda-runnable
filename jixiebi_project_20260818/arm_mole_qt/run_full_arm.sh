#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

END_CAMERA="/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0"
ASTRA_RGB_CAMERA="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
RA6M5_SERIAL="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
ASTRA_RGB_CALIBRATION="/home/jetson/jixiebi/calibration/astra_rgb/rgb_calib_params.npz"
# 640x480 @ 30 FPS, 17.5 mm curved mannequin visible aiming spot.
# Saved in calibration/mono_end_camera/laser_aim_reference_candidate.json.
LASER_OFFSET_X_PX=78
LASER_OFFSET_Y_PX=-14

for required_path in \
  "$END_CAMERA" \
  "$ASTRA_RGB_CAMERA" \
  "$RA6M5_SERIAL" \
  "$ASTRA_RGB_CALIBRATION"
do
  if [[ ! -e "$required_path" ]]; then
    echo "启动失败：设备或标定文件不存在：$required_path" >&2
    exit 2
  fi
done

echo "完整联动模式：已连接 CH340；点击 Qt 的‘开始’后机械臂会真实运动。" >&2
echo "首次联调请保持激光断电、清空机械臂周围并确认急停可用。" >&2

exec bash ./run_qt_app.sh \
  --camera-layout astra-rgbd \
  --camera "$END_CAMERA" \
  --width 640 --height 480 --fps 30 --ui-fps 20 \
  --flip none --rotate none \
  --camera-brightness 0 \
  --camera-gain 64 \
  --camera-backlight-compensation 0 \
  --facial-face-device 0 \
  --facial-face-imgsz 416 \
  --facial-face-refresh-frames 6 \
  --facial-stable-frames 4 \
  --facial-stable-seconds 0.35 \
  --spot-max-aspect-ratio 2.40 \
  --astra-always-on 1 \
  --astra-rgb-enabled 1 \
  --astra-rgb-camera "$ASTRA_RGB_CAMERA" \
  --astra-rgb-width 640 \
  --astra-rgb-height 480 \
  --astra-rgb-fps 30 \
  --astra-rgb-flip none \
  --astra-rgb-calibration "$ASTRA_RGB_CALIBRATION" \
  --astra-yolo-device 0 \
  --astra-yolo-imgsz 416 \
  --astra-yolo-interval 0.10 \
  --astra-depth-enabled 1 \
  --astra-depth-distortion-mode raw \
  --rgbd-distance-log \
  --rgbd-distance-log-interval 0.25 \
  --rgbd-continuous-stop-mm 120 \
  --rgbd-continuous-guard-interval 0.20 \
  --serial-safe-distance-interval 0.15 \
  --serial-port "$RA6M5_SERIAL" \
  --baudrate 115200 \
  --serial-protocol unified \
  --laser-offset-x-px "$LASER_OFFSET_X_PX" \
  --laser-offset-y-px "$LASER_OFFSET_Y_PX" \
  --rgbd-hardware-pair-verified \
  "$@"

"""Standalone RGB-D distance visualizer.

This process intentionally does not own the RA6M5 serial port.  In the formal
protocol-v5 application the Qt process is the single serial owner and consumes
the distance result produced by the RGB-D pipeline.
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from pyorbbecsdk import Config, OBSensorType, Pipeline

from depth_roi_distance import (
    RoiState,
    ScalarKalman1D,
    calc_distance,
    clamp_roi,
    draw_text,
    make_depth_image,
    make_status_text,
    normalize_roi,
    print_distance,
    update_distance_filter,
)
from orbbec_depth_click_3d import get_depth_intrinsics, read_depth_frame, rotate_180

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"
DEFAULT_MODEL = "../models/arm_face_yolo/best.pt"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run trained YOLO on Astra RGB and measure depth distance between arm_tip and face."
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--rgb-camera", default=DEFAULT_CAMERA)
    parser.add_argument("--rgb-width", type=int, default=640)
    parser.add_argument("--rgb-height", type=int, default=480)
    parser.add_argument("--rgb-fps", type=int, default=30)
    parser.add_argument("--fourcc", choices=["MJPG", "YUY2", "YUYV", "native"], default="MJPG")
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--window", default="Astra YOLO RGB-D Distance")
    parser.add_argument("--arm-name", default="arm_tip")
    parser.add_argument("--face-name", default="face")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="")
    parser.add_argument("--arm-bbox-scale", type=float, default=1.0)
    parser.add_argument("--face-bbox-scale", type=float, default=1.0)
    parser.add_argument("--arm-depth-offset-x", type=int, default=0)
    parser.add_argument("--arm-depth-offset-y", type=int, default=0)
    parser.add_argument("--face-depth-offset-x", type=int, default=0)
    parser.add_argument("--face-depth-offset-y", type=int, default=0)
    parser.add_argument("--min-depth-mm", type=float, default=20.0)
    parser.add_argument("--max-depth-mm", type=float, default=10000.0)
    parser.add_argument("--max-points", type=int, default=600)
    parser.add_argument("--distance-interval", type=float, default=0.2)
    parser.add_argument("--print-cooldown", type=float, default=0.5)
    parser.add_argument("--no-kalman", action="store_true")
    parser.add_argument("--kalman-distance-r-mm", type=float, default=20.0)
    parser.add_argument("--kalman-distance-process-mm", type=float, default=6.0)
    parser.add_argument("--kalman-distance-jump-mm", type=float, default=80.0)
    parser.add_argument(
        "--serial-port",
        default="",
        help=(
            "Disabled in this standalone tool. The formal Qt process is the "
            "only RA6M5 serial owner."
        ),
    )
    parser.add_argument("--rotate-180", action="store_true")
    parser.add_argument("--save-dir", default="yolo_rgb_depth_captures")
    return parser.parse_args()


def camera_value(value: str):
    return int(value) if str(value).isdigit() else value


def open_rgb_camera(args):
    cap = cv2.VideoCapture(camera_value(args.rgb_camera))
    if not cap.isOpened():
        raise RuntimeError("cannot open RGB camera {}".format(args.rgb_camera))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.rgb_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.rgb_height)
    cap.set(cv2.CAP_PROP_FPS, args.rgb_fps)
    if args.fourcc != "native":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def load_model(model_path: str):
    if YOLO is None:
        raise RuntimeError("ultralytics is not installed. Install it before running YOLO distance.")
    path = Path(model_path)
    if not path.exists():
        raise RuntimeError("model not found: {}".format(path))
    return YOLO(str(path))


def model_names(model):
    names = getattr(model, "names", {})
    return {int(k): str(v) for k, v in dict(names).items()}


def class_id_by_name(names, wanted: str):
    for class_id, name in names.items():
        if name == wanted:
            return class_id
    raise RuntimeError("class '{}' not found in model names: {}".format(wanted, names))


def best_detections(result, names, arm_id: int, face_id: int):
    detections = {}
    boxes = getattr(result, "boxes", None)
    if boxes is None or boxes.xyxy is None:
        return detections

    xyxy = boxes.xyxy.detach().cpu().numpy()
    cls = boxes.cls.detach().cpu().numpy().astype(np.int32)
    conf = boxes.conf.detach().cpu().numpy()
    for box, class_id, score in zip(xyxy, cls, conf):
        if class_id not in (arm_id, face_id):
            continue
        key = "arm" if class_id == arm_id else "face"
        prev = detections.get(key)
        if prev is None or float(score) > prev["conf"]:
            detections[key] = {
                "box": [float(v) for v in box],
                "conf": float(score),
                "class_id": int(class_id),
                "name": names.get(int(class_id), str(class_id)),
            }
    return detections


def scaled_box(box, scale: float, image_w: int, image_h: int):
    x0, y0, x1, y1 = [float(v) for v in box]
    cx = (x0 + x1) * 0.5
    cy = (y0 + y1) * 0.5
    w = max(2.0, (x1 - x0) * max(scale, 0.05))
    h = max(2.0, (y1 - y0) * max(scale, 0.05))
    return (
        max(0.0, min(float(image_w - 1), cx - w * 0.5)),
        max(0.0, min(float(image_h - 1), cy - h * 0.5)),
        max(0.0, min(float(image_w - 1), cx + w * 0.5)),
        max(0.0, min(float(image_h - 1), cy + h * 0.5)),
    )


def box_to_depth_roi(box, rgb_shape, depth_shape, scale: float, offset_x: int, offset_y: int):
    rgb_h, rgb_w = rgb_shape[:2]
    depth_h, depth_w = depth_shape[:2]
    x0, y0, x1, y1 = scaled_box(box, scale, rgb_w, rgb_h)
    dx0 = int(round(x0 * float(depth_w) / float(rgb_w))) + int(offset_x)
    dx1 = int(round(x1 * float(depth_w) / float(rgb_w))) + int(offset_x)
    dy0 = int(round(y0 * float(depth_h) / float(rgb_h))) + int(offset_y)
    dy1 = int(round(y1 * float(depth_h) / float(rgb_h))) + int(offset_y)
    return clamp_roi(normalize_roi((dx0, dy0), (dx1, dy1)), depth_w, depth_h)


def draw_rgb_detection(image, detection, color):
    x0, y0, x1, y1 = [int(round(v)) for v in detection["box"]]
    cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)
    label = "{} {:.2f}".format(detection["name"], detection["conf"])
    draw_text(image, label, (x0 + 4, max(18, y0 - 6)), 0.55)


def draw_depth_roi(image, roi, label, color):
    x0, y0, x1, y1 = roi
    cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)
    draw_text(image, label, (x0 + 4, max(18, y0 - 6)), 0.5)


def fit_to_height(image, target_height):
    h, w = image.shape[:2]
    if h == target_height:
        return image
    scale = float(target_height) / float(h)
    return cv2.resize(image, (max(1, int(round(w * scale))), target_height), interpolation=cv2.INTER_AREA)


def make_sheet(rgb, depth):
    target_h = min(rgb.shape[0], depth.shape[0])
    return np.hstack([fit_to_height(rgb, target_h), fit_to_height(depth, target_h)])


def reset_distance_state(state: RoiState) -> None:
    state.distance_info = None
    state.last_distance_time = 0.0
    if state.distance_filter is not None:
        state.distance_filter.reset()


def main() -> int:
    args = parse_args()
    if args.serial_port:
        print(
            "ERROR: standalone RGB-D serial output is disabled. "
            "Run the formal Qt application; it is the sole RA6M5 serial owner."
        )
        return 2

    model = load_model(args.model)
    names = model_names(model)
    arm_id = class_id_by_name(names, args.arm_name)
    face_id = class_id_by_name(names, args.face_name)

    rgb_cap = open_rgb_camera(args)
    pipeline = Pipeline()
    config = Config()
    profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    if profile_list is None:
        print("ERROR: no depth stream profile")
        rgb_cap.release()
        return 1

    depth_profile = profile_list.get_default_video_stream_profile()
    if depth_profile is None:
        print("ERROR: no default depth stream profile")
        rgb_cap.release()
        return 1

    print("model:", args.model)
    print("model classes:", names)
    print("RGB camera:", args.rgb_camera)
    print("depth profile:", depth_profile)
    config.enable_stream(depth_profile)
    pipeline.start(config)

    intr = get_depth_intrinsics(pipeline)
    print("depth intrinsics from SDK:", intr)
    if intr is None:
        pipeline.stop()
        rgb_cap.release()
        print("ERROR: no depth intrinsics")
        return 1

    state = RoiState(1.0, args.rotate_180)
    if not args.no_kalman:
        state.distance_filter = ScalarKalman1D(
            args.kalman_distance_r_mm,
            args.kalman_distance_process_mm,
            args.kalman_distance_jump_mm,
        )

    print("Press S to save current sheet, Q/Esc to quit.")
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    try:
        while True:
            ok, rgb = rgb_cap.read()
            if not ok or rgb is None:
                time.sleep(0.01)
                continue

            frames = pipeline.wait_for_frames(args.timeout_ms)
            if frames is None:
                continue
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue

            depth_mm = read_depth_frame(depth_frame)
            if args.rotate_180:
                rgb = rotate_180(rgb)
                depth_mm = rotate_180(depth_mm)

            depth_h, depth_w = depth_mm.shape[:2]
            state.update_shape(depth_w, depth_h)

            prediction = model.predict(
                source=rgb,
                imgsz=args.imgsz,
                conf=args.conf,
                iou=args.iou,
                device=args.device or None,
                verbose=False,
            )[0]
            detections = best_detections(prediction, names, arm_id, face_id)

            rgb_shown = rgb.copy()
            depth_shown = make_depth_image(depth_mm, args.min_depth_mm, args.max_depth_mm)
            state.rois.clear()

            if "arm" in detections:
                draw_rgb_detection(rgb_shown, detections["arm"], (255, 0, 0))
                state.rois["arm"] = box_to_depth_roi(
                    detections["arm"]["box"],
                    rgb.shape,
                    depth_mm.shape,
                    args.arm_bbox_scale,
                    args.arm_depth_offset_x,
                    args.arm_depth_offset_y,
                )
                draw_depth_roi(depth_shown, state.rois["arm"], "arm_tip", (255, 0, 0))
            if "face" in detections:
                draw_rgb_detection(rgb_shown, detections["face"], (255, 255, 0))
                state.rois["face"] = box_to_depth_roi(
                    detections["face"]["box"],
                    rgb.shape,
                    depth_mm.shape,
                    args.face_bbox_scale,
                    args.face_depth_offset_x,
                    args.face_depth_offset_y,
                )
                draw_depth_roi(depth_shown, state.rois["face"], "face", (255, 255, 0))

            missing = []
            if "arm" not in state.rois:
                missing.append(args.arm_name)
            if "face" not in state.rois:
                missing.append(args.face_name)

            now = time.time()
            if "arm" in state.rois and "face" in state.rois:
                if state.distance_info is None or now - state.last_distance_time >= args.distance_interval:
                    state.distance_info = calc_distance(depth_mm, state, intr, args)
                    state.distance_info = update_distance_filter(state.distance_info, state.distance_filter, now)
                    state.last_distance_time = now
                    print_distance(state.distance_info, state, args.print_cooldown)
            else:
                reset_distance_state(state)

            draw_text(rgb_shown, "YOLO conf={:.2f}".format(args.conf), (12, 28))
            if missing:
                draw_text(depth_shown, "missing: {}".format(",".join(missing)), (12, 28))
            else:
                draw_text(depth_shown, make_status_text(state.distance_info), (12, 28))

            sheet = make_sheet(rgb_shown, depth_shown)
            cv2.imshow(args.window, sheet)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("s"), ord("S")):
                save_dir = Path(args.save_dir)
                save_dir.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                path = save_dir / "yolo_rgb_depth_{}.png".format(stamp)
                if cv2.imwrite(str(path), sheet):
                    print("saved:", path)
    finally:
        rgb_cap.release()
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

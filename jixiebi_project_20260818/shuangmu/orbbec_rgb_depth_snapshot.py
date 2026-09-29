import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from pyorbbecsdk import Config, OBSensorType, Pipeline


MIN_DEPTH_MM = 20
MAX_DEPTH_MM = 10000


def parse_args():
    parser = argparse.ArgumentParser(description="Save RGB + depth snapshot from Astra Pro Plus.")
    parser.add_argument("--rgb-camera", type=int, default=0)
    parser.add_argument("--rgb-width", type=int, default=640)
    parser.add_argument("--rgb-height", type=int, default=480)
    parser.add_argument("--rgb-fps", type=int, default=30)
    parser.add_argument("--fourcc", choices=["MJPG", "YUY2", "YUYV", "native"], default="MJPG")
    parser.add_argument("--warmup", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--out", default="rgb_depth_snapshot")
    parser.add_argument("--rotate-180", action="store_true", help="Rotate saved RGB and depth images by 180 degrees.")
    return parser.parse_args()


def open_rgb_camera(args):
    cap = cv2.VideoCapture(args.rgb_camera)
    if not cap.isOpened():
        raise RuntimeError("cannot open RGB camera index {}".format(args.rgb_camera))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.rgb_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.rgb_height)
    cap.set(cv2.CAP_PROP_FPS, args.rgb_fps)
    if args.fourcc != "native":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def read_rgb_frame(cap, warmup):
    frame = None
    ok = False
    for _ in range(max(1, warmup)):
        ok, frame = cap.read()
        time.sleep(0.01)
    if not ok or frame is None:
        raise RuntimeError("failed to read RGB frame")
    return frame


def read_depth_frame(depth_frame):
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    scale = depth_frame.get_depth_scale()
    depth = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
    return depth.astype(np.float32) * float(scale)


def depth_to_colormap(depth_mm):
    valid = (depth_mm > MIN_DEPTH_MM) & (depth_mm < MAX_DEPTH_MM)
    shown_depth = np.where(valid, depth_mm, 0).astype(np.uint16)
    depth_u8 = cv2.normalize(shown_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
    invalid = ~valid
    color[invalid] = (0, 0, 0)
    return color, valid


def put_label(image, text):
    cv2.putText(image, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)


def fit_to_height(image, target_height):
    h, w = image.shape[:2]
    if h == target_height:
        return image
    scale = float(target_height) / float(h)
    return cv2.resize(image, (max(1, int(w * scale)), target_height), interpolation=cv2.INTER_AREA)


def make_sheet(rgb, depth_color, rgb_label):
    target_h = min(rgb.shape[0], depth_color.shape[0])
    rgb_fit = fit_to_height(rgb, target_h)
    depth_fit = fit_to_height(depth_color, target_h)
    put_label(rgb_fit, rgb_label)
    put_label(depth_fit, "Depth colormap")
    return np.hstack([rgb_fit, depth_fit])


def rotate_180(image):
    return cv2.rotate(image, cv2.ROTATE_180)


def main():
    args = parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    rgb_cap = open_rgb_camera(args)

    pipeline = Pipeline()
    config = Config()
    profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    if profiles is None:
        raise RuntimeError("no depth stream profile")
    depth_profile = profiles.get_default_video_stream_profile()
    if depth_profile is None:
        raise RuntimeError("no default depth stream profile")
    print("depth profile:", depth_profile)
    config.enable_stream(depth_profile)
    pipeline.start(config)

    try:
        rgb = read_rgb_frame(rgb_cap, args.warmup)

        depth_frame = None
        for _ in range(30):
            frames = pipeline.wait_for_frames(args.timeout_ms)
            if frames is None:
                continue
            depth_frame = frames.get_depth_frame()
            if depth_frame is not None:
                break
        if depth_frame is None:
            raise RuntimeError("failed to read depth frame")

        depth_mm = read_depth_frame(depth_frame)
        if args.rotate_180:
            rgb = rotate_180(rgb)
            depth_mm = rotate_180(depth_mm)
        depth_color, valid = depth_to_colormap(depth_mm)
        sheet = make_sheet(rgb.copy(), depth_color.copy(), "RGB /dev/video{}".format(args.rgb_camera))

        valid_depth = depth_mm[valid]
        if valid_depth.size > 0:
            print(
                "valid depth count={}, min={:.1f} mm, max={:.1f} mm, median={:.1f} mm".format(
                    int(valid_depth.size),
                    float(np.min(valid_depth)),
                    float(np.max(valid_depth)),
                    float(np.median(valid_depth)),
                )
            )
        else:
            print("valid depth count=0")

        rgb_path = out_dir / "rgb_frame.png"
        depth_color_path = out_dir / "depth_colormap.png"
        sheet_path = out_dir / "rgb_depth_sheet.png"
        raw_path = out_dir / "depth_raw_mm.npy"

        cv2.imwrite(str(rgb_path), rgb)
        cv2.imwrite(str(depth_color_path), depth_color)
        cv2.imwrite(str(sheet_path), sheet)
        np.save(str(raw_path), depth_mm)

        print("saved:", rgb_path)
        print("saved:", depth_color_path)
        print("saved:", sheet_path)
        print("saved:", raw_path)
    finally:
        rgb_cap.release()
        pipeline.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

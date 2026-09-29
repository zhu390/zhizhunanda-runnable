import argparse
import time
from typing import Optional, Tuple

import cv2
import numpy as np
from pyorbbecsdk import Config, OBSensorType, Pipeline

try:
    from pyorbbecsdk import OBFormat, PointCloudFilter
except Exception:
    OBFormat = None
    PointCloudFilter = None


MIN_DEPTH_MM = 20
MAX_DEPTH_MM = 10000


class ClickState:
    def __init__(self) -> None:
        self.xy = None  # type: Optional[Tuple[int, int]]
        self.last_print_time = 0.0


class Intrinsics:
    def __init__(self, fx: float, fy: float, cx: float, cy: float) -> None:
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def __repr__(self) -> str:
        return "Intrinsics(fx={:.3f}, fy={:.3f}, cx={:.3f}, cy={:.3f})".format(
            self.fx, self.fy, self.cx, self.cy
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Click on the Orbbec depth image and print the clicked point 3D coordinate."
    )
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--window", default="Orbbec Depth Click 3D")
    parser.add_argument("--preview-scale", type=float, default=1.0)
    parser.add_argument("--print-cooldown", type=float, default=0.3)
    parser.add_argument("--fx", type=float, default=0.0, help="Fallback depth fx, mm calculation only.")
    parser.add_argument("--fy", type=float, default=0.0, help="Fallback depth fy, mm calculation only.")
    parser.add_argument("--cx", type=float, default=0.0, help="Fallback depth cx, mm calculation only.")
    parser.add_argument("--cy", type=float, default=0.0, help="Fallback depth cy, mm calculation only.")
    parser.add_argument("--rotate-180", action="store_true", help="Rotate the displayed depth image by 180 degrees.")
    return parser.parse_args()


def mouse_callback(event, x, y, flags, state: ClickState):
    if event == cv2.EVENT_LBUTTONDOWN:
        state.xy = (int(x), int(y))


def make_depth_image(depth_mm: np.ndarray) -> np.ndarray:
    shown_depth = np.where(
        (depth_mm > MIN_DEPTH_MM) & (depth_mm < MAX_DEPTH_MM),
        depth_mm,
        0,
    ).astype(np.uint16)
    depth_u8 = cv2.normalize(shown_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    return cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)


def rotate_180(image: np.ndarray) -> np.ndarray:
    return cv2.rotate(image, cv2.ROTATE_180)


def display_to_raw_xy(display_x: int, display_y: int, width: int, height: int, rotate_180_enabled: bool):
    if rotate_180_enabled:
        return width - 1 - display_x, height - 1 - display_y
    return display_x, display_y


def read_depth_frame(depth_frame) -> np.ndarray:
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    scale = depth_frame.get_depth_scale()
    depth = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))
    return depth.astype(np.float32) * float(scale)


def attr_any(obj, names):
    for name in names:
        if hasattr(obj, name):
            value = getattr(obj, name)
            if callable(value):
                try:
                    value = value()
                except TypeError:
                    pass
            return value
    return None


def intrinsics_from_obj(obj) -> Optional[Intrinsics]:
    if obj is None:
        return None

    fx = attr_any(obj, ("fx", "focalLengthX", "focal_length_x"))
    fy = attr_any(obj, ("fy", "focalLengthY", "focal_length_y"))
    cx = attr_any(obj, ("cx", "principalPointX", "principal_point_x", "ppx"))
    cy = attr_any(obj, ("cy", "principalPointY", "principal_point_y", "ppy"))

    if fx is None or fy is None or cx is None or cy is None:
        return None
    return Intrinsics(float(fx), float(fy), float(cx), float(cy))


def get_depth_intrinsics(pipeline) -> Optional[Intrinsics]:
    try:
        camera_param = pipeline.get_camera_param()
    except Exception:
        return None

    candidates = [
        camera_param,
        attr_any(camera_param, ("depth_intrinsic", "depthIntrinsic", "depth_intrinsics", "depthIntrinsicParam")),
        attr_any(camera_param, ("depth_intrinsic_param", "depthIntrinsicParam")),
    ]

    for candidate in candidates:
        intr = intrinsics_from_obj(candidate)
        if intr is not None:
            return intr
    return None


def calc_point_from_intrinsics(x: int, y: int, z_mm: float, intr: Optional[Intrinsics]):
    if intr is None or z_mm <= 0:
        return None
    px = (float(x) - intr.cx) * z_mm / intr.fx
    py = (float(y) - intr.cy) * z_mm / intr.fy
    return px, py, z_mm


def calc_clicked_point_with_pointcloud(pipeline, frames, depth_frame, x: int, y: int):
    if PointCloudFilter is None or OBFormat is None:
        return None

    camera_param = pipeline.get_camera_param()
    point_cloud_filter = PointCloudFilter()
    point_cloud_filter.set_camera_param(camera_param)
    point_cloud_filter.set_position_data_scaled(depth_frame.get_depth_scale())
    point_cloud_filter.set_create_point_format(OBFormat.POINT)

    point_cloud_frame = point_cloud_filter.process(frames)
    points = np.asarray(point_cloud_filter.calculate(point_cloud_frame), dtype=np.float32)

    width = depth_frame.get_width()
    height = depth_frame.get_height()
    if points.shape[0] < width * height:
        return None

    point = points[y * width + x]
    if point.shape[0] < 3:
        return None
    return float(point[0]), float(point[1]), float(point[2])


def main() -> int:
    args = parse_args()

    config = Config()
    pipeline = Pipeline()

    profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    if profile_list is None:
        print("ERROR: no depth stream profile")
        return 1

    depth_profile = profile_list.get_default_video_stream_profile()
    if depth_profile is None:
        print("ERROR: no default depth stream profile")
        return 1

    print("depth profile:", depth_profile)
    config.enable_stream(depth_profile)
    pipeline.start(config)

    fallback_intrinsics = None
    if args.fx > 0 and args.fy > 0:
        fallback_intrinsics = Intrinsics(args.fx, args.fy, args.cx, args.cy)
        print("using manual fallback depth intrinsics:", fallback_intrinsics)
    else:
        fallback_intrinsics = get_depth_intrinsics(pipeline)
        print("depth intrinsics from SDK:", fallback_intrinsics)

    state = ClickState()
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(args.window, mouse_callback, state)

    print("Left-click a point in the Depth window. The terminal will print its 3D coordinate.")
    print("Press Q or Esc to quit.")

    try:
        while True:
            frames = pipeline.wait_for_frames(args.timeout_ms)
            if frames is None:
                continue

            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue

            depth_mm = read_depth_frame(depth_frame)
            height, width = depth_mm.shape[:2]
            shown = make_depth_image(depth_mm)
            if args.rotate_180:
                shown = rotate_180(shown)

            if state.xy is not None:
                display_x, display_y = state.xy
                if args.preview_scale != 1.0:
                    display_x = int(round(display_x / args.preview_scale))
                    display_y = int(round(display_y / args.preview_scale))

                raw_x, raw_y = display_to_raw_xy(display_x, display_y, width, height, args.rotate_180)

                if 0 <= display_x < width and 0 <= display_y < height and 0 <= raw_x < width and 0 <= raw_y < height:
                    cv2.drawMarker(
                        shown,
                        (display_x, display_y),
                        (255, 255, 255),
                        markerType=cv2.MARKER_CROSS,
                        markerSize=18,
                        thickness=2,
                    )
                    now = time.time()
                    if now - state.last_print_time >= args.print_cooldown:
                        z = float(depth_mm[raw_y, raw_x])
                        point = None
                        try:
                            point = calc_clicked_point_with_pointcloud(pipeline, frames, depth_frame, raw_x, raw_y)
                        except Exception as exc:
                            print("point cloud calc failed:", exc)
                        if point is None:
                            point = calc_point_from_intrinsics(raw_x, raw_y, z, fallback_intrinsics)

                        if point is None:
                            print(
                                "display_pixel=({}, {}), raw_pixel=({}, {}), depth_z={:.1f} mm, 3D point unavailable".format(
                                    display_x, display_y, raw_x, raw_y, z
                                )
                            )
                        else:
                            px, py, pz = point
                            print(
                                "display_pixel=({}, {}), raw_pixel=({}, {}), X={:.1f} mm, Y={:.1f} mm, Z={:.1f} mm, depth_z={:.1f} mm".format(
                                    display_x, display_y, raw_x, raw_y, px, py, pz, z
                                )
                            )
                        state.last_print_time = now
                        state.xy = None

            cv2.putText(
                shown,
                "left click: print 3D point | Q/Esc: quit",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            if args.preview_scale != 1.0:
                preview_w = max(1, int(width * args.preview_scale))
                preview_h = max(1, int(height * args.preview_scale))
                shown = cv2.resize(shown, (preview_w, preview_h), interpolation=cv2.INTER_AREA)

            cv2.imshow(args.window, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

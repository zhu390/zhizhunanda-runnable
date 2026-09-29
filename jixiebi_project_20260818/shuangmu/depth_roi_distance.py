import argparse
import json
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
from pyorbbecsdk import Config, OBSensorType, Pipeline

from orbbec_depth_click_3d import (
    Intrinsics,
    display_to_raw_xy,
    get_depth_intrinsics,
    read_depth_frame,
    rotate_180,
)


MIN_DEPTH_MM = 20.0
MAX_DEPTH_MM = 10000.0
DEFAULT_MAX_POINTS = 600
DEFAULT_ROI_FILE = "depth_roi_rois.json"

Roi = Tuple[int, int, int, int]


class RoiState:
    def __init__(self, preview_scale: float, rotate_180_enabled: bool) -> None:
        self.preview_scale = preview_scale
        self.rotate_180_enabled = rotate_180_enabled
        self.width = 0
        self.height = 0
        self.active = "arm"
        self.rois = {}  # type: Dict[str, Roi]
        self.drag_start = None  # type: Optional[Tuple[int, int]]
        self.drag_current = None  # type: Optional[Tuple[int, int]]
        self.last_print_time = 0.0
        self.last_distance_time = 0.0
        self.distance_info = None
        self.distance_filter = None
        self.roi_dirty = False
        self.auto_track_enabled = False

    def update_shape(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

    def preview_to_raw(self, x: int, y: int) -> Optional[Tuple[int, int]]:
        if self.width <= 0 or self.height <= 0:
            return None

        display_x = int(round(float(x) / self.preview_scale))
        display_y = int(round(float(y) / self.preview_scale))
        display_x = max(0, min(self.width - 1, display_x))
        display_y = max(0, min(self.height - 1, display_y))
        return display_to_raw_xy(display_x, display_y, self.width, self.height, self.rotate_180_enabled)

    def current_drag_roi(self) -> Optional[Roi]:
        if self.drag_start is None or self.drag_current is None:
            return None
        return normalize_roi(self.drag_start, self.drag_current)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure 3D distance between two selected ROIs on the Orbbec depth image."
    )
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--window", default="Astra ROI Distance")
    parser.add_argument("--preview-scale", type=float, default=1.0)
    parser.add_argument("--print-cooldown", type=float, default=0.5)
    parser.add_argument("--distance-interval", type=float, default=0.2)
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS)
    parser.add_argument("--roi-file", default=DEFAULT_ROI_FILE)
    parser.add_argument("--no-load-roi", action="store_true")
    parser.add_argument("--no-auto-save-roi", action="store_true")
    parser.add_argument("--auto-track-roi", action="store_true")
    parser.add_argument("--track-margin", type=int, default=45)
    parser.add_argument("--track-depth-window-mm", type=float, default=90.0)
    parser.add_argument("--track-min-area", type=int, default=20)
    parser.add_argument("--min-depth-mm", type=float, default=MIN_DEPTH_MM)
    parser.add_argument("--max-depth-mm", type=float, default=MAX_DEPTH_MM)
    parser.add_argument("--no-kalman", action="store_true")
    parser.add_argument("--kalman-distance-r-mm", type=float, default=20.0)
    parser.add_argument("--kalman-distance-process-mm", type=float, default=6.0)
    parser.add_argument("--kalman-distance-jump-mm", type=float, default=80.0)
    parser.add_argument("--fx", type=float, default=0.0, help="Fallback depth fx.")
    parser.add_argument("--fy", type=float, default=0.0, help="Fallback depth fy.")
    parser.add_argument("--cx", type=float, default=0.0, help="Fallback depth cx.")
    parser.add_argument("--cy", type=float, default=0.0, help="Fallback depth cy.")
    parser.add_argument("--rotate-180", action="store_true", help="Rotate the displayed depth image by 180 degrees.")
    return parser.parse_args()


def roi_to_list(roi: Roi):
    return [int(v) for v in roi]


def list_to_roi(values) -> Optional[Roi]:
    if not isinstance(values, list) or len(values) != 4:
        return None
    try:
        x0, y0, x1, y1 = [int(v) for v in values]
    except (TypeError, ValueError):
        return None
    return normalize_roi((x0, y0), (x1, y1))


def load_rois(path: Path) -> Dict[str, Roi]:
    if not path.exists():
        return {}
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print("WARNING: failed to load ROI file {}: {}".format(path, exc))
        return {}

    rois = {}
    raw_rois = data.get("rois", {})
    for name in ("arm", "face"):
        roi = list_to_roi(raw_rois.get(name))
        if roi is not None:
            rois[name] = roi
    return rois


def save_rois(path: Path, rois: Dict[str, Roi], width: int, height: int) -> bool:
    if "arm" not in rois or "face" not in rois:
        print("ROI file not saved: select both arm and face first.")
        return False

    data = {
        "version": 1,
        "image_size": [int(width), int(height)],
        "rois": {
            "arm": roi_to_list(rois["arm"]),
            "face": roi_to_list(rois["face"]),
        },
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
            f.write("\n")
        print("saved ROI file:", path)
        return True
    except OSError as exc:
        print("WARNING: failed to save ROI file {}: {}".format(path, exc))
        return False


def normalize_roi(p0: Tuple[int, int], p1: Tuple[int, int]) -> Roi:
    x0, y0 = p0
    x1, y1 = p1
    left = min(x0, x1)
    top = min(y0, y1)
    right = max(x0, x1)
    bottom = max(y0, y1)
    return left, top, right, bottom


def roi_size(roi: Roi) -> Tuple[int, int]:
    x0, y0, x1, y1 = roi
    return max(0, x1 - x0 + 1), max(0, y1 - y0 + 1)


def roi_center(roi: Roi) -> Tuple[float, float]:
    x0, y0, x1, y1 = roi
    return (float(x0 + x1) * 0.5, float(y0 + y1) * 0.5)


def clamp_roi(roi: Roi, width: int, height: int) -> Roi:
    x0, y0, x1, y1 = roi
    return (
        max(0, min(width - 1, x0)),
        max(0, min(height - 1, y0)),
        max(0, min(width - 1, x1)),
        max(0, min(height - 1, y1)),
    )


def raw_to_display_xy(x: int, y: int, width: int, height: int, rotate_180_enabled: bool) -> Tuple[int, int]:
    if rotate_180_enabled:
        return width - 1 - x, height - 1 - y
    return x, y


def raw_roi_to_display_roi(roi: Roi, width: int, height: int, rotate_180_enabled: bool) -> Roi:
    x0, y0, x1, y1 = roi
    d0 = raw_to_display_xy(x0, y0, width, height, rotate_180_enabled)
    d1 = raw_to_display_xy(x1, y1, width, height, rotate_180_enabled)
    return normalize_roi(d0, d1)


def mouse_callback(event, x, y, flags, state: RoiState):
    raw_xy = state.preview_to_raw(int(x), int(y))
    if raw_xy is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        state.drag_start = raw_xy
        state.drag_current = raw_xy
    elif event == cv2.EVENT_MOUSEMOVE and state.drag_start is not None:
        state.drag_current = raw_xy
    elif event == cv2.EVENT_LBUTTONUP and state.drag_start is not None:
        state.drag_current = raw_xy
        roi = state.current_drag_roi()
        if roi is not None:
            w, h = roi_size(roi)
            if w >= 3 and h >= 3:
                state.rois[state.active] = roi
                state.distance_info = None
                state.last_distance_time = 0.0
                if state.distance_filter is not None:
                    state.distance_filter.reset()
                state.roi_dirty = True
        state.drag_start = None
        state.drag_current = None


class ScalarKalman1D:
    def __init__(self, measurement_std: float, process_std: float, jump_reset: float) -> None:
        self.measurement_var = float(max(measurement_std, 0.1)) ** 2
        self.process_var = float(max(process_std, 0.01)) ** 2
        self.jump_reset = float(max(jump_reset, 1.0))
        self.x = None
        self.p = None
        self.last_time = None

    def reset(self) -> None:
        self.x = None
        self.p = None
        self.last_time = None

    def update(self, measurement: float, now: float) -> float:
        z = float(measurement)
        if self.x is None or self.p is None or self.last_time is None:
            self.x = z
            self.p = self.measurement_var
            self.last_time = now
            return self.x

        if abs(z - self.x) > self.jump_reset:
            self.reset()
            return self.update(z, now)

        dt = max(1e-3, min(float(now - self.last_time), 0.5))
        self.last_time = now

        self.p += self.process_var * dt
        kalman_gain = self.p / (self.p + self.measurement_var)
        self.x = self.x + kalman_gain * (z - self.x)
        self.p = (1.0 - kalman_gain) * self.p
        return self.x


def make_depth_image(depth_mm: np.ndarray, min_depth_mm: float, max_depth_mm: float) -> np.ndarray:
    valid = (depth_mm > min_depth_mm) & (depth_mm < max_depth_mm)
    shown_depth = np.where(valid, depth_mm, 0).astype(np.uint16)
    depth_u8 = cv2.normalize(shown_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
    color[~valid] = (0, 0, 0)
    return color


def roi_to_points(
    depth_mm: np.ndarray,
    roi: Roi,
    intr: Intrinsics,
    min_depth_mm: float,
    max_depth_mm: float,
    max_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    height, width = depth_mm.shape[:2]
    x0, y0, x1, y1 = roi
    x0 = max(0, min(width - 1, x0))
    x1 = max(0, min(width - 1, x1))
    y0 = max(0, min(height - 1, y0))
    y1 = max(0, min(height - 1, y1))
    if x1 < x0 or y1 < y0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.float32)

    roi_depth = depth_mm[y0 : y1 + 1, x0 : x1 + 1]
    valid = (roi_depth > min_depth_mm) & (roi_depth < max_depth_mm)
    ys, xs = np.nonzero(valid)
    if xs.size == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.float32)

    xs = xs.astype(np.float32) + float(x0)
    ys = ys.astype(np.float32) + float(y0)
    z = roi_depth[valid].astype(np.float32)

    if z.size > max_points > 0:
        idx = np.linspace(0, z.size - 1, max_points).astype(np.int32)
        xs = xs[idx]
        ys = ys[idx]
        z = z[idx]

    x = (xs - intr.cx) * z / intr.fx
    y = (ys - intr.cy) * z / intr.fy
    points = np.column_stack((x, y, z)).astype(np.float32)
    return points, z


def track_roi_by_depth(
    depth_mm: np.ndarray,
    roi: Roi,
    min_depth_mm: float,
    max_depth_mm: float,
    margin: int,
    depth_window_mm: float,
    min_area: int,
) -> Optional[Roi]:
    height, width = depth_mm.shape[:2]
    roi = clamp_roi(roi, width, height)
    x0, y0, x1, y1 = roi

    current_depth = depth_mm[y0 : y1 + 1, x0 : x1 + 1]
    current_valid = (current_depth > min_depth_mm) & (current_depth < max_depth_mm)
    if int(np.count_nonzero(current_valid)) < min_area:
        return None

    target_z = float(np.median(current_depth[current_valid]))
    sx0 = max(0, x0 - margin)
    sy0 = max(0, y0 - margin)
    sx1 = min(width - 1, x1 + margin)
    sy1 = min(height - 1, y1 + margin)

    search_depth = depth_mm[sy0 : sy1 + 1, sx0 : sx1 + 1]
    valid = (
        (search_depth > min_depth_mm)
        & (search_depth < max_depth_mm)
        & (np.abs(search_depth - target_z) <= depth_window_mm)
    )
    mask = valid.astype(np.uint8)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if num_labels <= 1:
        return None

    old_cx, old_cy = roi_center(roi)
    best_label = None
    best_score = None
    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        cx = float(centroids[label][0]) + float(sx0)
        cy = float(centroids[label][1]) + float(sy0)
        center_dist = np.hypot(cx - old_cx, cy - old_cy)
        score = center_dist - min(area, 300) * 0.02
        if best_score is None or score < best_score:
            best_score = score
            best_label = label

    if best_label is None:
        return None

    bx = int(stats[best_label, cv2.CC_STAT_LEFT]) + sx0
    by = int(stats[best_label, cv2.CC_STAT_TOP]) + sy0
    bw = int(stats[best_label, cv2.CC_STAT_WIDTH])
    bh = int(stats[best_label, cv2.CC_STAT_HEIGHT])
    tracked = (bx, by, bx + bw - 1, by + bh - 1)
    tw, th = roi_size(tracked)
    if tw < 3 or th < 3:
        return None
    return clamp_roi(tracked, width, height)


def auto_track_rois(depth_mm: np.ndarray, state: RoiState, args) -> None:
    updated = False
    for name in ("arm", "face"):
        if name not in state.rois:
            continue
        tracked = track_roi_by_depth(
            depth_mm,
            state.rois[name],
            args.min_depth_mm,
            args.max_depth_mm,
            args.track_margin,
            args.track_depth_window_mm,
            args.track_min_area,
        )
        if tracked is not None and tracked != state.rois[name]:
            state.rois[name] = tracked
            updated = True
    if updated:
        state.distance_info = None
        state.last_distance_time = 0.0


def nearest_distances_mm(a: np.ndarray, b: np.ndarray, chunk_size: int = 256) -> np.ndarray:
    if a.size == 0 or b.size == 0:
        return np.empty((0,), dtype=np.float32)

    nearest = np.empty((a.shape[0],), dtype=np.float32)
    b64 = b.astype(np.float64, copy=False)
    for start in range(0, a.shape[0], chunk_size):
        stop = min(start + chunk_size, a.shape[0])
        diff = a[start:stop, None, :].astype(np.float64, copy=False) - b64[None, :, :]
        dist2 = np.sum(diff * diff, axis=2)
        nearest[start:stop] = np.sqrt(np.min(dist2, axis=1)).astype(np.float32)
    return nearest


def calc_distance(depth_mm: np.ndarray, state: RoiState, intr: Optional[Intrinsics], args):
    if intr is None:
        return None
    if "arm" not in state.rois or "face" not in state.rois:
        return None

    arm_points, arm_z = roi_to_points(
        depth_mm,
        state.rois["arm"],
        intr,
        args.min_depth_mm,
        args.max_depth_mm,
        args.max_points,
    )
    face_points, face_z = roi_to_points(
        depth_mm,
        state.rois["face"],
        intr,
        args.min_depth_mm,
        args.max_depth_mm,
        args.max_points,
    )
    if arm_points.size == 0 or face_points.size == 0:
        return {
            "arm_count": int(arm_points.shape[0]),
            "face_count": int(face_points.shape[0]),
        }

    arm_to_face = nearest_distances_mm(arm_points, face_points)
    face_to_arm = nearest_distances_mm(face_points, arm_points)
    nearest = np.concatenate((arm_to_face, face_to_arm))

    return {
        "arm_count": int(arm_points.shape[0]),
        "face_count": int(face_points.shape[0]),
        "arm_z_median": float(np.median(arm_z)),
        "face_z_median": float(np.median(face_z)),
        "min_distance": float(np.min(nearest)),
        "p05_distance": float(np.percentile(nearest, 5)),
        "median_nearest": float(np.median(nearest)),
    }


def update_distance_filter(distance_info, distance_filter, now: float):
    if distance_filter is None:
        return distance_info
    if distance_info is None:
        distance_filter.reset()
        return None
    if distance_info.get("arm_count", 0) == 0 or distance_info.get("face_count", 0) == 0:
        distance_filter.reset()
        return distance_info
    filtered = distance_filter.update(distance_info["p05_distance"], now)
    distance_info["filtered_p05_distance"] = float(filtered)
    return distance_info


def draw_text(image: np.ndarray, text: str, origin: Tuple[int, int], scale: float = 0.55) -> None:
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def draw_roi(
    image: np.ndarray,
    roi: Roi,
    label: str,
    color: Tuple[int, int, int],
    state: RoiState,
    thickness: int = 2,
) -> None:
    display_roi = raw_roi_to_display_roi(roi, state.width, state.height, state.rotate_180_enabled)
    x0, y0, x1, y1 = display_roi
    cv2.rectangle(image, (x0, y0), (x1, y1), color, thickness)
    draw_text(image, label, (x0 + 4, max(18, y0 - 6)), 0.5)


def make_status_text(distance_info) -> str:
    if distance_info is None:
        return "select arm ROI with 1, face ROI with 2"
    if distance_info.get("arm_count", 0) == 0 or distance_info.get("face_count", 0) == 0:
        return "invalid depth in ROI: arm={} face={}".format(
            distance_info.get("arm_count", 0),
            distance_info.get("face_count", 0),
        )
    if "filtered_p05_distance" in distance_info:
        return "kf={:.1f} mm  raw={:.1f} mm  min={:.1f} mm".format(
            distance_info["filtered_p05_distance"],
            distance_info["p05_distance"],
            distance_info["min_distance"],
        )
    return "raw={:.1f} mm  min={:.1f} mm".format(distance_info["p05_distance"], distance_info["min_distance"])


def print_distance(distance_info, state: RoiState, cooldown: float) -> None:
    if distance_info is None:
        return
    now = time.time()
    if now - state.last_print_time < cooldown:
        return
    state.last_print_time = now

    if distance_info.get("arm_count", 0) == 0 or distance_info.get("face_count", 0) == 0:
        print(
            "distance unavailable: arm_valid={}, face_valid={}".format(
                distance_info.get("arm_count", 0),
                distance_info.get("face_count", 0),
            )
        )
        return

    if "filtered_p05_distance" in distance_info:
        print(
            "distance_kf={:.1f} mm, distance_p05={:.1f} mm, min={:.1f} mm, "
            "median_nearest={:.1f} mm, arm_z_median={:.1f} mm, face_z_median={:.1f} mm, "
            "arm_points={}, face_points={}".format(
                distance_info["filtered_p05_distance"],
                distance_info["p05_distance"],
                distance_info["min_distance"],
                distance_info["median_nearest"],
                distance_info["arm_z_median"],
                distance_info["face_z_median"],
                distance_info["arm_count"],
                distance_info["face_count"],
            )
        )
        return

    print(
        "distance_p05={:.1f} mm, min={:.1f} mm, median_nearest={:.1f} mm, "
        "arm_z_median={:.1f} mm, face_z_median={:.1f} mm, arm_points={}, face_points={}".format(
            distance_info["p05_distance"],
            distance_info["min_distance"],
            distance_info["median_nearest"],
            distance_info["arm_z_median"],
            distance_info["face_z_median"],
            distance_info["arm_count"],
            distance_info["face_count"],
        )
    )


def main() -> int:
    args = parse_args()
    if args.preview_scale <= 0:
        print("ERROR: --preview-scale must be > 0")
        return 1

    pipeline = Pipeline()
    config = Config()

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

    if args.fx > 0 and args.fy > 0:
        intr = Intrinsics(args.fx, args.fy, args.cx, args.cy)
        print("using manual fallback depth intrinsics:", intr)
    else:
        intr = get_depth_intrinsics(pipeline)
        print("depth intrinsics from SDK:", intr)

    if intr is None:
        print("ERROR: no depth intrinsics. Use --fx --fy --cx --cy as fallback.")
        pipeline.stop()
        return 1

    state = RoiState(args.preview_scale, args.rotate_180)
    if not args.no_kalman:
        state.distance_filter = ScalarKalman1D(
            args.kalman_distance_r_mm,
            args.kalman_distance_process_mm,
            args.kalman_distance_jump_mm,
        )
    state.auto_track_enabled = args.auto_track_roi
    roi_path = Path(args.roi_file)
    if not args.no_load_roi:
        loaded_rois = load_rois(roi_path)
        if loaded_rois:
            state.rois.update(loaded_rois)
            print("loaded ROI file:", roi_path, sorted(loaded_rois.keys()))

    cv2.namedWindow(args.window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(args.window, mouse_callback, state)

    print(
        "kalman:",
        "off" if state.distance_filter is None else "on R={}mm Q={}mm jump={}mm".format(
            args.kalman_distance_r_mm,
            args.kalman_distance_process_mm,
            args.kalman_distance_jump_mm,
        ),
    )
    print("Keys: 1=arm ROI, 2=face ROI, T=toggle auto track, S=save ROI, C=clear, Q/Esc=quit.")
    print("Drag rectangles on the depth image. Distance is printed after both ROIs are selected.")

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
            state.update_shape(width, height)

            if state.auto_track_enabled and state.drag_start is None:
                auto_track_rois(depth_mm, state, args)

            shown = make_depth_image(depth_mm, args.min_depth_mm, args.max_depth_mm)
            if args.rotate_180:
                shown = rotate_180(shown)

            if "arm" in state.rois and "face" in state.rois:
                now = time.time()
                if state.distance_info is None or now - state.last_distance_time >= args.distance_interval:
                    state.distance_info = calc_distance(depth_mm, state, intr, args)
                    state.distance_info = update_distance_filter(state.distance_info, state.distance_filter, now)
                    state.last_distance_time = now
                    print_distance(state.distance_info, state, args.print_cooldown)
            else:
                state.distance_info = None
                if state.distance_filter is not None:
                    state.distance_filter.reset()
            distance_info = state.distance_info

            if "arm" in state.rois:
                draw_roi(shown, state.rois["arm"], "arm", (0, 255, 255), state)
            if "face" in state.rois:
                draw_roi(shown, state.rois["face"], "face", (0, 80, 255), state)
            drag_roi = state.current_drag_roi()
            if drag_roi is not None:
                draw_roi(shown, drag_roi, state.active, (255, 255, 255), state, 1)

            if not args.no_auto_save_roi and state.roi_dirty and state.width > 0 and state.height > 0:
                if save_rois(roi_path, state.rois, state.width, state.height):
                    state.roi_dirty = False

            track_text = "auto" if state.auto_track_enabled else "manual"
            draw_text(
                shown,
                "active: {} | {} | 1 arm  2 face  T track  S save  C clear  Q quit".format(
                    state.active,
                    track_text,
                ),
                (12, 28),
            )
            draw_text(shown, make_status_text(distance_info), (12, 54))

            if args.preview_scale != 1.0:
                preview_w = max(1, int(width * args.preview_scale))
                preview_h = max(1, int(height * args.preview_scale))
                shown = cv2.resize(shown, (preview_w, preview_h), interpolation=cv2.INTER_AREA)

            cv2.imshow(args.window, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("1"), ord("a")):
                state.active = "arm"
            elif key in (ord("2"), ord("f")):
                state.active = "face"
            elif key in (ord("t"), ord("T")):
                state.auto_track_enabled = not state.auto_track_enabled
                state.distance_info = None
                state.last_distance_time = 0.0
                if state.distance_filter is not None:
                    state.distance_filter.reset()
                print("auto track:", "on" if state.auto_track_enabled else "off")
            elif key in (ord("s"), ord("S")):
                if save_rois(roi_path, state.rois, state.width, state.height):
                    state.roi_dirty = False
            elif key in (ord("c"), ord("C")):
                state.rois.clear()
                state.drag_start = None
                state.drag_current = None
                state.distance_info = None
                state.last_distance_time = 0.0
                if state.distance_filter is not None:
                    state.distance_filter.reset()
                state.roi_dirty = False
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

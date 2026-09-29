"""End-effector monocular target preview and isolated serial bench test.

The formal protocol-v5 application feeds detections to Qt, which is the only
RA6M5 serial owner.  Direct serial output here is opt-in laboratory debugging
and must never run beside the Qt serial session.
"""

import argparse
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from ._compat import dataclass
from .camera_utils import apply_flip, open_camera
from .reference_mole_preview import draw_detection_overlay
from .undistort import Undistorter
from .vision.reference_mole_detect import ReferenceMoleDetector
from .vision.target_stability import stable_center
from .vision_protocol import pack_unified_vision_error, pack_vision_error, pixel_error_from_target


@dataclass(frozen=True)
class DarkStickerMole:
    center: tuple
    radius: int
    area: float
    confidence: float


class ConstantVelocityKalman2D:
    def __init__(
        self,
        measurement_std_px: float = 3.0,
        process_std_px: float = 1.0,
        jump_reset_px: float = 15.0,
    ) -> None:
        self.measurement_var = float(max(measurement_std_px, 0.1)) ** 2
        self.process_var = float(max(process_std_px, 0.01)) ** 2
        self.jump_reset_px = float(max(jump_reset_px, 1.0))
        self.x = None
        self.p = None
        self.last_time = None

    def reset(self) -> None:
        self.x = None
        self.p = None
        self.last_time = None

    def update(self, measurement, now: float):
        z = np.asarray([[float(measurement[0])], [float(measurement[1])]], dtype=np.float64)
        if self.x is None or self.p is None or self.last_time is None:
            self.x = np.asarray([[z[0, 0]], [z[1, 0]], [0.0], [0.0]], dtype=np.float64)
            self.p = np.diag([self.measurement_var, self.measurement_var, 100.0, 100.0])
            self.last_time = now
            return int(round(self.x[0, 0])), int(round(self.x[1, 0]))

        predicted_position = self.x[:2, 0]
        if float(np.linalg.norm(z[:, 0] - predicted_position)) > self.jump_reset_px:
            self.reset()
            return self.update(measurement, now)

        dt = max(1e-3, min(float(now - self.last_time), 0.2))
        self.last_time = now

        f = np.asarray(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        h = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        q = np.diag(
            [
                self.process_var,
                self.process_var,
                self.process_var * 4.0,
                self.process_var * 4.0,
            ]
        )
        r = np.eye(2, dtype=np.float64) * self.measurement_var

        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q

        innovation = z - h @ self.x
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ innovation
        self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p
        return int(round(self.x[0, 0])), int(round(self.x[1, 0]))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Use the end-effector monocular camera to detect a mole and output u/v + dcx/dcy."
    )
    parser.add_argument("--camera", default="/dev/v4l/by-id/usb-Generic_USB_Camera_YHTek-video-index0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--flip", choices=["none", "0", "1", "-1"], default="none")
    parser.add_argument(
        "--rotate",
        choices=["none", "90cw", "90ccw", "180"],
        default="none",
        help="Rotate the frame after undistortion. Use 90cw when the live image is 90 degrees counterclockwise.",
    )
    parser.add_argument("--calib-params", default="../calibration/mono_end_camera/mono_calib_params.npz")
    parser.add_argument("--reference", default="reference/clean_face.png")
    parser.add_argument("--detector", choices=["auto", "reference", "dark"], default="auto")
    parser.add_argument("--dark-threshold", type=int, default=80)
    parser.add_argument("--dark-min-area", type=int, default=80)
    parser.add_argument("--dark-max-area", type=int, default=1200)
    parser.add_argument("--dark-max-aspect", type=float, default=2.5)
    parser.add_argument("--display", type=int, choices=[0, 1], default=1)
    parser.add_argument("--stable-window", type=int, default=5)
    parser.add_argument("--stable-required", type=int, default=4)
    parser.add_argument("--stable-radius", type=float, default=5.0)
    parser.add_argument("--no-kalman", action="store_true")
    parser.add_argument("--kalman-measurement-px", type=float, default=3.0)
    parser.add_argument("--kalman-process-px", type=float, default=1.0)
    parser.add_argument("--kalman-jump-reset-px", type=float, default=15.0)
    parser.add_argument("--print-interval", type=float, default=0.3)
    parser.add_argument("--send-interval", type=float, default=0.1)
    parser.add_argument(
        "--wait-arm-stable",
        action="store_true",
        help="Start in preview-only mode. Press D/Space, or use --start-on-ready with serial, before detecting.",
    )
    parser.add_argument(
        "--start-on-ready",
        action="store_true",
        help="Start detection after RA6M5 TARGET_READY (READY for legacy) is received.",
    )
    parser.add_argument(
        "--serial-port",
        default="",
        help="Isolated bench serial port. Empty means preview/print only.",
    )
    parser.add_argument(
        "--isolated-serial-debug",
        action="store_true",
        help="Acknowledge that formal Qt is stopped and this process alone owns the port.",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--serial-protocol", choices=["unified", "legacy"], default="unified")
    parser.add_argument("--heartbeat-interval", type=float, default=0.15)
    parser.add_argument(
        "--start-target-ctrl",
        action="store_true",
        help="Send TARGET_CTRL START in an isolated, manually prepared bench session.",
    )
    parser.add_argument(
        "--stop-target-ctrl-on-exit",
        type=int,
        choices=[0, 1],
        default=1,
        help="Stop/hold the target controller on exit if this script started it.",
    )
    parser.add_argument("--save-dir", default="test_results")
    parser.add_argument("--max-frames", type=int, default=0)
    return parser.parse_args()


def format_frame(frame: bytes) -> str:
    return " ".join("{:02X}".format(byte) for byte in frame)


def rotate_frame(frame, rotate: str):
    if rotate == "none":
        return frame
    if rotate == "90cw":
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if rotate == "90ccw":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if rotate == "180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    raise ValueError("unsupported rotate mode: {}".format(rotate))


def rotated_size(width: int, height: int, rotate: str):
    if rotate in ("90cw", "90ccw"):
        return height, width
    return width, height


def load_detector(reference_path: Path, width: int, height: int, rotate: str):
    reference = cv2.imread(str(reference_path))
    if reference is None:
        raise RuntimeError("cannot read reference image: {}".format(reference_path))
    reference = rotate_frame(reference, rotate)
    if reference.shape[1] != width or reference.shape[0] != height:
        raise RuntimeError(
            "reference image size {}x{} does not match runtime size {}x{}".format(
                reference.shape[1],
                reference.shape[0],
                width,
                height,
            )
        )
    return ReferenceMoleDetector(reference)


def detect_dark_sticker_moles(frame, args):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    threshold = max(0, min(255, int(args.dark_threshold)))
    mask = cv2.inRange(gray, 0, threshold)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

    moles = []
    for label in range(1, count):
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        w = int(stats[label, cv2.CC_STAT_WIDTH])
        h = int(stats[label, cv2.CC_STAT_HEIGHT])
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < args.dark_min_area or area > args.dark_max_area:
            continue

        aspect = max(w, h) / max(1, min(w, h))
        if aspect > args.dark_max_aspect:
            continue

        component_mask = labels[y : y + h, x : x + w] == label
        component_gray = gray[y : y + h, x : x + w][component_mask]
        if component_gray.size == 0:
            continue
        mean_gray = float(component_gray.mean())
        darkness = max(0.0, (float(threshold) - mean_gray) / max(float(threshold), 1.0))
        confidence = min(1.0, 0.25 + darkness)
        cx, cy = centroids[label]
        radius = max(2, int(round(max(w, h) * 0.5)))
        moles.append(
            DarkStickerMole(
                center=(int(round(cx)), int(round(cy))),
                radius=radius,
                area=float(area),
                confidence=float(confidence),
            )
        )

    moles.sort(key=lambda mole: mole.confidence * mole.area, reverse=True)
    return moles


def draw_protocol_overlay(image, state, source, stable, dcx, dcy, frame_hex, sent):
    shown = image.copy()
    color = (0, 255, 0) if state == "STABLE_TARGET" else (0, 255, 255)
    cv2.putText(
        shown,
        "{} [{}]".format(state, source),
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        color,
        2,
    )
    if stable is not None:
        cv2.putText(
            shown,
            "u={} v={}  dcx={} dcy={}  sent={}".format(
                stable[0],
                stable[1],
                dcx,
                dcy,
                "yes" if sent else "no",
            ),
            (10, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            shown,
            "frame {}".format(frame_hex),
            (10, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 0),
            1,
        )
    else:
        cv2.putText(
            shown,
            "no stable target (unified: valid=0; legacy: no frame)",
            (10, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
    cv2.putText(
        shown,
        "Q quit | S save",
        (10, shown.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 0),
        1,
    )
    return shown


def draw_wait_overlay(image):
    shown = image.copy()
    cv2.putText(
        shown,
        "WAIT_ARM_STABLE",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        shown,
        "preview only: no detection, no target frame",
        (10, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        shown,
        "Press D/Space after arm stops | P pause | Q quit",
        (10, shown.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 0),
        1,
    )
    return shown


def main() -> int:
    args = parse_args()
    if args.serial_port and not args.isolated_serial_debug:
        print(
            "ERROR: direct serial output is isolated debug only. Stop formal Qt and add "
            "--isolated-serial-debug.",
            flush=True,
        )
        return 2
    if args.start_target_ctrl and not args.serial_port:
        print("ERROR: --start-target-ctrl requires --serial-port", flush=True)
        return 2
    if (
        args.serial_protocol == "unified"
        and args.serial_port
        and args.start_on_ready
        and not args.start_target_ctrl
    ):
        print(
            "ERROR: unified --start-on-ready requires --start-target-ctrl in this isolated tool",
            flush=True,
        )
        return 2

    reference_path = Path(args.reference)
    target_width, target_height = rotated_size(args.width, args.height, args.rotate)

    detector = None
    if args.detector in ("auto", "reference"):
        try:
            detector = load_detector(reference_path, target_width, target_height, args.rotate)
        except Exception as exc:
            if args.detector == "reference":
                print("ERROR:", exc, flush=True)
                print("先用 mole_system.py 或 reference_capture.py 采集无黑痣基准图。", flush=True)
                return 2
            print("reference unavailable, fallback to dark detector:", exc, flush=True)

    cap, label = open_camera(args.camera, args.width, args.height, args.fps)
    undistorter = Undistorter.load(args.calib_params, (args.width, args.height))
    if undistorter is None:
        print("ERROR: cannot load calibration params: {}".format(args.calib_params), flush=True)
        cap.release()
        return 2

    sender = None
    if args.serial_port:
        from .serial_sender import SerialSender

        try:
            sender = SerialSender(
                args.serial_port,
                args.baudrate,
                protocol=args.serial_protocol,
                heartbeat_interval=args.heartbeat_interval,
                auto_heartbeat=args.serial_protocol == "unified",
            )
            print(
                "WARNING: isolated serial debug only; formal Qt must be stopped\n"
                "serial connected: {} @ {} protocol={}".format(
                    args.serial_port,
                    args.baudrate,
                    args.serial_protocol,
                ),
                flush=True,
            )
        except Exception as exc:
            print("serial open failed:", exc, flush=True)
            sender = None
    if args.serial_port and sender is None:
        cap.release()
        return 2

    target_ctrl_started = False
    target_ctrl_on = False
    target_ready = False
    if sender is not None and args.start_target_ctrl:
        target_ctrl_started = sender.send_target_ctrl(True)
        if not target_ctrl_started:
            print("target ctrl start failed: {}".format(sender.last_error), flush=True)
            sender.close()
            cap.release()
            return 2

    history = deque(maxlen=max(1, args.stable_window))
    target_filter = None
    if not args.no_kalman:
        target_filter = ConstantVelocityKalman2D(
            args.kalman_measurement_px,
            args.kalman_process_px,
            args.kalman_jump_reset_px,
        )
    detect_enabled = not args.wait_arm_stable
    last_print = 0.0
    last_wait_print = 0.0
    last_send = 0.0
    frame_count = 0
    win = "Mono Mole Target"
    if args.display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    print("camera opened:", label, flush=True)
    print("detector:", args.detector, flush=True)
    print(
        "kalman:",
        "off" if target_filter is None else "on R={}px jump={}px".format(
            args.kalman_measurement_px,
            args.kalman_jump_reset_px,
        ),
        flush=True,
    )
    if args.wait_arm_stable:
        print("mode: WAIT_ARM_STABLE, press D/Space after the arm stops", flush=True)
        if args.start_on_ready and sender is None:
            print("WARN: --start-on-ready requires --serial-port", flush=True)
    if detector is not None:
        print("reference:", reference_path, flush=True)
    if args.serial_protocol == "unified":
        print("protocol: A5 5A 01 TYPE SEQ LEN PAYLOAD CRC16_LO CRC16_HI", flush=True)
    else:
        print("protocol: FF 05 03 dcx_lo dcx_hi dcy_lo dcy_hi checksum FE", flush=True)

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame_count += 1
            frame = apply_flip(frame, args.flip)
            frame = undistorter.apply(frame)
            frame = rotate_frame(frame, args.rotate)
            frame_height, frame_width = frame.shape[:2]
            now = time.time()

            if sender is not None:
                for status in sender.poll_statuses():
                    print(
                        "RA6M5 status: {} protocol={} func=0x{:02X} value=0x{:02X} error=0x{:02X}".format(
                            status.label,
                            status.protocol,
                            status.func,
                            status.value,
                            status.error_code,
                        ),
                        flush=True,
                    )
                    recoverable_vision_lost = (
                        args.serial_protocol == "unified"
                        and int(status.error_code) == 0x0A
                        and int(status.func) in (0x03, 0x07)
                    )
                    if recoverable_vision_lost:
                        print(
                            "recoverable VISION_LOST: keep TARGET_READY session and continue valid=0/reacquire",
                            flush=True,
                        )
                        continue
                    if not status.ok:
                        target_ctrl_on = False
                        target_ready = False
                        print("RA6M5 rejected/stopped target session: {}".format(status.label), flush=True)
                        continue

                    if args.serial_protocol == "unified":
                        if status.label == "TARGET_CTRL_ON":
                            target_ctrl_on = True
                        elif status.label == "TARGET_READY":
                            target_ready = True
                        elif status.label == "TARGET_CTRL_OFF":
                            target_ctrl_on = False
                            target_ready = False

                    ready_received = (
                        target_ctrl_on and target_ready
                        if args.serial_protocol == "unified"
                        else status.label == "READY"
                    )
                    if args.wait_arm_stable and args.start_on_ready and ready_received:
                        detect_enabled = True
                        history.clear()
                        if target_filter is not None:
                            target_filter.reset()
                        print("arm stable: start mole detection", flush=True)

            if not detect_enabled:
                history.clear()
                shown = draw_wait_overlay(frame)
                if now - last_wait_print >= max(args.print_interval, 1.0):
                    last_wait_print = now
                    print("WAIT_ARM_STABLE preview_only", flush=True)
                if args.display:
                    cv2.imshow(win, shown)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key in (ord("d"), ord("D"), ord(" ")):
                        detect_enabled = True
                        history.clear()
                        if target_filter is not None:
                            target_filter.reset()
                        print("manual start: mole detection enabled", flush=True)
                    if key in (ord("s"), ord("S")):
                        save_dir = Path(args.save_dir)
                        save_dir.mkdir(parents=True, exist_ok=True)
                        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        path = save_dir / "mono_mole_wait_{}.png".format(stamp)
                        if cv2.imwrite(str(path), shown):
                            print("saved:", path, flush=True)
                if args.max_frames > 0 and frame_count >= args.max_frames:
                    break
                continue

            source = "dark"
            moles = []
            aligned_frame = frame
            alignment_ok = True
            alignment_reason = ""

            if detector is not None and args.detector in ("auto", "reference"):
                ref_moles, alignment, debug = detector.detect(frame, return_debug=True)
                if args.detector == "reference" or (alignment.ok and ref_moles):
                    moles = ref_moles
                    aligned_frame = debug["aligned"]
                    alignment_ok = alignment.ok
                    alignment_reason = alignment.reason
                    source = "reference"
                elif args.detector == "auto":
                    moles = detect_dark_sticker_moles(frame, args)
                    aligned_frame = frame
                    alignment_ok = True
                    source = "dark"
                    alignment_reason = ""
            else:
                moles = detect_dark_sticker_moles(frame, args)

            primary = moles[0] if moles else None
            history.append(primary.center if primary is not None and alignment_ok else None)
            stable = stable_center(history, args.stable_required, args.stable_radius)

            dcx = dcy = None
            frame_hex = ""
            sent = False
            raw_stable = stable
            unified_session_ready = (
                sender is not None
                and args.serial_protocol == "unified"
                and target_ctrl_on
                and target_ready
            )
            vision_send_allowed = sender is not None and (
                args.serial_protocol == "legacy" or unified_session_ready
            )

            if not alignment_ok:
                state = "ALIGN_FAILED"
                if target_filter is not None:
                    target_filter.reset()
            elif stable is not None:
                state = "STABLE_TARGET"
                if target_filter is not None:
                    stable = target_filter.update(stable, now)
                dcx, dcy = pixel_error_from_target(stable[0], stable[1], frame_width, frame_height)
                if args.serial_protocol == "unified":
                    protocol_frame = pack_unified_vision_error(0, dcx, dcy, True)
                else:
                    protocol_frame = pack_vision_error(dcx, dcy)
                frame_hex = format_frame(protocol_frame)
                if vision_send_allowed and now - last_send >= args.send_interval:
                    sent = sender.send_vision_error(dcx, dcy, valid=True)
                    last_send = now
            elif primary is not None:
                state = "CANDIDATE"
                if target_filter is not None:
                    target_filter.reset()
            else:
                state = "NO_TARGET"
                if target_filter is not None:
                    target_filter.reset()

            if (
                state != "STABLE_TARGET"
                and unified_session_ready
                and now - last_send >= args.send_interval
            ):
                protocol_frame = pack_unified_vision_error(0, 0, 0, False)
                frame_hex = format_frame(protocol_frame)
                sent = sender.send_no_target()
                last_send = now

            if now - last_print >= args.print_interval:
                last_print = now
                if state == "STABLE_TARGET":
                    print(
                        "STABLE_TARGET u={} v={} raw_u={} raw_v={} dcx={} dcy={} frame={}".format(
                            stable[0],
                            stable[1],
                            raw_stable[0],
                            raw_stable[1],
                            dcx,
                            dcy,
                            frame_hex,
                        ),
                        flush=True,
                    )
                elif state == "CANDIDATE":
                    print(
                        "CANDIDATE u={} v={} confidence={:.2f}".format(
                            primary.center[0],
                            primary.center[1],
                            primary.confidence,
                        ),
                        flush=True,
                    )
                elif state == "ALIGN_FAILED":
                    print("ALIGN_FAILED {}".format(alignment_reason), flush=True)
                else:
                    print("NO_TARGET", flush=True)

            overlay = draw_detection_overlay(aligned_frame, moles, stable, state)
            shown = draw_protocol_overlay(overlay, state, source, stable, dcx, dcy, frame_hex, sent)

            if args.display:
                cv2.imshow(win, shown)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key in (ord("p"), ord("P"), ord("h"), ord("H")):
                    detect_enabled = False
                    history.clear()
                    if target_filter is not None:
                        target_filter.reset()
                    print("mole detection paused: WAIT_ARM_STABLE", flush=True)
                if key in (ord("s"), ord("S")):
                    save_dir = Path(args.save_dir)
                    save_dir.mkdir(parents=True, exist_ok=True)
                    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    path = save_dir / "mono_mole_target_{}.png".format(stamp)
                    if cv2.imwrite(str(path), shown):
                        print("saved:", path, flush=True)

            if args.max_frames > 0 and frame_count >= args.max_frames:
                break
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        cap.release()
        if sender is not None:
            if target_ctrl_started and args.stop_target_ctrl_on_exit:
                sender.send_target_ctrl(False)
            sender.close()
        if args.display:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

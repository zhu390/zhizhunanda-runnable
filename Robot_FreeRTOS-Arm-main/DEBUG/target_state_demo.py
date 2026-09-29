import argparse
import datetime as dt
import pathlib
import queue
import sys
import threading
import time

import serial


ARM_PORT = "COM7"
JETSON_PORT = "COM14"
BAUD = 115200
DEFAULT_VISION_SEQUENCE = "9,-10:2;4,-6:1;0,0:0"

JETSON_SOF = 0xFF
JETSON_EOF = 0xFE
JETSON_LEN = 0x05
JETSON_FUNC_VISION_ERROR = 0x03
JETSON_CTRL_SOF = 0xAA
JETSON_CTRL_EOF = 0xBB
JETSON_FUNC_TARGET_CTRL = 0x01

RA6_SOF = 0xCC
RA6_EOF = 0xDD
RA6_READY = 0x01
RA6_ALIGN_DONE = 0x02
RA6_OUTPUT = 0x03
RA6_TARGET_CTRL = 0x04
RA6_ERROR = 0xFE


def now_stamp():
    return dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def hex_bytes(data):
    return " ".join(f"{b:02X}" for b in data)


class Logger:
    def __init__(self, debug_dir):
        debug_dir.mkdir(parents=True, exist_ok=True)
        ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = debug_dir / f"target_state_demo_{ts}.log"
        self._lock = threading.Lock()
        self._file = self.path.open("w", encoding="utf-8", newline="")
        self._console_muted = False

    def set_console_muted(self, muted):
        with self._lock:
            self._console_muted = bool(muted)

    def write(self, msg, force_console=False):
        line = f"[{now_stamp()}] {msg}"
        with self._lock:
            self._file.write(line)
            self._file.flush()
            if force_console or not self._console_muted:
                sys.stdout.write(line)
                sys.stdout.flush()

    def close(self):
        with self._lock:
            self._file.close()


class StatusReader:
    def __init__(self, ser, logger, stop_event):
        self.ser = ser
        self.logger = logger
        self.stop_event = stop_event
        self.events = queue.Queue()
        self._buf = bytearray()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(256)
            except Exception as exc:
                self.logger.write(f"[JETSON-RX-ERR] {exc}\n")
                return
            if not data:
                continue
            self.logger.write(f"[JETSON-RX] {hex_bytes(data)}\n")
            self._buf.extend(data)
            self._parse()

    def _parse(self):
        while len(self._buf) >= 4:
            if self._buf[0] != RA6_SOF:
                del self._buf[0]
                continue
            if self._buf[3] != RA6_EOF:
                del self._buf[0]
                continue
            func = self._buf[1]
            value = self._buf[2]
            del self._buf[:4]
            name = status_name(func, value)
            self.logger.write(f"[JETSON-STATUS] {name} func=0x{func:02X} value=0x{value:02X}\n")
            self.events.put((func, value, name))

    def wait_for(self, wanted, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                event = self.events.get(timeout=0.1)
            except queue.Empty:
                continue
            func, value, name = event
            if (func, value) == (RA6_ERROR, 0x01):
                raise RuntimeError("RA6 reported target ERROR")
            if event[:2] == wanted:
                return name
        raise TimeoutError(f"timeout waiting for {status_name(*wanted)}")


class ArmReader:
    def __init__(self, ser, logger, stop_event):
        self.ser = ser
        self.logger = logger
        self.stop_event = stop_event
        self.lines = queue.Queue()
        self._buf = bytearray()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(256)
            except Exception as exc:
                self.logger.write(f"[ARM-ERR] {exc}\n")
                return
            if not data:
                continue
            self._buf.extend(data)
            while b"\n" in self._buf:
                raw, _, rest = self._buf.partition(b"\n")
                self._buf = bytearray(rest)
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                self.logger.write(f"[ARM] {line}\n")
                self.lines.put(line)

    def wait_line_contains(self, needles, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            if "soft reset final FAIL" in line:
                raise RuntimeError(line)
            if "[ERROR] event_type:target_enable failed" in line:
                raise RuntimeError(line)
            for needle in needles:
                if needle in line:
                    return line
        raise TimeoutError(f"timeout waiting for ARM log containing {needles}")

    def wait_line_exact(self, wanted, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            if "soft reset final FAIL" in line:
                raise RuntimeError(line)
            if "target_enable rejected" in line:
                raise RuntimeError(line)
            if "[ERROR] event_type:target_enable failed" in line:
                raise RuntimeError(line)
            if line == wanted:
                return line
        raise TimeoutError(f"timeout waiting for exact ARM log '{wanted}'")


class VisionSender:
    def __init__(self, ser, logger, stop_event, period_s=0.2, tx_log_every=0):
        self.ser = ser
        self.logger = logger
        self.stop_event = stop_event
        self.period_s = period_s
        self.tx_log_every = max(0, tx_log_every)
        self.enabled = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.frame = make_vision_frame(0, 0)
        self.sequence = [(0, 0, 0.0)]
        self.sequence_index = 0
        self.sequence_step_started_at = time.monotonic()
        self.sequence_logged_index = None
        self.tx_count = 0
        self._lock = threading.Lock()

    def start(self):
        self.thread.start()

    def set_error(self, dcx, dcy):
        with self._lock:
            self.frame = make_vision_frame(dcx, dcy)
            self.sequence = [(int(dcx), int(dcy), 0.0)]
            self.sequence_index = 0
            self.sequence_step_started_at = time.monotonic()
            self.sequence_logged_index = None

    def set_sequence(self, sequence):
        if not sequence:
            raise ValueError("vision sequence must not be empty")
        with self._lock:
            self.sequence = [(int(dcx), int(dcy), max(0.0, float(duration_s)))
                             for dcx, dcy, duration_s in sequence]
            self.sequence_index = 0
            self.sequence_step_started_at = time.monotonic()
            self.sequence_logged_index = None
            dcx, dcy, _ = self.sequence[0]
            self.frame = make_vision_frame(dcx, dcy)

    def enable(self):
        if not self.enabled.is_set():
            with self._lock:
                self.tx_count = 0
                self.sequence_index = 0
                self.sequence_step_started_at = time.monotonic()
                self.sequence_logged_index = None
            self.logger.write(f"[JETSON-TX] vision sender enabled: period={self.period_s:.3f}s.\n")
        self.enabled.set()

    def disable(self):
        was_enabled = self.enabled.is_set()
        self.enabled.clear()
        if was_enabled:
            with self._lock:
                tx_count = self.tx_count
            self.logger.write(f"[JETSON-TX] vision sender disabled after {tx_count} frames.\n")

    def send_once(self):
        with self._lock:
            self._update_sequence_locked()
            frame = self.frame
            self.tx_count += 1
            tx_count = self.tx_count
        self.ser.write(frame)
        self.ser.flush()
        if self.tx_log_every != 0 and (tx_count == 1 or (tx_count % self.tx_log_every) == 0):
            self.logger.write(f"[JETSON-TX] #{tx_count} {hex_bytes(frame)}\n")

    def _update_sequence_locked(self):
        now = time.monotonic()
        while self.sequence_index < len(self.sequence) - 1:
            _, _, duration_s = self.sequence[self.sequence_index]
            if duration_s <= 0.0 or (now - self.sequence_step_started_at) < duration_s:
                break
            self.sequence_index += 1
            self.sequence_step_started_at = now

        dcx, dcy, duration_s = self.sequence[self.sequence_index]
        self.frame = make_vision_frame(dcx, dcy)
        if self.sequence_logged_index != self.sequence_index:
            self.sequence_logged_index = self.sequence_index
            self.logger.write(
                f"[JETSON-TX] vision step {self.sequence_index + 1}/{len(self.sequence)}: "
                f"dcx={dcx} dcy={dcy} hold={duration_s:.1f}s "
                f"frame={hex_bytes(self.frame)}\n"
            )

    def _run(self):
        while not self.stop_event.is_set():
            if self.enabled.is_set():
                try:
                    self.send_once()
                except Exception as exc:
                    self.logger.write(f"[JETSON-TX-ERR] {exc}\n")
                    return
            time.sleep(self.period_s)


def status_name(func, value):
    names = {
        (RA6_READY, 0x01): "READY",
        (RA6_ALIGN_DONE, 0x01): "ALIGN_DONE",
        (RA6_OUTPUT, 0x01): "OUTPUT_ON",
        (RA6_OUTPUT, 0x00): "OUTPUT_OFF",
        (RA6_TARGET_CTRL, 0x01): "TARGET_CTRL_ON",
        (RA6_TARGET_CTRL, 0x00): "TARGET_CTRL_OFF",
        (RA6_ERROR, 0x01): "ERROR",
    }
    return names.get((func, value), "UNKNOWN")


def make_vision_frame(dcx, dcy):
    payload = int(dcx).to_bytes(2, "little", signed=True) + int(dcy).to_bytes(2, "little", signed=True)
    checksum = (JETSON_LEN + JETSON_FUNC_VISION_ERROR + sum(payload)) & 0xFF
    return bytes([JETSON_SOF, JETSON_LEN, JETSON_FUNC_VISION_ERROR]) + payload + bytes([checksum, JETSON_EOF])


def make_target_ctrl_frame(enable):
    return bytes([JETSON_CTRL_SOF, JETSON_FUNC_TARGET_CTRL, 0x01 if enable else 0x00, JETSON_CTRL_EOF])


def send_target_ctrl(ser, logger, enable):
    frame = make_target_ctrl_frame(enable)
    action = "START" if enable else "STOP"
    logger.write(f"[JETSON-TX] TARGET_CTRL {action}: {hex_bytes(frame)}\n")
    ser.write(frame)
    ser.flush()


def send_cmd(ser, logger, cmd):
    logger.write(f"[ARM-TX] {cmd}\n")
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()


def parse_vision_sequence(text, default_hold_s, fallback_dcx, fallback_dcy):
    if text is None or text.strip() == "":
        return [(fallback_dcx, fallback_dcy, 0.0)]

    sequence = []
    for raw_item in text.split(";"):
        item = raw_item.strip()
        if not item:
            continue

        if ":" in item:
            pair_text, duration_text = item.split(":", 1)
            duration_s = float(duration_text.strip())
        else:
            pair_text = item
            duration_s = default_hold_s

        parts = [part.strip() for part in pair_text.split(",")]
        if len(parts) != 2:
            raise ValueError(f"invalid vision sequence item '{item}', expected dcx,dcy[:seconds]")

        sequence.append((int(parts[0]), int(parts[1]), duration_s))

    if not sequence:
        raise ValueError("vision sequence is empty")
    return sequence


def format_vision_sequence(sequence):
    return " -> ".join(f"({dcx},{dcy})/{duration_s:.1f}s" for dcx, dcy, duration_s in sequence)


def wait_manual(prompt, logger):
    logger.write(f"[PROMPT] {prompt}\n", force_console=True)
    logger.set_console_muted(True)
    try:
        input("按 Enter 继续...")
    finally:
        logger.set_console_muted(False)
    logger.write("[PROMPT] Enter received; console logging resumed.\n")


def final_soft_reset(args, arm, arm_reader, logger):
    if args.skip_final_reset:
        logger.write("[STEP] final soft_reset skipped by --skip-final-reset.\n")
        return

    logger.write("[STEP] final soft_reset to HOME...\n")
    send_cmd(arm, logger, "soft_reset")
    arm_reader.wait_line_contains(["soft reset final verify PASS"], args.soft_reset_timeout)
    args.final_soft_reset_done = True
    logger.write("[DONE] final soft_reset PASS; robot returned HOME.\n")


def run_with_status(args, arm, jetson, arm_reader, status_reader, vision_sender, logger):
    send_target_ctrl(jetson, logger, False)
    try:
        status_reader.wait_for((RA6_TARGET_CTRL, 0x00), 3.0)
    except TimeoutError:
        logger.write("[WARN] TARGET_CTRL_OFF not seen during initial cleanup; continuing.\n")
    time.sleep(0.2)
    send_cmd(arm, logger, "laser_off")
    time.sleep(0.2)

    send_cmd(arm, logger, "soft_reset")
    try:
        arm_reader.wait_line_contains(["soft reset final verify PASS"], args.soft_reset_timeout)
    except TimeoutError:
        logger.write("[WARN] soft_reset PASS log not seen before timeout; continuing to Jetson target start.\n")
    except Exception:
        raise

    send_target_ctrl(jetson, logger, True)
    status_reader.wait_for((RA6_TARGET_CTRL, 0x01), args.target_ctrl_timeout)

    logger.write("[STEP] waiting READY from RA6...\n")
    status_reader.wait_for((RA6_READY, 0x01), args.ready_timeout)

    wait_manual("target_enable 已完成，RA6M5 已到预定位 READY。确认后按 Enter 开始发送视觉误差校准。", logger)
    vision_sender.set_sequence(args.vision_sequence_steps)
    vision_sender.enable()
    logger.write(f"[STEP] sending vision sequence {format_vision_sequence(args.vision_sequence_steps)}, waiting ALIGN_DONE...\n")
    status_reader.wait_for((RA6_ALIGN_DONE, 0x01), args.align_timeout)

    wait_manual("已收到 ALIGN_DONE。确认画面和机械臂位置安全后，按 Enter 进入 P000 发射许可步骤。", logger)
    logger.write("[PROMPT] 请按住 P000 KEY，脚本会自动等待 OUTPUT_ON，不需要再按 Enter。\n")
    logger.write("[STEP] waiting OUTPUT_ON from RA6...\n")
    status_reader.wait_for((RA6_OUTPUT, 0x01), args.output_timeout)

    logger.write("[PROMPT] 已进入 OUTPUT。请松开 P000 KEY，脚本会自动等待 OUTPUT_OFF，不需要按 Enter。\n")
    logger.write("[STEP] waiting OUTPUT_OFF from RA6...\n")
    status_reader.wait_for((RA6_OUTPUT, 0x00), args.output_timeout)

    vision_sender.disable()
    send_target_ctrl(jetson, logger, False)
    status_reader.wait_for((RA6_TARGET_CTRL, 0x00), args.target_ctrl_timeout)
    time.sleep(0.2)
    send_cmd(arm, logger, "laser_off")
    time.sleep(0.2)
    final_soft_reset(args, arm, arm_reader, logger)
    logger.write("[DONE] OUTPUT_OFF received; target disabled; demo sequence complete.\n")


def run_without_status(args, arm, jetson, arm_reader, vision_sender, logger):
    send_target_ctrl(jetson, logger, False)
    time.sleep(0.2)
    send_cmd(arm, logger, "laser_off")
    time.sleep(0.2)
    send_cmd(arm, logger, "soft_reset")
    logger.write(f"[STEP] no-status mode: waiting {args.soft_reset_timeout:.1f}s for soft_reset.\n")
    time.sleep(args.soft_reset_timeout)
    send_target_ctrl(jetson, logger, True)
    logger.write(f"[STEP] no-status mode: waiting {args.ready_timeout:.1f}s for pre-position.\n")
    time.sleep(args.ready_timeout)

    vision_sender.set_sequence(args.vision_sequence_steps)
    vision_sender.enable()
    logger.write(f"[STEP] no-status mode: sending vision sequence {format_vision_sequence(args.vision_sequence_steps)} for {args.align_timeout:.1f}s.\n")
    time.sleep(args.align_timeout)

    wait_manual("请按住 P000 KEY，观察是否进入激光输出。", logger)
    time.sleep(args.output_timeout)
    wait_manual("请松开 P000 KEY，观察 P015 是否关闭。", logger)
    time.sleep(args.output_timeout)

    vision_sender.disable()
    send_target_ctrl(jetson, logger, False)
    time.sleep(0.2)
    send_cmd(arm, logger, "laser_off")
    if args.skip_final_reset:
        logger.write("[STEP] no-status mode: final soft_reset skipped by --skip-final-reset.\n")
    else:
        send_cmd(arm, logger, "soft_reset")
        logger.write(f"[STEP] no-status mode: waiting {args.soft_reset_timeout:.1f}s for final soft_reset.\n")
        time.sleep(args.soft_reset_timeout)
        args.final_soft_reset_done = True
    logger.write("[DONE] no-status demo sequence complete.\n")


def parse_args():
    default_debug = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="RA6M5 target demo: COM7 arm control + Jetson UART vision/control")
    parser.add_argument("--arm-port", default=ARM_PORT)
    parser.add_argument("--jetson-port", default=JETSON_PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--debug-dir", type=pathlib.Path, default=default_debug)
    parser.add_argument("--no-status-wait", action="store_true")
    parser.add_argument("--skip-final-reset", action="store_true",
                        help="Do not send final soft_reset after Jetson target stop.")
    parser.add_argument("--vision-period", type=float, default=0.2,
                        help="Seconds between simulated Jetson vision frames while enabled.")
    parser.add_argument("--vision-dcx", type=int, default=80,
                        help="Fallback simulated Jetson dcx pixels when --vision-sequence is empty.")
    parser.add_argument("--vision-dcy", type=int, default=-80,
                        help="Fallback simulated Jetson dcy pixels when --vision-sequence is empty.")
    parser.add_argument("--vision-sequence", default=DEFAULT_VISION_SEQUENCE,
                        help=("Semicolon-separated simulated vision errors, format "
                              "'dcx,dcy[:seconds];...'. The last step repeats until ALIGN_DONE. "
                              "Use an empty string to fall back to --vision-dcx/--vision-dcy."))
    parser.add_argument("--vision-step-hold", type=float, default=1.5,
                        help="Default hold seconds for --vision-sequence items without ':seconds'.")
    parser.add_argument("--tx-log-every", type=int, default=0,
                        help="Log one JETSON-TX line every N frames; 0 disables per-frame TX logs.")
    parser.add_argument("--soft-reset-timeout", type=float, default=45.0)
    parser.add_argument("--target-ctrl-timeout", type=float, default=5.0)
    parser.add_argument("--ready-timeout", type=float, default=25.0)
    parser.add_argument("--align-timeout", type=float, default=20.0)
    parser.add_argument("--output-timeout", type=float, default=10.0)
    args = parser.parse_args()
    args.vision_sequence_steps = parse_vision_sequence(args.vision_sequence,
                                                       args.vision_step_hold,
                                                       args.vision_dcx,
                                                       args.vision_dcy)
    return args


def main():
    args = parse_args()
    args.final_soft_reset_done = False
    logger = Logger(args.debug_dir)
    stop_event = threading.Event()

    logger.write(f"[INFO] log file: {logger.path}\n")
    logger.write(f"[INFO] ARM={args.arm_port} JETSON={args.jetson_port} baud={args.baud}\n")
    logger.write(f"[INFO] start frame: {hex_bytes(make_target_ctrl_frame(True))}\n")
    logger.write(f"[INFO] stop frame: {hex_bytes(make_target_ctrl_frame(False))}\n")
    logger.write(f"[INFO] vision sequence: {format_vision_sequence(args.vision_sequence_steps)}\n")

    arm = None
    jetson = None
    arm_reader = None
    vision_sender = None
    try:
        arm = serial.Serial(args.arm_port, args.baud, timeout=0.05)
        jetson = serial.Serial(args.jetson_port, args.baud, timeout=0.05)

        arm_reader = ArmReader(arm, logger, stop_event)
        status_reader = StatusReader(jetson, logger, stop_event)
        vision_sender = VisionSender(jetson, logger, stop_event,
                                     period_s=args.vision_period,
                                     tx_log_every=args.tx_log_every)

        arm_reader.start()
        status_reader.start()
        vision_sender.start()
        time.sleep(0.5)

        if args.no_status_wait:
            run_without_status(args, arm, jetson, arm_reader, vision_sender, logger)
        else:
            run_with_status(args, arm, jetson, arm_reader, status_reader, vision_sender, logger)
    except KeyboardInterrupt:
        logger.write("[ABORT] keyboard interrupt.\n")
    except Exception as exc:
        logger.write(f"[FATAL] {type(exc).__name__}: {exc}\n")
    finally:
        if vision_sender is not None:
            vision_sender.disable()
        if jetson is not None and jetson.is_open:
            try:
                send_target_ctrl(jetson, logger, False)
            except Exception:
                pass
        if arm is not None and arm.is_open:
            try:
                send_cmd(arm, logger, "laser_off")
            except Exception:
                pass
            if not args.skip_final_reset and not args.final_soft_reset_done:
                try:
                    logger.write("[STEP] cleanup soft_reset to HOME...\n")
                    send_cmd(arm, logger, "soft_reset")
                    if arm_reader is not None:
                        arm_reader.wait_line_contains(["soft reset final verify PASS"], args.soft_reset_timeout)
                        logger.write("[DONE] cleanup soft_reset PASS; robot returned HOME.\n")
                    else:
                        time.sleep(args.soft_reset_timeout)
                    args.final_soft_reset_done = True
                except Exception as exc:
                    logger.write(f"[WARN] cleanup soft_reset failed: {exc}\n")
        stop_event.set()
        time.sleep(0.2)
        if jetson is not None and jetson.is_open:
            jetson.close()
        if arm is not None and arm.is_open:
            arm.close()
        logger.write("[INFO] ports closed.\n")
        logger.close()


if __name__ == "__main__":
    main()

import argparse
import datetime as dt
import pathlib
import queue
import re
import sys
import threading
import time

import serial


ARM_PORT = "COM7"
JETSON_PORT = "COM14"
BAUD = 115200

JETSON_SOF = 0xFF
JETSON_EOF = 0xFE
JETSON_LEN = 0x05
JETSON_FUNC_VISION_ERROR = 0x03

RA6_SOF = 0xCC
RA6_EOF = 0xDD
RA6_READY = 0x01
RA6_ALIGN_DONE = 0x02
RA6_OUTPUT = 0x03
RA6_ERROR = 0xFE

AUTO_TIMEOUT_S = 45.0

DEFAULT_MOTION_GROUPS = [
    ("前伸到预定位，展示平滑起步和末端稳定", ["auto 0 -50 0"]),
    ("回 HOME，作为下一组动作的干净起点", ["soft_reset"]),
    ("右前方斜向运动，展示 X/Y 联动", ["auto 45 -70 0"]),
    ("左前方斜向运动，展示跨中线平滑移动", ["auto -45 -70 0"]),
    ("回 HOME，切换到 Z 方向展示", ["soft_reset"]),
    ("前伸并上抬，展示 X/Y/Z 协调", ["auto 0 -70 15"]),
    ("前伸并下压，展示 Z 方向反向切换", ["auto 0 -70 -15"]),
    ("回 HOME，结束运动展示", ["soft_reset"]),
]


def now_stamp():
    return dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def hex_bytes(data):
    return " ".join(f"{b:02X}" for b in data)


class Logger:
    def __init__(self, debug_dir):
        debug_dir.mkdir(parents=True, exist_ok=True)
        ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = debug_dir / f"shooting_robot_demo_{ts}.log"
        self._lock = threading.Lock()
        self._file = self.path.open("w", encoding="utf-8", newline="")

    def write(self, msg):
        line = f"[{now_stamp()}] {msg}"
        with self._lock:
            self._file.write(line)
            self._file.flush()
            sys.stdout.write(line)
            sys.stdout.flush()

    def close(self):
        with self._lock:
            self._file.close()


def should_fail_on_arm_line(line):
    lower = line.lower()
    if "soft reset final fail" in lower:
        return True
    if "reject auto" in lower:
        return True
    if "target_enable rejected" in lower:
        return True
    if "inverse failed" in lower:
        return True
    if "[error] event_type:target_enable failed" in lower:
        return True
    if line.startswith("ERROR:"):
        return True
    return False


class ArmReader:
    def __init__(self, ser, logger, stop_event, verbose_arm_path=False):
        self.ser = ser
        self.logger = logger
        self.stop_event = stop_event
        self.verbose_arm_path = verbose_arm_path
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
                self.lines.put(line)
                if self.verbose_arm_path or not is_arm_path_line(line):
                    self.logger.write(f"[ARM] {line}\n")

    def wait_line_contains(self, needles, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            if should_fail_on_arm_line(line):
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
            if should_fail_on_arm_line(line):
                raise RuntimeError(line)
            if line == wanted:
                return line
        raise TimeoutError(f"timeout waiting for exact ARM log '{wanted}'")


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
        self.tx_count = 0
        self._lock = threading.Lock()

    def start(self):
        self.thread.start()

    def set_error(self, dcx, dcy):
        with self._lock:
            self.frame = make_vision_frame(dcx, dcy)

    def enable(self):
        if not self.enabled.is_set():
            with self._lock:
                self.tx_count = 0
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
            frame = self.frame
            self.tx_count += 1
            tx_count = self.tx_count
        self.ser.write(frame)
        self.ser.flush()
        if self.tx_log_every != 0 and (tx_count == 1 or (tx_count % self.tx_log_every) == 0):
            self.logger.write(f"[JETSON-TX] #{tx_count} {hex_bytes(frame)}\n")

    def _run(self):
        while not self.stop_event.is_set():
            if self.enabled.is_set():
                try:
                    self.send_once()
                except Exception as exc:
                    self.logger.write(f"[JETSON-TX-ERR] {exc}\n")
                    return
            time.sleep(self.period_s)


def is_arm_path_line(line):
    return re.match(r"^\[\d+\]\s+<", line) is not None


def status_name(func, value):
    names = {
        (RA6_READY, 0x01): "READY",
        (RA6_ALIGN_DONE, 0x01): "ALIGN_DONE",
        (RA6_OUTPUT, 0x01): "OUTPUT_ON",
        (RA6_OUTPUT, 0x00): "OUTPUT_OFF",
        (RA6_ERROR, 0x01): "ERROR",
    }
    return names.get((func, value), "UNKNOWN")


def make_vision_frame(dcx, dcy):
    payload = int(dcx).to_bytes(2, "little", signed=True) + int(dcy).to_bytes(2, "little", signed=True)
    checksum = (JETSON_LEN + JETSON_FUNC_VISION_ERROR + sum(payload)) & 0xFF
    return bytes([JETSON_SOF, JETSON_LEN, JETSON_FUNC_VISION_ERROR]) + payload + bytes([checksum, JETSON_EOF])


def send_cmd(ser, logger, cmd):
    logger.write(f"[ARM-TX] {cmd}\n")
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()


def wait_manual(prompt, logger):
    logger.write(f"[PROMPT] {prompt}\n")
    input(prompt + "\n按 Enter 继续...")


def safe_disable(arm, logger):
    if arm is None or not arm.is_open:
        return
    try:
        send_cmd(arm, logger, "target_disable")
        time.sleep(0.2)
        send_cmd(arm, logger, "laser_off")
    except Exception as exc:
        logger.write(f"[WARN] safe disable failed: {exc}\n")


def initial_soft_reset(args, arm, arm_reader, logger):
    logger.write("[STEP] initial soft_reset to HOME...\n")
    send_cmd(arm, logger, "soft_reset")
    arm_reader.wait_line_contains(["soft reset final verify PASS"], args.soft_reset_timeout)
    logger.write("[DONE] initial soft_reset PASS; pose valid.\n")


def final_soft_reset(args, arm, arm_reader, logger):
    if args.skip_final_reset:
        logger.write("[STEP] final soft_reset skipped by --skip-final-reset.\n")
        return
    logger.write("[STEP] final soft_reset to HOME...\n")
    send_cmd(arm, logger, "soft_reset")
    arm_reader.wait_line_contains(["soft reset final verify PASS"], args.soft_reset_timeout)
    logger.write("[DONE] final soft_reset PASS; robot returned HOME.\n")


def run_motion_cmd(args, arm, arm_reader, logger, cmd):
    lower = cmd.lower()
    if lower.startswith("sleep ") or lower.startswith("wait "):
        _, value = cmd.split(maxsplit=1)
        delay_s = float(value)
        logger.write(f"[STEP] sleep {delay_s:.2f}s\n")
        time.sleep(delay_s)
        return

    send_cmd(arm, logger, cmd)
    if lower.startswith("auto ") or lower.startswith("circle") or lower.startswith("time_func"):
        arm_reader.wait_line_contains(["robot pid run finished!!"], AUTO_TIMEOUT_S)
    elif lower == "soft_reset":
        arm_reader.wait_line_contains(["soft reset final verify PASS"], args.soft_reset_timeout)
    else:
        time.sleep(0.2)


def run_motion_group(args, arm, arm_reader, logger, title, commands):
    command_text = "\n".join(f"  {cmd}" for cmd in commands)
    wait_manual(f"准备拍摄：{title}\n即将执行：\n{command_text}", logger)
    for cmd in commands:
        run_motion_cmd(args, arm, arm_reader, logger, cmd)
    logger.write(f"[DONE] command group finished: {title}\n")
    time.sleep(0.5)


def load_motion_groups(sequence_file):
    groups = []
    title_lines = []
    commands = []

    def flush_group():
        nonlocal title_lines, commands
        if commands:
            title = " / ".join(title_lines) if title_lines else commands[0]
            groups.append((title, commands))
        title_lines = []
        commands = []

    with sequence_file.open("r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                flush_group()
                continue
            if line.startswith("#"):
                title = line.lstrip("#").strip()
                if title:
                    title_lines.append(title)
                continue
            commands.append(line)

    flush_group()
    if not groups:
        raise ValueError(f"sequence file has no commands: {sequence_file}")
    return groups


def run_motion_sequence(args, arm, arm_reader, logger):
    groups = load_motion_groups(args.sequence_file) if args.sequence_file and args.sequence_file.exists() else DEFAULT_MOTION_GROUPS
    for title, commands in groups:
        run_motion_group(args, arm, arm_reader, logger, title, commands)


def run_target_sequence(args, arm, arm_reader, status_reader, vision_sender, logger):
    wait_manual("准备拍摄：视觉定靶状态机。请把镜头对准靶纸、末端和 P000 按键区域。", logger)
    send_cmd(arm, logger, "target_enable")
    arm_reader.wait_line_exact("target_enable", 3.0)

    logger.write("[STEP] waiting READY from RA6...\n")
    status_reader.wait_for((RA6_READY, 0x01), args.ready_timeout)

    vision_sender.set_error(0, 0)
    vision_sender.enable()
    logger.write("[STEP] sending aligned vision frames, waiting ALIGN_DONE...\n")
    status_reader.wait_for((RA6_ALIGN_DONE, 0x01), args.align_timeout)

    logger.write("[PROMPT] 已对准。请按住 P000 KEY，脚本会自动等待 OUTPUT_ON，不需要按 Enter。\n")
    logger.write("[STEP] waiting OUTPUT_ON from RA6...\n")
    status_reader.wait_for((RA6_OUTPUT, 0x01), args.output_timeout)

    logger.write("[PROMPT] 已进入 OUTPUT。请松开 P000 KEY，脚本会自动等待 OUTPUT_OFF，不需要按 Enter。\n")
    logger.write("[STEP] waiting OUTPUT_OFF from RA6...\n")
    status_reader.wait_for((RA6_OUTPUT, 0x00), args.output_timeout)

    vision_sender.disable()
    send_cmd(arm, logger, "target_disable")
    time.sleep(0.2)
    send_cmd(arm, logger, "laser_off")
    logger.write("[DONE] target state demo finished.\n")


def run_demo(args, arm, arm_reader, status_reader, vision_sender, logger):
    safe_disable(arm, logger)
    time.sleep(0.2)
    initial_soft_reset(args, arm, arm_reader, logger)

    if args.mode in ("full", "motion"):
        run_motion_sequence(args, arm, arm_reader, logger)

    if args.mode in ("full", "target"):
        run_target_sequence(args, arm, arm_reader, status_reader, vision_sender, logger)

    if args.mode != "motion":
        final_soft_reset(args, arm, arm_reader, logger)
    logger.write(f"[DONE] shooting demo complete: mode={args.mode}.\n")


def parse_args():
    default_debug = pathlib.Path(__file__).resolve().parent
    default_sequence = default_debug / "shooting_sequence.txt"
    parser = argparse.ArgumentParser(description="RA6M5 robot shooting demo script")
    parser.add_argument("--arm-port", default=ARM_PORT)
    parser.add_argument("--jetson-port", default=JETSON_PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--mode", choices=("full", "motion", "target"), default="full")
    parser.add_argument("--sequence-file", type=pathlib.Path, default=default_sequence,
                        help="UTF-8 text file for motion groups. Blank lines split groups; # lines are group titles.")
    parser.add_argument("--debug-dir", type=pathlib.Path, default=default_debug)
    parser.add_argument("--skip-final-reset", action="store_true",
                        help="Do not send final soft_reset after demo.")
    parser.add_argument("--verbose-arm-path", action="store_true",
                        help="Log every ARM path interpolation line.")
    parser.add_argument("--vision-period", type=float, default=0.2,
                        help="Seconds between simulated Jetson vision frames while enabled.")
    parser.add_argument("--tx-log-every", type=int, default=0,
                        help="Log one JETSON-TX line every N frames; 0 disables per-frame TX logs.")
    parser.add_argument("--soft-reset-timeout", type=float, default=45.0)
    parser.add_argument("--ready-timeout", type=float, default=25.0)
    parser.add_argument("--align-timeout", type=float, default=10.0)
    parser.add_argument("--output-timeout", type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()
    logger = Logger(args.debug_dir)
    stop_event = threading.Event()
    needs_jetson = args.mode in ("full", "target")

    logger.write(f"[INFO] log file: {logger.path}\n")
    logger.write(f"[INFO] ARM={args.arm_port} baud={args.baud} mode={args.mode}\n")
    if needs_jetson:
        logger.write(f"[INFO] JETSON={args.jetson_port} aligned frame: {hex_bytes(make_vision_frame(0, 0))}\n")

    arm = None
    jetson = None
    vision_sender = None
    try:
        arm = serial.Serial(args.arm_port, args.baud, timeout=0.05)
        arm_reader = ArmReader(arm, logger, stop_event, args.verbose_arm_path)
        arm_reader.start()

        status_reader = None
        if needs_jetson:
            jetson = serial.Serial(args.jetson_port, args.baud, timeout=0.05)
            status_reader = StatusReader(jetson, logger, stop_event)
            vision_sender = VisionSender(jetson, logger, stop_event,
                                         period_s=args.vision_period,
                                         tx_log_every=args.tx_log_every)
            status_reader.start()
            vision_sender.start()

        time.sleep(0.5)
        run_demo(args, arm, arm_reader, status_reader, vision_sender, logger)
    except KeyboardInterrupt:
        logger.write("[ABORT] keyboard interrupt.\n")
    except Exception as exc:
        logger.write(f"[FATAL] {type(exc).__name__}: {exc}\n")
    finally:
        if vision_sender is not None:
            vision_sender.disable()
        stop_event.set()
        time.sleep(0.2)
        safe_disable(arm, logger)
        if arm is not None and arm.is_open:
            arm.close()
        if jetson is not None and jetson.is_open:
            jetson.close()
        logger.write("[INFO] ports closed.\n")
        logger.close()


if __name__ == "__main__":
    main()

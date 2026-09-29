import argparse
import datetime as dt
import math
import pathlib
import queue
import re
import statistics
import sys
import threading
import time
from dataclasses import dataclass

import serial


ARM_PORT = "COM7"
BAUD = 115200
DEFAULT_CYCLES = 50
DEFAULT_X = 23.15
DEFAULT_Y = -123.45
DEFAULT_Z = -15.20
JOINT_COUNT = 5

RESULT_RE = re.compile(r"result:\s*(.*)$")
READ_ALL_RE = re.compile(r"\bJ([1-5])\s+angle=([-+]?\d+(?:\.\d+)?)")
AVE_ERROR_RE = re.compile(r"\[j(?:oint|pint)\s+([1-5])\]\s+ave_error:([-+]?\d+(?:\.\d+)?)", re.IGNORECASE)


@dataclass
class JointSummary:
    joint: int
    mean_angle: float
    stddev_deg: float
    range_deg: float
    mean_abs_arrival_error: float
    max_abs_arrival_error: float


@dataclass
class CycleResult:
    cycle: int
    target_angles: list[float]
    read_angles: list[float]
    errors: list[float]
    ave_errors: dict[int, float]
    ok: bool
    message: str = ""


def now_stamp() -> str:
    return dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def circular_diff_deg(measured: float, target: float) -> float:
    """Return measured-target as the shortest signed angular difference."""
    return (float(measured) - float(target) + 180.0) % 360.0 - 180.0


def normalize_angle_deg(angle: float) -> float:
    return float(angle) % 360.0


def parse_last_result_angles(lines: list[str]) -> list[float] | None:
    last: list[float] | None = None
    for line in lines:
        match = RESULT_RE.search(line)
        if not match:
            continue
        values = [float(item) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", match.group(1))]
        if len(values) >= JOINT_COUNT:
            last = values[:JOINT_COUNT]
    return last


def parse_read_all_angles(lines: list[str]) -> list[float] | None:
    values: dict[int, float] = {}
    for line in lines:
        match = READ_ALL_RE.search(line)
        if match:
            values[int(match.group(1))] = float(match.group(2))
    if all(joint in values for joint in range(1, JOINT_COUNT + 1)):
        return [values[joint] for joint in range(1, JOINT_COUNT + 1)]
    return None


def parse_ave_error_line(line: str) -> tuple[int, float] | None:
    match = AVE_ERROR_RE.search(line)
    if not match:
        return None
    return int(match.group(1)), float(match.group(2))


def summarize_samples(samples: list[list[float]], arrival_errors: list[list[float]] | None = None) -> list[JointSummary]:
    if not samples:
        return []

    summaries: list[JointSummary] = []
    for joint_index in range(JOINT_COUNT):
        base = samples[0][joint_index]
        deltas = [circular_diff_deg(sample[joint_index], base) for sample in samples]
        mean_delta = statistics.fmean(deltas)
        mean_angle = normalize_angle_deg(base + mean_delta)
        centered = [delta - mean_delta for delta in deltas]
        stddev = math.sqrt(statistics.fmean([value * value for value in centered])) if len(centered) > 1 else 0.0
        angle_range = max(deltas) - min(deltas) if deltas else 0.0

        if arrival_errors:
            abs_errors = [abs(error[joint_index]) for error in arrival_errors if len(error) > joint_index]
        else:
            abs_errors = []
        summaries.append(
            JointSummary(
                joint=joint_index + 1,
                mean_angle=mean_angle,
                stddev_deg=stddev,
                range_deg=angle_range,
                mean_abs_arrival_error=statistics.fmean(abs_errors) if abs_errors else 0.0,
                max_abs_arrival_error=max(abs_errors) if abs_errors else 0.0,
            )
        )
    return summaries


class Logger:
    def __init__(self, debug_dir: pathlib.Path):
        debug_dir.mkdir(parents=True, exist_ok=True)
        ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = debug_dir / f"repeat_position_error_{ts}.txt"
        self._lock = threading.Lock()
        self._file = self.path.open("w", encoding="utf-8", newline="")

    def write(self, msg: str) -> None:
        with self._lock:
            self._file.write(msg)
            self._file.flush()
            sys.stdout.write(msg)
            sys.stdout.flush()

    def close(self) -> None:
        with self._lock:
            self._file.close()


class ArmReader:
    def __init__(self, ser: serial.Serial, logger: Logger, stop_event: threading.Event):
        self.ser = ser
        self.logger = logger
        self.stop_event = stop_event
        self.lines: queue.Queue[str] = queue.Queue()
        self._buf = bytearray()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(512)
            except Exception as exc:
                self.logger.write(f"[{now_stamp()}] [ARM-ERR] {exc}\n")
                return
            if not data:
                continue
            self._buf.extend(data)
            while b"\n" in self._buf:
                raw, _, rest = self._buf.partition(b"\n")
                self._buf = bytearray(rest)
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                self.logger.write(f"[{now_stamp()}] [ARM-RX] {line}\n")
                self.lines.put(line)

    def drain(self) -> None:
        while True:
            try:
                self.lines.get_nowait()
            except queue.Empty:
                return

    def wait_line_contains(self, needles: list[str], timeout_s: float, sink: list[str]) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            sink.append(line)
            lower = line.lower()
            if "soft reset final fail" in lower:
                raise RuntimeError(line)
            if "inverse failed" in lower:
                raise RuntimeError(line)
            if "reject auto" in lower:
                raise RuntimeError(line)
            if "read_all ret" in lower and "ret=0" not in lower:
                raise RuntimeError(line)
            for needle in needles:
                if needle in line:
                    return line
        raise TimeoutError(f"timeout waiting for {needles}")

    def collect_for(self, duration_s: float, sink: list[str]) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=0.05)
            except queue.Empty:
                continue
            sink.append(line)


def send_cmd(ser: serial.Serial, logger: Logger, cmd: str) -> None:
    logger.write(f"[{now_stamp()}] [ARM-TX] {cmd}\n")
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()


def run_cycle(
    cycle: int,
    ser: serial.Serial,
    reader: ArmReader,
    logger: Logger,
    auto_cmd: str,
    soft_reset_timeout: float,
    auto_timeout: float,
) -> CycleResult:
    cycle_lines: list[str] = []
    ave_errors: dict[int, float] = {}

    logger.write(f"\n===== CYCLE {cycle} START =====\n")
    reader.drain()

    send_cmd(ser, logger, "soft_reset")
    reader.wait_line_contains(["soft reset final verify PASS"], soft_reset_timeout, cycle_lines)

    send_cmd(ser, logger, auto_cmd)
    reader.wait_line_contains(["robot pid run finished!!"], auto_timeout, cycle_lines)
    for line in cycle_lines:
        parsed = parse_ave_error_line(line)
        if parsed:
            joint, value = parsed
            ave_errors[joint] = value

    target_angles = parse_last_result_angles(cycle_lines)
    if target_angles is None:
        raise RuntimeError("AUTO target result line not found")

    send_cmd(ser, logger, "read_all")
    reader.wait_line_contains(["J5 angle="], 8.0, cycle_lines)
    reader.collect_for(0.2, cycle_lines)

    read_angles = parse_read_all_angles(cycle_lines)
    if read_angles is None:
        raise RuntimeError("read_all J1-J5 angles not found")

    errors = [circular_diff_deg(read_angles[i], target_angles[i]) for i in range(JOINT_COUNT)]
    logger.write(format_cycle_result(cycle, target_angles, read_angles, errors, ave_errors))
    logger.write(f"===== CYCLE {cycle} END =====\n")

    return CycleResult(
        cycle=cycle,
        target_angles=target_angles,
        read_angles=read_angles,
        errors=errors,
        ave_errors=ave_errors,
        ok=True,
    )


def format_float_list(values: list[float]) -> str:
    return " ".join(f"{value:8.3f}" for value in values)


def format_cycle_result(
    cycle: int,
    target_angles: list[float],
    read_angles: list[float],
    errors: list[float],
    ave_errors: dict[int, float],
) -> str:
    ave_text = " ".join(f"J{joint}={ave_errors.get(joint, float('nan')):.3f}" for joint in range(1, JOINT_COUNT + 1))
    return (
        f"[CYCLE {cycle:02d}] target_deg: {format_float_list(target_angles)}\n"
        f"[CYCLE {cycle:02d}] read_deg:   {format_float_list(read_angles)}\n"
        f"[CYCLE {cycle:02d}] error_deg:  {format_float_list(errors)}\n"
        f"[CYCLE {cycle:02d}] ave_error:  {ave_text}\n"
    )


def write_summary(logger: Logger, results: list[CycleResult]) -> None:
    ok_results = [result for result in results if result.ok]
    logger.write("\n========== SUMMARY ==========\n")
    logger.write(f"valid cycles: {len(ok_results)} / {len(results)}\n")
    if not ok_results:
        logger.write("no valid cycle data\n")
        return

    read_samples = [result.read_angles for result in ok_results]
    arrival_errors = [result.errors for result in ok_results]
    summaries = summarize_samples(read_samples, arrival_errors)

    logger.write("joint | mean_read_deg | repeat_std_deg | repeat_range_deg | mean_arrival_err_deg | max_arrival_err_deg\n")
    for item in summaries:
        logger.write(
            f"J{item.joint:<4d}|"
            f" {item.mean_angle:13.3f} |"
            f" {item.stddev_deg:14.4f} |"
            f" {item.range_deg:16.4f} |"
            f" {item.mean_abs_arrival_error:20.4f} |"
            f" {item.max_abs_arrival_error:19.4f}\n"
        )

    mean_arrival = statistics.fmean(item.mean_abs_arrival_error for item in summaries)
    mean_repeat_std = statistics.fmean(item.stddev_deg for item in summaries)
    logger.write(f"\n平均到达误差 = {mean_arrival:.4f} deg\n")
    logger.write(f"平均重复性标准差 = {mean_repeat_std:.4f} deg\n")
    logger.write("说明：以上为关节角误差，不等同于末端 mm 重复定位误差。\n")


def parse_args() -> argparse.Namespace:
    default_debug_dir = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="RA6M5 repeated joint-position error test over COM7")
    parser.add_argument("--port", default=ARM_PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--x", type=float, default=DEFAULT_X)
    parser.add_argument("--y", type=float, default=DEFAULT_Y)
    parser.add_argument("--z", type=float, default=DEFAULT_Z)
    parser.add_argument("--debug-dir", type=pathlib.Path, default=default_debug_dir)
    parser.add_argument("--auto-timeout", type=float, default=60.0)
    parser.add_argument("--soft-reset-timeout", type=float, default=20.0)
    parser.add_argument("--continue-on-fail", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    auto_cmd = f"auto {args.x:.2f} {args.y:.2f} {args.z:.2f}"
    logger = Logger(args.debug_dir)
    stop_event = threading.Event()
    ser: serial.Serial | None = None
    results: list[CycleResult] = []

    try:
        logger.write(f"[INFO] log file: {logger.path}\n")
        logger.write(f"[INFO] ARM={args.port} baud={args.baud} cycles={args.cycles} auto='{auto_cmd}'\n")
        ser = serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0)
        reader = ArmReader(ser, logger, stop_event)
        reader.start()
        time.sleep(0.8)

        for cycle in range(1, args.cycles + 1):
            try:
                result = run_cycle(
                    cycle=cycle,
                    ser=ser,
                    reader=reader,
                    logger=logger,
                    auto_cmd=auto_cmd,
                    soft_reset_timeout=args.soft_reset_timeout,
                    auto_timeout=args.auto_timeout,
                )
                results.append(result)
            except Exception as exc:
                message = str(exc)
                logger.write(f"[CYCLE {cycle:02d}] FAILED: {message}\n")
                results.append(CycleResult(cycle, [], [], [], {}, False, message))
                if not args.continue_on_fail:
                    break

        write_summary(logger, results)
        logger.write(f"[DONE] repeat position test finished. log={logger.path}\n")
        return 0 if all(result.ok for result in results) else 1
    except Exception as exc:
        logger.write(f"[FATAL] {exc}\n")
        return 1
    finally:
        stop_event.set()
        time.sleep(0.2)
        if ser is not None and ser.is_open:
            ser.close()
        logger.close()


if __name__ == "__main__":
    raise SystemExit(main())

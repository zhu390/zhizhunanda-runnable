import argparse
import datetime as dt
from dataclasses import dataclass
import pathlib
import queue
import secrets
import struct
import sys
import threading
import time

import serial


SOF = b"\xA5\x5A"
VERSION = 0x01
MAX_PAYLOAD = 32

MSG_HEARTBEAT = 0x01
MSG_TARGET_CTRL = 0x02
MSG_VISION_ERROR = 0x03
MSG_CAPTURE_CTRL = 0x04
MSG_SAFE_DISTANCE = 0x05
MSG_WORKFLOW_CTRL = 0x06
MSG_STATUS = 0x81
MSG_ERROR = 0xFE

EVENT_READY = 0x01
EVENT_ALIGN_DONE = 0x02
EVENT_OUTPUT = 0x03
EVENT_TARGET_CTRL = 0x04
EVENT_SAFE_DISTANCE = 0x06
EVENT_VISION_STATE = 0x07
EVENT_CAPTURE_POINT = 0x10
EVENT_CAPTURE_HOME = 0x11
EVENT_SELECTED_VIEW = 0x12
EVENT_WORKFLOW = 0x20
EVENT_COMMAND_ACK = 0x21

WORKFLOW_START = 0x01
WORKFLOW_FINISH = 0x02
WORKFLOW_ABORT = 0x03
WORKFLOW_START_ACCEPTED = 0x01
WORKFLOW_MEASURE_READY = 0x02
WORKFLOW_SAFE_LATCHED = 0x03
WORKFLOW_RETURN_HOME_DONE = 0x05
WORKFLOW_ABORTED_HOLD = 0x06

CAPTURE_GOTO = 0x01
CAPTURE_HOME = 0x02
CAPTURE_SELECT = 0x03

DEFAULT_ARM_PORT = "COM7"
DEFAULT_JETSON_PORT = "COM14"
DEFAULT_VISION_SEQUENCE = "20,-15:2;8,-6:2;0,0:0"


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def build_frame(msg_type, seq, payload=b""):
    payload = bytes(payload)
    if not 0 <= msg_type <= 0xFF or not 0 <= seq <= 0xFF:
        raise ValueError("msg_type and seq must fit uint8")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes((VERSION, msg_type, seq, len(payload))) + payload
    crc = crc16_modbus(body)
    return SOF + body + struct.pack("<H", crc)


def hex_bytes(data):
    return " ".join(f"{byte:02X}" for byte in data)


@dataclass(frozen=True)
class Frame:
    msg_type: int
    seq: int
    payload: bytes


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == SOF[:1] else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 8:
                break

            payload_len = self.buffer[5]
            if payload_len > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            frame_len = 8 + payload_len
            if len(self.buffer) < frame_len:
                break

            raw = bytes(self.buffer[:frame_len])
            body = raw[2 : 6 + payload_len]
            rx_crc = struct.unpack_from("<H", raw, 6 + payload_len)[0]
            if raw[2] != VERSION or crc16_modbus(body) != rx_crc:
                del self.buffer[0]
                continue

            frames.append(Frame(raw[3], raw[4], raw[6 : 6 + payload_len]))
            del self.buffer[:frame_len]
        return frames


class Logger:
    def __init__(self, debug_dir):
        debug_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = debug_dir / f"formal_workflow_demo_{stamp}.log"
        self.lock = threading.Lock()
        self.file = self.path.open("w", encoding="utf-8", newline="")

    def write(self, message):
        line = f"[{dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {message}\n"
        with self.lock:
            self.file.write(line)
            self.file.flush()
            sys.stdout.write(line)
            sys.stdout.flush()

    def close(self):
        with self.lock:
            self.file.close()


class SequenceCounter:
    def __init__(self):
        self.value = secrets.randbelow(0xFF) + 1
        self.lock = threading.Lock()

    def next(self):
        with self.lock:
            value = self.value
            self.value = 1 if self.value >= 0xFF else self.value + 1
            return value


class PortWriter:
    def __init__(self, port):
        self.port = port
        self.lock = threading.Lock()

    def send(self, frame):
        with self.lock:
            self.port.write(frame)
            self.port.flush()


STATUS_NAMES = {
    EVENT_READY: "TARGET_READY",
    EVENT_ALIGN_DONE: "ALIGN_DONE",
    EVENT_OUTPUT: "OUTPUT",
    EVENT_TARGET_CTRL: "TARGET_CTRL",
    EVENT_SAFE_DISTANCE: "SAFE_DISTANCE",
    EVENT_VISION_STATE: "VISION_STATE",
    EVENT_CAPTURE_POINT: "CAPTURE_POINT_READY",
    EVENT_CAPTURE_HOME: "CAPTURE_HOME_READY",
    EVENT_SELECTED_VIEW: "SELECTED_VIEW_READY",
    EVENT_WORKFLOW: "WORKFLOW",
    EVENT_COMMAND_ACK: "COMMAND_ACK",
}


class StatusReader:
    def __init__(self, port, logger, stop_event):
        self.port = port
        self.logger = logger
        self.stop_event = stop_event
        self.parser = FrameParser()
        self.events = queue.Queue()
        self.pending = []
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                data = self.port.read(256)
            except Exception as exc:
                self.logger.write(f"[JETSON-RX-ERROR] {exc}")
                return
            if not data:
                continue
            for frame in self.parser.feed(data):
                if frame.msg_type == MSG_STATUS and len(frame.payload) == 3:
                    event, value, error = frame.payload
                    name = STATUS_NAMES.get(event, f"EVENT_0x{event:02X}")
                    self.logger.write(
                        f"[STATUS] seq={frame.seq:02X} {name} value={value:02X} error={error:02X}"
                    )
                elif frame.msg_type == MSG_ERROR and len(frame.payload) == 1:
                    self.logger.write(
                        f"[ERROR] seq={frame.seq:02X} code={frame.payload[0]:02X}"
                    )
                else:
                    self.logger.write(
                        f"[JETSON-RX] type={frame.msg_type:02X} seq={frame.seq:02X} "
                        f"payload={hex_bytes(frame.payload)}"
                    )
                self.events.put(frame)

    @staticmethod
    def _status_matches(frame, event, value, seq):
        if frame.msg_type != MSG_STATUS or len(frame.payload) != 3:
            return False
        return ((seq is None or frame.seq == seq) and
                frame.payload[0] == event and
                (value is None or frame.payload[1] == value))

    def wait_status(self, event, value=None, seq=None, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for index, frame in enumerate(self.pending):
                if self._status_matches(frame, event, value, seq):
                    return self.pending.pop(index)
                if frame.msg_type == MSG_ERROR:
                    self.pending.pop(index)
                    raise RuntimeError(
                        f"RA6 ERROR seq={frame.seq:02X} code={frame.payload[0]:02X}"
                    )
            try:
                frame = self.events.get(timeout=0.1)
            except queue.Empty:
                continue
            if self._status_matches(frame, event, value, seq):
                return frame
            if frame.msg_type == MSG_ERROR:
                raise RuntimeError(
                    f"RA6 ERROR seq={frame.seq:02X} code={frame.payload[0]:02X}"
                )
            self.pending.append(frame)
        name = STATUS_NAMES.get(event, f"EVENT_0x{event:02X}")
        raise TimeoutError(f"timeout waiting {name} value={value} seq={seq}")


class ArmLogReader:
    def __init__(self, port, logger, stop_event):
        self.port = port
        self.logger = logger
        self.stop_event = stop_event
        self.buffer = bytearray()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                data = self.port.read(256)
            except Exception as exc:
                self.logger.write(f"[ARM-RX-ERROR] {exc}")
                return
            if not data:
                continue
            self.buffer.extend(data)
            while b"\n" in self.buffer:
                raw, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                self.logger.write(f"[ARM] {raw.decode('utf-8', errors='replace').rstrip()}")


class HeartbeatSender:
    def __init__(self, writer, sequences, stop_event, period):
        self.writer = writer
        self.sequences = sequences
        self.stop_event = stop_event
        self.period = period
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            tick = int(time.monotonic() * 1000) & 0xFFFFFFFF
            frame = build_frame(MSG_HEARTBEAT, self.sequences.next(), struct.pack("<I", tick))
            try:
                self.writer.send(frame)
            except Exception:
                return
            self.stop_event.wait(self.period)


def parse_vision_sequence(text):
    result = []
    for item in text.split(";"):
        pair, _, duration = item.strip().partition(":")
        dcx, dcy = (int(value.strip()) for value in pair.split(","))
        result.append((dcx, dcy, float(duration) if duration else 1.5))
    if not result:
        raise ValueError("vision sequence is empty")
    return result


class VisionSender:
    def __init__(self, writer, sequences, stop_event, period, steps):
        self.writer = writer
        self.sequences = sequences
        self.stop_event = stop_event
        self.period = period
        self.steps = steps
        self.enabled = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        index = 0
        entered = time.monotonic()
        while not self.stop_event.is_set():
            if not self.enabled.wait(0.05):
                index = 0
                entered = time.monotonic()
                continue
            dcx, dcy, duration = self.steps[index]
            if duration > 0 and index < len(self.steps) - 1 and time.monotonic() - entered >= duration:
                index += 1
                entered = time.monotonic()
                dcx, dcy, duration = self.steps[index]
            payload = struct.pack("<hhB", dcx, dcy, 1)
            self.writer.send(build_frame(MSG_VISION_ERROR, self.sequences.next(), payload))
            self.stop_event.wait(self.period)


def send_arm_command(port, logger, command):
    if port is None:
        return
    logger.write(f"[ARM-TX] {command}")
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def send_control(writer, reader, sequences, logger, msg_type, payload, label, timeout=5.0):
    seq = sequences.next()
    frame = build_frame(msg_type, seq, payload)
    logger.write(f"[JETSON-TX] {label} seq={seq:02X}: {hex_bytes(frame)}")
    writer.send(frame)
    ack = reader.wait_status(EVENT_COMMAND_ACK, seq=seq, timeout=timeout)
    _, accepted, error = ack.payload
    if accepted != 1 or error != 0:
        raise RuntimeError(f"{label} rejected: error=0x{error:02X}")
    return seq


def wait_business(reader, event, value, seq, timeout, logger, label):
    reader.wait_status(event, value=value, seq=seq, timeout=timeout)
    logger.write(f"[STEP] {label}")


def maybe_prompt(args, logger, message):
    logger.write(f"[PROMPT] {message}")
    if not args.auto_advance:
        input("按 Enter 继续...")


def run_demo(args, arm, writer, reader, sequences, vision, logger):
    send_arm_command(arm, logger, "laser_off")
    time.sleep(0.2)

    start_seq = send_control(writer, reader, sequences, logger,
                             MSG_WORKFLOW_CTRL, bytes((WORKFLOW_START,)),
                             "WORKFLOW START")
    wait_business(reader, EVENT_WORKFLOW, WORKFLOW_START_ACCEPTED, start_seq,
                  5.0, logger, "workflow start accepted")
    wait_business(reader, EVENT_WORKFLOW, WORKFLOW_MEASURE_READY, start_seq,
                  args.measure_timeout, logger, "public measurement position ready")

    for index in range(3):
        seq = sequences.next()
        payload = struct.pack("<HB", args.safe_distance_mm, 1)
        writer.send(build_frame(MSG_SAFE_DISTANCE, seq, payload))
        logger.write(
            f"[JETSON-TX] SAFE_DISTANCE {index + 1}/3 seq={seq:02X} "
            f"distance={args.safe_distance_mm}mm"
        )
        time.sleep(args.telemetry_period)
    wait_business(reader, EVENT_WORKFLOW, WORKFLOW_SAFE_LATCHED, start_seq,
                  5.0, logger, "safe distance latched")

    maybe_prompt(args, logger, "开始三视图逐点采集")
    capture_seq = send_control(writer, reader, sequences, logger,
                               MSG_CAPTURE_CTRL, bytes((CAPTURE_HOME, 0)),
                               "CAPTURE HOME", timeout=5.0)
    wait_business(reader, EVENT_CAPTURE_HOME, 1, capture_seq,
                  args.home_timeout, logger, "capture HOME ready")

    for point_id, name in ((1, "left"), (2, "front"), (3, "right")):
        capture_seq = send_control(writer, reader, sequences, logger,
                                   MSG_CAPTURE_CTRL, bytes((CAPTURE_GOTO, point_id)),
                                   f"CAPTURE GOTO {name}")
        wait_business(reader, EVENT_CAPTURE_POINT, point_id, capture_seq,
                      args.motion_timeout, logger, f"{name} view ready; simulated image saved")
        time.sleep(args.capture_hold)

    capture_seq = send_control(writer, reader, sequences, logger,
                               MSG_CAPTURE_CTRL, bytes((CAPTURE_HOME, 0)),
                               "CAPTURE HOME after views")
    wait_business(reader, EVENT_CAPTURE_HOME, 1, capture_seq,
                  args.home_timeout, logger, "three-view capture completed at HOME")

    select_seq = send_control(writer, reader, sequences, logger,
                              MSG_CAPTURE_CTRL, bytes((CAPTURE_SELECT, args.view_id)),
                              f"SELECT VIEW {args.view_id}")
    wait_business(reader, EVENT_SELECTED_VIEW, args.view_id, select_seq,
                  args.motion_timeout, logger, f"selected view {args.view_id} ready")

    target_seq = send_control(writer, reader, sequences, logger,
                              MSG_TARGET_CTRL, b"\x01", "TARGET START")
    wait_business(reader, EVENT_TARGET_CTRL, 1, target_seq,
                  5.0, logger, "target control enabled")
    wait_business(reader, EVENT_READY, 1, target_seq,
                  5.0, logger, "target ready; vision transmission allowed")

    vision.enabled.set()
    logger.write("[STEP] vision sequence enabled; waiting ALIGN_DONE")
    wait_business(reader, EVENT_ALIGN_DONE, 1, target_seq,
                  args.align_timeout, logger, "alignment confirmed")

    maybe_prompt(args, logger, "确认安全后按住 P000；脚本随后等待 OUTPUT_ON")
    wait_business(reader, EVENT_OUTPUT, 1, target_seq,
                  args.key_timeout, logger, "laser output on")
    logger.write("[PROMPT] 请松开 P000；脚本等待 OUTPUT_OFF")
    wait_business(reader, EVENT_OUTPUT, 0, target_seq,
                  args.output_off_timeout, logger, "laser output off")
    vision.enabled.clear()

    if args.skip_final_reset:
        finish_seq = send_control(writer, reader, sequences, logger,
                                  MSG_WORKFLOW_CTRL, bytes((WORKFLOW_ABORT,)),
                                  "WORKFLOW ABORT_HOLD")
        wait_business(reader, EVENT_WORKFLOW, WORKFLOW_ABORTED_HOLD, finish_seq,
                      5.0, logger, "workflow held without HOME")
    else:
        finish_seq = send_control(writer, reader, sequences, logger,
                                  MSG_WORKFLOW_CTRL, bytes((WORKFLOW_FINISH,)),
                                  "WORKFLOW FINISH_RETURN_HOME")
        wait_business(reader, EVENT_WORKFLOW, WORKFLOW_RETURN_HOME_DONE, finish_seq,
                      args.home_timeout, logger, "final soft_reset PASS; workflow IDLE")


def parse_args():
    parser = argparse.ArgumentParser(
        description="PC Jetson simulator for the formal RA6M5 A5 5A competition workflow"
    )
    parser.add_argument("--arm-port", default=DEFAULT_ARM_PORT,
                        help="RA6M5 text log port, or 'none' to disable")
    parser.add_argument("--jetson-port", default=DEFAULT_JETSON_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--debug-dir", type=pathlib.Path,
                        default=pathlib.Path(__file__).resolve().parent)
    parser.add_argument("--view-id", type=int, choices=(1, 2, 3), default=2)
    parser.add_argument("--safe-distance-mm", type=int, default=160)
    parser.add_argument("--heartbeat-period", type=float, default=0.2)
    parser.add_argument("--telemetry-period", type=float, default=0.2)
    parser.add_argument("--vision-sequence", default=DEFAULT_VISION_SEQUENCE)
    parser.add_argument("--capture-hold", type=float, default=1.0)
    parser.add_argument("--measure-timeout", type=float, default=65.0)
    parser.add_argument("--home-timeout", type=float, default=50.0)
    parser.add_argument("--motion-timeout", type=float, default=20.0)
    parser.add_argument("--align-timeout", type=float, default=30.0)
    parser.add_argument("--key-timeout", type=float, default=30.0)
    parser.add_argument("--output-off-timeout", type=float, default=12.0)
    parser.add_argument("--auto-advance", action="store_true")
    parser.add_argument("--skip-final-reset", action="store_true")
    args = parser.parse_args()
    if not 150 <= args.safe_distance_mm <= 65535:
        parser.error("--safe-distance-mm must be at least the firmware threshold of 150 mm")
    return args


def main():
    args = parse_args()
    logger = Logger(args.debug_dir)
    stop_event = threading.Event()
    sequences = SequenceCounter()
    arm = None
    jetson = None
    writer = None
    workflow_started = False
    completed = False

    logger.write(f"[INFO] log={logger.path}")
    logger.write(
        f"[INFO] formal A5 5A protocol, ARM={args.arm_port}, "
        f"JETSON={args.jetson_port}, view={args.view_id}"
    )

    try:
        if args.arm_port.lower() != "none":
            arm = serial.Serial(args.arm_port, args.baud, timeout=0.05)
            ArmLogReader(arm, logger, stop_event).start()
        jetson = serial.Serial(args.jetson_port, args.baud, timeout=0.05)
        writer = PortWriter(jetson)
        reader = StatusReader(jetson, logger, stop_event)
        reader.start()
        heartbeat = HeartbeatSender(writer, sequences, stop_event, args.heartbeat_period)
        heartbeat.start()
        vision = VisionSender(writer, sequences, stop_event, args.telemetry_period,
                              parse_vision_sequence(args.vision_sequence))
        vision.start()
        time.sleep(0.6)

        workflow_started = True
        run_demo(args, arm, writer, reader, sequences, vision, logger)
        completed = True
        logger.write("[DONE] formal competition workflow completed")
    except KeyboardInterrupt:
        logger.write("[ABORT] keyboard interrupt")
    except Exception as exc:
        logger.write(f"[FATAL] {type(exc).__name__}: {exc}")
    finally:
        if workflow_started and not completed and writer is not None:
            try:
                seq = sequences.next()
                frame = build_frame(MSG_WORKFLOW_CTRL, seq, bytes((WORKFLOW_ABORT,)))
                writer.send(frame)
                logger.write(f"[CLEANUP] WORKFLOW ABORT_HOLD seq={seq:02X} sent")
            except Exception as exc:
                logger.write(f"[WARN] cleanup abort failed: {exc}")
        try:
            send_arm_command(arm, logger, "laser_off")
        except Exception as exc:
            logger.write(f"[WARN] cleanup laser_off failed: {exc}")
        stop_event.set()
        time.sleep(0.2)
        if jetson is not None and jetson.is_open:
            jetson.close()
        if arm is not None and arm.is_open:
            arm.close()
        logger.write("[INFO] ports closed")
        logger.close()


if __name__ == "__main__":
    main()

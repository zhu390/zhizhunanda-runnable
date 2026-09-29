"""Command-line helper for RA6M5 protocol-v5 bring-up.

The formal path uses one :class:`SerialSender` session.  Its background worker
owns heartbeat transmission, receive parsing, ACK retry, and control timeouts.
This CLI deliberately does not create a second keepalive loop.
"""

import argparse
import time
from typing import Callable, List, Optional, Tuple

from .serial_sender import PendingControl, SerialSender
from .vision_protocol import (
    ERROR_NONE,
    MSG_ERROR,
    Ra6Status,
    STATUS_COMMAND_ACK,
    STATUS_SAFE_DISTANCE,
)


CONTROL_COMMANDS = {
    "workflow-start",
    "workflow-finish",
    "workflow-abort",
    "target-start",
    "target-stop",
    "capture-move",
    "capture-home",
    "capture-select",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "RA6M5 protocol-v5 bring-up tool. Unified-mode heartbeat and "
            "control retries run automatically in the serial background worker."
        )
    )
    parser.add_argument("--serial-port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--protocol",
        choices=["unified", "legacy"],
        default="unified",
        help="legacy is appendix/debug only; the formal system uses unified",
    )
    parser.add_argument(
        "--command",
        required=True,
        choices=[
            "listen",
            "heartbeat",
            "workflow-start",
            "workflow-finish",
            "workflow-abort",
            "safe-distance",
            "target-start",
            "target-stop",
            "vision-error",
            "capture-move",
            "capture-home",
            "capture-select",
        ],
    )
    parser.add_argument("--point-id", type=int, default=1)
    parser.add_argument("--dcx", type=int, default=0)
    parser.add_argument("--dcy", type=int, default=0)
    parser.add_argument("--valid", type=int, choices=[0, 1], default=1)
    parser.add_argument("--distance-mm", type=int, default=120)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="extra status-listen time after the command has completed",
    )
    parser.add_argument("--heartbeat-interval", type=float, default=0.15)
    parser.add_argument("--safe-distance-interval", type=float, default=0.15)
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=0.0,
        help=(
            "control/safe-distance wait timeout; 0 uses the protocol action "
            "timeout (safe-distance defaults to 10 s)"
        ),
    )
    return parser.parse_args()


def _print_status(status: Ra6Status) -> None:
    if status.protocol == "legacy":
        detail = "func=0x{:02X} value=0x{:02X}".format(
            status.func,
            status.value,
        )
    else:
        seq = "--" if status.seq is None else "{:02X}".format(status.seq)
        detail = (
            "seq=0x{} event=0x{:02X} value=0x{:02X} error=0x{:02X}".format(
                seq,
                status.func,
                status.value,
                status.error_code,
            )
        )
    print("STATUS {} ({})".format(status.label, detail), flush=True)


def _drain(sender: SerialSender) -> Tuple[List[Ra6Status], List[str]]:
    # SerialSender's background worker is the sole serial reader.  Calling
    # poll_statuses() here would also attempt a foreground read; two readers
    # can observe ACK and business frames out of order when the UART delivers
    # them in adjacent chunks.  Consume only the worker's already-parsed queue.
    with sender._rx_lock:  # type: ignore[attr-defined]
        statuses = list(sender._received_statuses)  # type: ignore[attr-defined]
        sender._received_statuses.clear()  # type: ignore[attr-defined]
    events = sender.pop_transport_events()
    for event in events:
        print("TRANSPORT", event, flush=True)
    for status in statuses:
        _print_status(status)
    return statuses, events


def _is_async_error(status: Ra6Status) -> bool:
    return status.protocol == "unified_error" or status.msg_type == MSG_ERROR


def _wait_for_control(
    sender: SerialSender,
    pending: Optional[PendingControl],
    wait_timeout: float,
) -> bool:
    """Wait for same-SEQ COMMAND_ACK and the command's business completion."""
    if pending is None:
        _drain(sender)
        print("ERROR 控制命令未进入等待队列，无法确认 ACK 和业务完成状态", flush=True)
        return False

    timeout = (
        float(wait_timeout)
        if wait_timeout > 0
        else float(pending.completion_timeout_s) + 2.0
    )
    deadline = time.monotonic() + timeout
    saw_ack = False
    saw_completion = False

    while time.monotonic() < deadline:
        statuses, _events = _drain(sender)
        for status in statuses:
            if _is_async_error(status):
                print("ERROR 收到 RA6M5 异步错误：{}".format(status.label), flush=True)
                return False
            if status.seq != pending.seq:
                continue
            if status.func == STATUS_COMMAND_ACK:
                if not status.command_accepted:
                    print(
                        "ERROR 控制命令被拒绝：{} seq={} error=0x{:02X}".format(
                            pending.label,
                            pending.seq,
                            status.error_code,
                        ),
                        flush=True,
                    )
                    return False
                saw_ack = True
            if (
                status.func == pending.expected_event
                and status.value == pending.expected_value
            ):
                if status.error_code != ERROR_NONE:
                    print(
                        "ERROR 业务动作失败：{} error=0x{:02X}".format(
                            status.label,
                            status.error_code,
                        ),
                        flush=True,
                    )
                    return False
                saw_completion = True

        if saw_ack and saw_completion:
            return True

        # SerialSender clears the request on success, rejection, asynchronous
        # error, or timeout.  If either required response was not observed,
        # disappearance therefore cannot be reported as a successful command.
        if not sender.control_busy:
            # The worker clears ``pending`` while interpreting the frame and
            # appends parsed statuses to its public-facing queue immediately
            # afterwards.  Allow that tiny hand-off to finish before deciding.
            time.sleep(0.04)
            final_statuses, _final_events = _drain(sender)
            for status in final_statuses:
                if _is_async_error(status):
                    print("ERROR 收到 RA6M5 异步错误：{}".format(status.label), flush=True)
                    return False
                if status.seq != pending.seq:
                    continue
                if status.func == STATUS_COMMAND_ACK:
                    if not status.command_accepted:
                        print(
                            "ERROR 控制命令被拒绝：{} seq={} error=0x{:02X}".format(
                                pending.label,
                                pending.seq,
                                status.error_code,
                            ),
                            flush=True,
                        )
                        return False
                    saw_ack = True
                if (
                    status.func == pending.expected_event
                    and status.value == pending.expected_value
                    and status.error_code == ERROR_NONE
                ):
                    saw_completion = True
            if saw_ack and saw_completion:
                return True
            if not saw_ack:
                print(
                    "ERROR {} 未收到同 SEQ 的 COMMAND_ACK".format(pending.label),
                    flush=True,
                )
            elif not saw_completion:
                print(
                    "ERROR {} 已 ACK，但未收到业务完成状态".format(pending.label),
                    flush=True,
                )
            return False
        time.sleep(0.02)

    _drain(sender)
    print(
        "ERROR 等待控制命令超时：{} seq={} timeout={:.1f}s".format(
            pending.label,
            pending.seq,
            timeout,
        ),
        flush=True,
    )
    return False


def _send_control_and_wait(
    sender: SerialSender,
    send: Callable[[], bool],
    wait_timeout: float,
) -> bool:
    if not send():
        print("ERROR 发送失败：{}".format(sender.last_error), flush=True)
        return False
    pending = sender.pending_control
    if sender.protocol == "legacy":
        # The only supported legacy controls are appendix target-start/stop
        # frames; that format has no ACK or business completion response.
        print("WARNING legacy 调试帧没有协议(5)的 ACK/业务完成保证", flush=True)
        return True
    return _wait_for_control(sender, pending, wait_timeout)


def _run_formal_safe_distance(sender: SerialSender, args: argparse.Namespace) -> bool:
    """Open this process's measurement window, then stream samples to verdict."""
    if sender.protocol != "unified":
        print("ERROR safe-distance 正式流程只支持 unified 协议", flush=True)
        return False

    print(
        "SAFE_DISTANCE 正式流程：先发送本次 WORKFLOW START_MEASURE，"
        "等待 MEASURE_POSITION_READY 后才发送测距样本。",
        flush=True,
    )
    if not _send_control_and_wait(
        sender,
        sender.send_workflow_start,
        args.wait_timeout,
    ):
        return False
    if not sender.measure_window_open:
        print("ERROR 未建立本次 MEASURE_POSITION_READY 测距窗口", flush=True)
        return False

    timeout = float(args.wait_timeout) if args.wait_timeout > 0 else 10.0
    interval = max(0.10, min(0.20, float(args.safe_distance_interval)))
    deadline = time.monotonic() + timeout
    next_sample = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if sender.measure_window_open and now >= next_sample:
            if not sender.send_safe_distance(args.distance_mm, bool(args.valid)):
                print("ERROR 测距样本发送失败：{}".format(sender.last_error), flush=True)
                return False
            next_sample = now + interval

        statuses, _events = _drain(sender)
        for status in statuses:
            if _is_async_error(status):
                print("ERROR 收到 RA6M5 异步错误：{}".format(status.label), flush=True)
                return False
            if status.func != STATUS_SAFE_DISTANCE:
                continue
            if status.value == 0x01 and status.error_code == ERROR_NONE:
                # Give the following WORKFLOW DISTANCE_SAFE_LATCHED frame a
                # brief chance to arrive so bring-up logs show the full gate.
                _listen(sender, 0.30)
                return True
            print(
                "ERROR RA6M5 判定距离不安全：{} error=0x{:02X}".format(
                    status.label,
                    status.error_code,
                ),
                flush=True,
            )
            return False

        if not sender.measure_window_open:
            print("ERROR 测距窗口在收到 SAFE/UNSAFE 判定前关闭", flush=True)
            return False
        time.sleep(0.02)

    _drain(sender)
    print("ERROR 等待 SAFE_DISTANCE 判定超时（{:.1f}s）".format(timeout), flush=True)
    return False


def _listen(sender: SerialSender, duration: float) -> bool:
    deadline = time.monotonic() + max(0.0, float(duration))
    while True:
        statuses, _events = _drain(sender)
        for status in statuses:
            if _is_async_error(status):
                print("ERROR 收到 RA6M5 异步错误：{}".format(status.label), flush=True)
                return False
        if time.monotonic() >= deadline:
            break
        time.sleep(0.02)
    return True


def _run_command(sender: SerialSender, args: argparse.Namespace) -> bool:
    command = args.command
    if command == "listen":
        return _listen(sender, args.duration)
    if command == "heartbeat":
        if sender.protocol != "unified":
            print("ERROR legacy 协议没有 HEARTBEAT", flush=True)
            return False
        # The background worker already started its heartbeat loop.  Waiting
        # briefly verifies that path without manually injecting a second loop.
        print("HEARTBEAT 由串口后台线程持续发送；CLI 不再手动重复发送。", flush=True)
        return _listen(sender, args.duration if args.duration > 0 else 0.25)
    if command == "safe-distance":
        return _run_formal_safe_distance(sender, args)

    control_senders = {
        "workflow-start": sender.send_workflow_start,
        "workflow-finish": sender.send_workflow_finish,
        "workflow-abort": sender.send_workflow_abort,
        "target-start": lambda: sender.send_target_ctrl(True),
        "target-stop": lambda: sender.send_target_ctrl(False),
        "capture-move": lambda: sender.send_capture_move(args.point_id),
        "capture-home": sender.send_capture_home,
        "capture-select": lambda: sender.send_capture_select(args.point_id),
    }
    if command in CONTROL_COMMANDS:
        return _send_control_and_wait(
            sender,
            control_senders[command],
            args.wait_timeout,
        )

    if command == "vision-error":
        if not sender.send_vision_error(args.dcx, args.dcy, bool(args.valid)):
            print("ERROR 发送失败：{}".format(sender.last_error), flush=True)
            return False
        return True
    raise RuntimeError("unsupported command: {}".format(command))


def main() -> int:
    args = parse_args()
    if args.duration < 0 or args.wait_timeout < 0:
        print("ERROR duration/wait-timeout 不能为负数", flush=True)
        return 2
    if args.protocol == "legacy":
        print("WARNING legacy 仅用于协议附录调试，不属于正式运行路径", flush=True)

    try:
        sender = SerialSender(
            args.serial_port,
            args.baudrate,
            protocol=args.protocol,
            heartbeat_interval=args.heartbeat_interval,
            auto_heartbeat=True,
        )
    except Exception as exc:
        print("ERROR 串口打开失败：{}".format(exc), flush=True)
        return 2

    print(
        "serial connected: {} @ {} protocol={}".format(
            args.serial_port,
            args.baudrate,
            args.protocol,
        ),
        flush=True,
    )
    try:
        ok = _run_command(sender, args)
        if ok and args.command not in ("listen", "heartbeat", "safe-distance"):
            ok = _listen(sender, args.duration)
        if not ok:
            if sender.last_error:
                print("SERIAL", sender.last_error, flush=True)
            return 1
        return 0
    except KeyboardInterrupt:
        print("\n用户中断", flush=True)
        return 130
    finally:
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())

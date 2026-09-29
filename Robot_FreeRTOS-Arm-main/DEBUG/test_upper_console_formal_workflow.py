from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import (  # noqa: E402
    EVENT_ALIGN_DONE,
    EVENT_CAPTURE_HOME,
    EVENT_CAPTURE_POINT,
    EVENT_COMMAND_ACK,
    EVENT_OUTPUT,
    EVENT_READY,
    EVENT_SELECTED_VIEW,
    EVENT_TARGET_CTRL,
    EVENT_WORKFLOW,
    FormalStatusMailbox,
    FormalWorkflowConfig,
    FormalWorkflowProtocolError,
    FormalWorkflowRunner,
    FormalWorkflowStopped,
    JETSON_MSG_CAPTURE_CTRL,
    JETSON_MSG_ERROR,
    JETSON_MSG_STATUS,
    JETSON_MSG_TARGET_CTRL,
    JETSON_MSG_WORKFLOW_CTRL,
    Ra6Status,
    WORKFLOW_FINISH,
    WORKFLOW_MEASURE_READY,
    WORKFLOW_RETURN_HOME_DONE,
    WORKFLOW_SAFE_LATCHED,
    WORKFLOW_START,
    WORKFLOW_START_ACCEPTED,
)


def test_formal_workflow_runner_uses_strict_competition_sequence() -> None:
    controls: list[tuple[int, bytes, str]] = []
    waits: list[tuple[int, int | None, int]] = []
    distances: list[int] = []
    vision_actions: list[object] = []
    stages: list[str] = []
    pauses: list[float] = []
    next_seq = 0

    def send_control(msg_type: int, payload: bytes, label: str, _timeout: float = 5.0) -> int:
        nonlocal next_seq
        next_seq += 1
        controls.append((msg_type, bytes(payload), label))
        return next_seq

    def wait_status(event: int, value: int | None, seq: int, _timeout: float) -> Ra6Status:
        waits.append((event, value, seq))
        return Ra6Status(
            raw=b"",
            func=event,
            value=0 if value is None else value,
            name="TEST",
            seq=seq,
            event=event,
            error=0,
            msg_type=JETSON_MSG_STATUS,
            is_formal=True,
        )

    runner = FormalWorkflowRunner(
        send_control=send_control,
        wait_status=wait_status,
        send_distance=lambda value: distances.append(value),
        start_vision=lambda steps: vision_actions.append(("start", tuple(steps))),
        stop_vision=lambda: vision_actions.append("stop"),
        set_stage=stages.append,
        should_stop=lambda: False,
        pause=pauses.append,
    )
    config = FormalWorkflowConfig(
        view_id=2,
        safe_distance_mm=160,
        vision_steps=((20, -15, 2.0), (8, -6, 2.0), (0, 0, 0.0)),
    )

    runner.run(config)

    assert [(msg_type, payload) for msg_type, payload, _ in controls] == [
        (JETSON_MSG_WORKFLOW_CTRL, bytes([WORKFLOW_START])),
        (JETSON_MSG_CAPTURE_CTRL, b"\x02\x00"),
        (JETSON_MSG_CAPTURE_CTRL, b"\x01\x01"),
        (JETSON_MSG_CAPTURE_CTRL, b"\x01\x02"),
        (JETSON_MSG_CAPTURE_CTRL, b"\x01\x03"),
        (JETSON_MSG_CAPTURE_CTRL, b"\x02\x00"),
        (JETSON_MSG_CAPTURE_CTRL, b"\x03\x02"),
        (JETSON_MSG_TARGET_CTRL, b"\x01"),
        (JETSON_MSG_WORKFLOW_CTRL, bytes([WORKFLOW_FINISH])),
    ]
    assert distances == [160, 160, 160]
    assert pauses.count(0.2) == 3
    assert pauses.count(1.0) == 3
    assert (EVENT_WORKFLOW, WORKFLOW_START_ACCEPTED, 1) in waits
    assert (EVENT_WORKFLOW, WORKFLOW_MEASURE_READY, 1) in waits
    assert (EVENT_WORKFLOW, WORKFLOW_SAFE_LATCHED, 1) in waits
    assert (EVENT_CAPTURE_HOME, 1, 2) in waits
    assert (EVENT_CAPTURE_POINT, 1, 3) in waits
    assert (EVENT_CAPTURE_POINT, 2, 4) in waits
    assert (EVENT_CAPTURE_POINT, 3, 5) in waits
    assert (EVENT_SELECTED_VIEW, 2, 7) in waits
    assert (EVENT_TARGET_CTRL, 1, 8) in waits
    assert (EVENT_READY, 1, 8) in waits
    assert (EVENT_ALIGN_DONE, 1, 8) in waits
    assert (EVENT_OUTPUT, 1, 8) in waits
    assert (EVENT_OUTPUT, 0, 8) in waits
    assert (EVENT_WORKFLOW, WORKFLOW_RETURN_HOME_DONE, 9) in waits
    assert vision_actions[0][0] == "start"
    assert vision_actions[-1] == "stop"
    assert any("请按住 P000" in stage for stage in stages)
    assert any("请松开 P000" in stage for stage in stages)
    assert stages[-1] == "流程完成，机械臂已回 HOME"


def test_formal_status_mailbox_matches_seq_and_surfaces_async_error() -> None:
    mailbox = FormalStatusMailbox()
    stop_event = threading.Event()
    mailbox.publish(
        Ra6Status(
            raw=b"",
            func=EVENT_COMMAND_ACK,
            value=1,
            name="COMMAND_ACK",
            seq=0x32,
            event=EVENT_COMMAND_ACK,
            error=0,
            msg_type=JETSON_MSG_STATUS,
            is_formal=True,
        )
    )
    status = mailbox.wait(EVENT_COMMAND_ACK, None, 0x32, 0.1, stop_event)
    assert status.seq == 0x32

    mailbox.publish(
        Ra6Status(
            raw=b"",
            func=0xFE,
            value=0x08,
            name="SAFETY_ERROR",
            seq=0,
            event=0xFE,
            error=0x08,
            msg_type=JETSON_MSG_ERROR,
            is_formal=True,
        )
    )
    try:
        mailbox.wait(EVENT_READY, 1, 0x33, 0.1, stop_event)
    except FormalWorkflowProtocolError as exc:
        assert "安全保护触发" in str(exc)
    else:
        raise AssertionError("async formal error must stop the workflow")


def test_formal_workflow_runner_honors_stop_request() -> None:
    runner = FormalWorkflowRunner(
        send_control=lambda *_args: 1,
        wait_status=lambda *_args: None,
        send_distance=lambda _value: None,
        start_vision=lambda _steps: None,
        stop_vision=lambda: None,
        set_stage=lambda _stage: None,
        should_stop=lambda: True,
        pause=lambda _seconds: None,
    )
    try:
        runner.run(FormalWorkflowConfig(2, 160, ((0, 0, 0.0),)))
    except FormalWorkflowStopped:
        pass
    else:
        raise AssertionError("stop request must interrupt the workflow")


if __name__ == "__main__":
    test_formal_workflow_runner_uses_strict_competition_sequence()
    test_formal_status_mailbox_matches_seq_and_surfaces_async_error()
    test_formal_workflow_runner_honors_stop_request()
    print("upper console formal workflow checks passed")

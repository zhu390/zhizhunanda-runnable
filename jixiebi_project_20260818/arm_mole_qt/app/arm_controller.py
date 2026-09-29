from dataclasses import dataclass
from typing import List, Optional, Tuple


PREVIEW_SEND_REPEAT = 1


@dataclass(frozen=True)
class ArmTarget:
    pixel: Tuple[int, int]
    frame_size: Optional[Tuple[int, int]] = None
    robot: Optional[Tuple[float, float, float]] = None


class ArmController:
    """Qt-side wrapper for the Jetson <-> RA6M5 vision serial protocol."""

    def __init__(
        self,
        enabled: bool = False,
        port: str = "",
        baudrate: int = 115200,
        timeout: float = 0.02,
        serial_protocol: str = "unified",
        laser_offset_mm: float = 27.81,
        mm_per_pixel_x: float = 0.0,
        laser_offset_x_px: Optional[int] = None,
        laser_offset_y_px: Optional[int] = None,
        heartbeat_interval: float = 0.15,
        vision_interval: float = 0.15,
        vision_max_age: float = 0.5,
        continuous_safe_distance: bool = False,
    ) -> None:
        self.enabled = enabled
        self.port = port
        self.baudrate = baudrate
        self.serial_protocol = serial_protocol
        self.laser_offset_mm = float(laser_offset_mm)
        self.mm_per_pixel_x = float(mm_per_pixel_x)
        self.laser_offset_x_px = laser_offset_x_px
        self.laser_offset_y_px = (
            None if laser_offset_y_px is None else int(laser_offset_y_px)
        )
        self.continuous_safe_distance = bool(continuous_safe_distance)
        self._sender = None
        self._open_error = ""

        if not enabled:
            return

        try:
            from jetson_vision.serial_sender import SerialSender

            self._sender = SerialSender(
                port,
                baudrate,
                timeout=timeout,
                protocol=serial_protocol,
                heartbeat_interval=heartbeat_interval,
                vision_interval=vision_interval,
                vision_max_age=vision_max_age,
                auto_vision=True,
            )
        except Exception as exc:
            self.enabled = False
            self._open_error = str(exc)

    @property
    def connected(self) -> bool:
        return bool(self.enabled and self._sender is not None and self._sender.is_open)

    def connection_text(self) -> str:
        if self.connected:
            return f"RA6M5 串口已连接：{self.port} @ {self.baudrate} / {self.serial_protocol}"
        if self._open_error:
            return f"RA6M5 串口打开失败：{self._open_error}"
        return "RA6M5 串口未配置"

    def laser_offset_px(self) -> Optional[int]:
        """Return the explicitly calibrated X pixel offset, if available.

        The physical camera/laser spacing cannot safely be converted with one
        mm-per-pixel value at the 15--20 mm working distance: perspective and
        the laser mounting angle determine the actual image location.  Formal
        VISION_ERROR samples therefore require a measured pixel offset.
        """

        if self.laser_offset_x_px is None:
            return None
        return int(self.laser_offset_x_px)

    @property
    def aim_reference_configured(self) -> bool:
        return (
            self.laser_offset_x_px is not None
            and self.laser_offset_y_px is not None
        )

    def aim_reference_pixel(
        self,
        frame_size: Tuple[int, int],
    ) -> Optional[Tuple[int, int]]:
        """Return the calibrated laser aim pixel for this frame size."""

        width, height = (int(frame_size[0]), int(frame_size[1]))
        if width <= 0 or height <= 0:
            raise ValueError("画面尺寸必须为正数")
        offset_x_px = self.laser_offset_px()
        if not self.aim_reference_configured or offset_x_px is None:
            return None
        return (
            width // 2 + offset_x_px,
            height // 2 + int(self.laser_offset_y_px),
        )

    def aim_corrected_error(self, target: ArmTarget) -> Tuple[int, int]:
        """Compute target error relative to the measured laser aim pixel."""

        return self._vision_error(target, laser_corrected=True)

    def laser_offset_text(self) -> str:
        offset_px = self.laser_offset_px()
        if not self.aim_reference_configured or offset_px is None:
            return "未标定（需显式配置激光瞄准 X/Y 像素偏移）"
        return f"已标定：X {offset_px} px / Y {self.laser_offset_y_px} px"

    def _vision_error(self, target: ArmTarget, laser_corrected: bool) -> Tuple[int, int]:
        if target.frame_size is None:
            raise ValueError("缺少画面尺寸，无法计算 dcx/dcy")

        from jetson_vision.vision_protocol import pixel_error_from_target

        x, y = target.pixel
        width, height = target.frame_size
        if not laser_corrected:
            return pixel_error_from_target(x, y, width, height)

        aim_pixel = self.aim_reference_pixel((width, height))
        if aim_pixel is None:
            raise ValueError(
                "激光瞄准参考像素未标定：必须显式配置 "
                "--laser-offset-x-px 和 --laser-offset-y-px"
            )
        return int(x) - aim_pixel[0], int(y) - aim_pixel[1]

    @property
    def control_busy(self) -> bool:
        return bool(self.connected and self._sender.control_busy)

    @property
    def abort_pending(self) -> bool:
        if not self.connected:
            return False
        pending = self._sender.pending_control
        return bool(pending is not None and pending.label.startswith("WORKFLOW_ABORT_HOLD"))

    @property
    def background_vision_enabled(self) -> bool:
        return bool(
            self.connected
            and self.serial_protocol == "unified"
            and self._sender.background_vision_enabled
        )

    @property
    def last_safe_distance_seq(self) -> Optional[int]:
        if not self.connected:
            return None
        return self._sender.last_safe_distance_seq

    def service_transport(self) -> List[str]:
        if self._sender is None:
            return []
        # A terminal I/O event may already be queued when pyserial marks the
        # port closed.  Always expose queued events to Qt so it can fail closed;
        # only retry controls while the physical handle is still open.
        if self.connected:
            self._sender.service_control()
        return self._sender.pop_transport_events()

    def preview_move(self, target: ArmTarget) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        try:
            dcx, dcy = self.aim_corrected_error(target)
        except ValueError as exc:
            return False, str(exc)

        for index in range(PREVIEW_SEND_REPEAT):
            ok = self._sender.send_vision_error(dcx, dcy)
            if not ok:
                return False, f"第 {index + 1}/{PREVIEW_SEND_REPEAT} 次视觉误差帧发送失败：{self._sender.last_error}"

        return True, f"已发送激光参考视觉误差：dcx={dcx}, dcy={dcy}"

    def send_target_sample(self, target: Optional[ArmTarget]) -> Tuple[bool, str]:
        """Send one formal VISION_ERROR sample, including explicit no-target."""
        if not self.connected:
            return False, self.connection_text()
        if target is None or not self.aim_reference_configured:
            ok = self._sender.send_no_target()
            if not ok:
                return False, self._sender.last_error
            if target is not None:
                return True, "激光瞄准参考像素未标定，已 fail-closed 发送 VISION_ERROR valid=0"
            return True, "已发送 VISION_ERROR valid=0"
        try:
            dcx, dcy = self.aim_corrected_error(target)
        except ValueError as exc:
            return False, str(exc)
        ok = self._sender.send_vision_error(dcx, dcy, valid=True)
        if ok:
            return True, f"已发送 VISION_ERROR dcx={dcx} dcy={dcy} valid=1"
        return False, self._sender.last_error

    def publish_target_sample(
        self,
        target: Optional[ArmTarget],
        sample_time: float,
    ) -> Tuple[bool, str]:
        """Publish a latest-only sample for the independent serial scheduler."""

        if not self.connected:
            return False, self.connection_text()
        if not self.background_vision_enabled:
            return False, "后台 VISION_ERROR 调度未启用"
        if target is None or not self.aim_reference_configured:
            dcx, dcy, valid = 0, 0, False
        else:
            try:
                dcx, dcy = self.aim_corrected_error(target)
            except ValueError as exc:
                return False, str(exc)
            valid = True
        ok = self._sender.publish_vision_sample(
            dcx,
            dcy,
            valid,
            sample_time,
        )
        if ok:
            return True, (
                "已发布最新视觉结果 valid=1"
                if valid
                else "已发布无目标结果 valid=0"
            )
        return False, self._sender.last_error

    def send_heartbeat(self) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_heartbeat()
        if ok:
            return True, "已发送 HEARTBEAT"
        return False, self._sender.last_error

    def send_safe_distance(self, distance_mm: int, valid: bool) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_safe_distance(distance_mm, valid)
        if ok:
            return True, f"已发送 SAFE_DISTANCE distance={distance_mm} valid={1 if valid else 0}"
        return False, self._sender.last_error

    def capture_move(self, point_id: int) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_capture_move(point_id)
        if ok:
            return True, f"已请求移动到第 {point_id} 个拍摄点"
        return False, self._sender.last_error

    def start_workflow(self) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_workflow_start(
            continuous=self.continuous_safe_distance
        )
        if ok:
            action = (
                "START_MEASURE_CONTINUOUS"
                if self.continuous_safe_distance
                else "START_MEASURE"
            )
            return True, "已发送 WORKFLOW_CTRL {}，等待公共测距位".format(action)
        return False, self._sender.last_error

    def finish_workflow(self) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_workflow_finish()
        if ok:
            return True, "已发送 WORKFLOW_CTRL FINISH_RETURN_HOME"
        return False, self._sender.last_error

    def abort_workflow(self) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_workflow_abort()
        if ok:
            return True, "已发送 WORKFLOW_CTRL ABORT_HOLD，机械臂应关输出并原地保持"
        return False, self._sender.last_error

    def capture_home(self) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_capture_home()
        if ok:
            return True, "已请求 CAPTURE HOME，等待 HOME_READY"
        return False, self._sender.last_error

    def capture_select(self, point_id: int) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_capture_select(point_id)
        if ok:
            return True, f"已选择第 {point_id} 个视图，等待 RA6M5 进入对应定靶预开始点"
        return False, self._sender.last_error

    def fire_laser(self, target: ArmTarget) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        if self.serial_protocol == "unified":
            return False, "正式协议禁止 Jetson 直接开 P801；请使用 TARGET_CTRL START 并等待 ALIGN_DONE/P000"
        try:
            dcx, dcy = self.aim_corrected_error(target)
        except ValueError as exc:
            return False, str(exc)

        if not self._sender.send_vision_error(dcx, dcy):
            return False, f"点痣前视觉误差帧发送失败：{self._sender.last_error}"
        if not self._sender.send_laser_output(True):
            return False, f"激光输出请求发送失败：{self._sender.last_error}"
        return True, f"已请求点痣：dcx={dcx}, dcy={dcy}，laser_offset={self.laser_offset_text()}"

    def start_target_state_machine(self) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        if self.serial_protocol == "unified" and not self.aim_reference_configured:
            return False, (
                "激光瞄准参考像素未标定：必须显式配置 "
                "--laser-offset-x-px 和 --laser-offset-y-px；"
                "TARGET_CTRL START 未发送"
            )
        ok = self._sender.send_target_ctrl(True)
        if ok:
            if self.serial_protocol == "unified":
                return True, "已发送 TARGET_CTRL enable=1 新协议帧"
            return True, "已发送定靶状态机启动帧：AA 01 01 BB"
        return False, f"定靶状态机启动帧发送失败：{self._sender.last_error}"

    def stop_target_state_machine(self) -> Tuple[bool, str]:
        if not self.connected:
            return False, self.connection_text()
        ok = self._sender.send_target_ctrl(False)
        if ok:
            if self.serial_protocol == "unified":
                return True, "已发送 TARGET_CTRL enable=0 新协议帧"
            return True, "已发送定靶状态机关闭帧：AA 01 00 BB"
        return False, f"定靶状态机关闭帧发送失败：{self._sender.last_error}"

    def poll_statuses(self) -> List[object]:
        if not self.connected:
            return []
        return self._sender.poll_statuses()

    def emergency_stop(self) -> Tuple[bool, str]:
        if self.serial_protocol == "unified":
            return self.abort_workflow()
        return False, "旧调试协议没有软件安全停止，请使用独立硬件急停"

    def close(self) -> None:
        if self._sender is not None:
            self._sender.close()

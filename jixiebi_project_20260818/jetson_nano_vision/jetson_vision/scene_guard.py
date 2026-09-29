"""Patient/scene stability guard used from the first valid protocol-v5 SAFE sample.

The guard deliberately depends only on normalized face boxes and monotonic
timestamps.  Camera/UI code can therefore test it without PyQt, OpenCV or a
model runtime.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class SceneGuardDecision:
    abort: bool
    reason: str = ""
    consumed: bool = False


class FaceSceneGuard:
    """Detect a material patient/scene change relative to a latched face box."""

    def __init__(
        self,
        center_shift_ratio: float = 0.08,
        scale_ratio: float = 1.25,
        missing_timeout_s: float = 0.8,
    ) -> None:
        self.center_shift_ratio = max(0.01, float(center_shift_ratio))
        self.scale_ratio = max(1.01, float(scale_ratio))
        self.missing_timeout_s = max(0.1, float(missing_timeout_s))
        self.reset()

    @property
    def active(self) -> bool:
        return self._baseline is not None

    @property
    def last_sample_time(self) -> float:
        return self._last_sample_time

    def reset(self) -> None:
        self._baseline: Optional[Tuple[float, float, float, float]] = None
        self._last_sample_time = 0.0
        self._missing_since = 0.0

    def latch(
        self,
        face_box: Sequence[float],
        frame_size: Tuple[int, int],
        sample_time: float,
    ) -> bool:
        normalized = self._normalize(face_box, frame_size)
        timestamp = float(sample_time)
        if normalized is None or not math.isfinite(timestamp) or timestamp <= 0.0:
            self.reset()
            return False
        self._baseline = normalized
        self._last_sample_time = timestamp
        self._missing_since = 0.0
        return True

    def update(
        self,
        face_box: Optional[Sequence[float]],
        frame_size: Tuple[int, int],
        sample_time: float,
    ) -> SceneGuardDecision:
        if self._baseline is None:
            return SceneGuardDecision(False, "场景守卫尚未锁存")
        timestamp = float(sample_time)
        if (
            not math.isfinite(timestamp)
            or timestamp <= 0.0
            or timestamp <= self._last_sample_time
        ):
            return SceneGuardDecision(False, "忽略重复或倒退的场景样本")
        self._last_sample_time = timestamp

        normalized = self._normalize(face_box, frame_size) if face_box is not None else None
        if normalized is None:
            if self._missing_since <= 0.0:
                self._missing_since = timestamp
            if timestamp - self._missing_since >= self.missing_timeout_s:
                return SceneGuardDecision(True, "人脸持续丢失，患者或场景可能已改变", True)
            return SceneGuardDecision(False, "暂时未检测到人脸", True)

        self._missing_since = 0.0
        base_cx, base_cy, base_w, base_h = self._baseline
        cx, cy, width, height = normalized
        center_shift = math.hypot(cx - base_cx, cy - base_cy)
        if center_shift > self.center_shift_ratio:
            return SceneGuardDecision(
                True,
                "人脸中心位移 {:.3f} 超过阈值 {:.3f}".format(
                    center_shift,
                    self.center_shift_ratio,
                ),
                True,
            )

        width_ratio = self._symmetric_ratio(width, base_w)
        height_ratio = self._symmetric_ratio(height, base_h)
        observed_ratio = max(width_ratio, height_ratio)
        if observed_ratio > self.scale_ratio:
            return SceneGuardDecision(
                True,
                "人脸尺度变化 {:.3f} 超过阈值 {:.3f}".format(
                    observed_ratio,
                    self.scale_ratio,
                ),
                True,
            )
        return SceneGuardDecision(False, "患者位置稳定", True)

    def poll(self, now: float) -> SceneGuardDecision:
        """Detect a stalled RGB/YOLO stream even when no new callback arrives."""

        if self._baseline is None:
            return SceneGuardDecision(False, "场景守卫尚未锁存")
        timestamp = float(now)
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            return SceneGuardDecision(False, "无效监测时间")
        if self._missing_since > 0.0 and timestamp - self._missing_since >= self.missing_timeout_s:
            return SceneGuardDecision(True, "人脸持续丢失，患者或场景可能已改变")
        if timestamp - self._last_sample_time >= self.missing_timeout_s:
            return SceneGuardDecision(True, "场景监测画面或 YOLO 结果已中断")
        return SceneGuardDecision(False, "场景监测正常")

    @staticmethod
    def _normalize(
        face_box: Optional[Sequence[float]],
        frame_size: Tuple[int, int],
    ) -> Optional[Tuple[float, float, float, float]]:
        if face_box is None or len(face_box) != 4:
            return None
        frame_width, frame_height = int(frame_size[0]), int(frame_size[1])
        if frame_width <= 0 or frame_height <= 0:
            return None
        try:
            x1, y1, x2, y2 = (float(value) for value in face_box)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
            return None
        width = (x2 - x1) / frame_width
        height = (y2 - y1) / frame_height
        if width <= 0.0 or height <= 0.0:
            return None
        return (
            ((x1 + x2) * 0.5) / frame_width,
            ((y1 + y2) * 0.5) / frame_height,
            width,
            height,
        )

    @staticmethod
    def _symmetric_ratio(value: float, baseline: float) -> float:
        if value <= 0.0 or baseline <= 0.0:
            return float("inf")
        return max(value / baseline, baseline / value)

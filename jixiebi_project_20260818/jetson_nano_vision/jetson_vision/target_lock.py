"""Session-local identity lock for one automatically selected treatment target."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class TargetLockDecision:
    index: Optional[int]
    locked: bool
    consumed: bool
    reason: str = ""


class TargetIdentityLock:
    """Keep following one candidate even if detector result ordering changes."""

    def __init__(
        self,
        max_step_ratio: float = 0.10,
        max_radius_ratio: float = 1.5,
        max_step_radius_factor: float = 1.0,
        reacquire_radius_factor: float = 0.5,
        ambiguity_radius_factor: float = 0.75,
        competitor_exclusion_radius_factor: float = 1.25,
        stable_frames: int = 2,
        track_id_rebind_timeout_s: float = 1.0,
        track_id_rebind_radius_factor: float = 2.0,
    ) -> None:
        self.max_step_ratio = max(0.005, float(max_step_ratio))
        self.max_radius_ratio = max(1.05, float(max_radius_ratio))
        self.max_step_radius_factor = max(0.25, float(max_step_radius_factor))
        self.reacquire_radius_factor = max(
            0.1,
            min(1.0, float(reacquire_radius_factor)),
        )
        self.ambiguity_radius_factor = max(0.0, float(ambiguity_radius_factor))
        self.competitor_exclusion_radius_factor = max(
            0.5,
            float(competitor_exclusion_radius_factor),
        )
        self.stable_frames = max(1, int(stable_frames))
        self.track_id_rebind_timeout_s = max(
            0.1,
            float(track_id_rebind_timeout_s),
        )
        self.track_id_rebind_radius_factor = max(
            0.5,
            float(track_id_rebind_radius_factor),
        )
        self.reset()

    @property
    def active(self) -> bool:
        return self._anchor is not None

    @property
    def lost(self) -> bool:
        return bool(self._lost)

    @property
    def track_id(self) -> int:
        return int(self._track_id)

    @property
    def pending_count(self) -> int:
        return int(self._pending_count)

    @property
    def anchor(self) -> Optional[Tuple[float, float, float]]:
        """Return the immutable normalized anchor snapshot for diagnostics."""

        if self._anchor is None:
            return None
        return tuple(self._anchor)

    @property
    def last_locked_sample_time(self) -> float:
        return float(self._last_locked_sample_time)

    def reset(self) -> None:
        self._anchor: Optional[Tuple[float, float, float]] = None
        self._pending: Optional[Tuple[float, float, float]] = None
        self._pending_track_id = 0
        self._pending_count = 0
        self._lost = False
        self._track_id = 0
        self._competitors = []
        self._last_sample_time = 0.0
        self._last_locked_sample_time = 0.0

    @staticmethod
    def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _radius_ratio(a: float, b: float) -> float:
        return max(a / b, b / a)

    def _step_gate(self, radius: float, *, reacquiring: bool = False) -> float:
        gate = min(self.max_step_ratio, radius * self.max_step_radius_factor)
        if reacquiring:
            gate *= self.reacquire_radius_factor
        return gate

    def _clear_pending(self) -> None:
        self._pending = None
        self._pending_track_id = 0
        self._pending_count = 0

    def _track_id_rebind_gate(self, anchor_radius: float, radius: float) -> float:
        """Return the deliberately small spatial gate for a changed track id."""

        return min(
            self.max_step_ratio,
            max(anchor_radius, radius) * self.track_id_rebind_radius_factor,
        )

    def _confirm_track_id_rebind(
        self,
        normalized,
        timestamp: float,
    ) -> TargetLockDecision:
        """Conservatively bind a new upstream id to the existing target.

        Upstream tracking ids may be recreated after a few blurred/missed
        frames.  Rebinding is allowed only for a short interval, with exactly
        one already-stable candidate that remains close to the immutable old
        anchor for multiple consecutive samples.  Until confirmation finishes
        the caller receives no target and therefore continues to send
        ``valid=0``.
        """

        if len(normalized) != 1:
            self._clear_pending()
            return TargetLockDecision(
                None,
                True,
                True,
                "锁定跟踪ID丢失且候选不唯一，禁止重绑定",
            )

        index, x, y, radius, track_id = normalized[0]
        if track_id <= 0 or track_id == self._track_id:
            self._clear_pending()
            return TargetLockDecision(
                None,
                True,
                True,
                "锁定跟踪ID暂时丢失，候选ID无效，禁止重绑定",
            )

        lost_age = timestamp - self._last_locked_sample_time
        if (
            self._last_locked_sample_time <= 0.0
            or lost_age > self.track_id_rebind_timeout_s
        ):
            self._clear_pending()
            return TargetLockDecision(
                None,
                True,
                True,
                "锁定跟踪ID丢失超时，禁止重绑定",
            )

        anchor_radius = self._anchor[2]
        candidate = (x, y, radius)
        if self._distance(candidate, self._anchor) > self._track_id_rebind_gate(
            anchor_radius,
            radius,
        ):
            self._clear_pending()
            return TargetLockDecision(
                None,
                True,
                True,
                "新跟踪ID候选远离旧锚点，禁止重绑定",
            )
        if self._radius_ratio(radius, anchor_radius) > self.max_radius_ratio:
            self._clear_pending()
            return TargetLockDecision(
                None,
                True,
                True,
                "新跟踪ID候选与旧锚点尺度不一致，禁止重绑定",
            )
        if self._matches_known_competitor(candidate):
            self._clear_pending()
            return TargetLockDecision(
                None,
                True,
                True,
                "新跟踪ID候选匹配本会话已排除的其他痣，禁止重绑定",
            )

        if self._pending is None or self._pending_track_id != track_id:
            self._pending = candidate
            self._pending_track_id = track_id
            self._pending_count = 1
        else:
            pending_gate = self._track_id_rebind_gate(
                self._pending[2],
                radius,
            )
            if (
                self._distance(candidate, self._pending) <= pending_gate
                and self._radius_ratio(radius, self._pending[2])
                <= self.max_radius_ratio
            ):
                self._pending = candidate
                self._pending_count += 1
            else:
                self._pending = candidate
                self._pending_count = 1

        required = max(2, self.stable_frames)
        if self._pending_count < required:
            return TargetLockDecision(
                None,
                True,
                True,
                "新跟踪ID重绑定确认 {}/{}（确认前禁止切换）".format(
                    self._pending_count,
                    required,
                ),
            )

        previous_track_id = self._track_id
        self._anchor = self._pending
        self._track_id = int(self._pending_track_id)
        self._lost = False
        self._last_locked_sample_time = timestamp
        self._clear_pending()
        return TargetLockDecision(
            index,
            True,
            True,
            "新跟踪ID保守重绑定完成 {}->{}".format(
                previous_track_id,
                self._track_id,
            ),
        )

    def _matches_known_competitor(self, candidate: Tuple[float, float, float]) -> bool:
        for competitor in self._competitors:
            exclusion = (
                max(candidate[2], competitor[2])
                * self.competitor_exclusion_radius_factor
            )
            if self._distance(candidate, competitor) <= exclusion:
                return True
        return False

    def _remember_competitors(self, normalized, selected_index: int) -> None:
        """Remember other visible moles so they can never inherit this lock."""

        for index, x, y, radius, _track_id in normalized:
            if index == selected_index:
                continue
            candidate = (x, y, radius)
            merged = False
            for position, previous in enumerate(self._competitors):
                if self._distance(candidate, previous) <= max(radius, previous[2]):
                    self._competitors[position] = candidate
                    merged = True
                    break
            if not merged:
                self._competitors.append(candidate)
        if len(self._competitors) > 64:
            self._competitors = self._competitors[-64:]

    def _confirm_candidate(
        self,
        candidate: Tuple[float, float, float],
        index: int,
        *,
        reacquiring: bool,
        track_id: int = 0,
    ) -> TargetLockDecision:
        if self.stable_frames <= 1:
            self._anchor = candidate
            self._lost = False
            self._track_id = int(track_id)
            self._clear_pending()
            return TargetLockDecision(index, True, True, "目标稳定确认完成")

        if self._pending is None or self._pending_track_id != int(track_id):
            self._pending = candidate
            self._pending_track_id = int(track_id)
            self._pending_count = 1
        else:
            pending_gate = (
                self.max_step_ratio
                if int(track_id) > 0
                else self._step_gate(
                    self._pending[2],
                    reacquiring=reacquiring,
                )
            )
            if (
                self._distance(candidate, self._pending) <= pending_gate
                and self._radius_ratio(candidate[2], self._pending[2]) <= self.max_radius_ratio
            ):
                self._pending = candidate
                self._pending_count += 1
            else:
                self._pending = candidate
                self._pending_count = 1

        if self._pending_count < self.stable_frames:
            stage = "重捕" if reacquiring else "初始"
            return TargetLockDecision(
                None,
                self.active,
                True,
                f"{stage}目标稳定确认 {self._pending_count}/{self.stable_frames}",
            )
        self._anchor = self._pending
        self._lost = False
        self._track_id = int(self._pending_track_id)
        self._clear_pending()
        return TargetLockDecision(index, True, True, "目标稳定确认完成")

    def update(
        self,
        candidates: Sequence[Tuple[float, float, float]],
        frame_size: Tuple[int, int],
        sample_time: float,
        candidate_ids: Optional[Sequence[int]] = None,
    ) -> TargetLockDecision:
        timestamp = float(sample_time)
        width, height = int(frame_size[0]), int(frame_size[1])
        if (
            not math.isfinite(timestamp)
            or timestamp <= 0.0
            or timestamp <= self._last_sample_time
        ):
            return TargetLockDecision(None, self.active, False, "忽略重复或倒退的目标样本")
        self._last_sample_time = timestamp
        diagonal = math.hypot(width, height)
        if width <= 0 or height <= 0 or diagonal <= 0.0:
            return TargetLockDecision(None, self.active, True, "目标画面尺寸无效")

        normalized = []
        for index, candidate in enumerate(candidates):
            if len(candidate) != 3:
                continue
            try:
                x, y, radius = (float(value) for value in candidate)
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (x, y, radius)) or radius <= 0.0:
                continue
            # All spatial quantities share the same diagonal normalization so
            # the continuation gate can be tied to the mole radius in pixels.
            track_id = 0
            if candidate_ids is not None and index < len(candidate_ids):
                try:
                    parsed_track_id = int(candidate_ids[index])
                except (TypeError, ValueError):
                    parsed_track_id = 0
                if parsed_track_id > 0:
                    track_id = parsed_track_id
            normalized.append(
                (index, x / diagonal, y / diagonal, radius / diagonal, track_id)
            )

        if not normalized:
            if self._anchor is not None:
                self._lost = True
            self._clear_pending()
            return TargetLockDecision(None, self.active, True, "锁定目标暂时丢失")
        if self._anchor is None:
            if self._pending is None:
                index, x, y, radius, track_id = normalized[0]
            else:
                index, x, y, radius, track_id = min(
                    normalized,
                    key=lambda item: math.hypot(
                        item[1] - self._pending[0],
                        item[2] - self._pending[1],
                    ),
                )
            decision = self._confirm_candidate(
                (x, y, radius),
                index,
                reacquiring=False,
                track_id=track_id,
            )
            if decision.index is not None:
                self._last_locked_sample_time = timestamp
                self._remember_competitors(normalized, decision.index)
            return decision

        # The facial candidate tracker has already performed frame-to-frame
        # one-to-one association and assigns a session-local track id.  Once
        # such an id is locked, only that exact id may continue the session.
        # This avoids the old failure mode where one skipped/jittery frame left
        # the anchor permanently stranded behind a 0.5 x mole-radius re-capture
        # gate.  A missing id still fails closed; a different mole can never
        # inherit the lock.
        if self._track_id > 0:
            tracked = [item for item in normalized if item[4] == self._track_id]
            if len(tracked) != 1:
                self._lost = True
                if len(tracked) > 1:
                    self._clear_pending()
                    reason = "锁定跟踪ID出现重复候选，禁止自动选择"
                    return TargetLockDecision(None, True, True, reason)
                return self._confirm_track_id_rebind(normalized, timestamp)

            index, x, y, radius, _track_id = tracked[0]
            anchor_x, anchor_y, anchor_radius = self._anchor
            step = math.hypot(x - anchor_x, y - anchor_y)
            radius_ratio = self._radius_ratio(radius, anchor_radius)
            if step > self.max_step_ratio:
                self._lost = True
                self._clear_pending()
                return TargetLockDecision(
                    None,
                    True,
                    True,
                    "锁定跟踪ID目标超出归一化硬位移上限",
                )
            if radius_ratio > self.max_radius_ratio:
                self._lost = True
                self._clear_pending()
                return TargetLockDecision(
                    None,
                    True,
                    True,
                    "锁定跟踪ID目标尺度不一致",
                )

            # A second stable candidate almost coincident with the selected
            # one remains ambiguous even when ids are available.
            for other in normalized:
                if other[0] == index:
                    continue
                separation = math.hypot(other[1] - x, other[2] - y)
                if separation <= max(radius, other[3]) * self.ambiguity_radius_factor:
                    self._lost = True
                    self._clear_pending()
                    return TargetLockDecision(
                        None,
                        True,
                        True,
                        "锁定点附近存在歧义候选，禁止自动换痣",
                    )

            candidate = (x, y, radius)
            if self._matches_known_competitor(candidate):
                self._lost = True
                self._clear_pending()
                return TargetLockDecision(
                    None,
                    True,
                    True,
                    "跟踪ID候选匹配本会话已排除的其他痣",
                )
            self._anchor = candidate
            self._lost = False
            self._last_locked_sample_time = timestamp
            self._clear_pending()
            self._remember_competitors(normalized, index)
            return TargetLockDecision(index, True, True, "跟踪ID连续，目标续跟成功")

        anchor_x, anchor_y, anchor_radius = self._anchor
        ranked = sorted(
            normalized,
            key=lambda item: math.hypot(item[1] - anchor_x, item[2] - anchor_y),
        )
        index, x, y, radius, _track_id = ranked[0]
        step = math.hypot(x - anchor_x, y - anchor_y)
        radius_ratio = self._radius_ratio(radius, anchor_radius)
        step_gate = self._step_gate(anchor_radius, reacquiring=self._lost)
        if step > step_gate:
            self._lost = True
            self._clear_pending()
            return TargetLockDecision(None, True, True, "原锁定目标超出最近邻续跟范围")
        if radius_ratio > self.max_radius_ratio:
            self._lost = True
            self._clear_pending()
            return TargetLockDecision(None, True, True, "最近候选与原锁定目标尺度不一致")

        if len(ranked) > 1:
            second = ranked[1]
            second_step = math.hypot(second[1] - anchor_x, second[2] - anchor_y)
            if (
                second_step <= step_gate
                and second_step - step <= anchor_radius * self.ambiguity_radius_factor
            ):
                self._lost = True
                self._clear_pending()
                return TargetLockDecision(None, True, True, "锁定点附近存在歧义候选，禁止自动换痣")

        candidate = (x, y, radius)
        if self._matches_known_competitor(candidate):
            self._lost = True
            self._clear_pending()
            return TargetLockDecision(None, True, True, "候选匹配本会话已排除的其他痣")
        if self._lost:
            decision = self._confirm_candidate(
                candidate,
                index,
                reacquiring=True,
                track_id=0,
            )
            if decision.index is not None:
                self._last_locked_sample_time = timestamp
                self._remember_competitors(normalized, decision.index)
            return decision
        self._anchor = candidate
        self._last_locked_sample_time = timestamp
        self._clear_pending()
        self._remember_competitors(normalized, index)
        return TargetLockDecision(index, True, True, "原锁定目标续跟成功")

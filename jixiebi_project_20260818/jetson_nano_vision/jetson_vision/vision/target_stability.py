from collections import deque
from typing import Iterable, List, Optional, Tuple

import numpy as np

from .._compat import dataclass
from .mole_detect import DetectedMole


def stable_center(
    history: Iterable[Optional[Tuple[int, int]]],
    required: int,
    radius: float,
) -> Optional[Tuple[int, int]]:
    """计算多帧稳定中心；有效样本不足或离散范围过大时返回 None。"""
    centers = [center for center in history if center is not None]
    if len(centers) < max(1, required):
        return None
    points = np.asarray(centers, dtype=np.float32)
    mean = points.mean(axis=0)
    distances = np.linalg.norm(points - mean, axis=1)
    if float(distances.max()) > radius:
        return None
    return int(round(float(mean[0]))), int(round(float(mean[1])))


@dataclass
class _CandidateTrack:
    track_id: int
    samples: object
    consecutive_hits: int = 1
    first_seen_time: Optional[float] = None
    missed: int = 0
    stable_since: Optional[float] = None
    last_sample_time: Optional[float] = None
    # Initial acquisition must be stationary, but a confirmed treatment
    # target is expected to move across the image while RA6M5 aligns the arm.
    # Keep that distinction explicit so motion is not mistaken for jitter.
    confirmed: bool = False
    # Once a track has passed the full initial gate, a very short detector
    # dropout may use a smaller *fresh-frame* re-acquisition gate.  No sample
    # is published while it is missing; this flag only avoids paying the full
    # initial 0.35 s gate again after the same face-relative target returns.
    ever_confirmed: bool = False
    fast_reacquiring: bool = False
    # (u, v, radius_ratio) inside the current face-mask bounding box.  The
    # camera moves with the arm, so raw image coordinates can jump even when
    # the physical mark stays at the same place on the face.
    reference_position: Optional[Tuple[float, float, float]] = None


class CandidateStabilityTracker:
    """Confirm dark-spot candidates only after stable multi-frame observations."""

    def __init__(
        self,
        required_hits: int = 4,
        history_size: int = 7,
        match_radius: float = 18.0,
        stable_radius: float = 6.0,
        max_missed: int = 2,
        min_stable_duration_s: float = 0.0,
        max_gap_s: float = 0.0,
        max_radius_ratio: float = 1.5,
        association_ambiguity_px: float = 3.0,
        small_target_max_radius: float = 10.0,
        small_target_radius_delta: float = 6.0,
        fast_reacquire_hits: int = 0,
        fast_reacquire_min_duration_s: float = 0.0,
        fast_reacquire_max_gap_s: float = 0.0,
    ) -> None:
        self.required_hits = max(1, int(required_hits))
        self.history_size = max(self.required_hits, int(history_size))
        self.match_radius = max(1.0, float(match_radius))
        self.stable_radius = max(0.5, float(stable_radius))
        self.max_missed = max(0, int(max_missed))
        self.min_stable_duration_s = max(0.0, float(min_stable_duration_s))
        self.max_gap_s = max(0.0, float(max_gap_s))
        self.max_radius_ratio = max(1.05, float(max_radius_ratio))
        self.small_target_max_radius = max(0.0, float(small_target_max_radius))
        self.small_target_radius_delta = max(
            0.0,
            float(small_target_radius_delta),
        )
        self.association_ambiguity_px = max(
            0.0,
            float(association_ambiguity_px),
        )
        # Zero preserves the conservative legacy behavior.  Production Qt
        # opts in explicitly after the full initial lock has succeeded.
        self.fast_reacquire_hits = max(0, int(fast_reacquire_hits))
        self.fast_reacquire_min_duration_s = max(
            0.0,
            float(fast_reacquire_min_duration_s),
        )
        self.fast_reacquire_max_gap_s = max(
            0.0,
            float(fast_reacquire_max_gap_s),
        )
        self._tracks = []
        self._next_id = 1

    def reset(self) -> None:
        self._tracks = []
        self._next_id = 1

    @staticmethod
    def _parse_reference_box(reference_box):
        if reference_box is None:
            return None
        try:
            item_count = len(reference_box)
        except TypeError:
            return None
        if item_count != 4:
            return None
        try:
            x, y, width, height = (float(value) for value in reference_box)
        except (TypeError, ValueError):
            return None
        if (
            not all(np.isfinite(value) for value in (x, y, width, height))
            or width <= 1.0
            or height <= 1.0
        ):
            return None
        return x, y, width, height

    @staticmethod
    def _relative_position(candidate: DetectedMole, reference_box):
        if reference_box is None:
            return None
        x, y, width, height = reference_box
        scale = max(1.0, min(width, height))
        return (
            (float(candidate.center[0]) - x) / width,
            (float(candidate.center[1]) - y) / height,
            float(candidate.radius) / scale,
        )

    @staticmethod
    def _project_reference_position(position, reference_box):
        x, y, width, height = reference_box
        scale = max(1.0, min(width, height))
        return (
            x + position[0] * width,
            y + position[1] * height,
            position[2] * scale,
        )

    @staticmethod
    def _ratio(a: float, b: float) -> float:
        if a <= 0.0 or b <= 0.0:
            return float("inf")
        return max(a / b, b / a)

    def _small_radius_change_allowed(self, a: float, b: float) -> bool:
        """Allow threshold fragmentation for tiny spots, not scale jumps.

        The same 12x17 px printed mark alternates between a full r=8 contour
        and an r=2..5 fragment in soft MJPEG frames.  Ratio-only association
        rejects that observation.  The absolute exception is deliberately
        limited to radii <=10 px; e.g. a real r=7 -> r=12 scale change still
        fails closed.
        """

        return (
            max(float(a), float(b)) <= self.small_target_max_radius
            and abs(float(a) - float(b)) <= self.small_target_radius_delta
        )

    def update(
        self,
        candidates: Iterable[DetectedMole],
        sample_time: Optional[float] = None,
        reference_box: Optional[Tuple[float, float, float, float]] = None,
    ) -> List[DetectedMole]:
        current_time = None
        if sample_time is not None:
            try:
                parsed_time = float(sample_time)
            except (TypeError, ValueError):
                parsed_time = float("nan")
            if np.isfinite(parsed_time):
                current_time = parsed_time

        current = list(candidates)
        current_reference_box = self._parse_reference_box(reference_box)
        current_reference_positions = [
            self._relative_position(candidate, current_reference_box)
            for candidate in current
        ]
        unmatched_tracks = set(range(len(self._tracks)))
        unmatched_candidates = set(range(len(current)))
        possible_matches = []
        for track_index, track in enumerate(self._tracks):
            if not track.samples:
                continue
            tx, ty = track.samples[-1].center
            for candidate_index, candidate in enumerate(current):
                cx, cy = candidate.center
                raw_distance = float(np.hypot(cx - tx, cy - ty))
                distance = raw_distance
                previous_radius = float(track.samples[-1].radius)
                candidate_reference_position = current_reference_positions[
                    candidate_index
                ]
                if (
                    current_reference_box is not None
                    and track.reference_position is not None
                    and candidate_reference_position is not None
                ):
                    predicted_x, predicted_y, predicted_radius = (
                        self._project_reference_position(
                            track.reference_position,
                            current_reference_box,
                        )
                    )
                    face_compensated_distance = float(
                        np.hypot(cx - predicted_x, cy - predicted_y)
                    )
                    # When both face references are trustworthy, use the
                    # anatomical position exclusively.  Falling back to a
                    # coincident old screen pixel here could let a different
                    # facial mark inherit the selected ID after camera motion.
                    distance = face_compensated_distance
                    previous_radius = float(predicted_radius)
                    radius_ratio = self._ratio(
                        candidate_reference_position[2],
                        track.reference_position[2],
                    )
                else:
                    radius_ratio = self._ratio(
                        float(candidate.radius),
                        float(track.samples[-1].radius),
                    )
                if (
                    radius_ratio > self.max_radius_ratio
                    and not self._small_radius_change_allowed(
                        float(candidate.radius),
                        float(previous_radius),
                    )
                ):
                    continue
                dynamic_radius = max(
                    self.match_radius,
                    float(candidate.radius + previous_radius) * 2.0,
                )
                if distance <= dynamic_radius:
                    possible_matches.append((distance, track_index, candidate_index))

        # If two candidates (or two old tracks) are almost equally plausible,
        # fail closed instead of letting greedy ordering silently swap IDs.
        by_track = {}
        by_candidate = {}
        for match in possible_matches:
            by_track.setdefault(match[1], []).append(match)
            by_candidate.setdefault(match[2], []).append(match)
        ambiguous_tracks = set()
        ambiguous_candidates = set()
        ambiguity_blocked_candidates = set()
        provisional_track_ids_to_drop = set()
        for track_index, matches in by_track.items():
            ranked = sorted(matches)
            if len(ranked) > 1:
                first_candidate = current[ranked[0][2]]
                margin = max(
                    self.association_ambiguity_px,
                    float(first_candidate.radius) * 0.5,
                )
                if ranked[1][0] - ranked[0][0] <= margin:
                    ambiguous_tracks.add(track_index)
                    ambiguity_blocked_candidates.update(
                        match[2] for match in matches
                    )
        for candidate_index, matches in by_candidate.items():
            ranked = sorted(matches)
            if len(ranked) > 1:
                candidate = current[candidate_index]
                margin = max(
                    self.association_ambiguity_px,
                    float(candidate.radius) * 0.5,
                )
                close_matches = [
                    match
                    for match in ranked
                    if match[0] - ranked[0][0] <= margin
                ]
                if len(close_matches) <= 1:
                    continue

                # A short ambiguous frame used to create a fresh duplicate
                # track next to the already confirmed target.  On subsequent
                # frames the one raw contour then matched both internal
                # tracks, so the fail-closed ambiguity gate repeatedly
                # dropped an otherwise continuous mole.  The ambiguous frame
                # itself must still publish no coordinate: two contours may
                # really have merged.  When exactly one competing identity
                # has ever passed the full stability gate, only discard the
                # never-confirmed bookkeeping duplicates; the trusted track
                # must then re-acquire from fresh observations on later
                # frames.  Two trusted identities remain fail-closed.
                trusted_matches = [
                    match
                    for match in close_matches
                    if self._tracks[match[1]].ever_confirmed
                ]
                if len(trusted_matches) == 1:
                    provisional_track_ids_to_drop.update(
                        self._tracks[match[1]].track_id
                        for match in close_matches
                        if not self._tracks[match[1]].ever_confirmed
                    )
                ambiguous_candidates.add(candidate_index)
                ambiguity_blocked_candidates.add(candidate_index)

        for _, track_index, candidate_index in sorted(possible_matches):
            if track_index not in unmatched_tracks or candidate_index not in unmatched_candidates:
                continue
            if (
                track_index in ambiguous_tracks
                or candidate_index in ambiguous_candidates
            ):
                continue
            track = self._tracks[track_index]
            candidate = current[candidate_index]
            previous = track.samples[-1]
            was_missed = track.missed > 0
            time_regressed = (
                current_time is not None
                and track.last_sample_time is not None
                and current_time < track.last_sample_time
            )
            if was_missed or time_regressed:
                # A gap (or a non-monotonic timestamp) breaks the consecutive
                # confirmation run.  Keep the track identity, but do not let
                # samples from before the break satisfy required_hits.
                track.samples.clear()
                track.consecutive_hits = 1
                track.confirmed = False
                gap_s = float("inf")
                if (
                    current_time is not None
                    and track.last_sample_time is not None
                    and not time_regressed
                ):
                    gap_s = max(0.0, current_time - track.last_sample_time)
                track.fast_reacquiring = bool(
                    track.ever_confirmed
                    and self.fast_reacquire_hits > 0
                    and self.fast_reacquire_max_gap_s > 0.0
                    and gap_s <= self.fast_reacquire_max_gap_s
                )
            else:
                track.consecutive_hits += 1
            track.samples.append(candidate)
            track.missed = 0
            track.reference_position = current_reference_positions[candidate_index]
            if current_time is not None:
                moved = float(
                    np.hypot(
                        candidate.center[0] - previous.center[0],
                        candidate.center[1] - previous.center[1],
                    )
                )
                if (
                    track.stable_since is None
                    or was_missed
                    or moved > self.stable_radius
                    or time_regressed
                ):
                    track.stable_since = current_time
                if track.first_seen_time is None or time_regressed:
                    track.first_seen_time = current_time
                track.last_sample_time = current_time
            unmatched_tracks.remove(track_index)
            unmatched_candidates.remove(candidate_index)

        for track_index in unmatched_tracks:
            track = self._tracks[track_index]
            track.missed += 1
            track.consecutive_hits = 0
            # A frame with no fresh observation must publish no target.  Keep
            # the track ID for conservative re-association, but require the
            # normal stationary confirmation again before it becomes valid.
            track.confirmed = False

        for candidate_index in unmatched_candidates:
            # An ambiguous observation is evidence that identity is unknown,
            # not evidence of a brand-new physical mark.  Spawning a track
            # here creates another competitor every frame and can permanently
            # starve the original confirmed target.
            if candidate_index in ambiguity_blocked_candidates:
                continue
            samples = deque(maxlen=self.history_size)
            samples.append(current[candidate_index])
            self._tracks.append(
                _CandidateTrack(
                    track_id=self._next_id,
                    samples=samples,
                    consecutive_hits=1,
                    first_seen_time=current_time,
                    missed=0,
                    stable_since=current_time,
                    last_sample_time=current_time,
                    reference_position=current_reference_positions[candidate_index],
                )
            )
            self._next_id += 1

        retained_tracks = []
        for track in self._tracks:
            if track.track_id in provisional_track_ids_to_drop:
                continue
            if track.missed > self.max_missed:
                continue
            if (
                self.max_gap_s > 0.0
                and current_time is not None
                and track.last_sample_time is not None
                and current_time - track.last_sample_time > self.max_gap_s
            ):
                continue
            retained_tracks.append(track)
        self._tracks = retained_tracks

        stable = []
        for track in self._tracks:
            if track.missed != 0 or not track.samples:
                continue

            if not track.confirmed:
                required_hits = self.required_hits
                required_duration_s = self.min_stable_duration_s
                if track.fast_reacquiring and track.ever_confirmed:
                    required_hits = max(1, self.fast_reacquire_hits)
                    required_duration_s = self.fast_reacquire_min_duration_s
                if (
                    track.consecutive_hits < required_hits
                    or len(track.samples) < required_hits
                ):
                    continue
                if required_duration_s > 0.0:
                    if track.stable_since is None or track.last_sample_time is None:
                        continue
                    if (
                        track.last_sample_time - track.stable_since
                        < required_duration_s
                    ):
                        continue
                recent = list(track.samples)[-required_hits:]
                points = np.asarray(
                    [sample.center for sample in recent],
                    dtype=np.float32,
                )
                median = np.median(points, axis=0)
                distances = np.linalg.norm(points - median, axis=1)
                if float(distances.max()) > self.stable_radius:
                    continue
                track.confirmed = True
                track.ever_confirmed = True
                track.fast_reacquiring = False
                representative = DetectedMole(
                    center=(
                        int(round(float(median[0]))),
                        int(round(float(median[1]))),
                    ),
                    radius=int(
                        round(float(np.median([item.radius for item in recent])))
                    ),
                    area=float(np.median([item.area for item in recent])),
                    circularity=float(
                        np.median([item.circularity for item in recent])
                    ),
                    contrast=float(np.median([item.contrast for item in recent])),
                )
            else:
                # Once acquired, publish the newest matched observation rather
                # than repeatedly applying a stationary 6 px window.  This is
                # still fail-closed: any missing/ambiguous frame above clears
                # ``confirmed`` and yields no target until re-confirmation.
                representative = track.samples[-1]

            published_required_count = self.required_hits
            if track.ever_confirmed and len(track.samples) < self.required_hits:
                published_required_count = max(1, self.fast_reacquire_hits)

            track_age_s = 0.0
            if track.first_seen_time is not None and track.last_sample_time is not None:
                track_age_s = max(
                    0.0,
                    float(track.last_sample_time - track.first_seen_time),
                )
            stable.append(
                DetectedMole(
                    center=representative.center,
                    radius=int(representative.radius),
                    area=float(representative.area),
                    circularity=float(representative.circularity),
                    contrast=float(representative.contrast),
                    track_id=track.track_id,
                    stable_count=track.consecutive_hits,
                    required_count=published_required_count,
                    track_age_s=track_age_s,
                )
            )

        stable.sort(key=lambda item: item.contrast * item.area, reverse=True)
        return stable

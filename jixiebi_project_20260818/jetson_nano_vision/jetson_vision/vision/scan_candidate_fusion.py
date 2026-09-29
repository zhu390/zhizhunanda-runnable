"""Fuse dark-spot candidates observed during a bounded face scan.

This module is intentionally independent from Qt and the RA6M5 protocol.  It
does not publish a live target, pixel error, ``valid`` flag, or any other value
that can drive the robot.  ``ScanCandidateFusion.finish`` only produces a
discovery-stage summary that a later, separately gated target-acquisition step
may use.

Tracks are associated in face-relative coordinates.  Consequently, a fixed
mark can keep the same identity while the face bounding box moves laterally in
the camera image.  Raw screen coordinates are deliberately not retained in the
public summary.
"""

import math
from statistics import median
from typing import Iterable, List, Optional, Tuple

from .._compat import dataclass
from .mole_detect import DetectedMole


ReferenceBox = Tuple[float, float, float, float]


class ScanReferenceBoxSmoother:
    """Low-pass a noisy face-mask box during discovery scanning.

    Haar/skin segmentation can alternate between a narrow central face mask
    and the full silhouette while the camera image itself barely moves.  That
    discontinuity must not be interpreted as a mole jumping anatomically.
    Missing boxes still return ``None`` (fail closed); the last filtered state
    is only retained so a later valid box can resume smoothly.
    """

    def __init__(self, alpha: float = 0.10) -> None:
        parsed_alpha = float(alpha)
        if not math.isfinite(parsed_alpha):
            raise ValueError("alpha must be a finite number")
        self.alpha = min(1.0, max(0.01, parsed_alpha))
        self.reset()

    def reset(self) -> None:
        self._box: Optional[ReferenceBox] = None

    @staticmethod
    def _parse(reference_box) -> Optional[ReferenceBox]:
        if reference_box is None:
            return None
        try:
            if len(reference_box) != 4:
                raise ValueError("reference_box must contain x/y/width/height")
            values = tuple(float(value) for value in reference_box)
        except TypeError:
            raise ValueError("reference_box must contain x/y/width/height")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("reference_box must contain finite values")
        if values[2] <= 1.0 or values[3] <= 1.0:
            raise ValueError("reference_box width/height must be greater than 1")
        return values

    def update(self, reference_box) -> Optional[ReferenceBox]:
        parsed = self._parse(reference_box)
        if parsed is None:
            return None
        if self._box is None:
            self._box = parsed
        else:
            alpha = self.alpha
            self._box = tuple(
                (1.0 - alpha) * old + alpha * new
                for old, new in zip(self._box, parsed)
            )
        return self._box


@dataclass(frozen=True)
class ScanCandidateSummary:
    """Read-only discovery summary for one face-relative candidate track."""

    track_id: int
    relative_center: Tuple[float, float]
    relative_radius: float
    hits: int
    hit_ratio: float
    first_seen_time: float
    last_seen_time: float
    observation_span_s: float
    view_span_face_widths: float
    median_contrast: float
    median_area_px2: float
    area_consistency: float
    confidence: float
    eligible: bool
    rejection_reasons: Tuple[str, ...]


@dataclass(frozen=True)
class ScanFusionResult:
    """Result of a completed discovery scan.

    ``selected`` is ``None`` unless one candidate wins by the configured
    confidence margin.  This fail-closed result intentionally has no dcx/dcy,
    frame pixel, or VISION_ERROR validity fields.
    """

    selected: Optional[ScanCandidateSummary]
    candidates: Tuple[ScanCandidateSummary, ...]
    reason: str
    confidence_margin: float

    @property
    def unique(self) -> bool:
        return self.selected is not None


@dataclass(frozen=True)
class _Observation:
    relative_x: float
    relative_y: float
    relative_radius: float
    normalized_area: float
    area_px2: float
    contrast: float
    face_center_x: float
    face_width: float
    sample_time: float
    frame_index: int


@dataclass
class _Track:
    track_id: int
    observations: List[_Observation]
    missed_frames: int = 0
    active: bool = True


class ScanCandidateFusion:
    """Associate and rank multiple mole candidates across a lateral scan.

    Call :meth:`update` once for every new camera result.  Candidate misses are
    represented by an empty iterable.  ``reference_box=None`` is accepted only
    as a fail-closed face-reference miss: no candidates from that frame are
    consumed.  A short miss does not destroy a track; longer gaps create a new
    track and therefore tend to make :meth:`finish` ambiguous rather than
    silently joining unrelated observations.
    """

    def __init__(
        self,
        min_hits: int = 5,
        min_view_span_face_widths: float = 0.30,
        min_contrast: float = 18.0,
        good_contrast: float = 55.0,
        min_area_consistency: float = 0.55,
        min_confidence: float = 0.72,
        unique_confidence_margin: float = 0.12,
        max_missed_frames: int = 3,
        max_gap_s: float = 0.65,
        max_relative_dx: float = 0.12,
        max_relative_dy: float = 0.085,
        max_area_ratio: float = 3.0,
        association_ambiguity: float = 0.10,
        face_edge_margin: float = 0.06,
        prediction_window: int = 5,
        max_relative_speed_x_per_s: float = 1.5,
        max_relative_speed_y_per_s: float = 1.0,
        max_gap_gate_scale: float = 1.75,
        reference_box_alpha: float = 1.0,
        stitch_max_gap_s: float = 0.0,
        stitch_max_relative_dx: float = 0.18,
        stitch_max_relative_dy: float = 0.08,
        stitch_max_radius_ratio: float = 1.8,
        stitch_max_contrast_delta: float = 25.0,
        stitch_ambiguity: float = 0.10,
    ) -> None:
        self.min_hits = max(2, int(min_hits))
        self.min_view_span_face_widths = max(
            0.0,
            float(min_view_span_face_widths),
        )
        self.min_contrast = max(0.0, float(min_contrast))
        self.good_contrast = max(self.min_contrast + 1.0, float(good_contrast))
        self.min_area_consistency = min(
            1.0,
            max(0.0, float(min_area_consistency)),
        )
        self.min_confidence = min(1.0, max(0.0, float(min_confidence)))
        self.unique_confidence_margin = min(
            1.0,
            max(0.0, float(unique_confidence_margin)),
        )
        self.max_missed_frames = max(0, int(max_missed_frames))
        self.max_gap_s = max(0.0, float(max_gap_s))
        self.max_relative_dx = max(1e-4, float(max_relative_dx))
        self.max_relative_dy = max(1e-4, float(max_relative_dy))
        self.max_area_ratio = max(1.01, float(max_area_ratio))
        self.association_ambiguity = max(0.0, float(association_ambiguity))
        self.face_edge_margin = max(0.0, float(face_edge_margin))
        self.prediction_window = max(2, min(5, int(prediction_window)))
        self.max_relative_speed_x_per_s = max(
            0.0,
            float(max_relative_speed_x_per_s),
        )
        self.max_relative_speed_y_per_s = max(
            0.0,
            float(max_relative_speed_y_per_s),
        )
        self.max_gap_gate_scale = max(1.0, float(max_gap_gate_scale))
        self.reference_box_alpha = min(
            1.0,
            max(0.01, float(reference_box_alpha)),
        )
        self.stitch_max_gap_s = max(0.0, float(stitch_max_gap_s))
        self.stitch_max_relative_dx = max(
            1e-4,
            float(stitch_max_relative_dx),
        )
        self.stitch_max_relative_dy = max(
            1e-4,
            float(stitch_max_relative_dy),
        )
        self.stitch_max_radius_ratio = max(
            1.01,
            float(stitch_max_radius_ratio),
        )
        self.stitch_max_contrast_delta = max(
            0.0,
            float(stitch_max_contrast_delta),
        )
        self.stitch_ambiguity = max(0.0, float(stitch_ambiguity))
        self._reference_smoother = ScanReferenceBoxSmoother(
            self.reference_box_alpha
        )
        self.reset()

    def reset(self) -> None:
        self._tracks: List[_Track] = []
        self._next_track_id = 1
        self._last_sample_time: Optional[float] = None
        self._frame_index = 0
        self._reference_smoother.reset()

    @staticmethod
    def _finite_float(name: str, value) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("{} must be a finite number".format(name))
        if not math.isfinite(parsed):
            raise ValueError("{} must be a finite number".format(name))
        return parsed

    @classmethod
    def _parse_reference_box(cls, reference_box) -> Optional[ReferenceBox]:
        if reference_box is None:
            return None
        if hasattr(reference_box, "roi"):
            reference_box = reference_box.roi
        elif all(
            hasattr(reference_box, name)
            for name in ("x", "y", "width", "height")
        ):
            reference_box = (
                reference_box.x,
                reference_box.y,
                reference_box.width,
                reference_box.height,
            )
        try:
            if len(reference_box) != 4:
                raise ValueError("reference_box must contain x/y/width/height")
            x, y, width, height = (
                cls._finite_float("reference_box", value)
                for value in reference_box
            )
        except TypeError:
            raise ValueError("reference_box must contain x/y/width/height")
        if width <= 1.0 or height <= 1.0:
            raise ValueError("reference_box width/height must be greater than 1")
        return x, y, width, height

    @staticmethod
    def _ratio(a: float, b: float) -> float:
        if a <= 0.0 or b <= 0.0:
            return float("inf")
        return max(a / b, b / a)

    @staticmethod
    def _clamped_linear_velocity(
        observations,
        attribute: str,
        max_abs_speed: float,
    ) -> float:
        """Estimate a short-window constant velocity in face coordinates."""

        if len(observations) < 2 or max_abs_speed <= 0.0:
            return 0.0
        times = [float(item.sample_time) for item in observations]
        values = [float(getattr(item, attribute)) for item in observations]
        mean_time = sum(times) / float(len(times))
        mean_value = sum(values) / float(len(values))
        denominator = sum((item - mean_time) ** 2 for item in times)
        if denominator <= 1e-12:
            return 0.0
        velocity = sum(
            (item_time - mean_time) * (item_value - mean_value)
            for item_time, item_value in zip(times, values)
        ) / denominator
        return max(-max_abs_speed, min(max_abs_speed, velocity))

    def _track_prediction(
        self,
        track: _Track,
        sample_time: float,
    ) -> Tuple[float, float, float, float, float]:
        """Predict the next face-relative location and a bounded gap gate.

        A true mark drifts inside the face box during yaw.  Using the last raw
        relative point (or a fixed median) would therefore fragment a scan
        track.  A 2--5 observation linear fit follows that drift.  The match
        gate grows only for time beyond the recent frame cadence and is capped,
        so a long gap cannot absorb a remote dark spot.
        """

        recent = track.observations[-self.prediction_window :]
        last = recent[-1]
        velocity_x = self._clamped_linear_velocity(
            recent,
            "relative_x",
            self.max_relative_speed_x_per_s,
        )
        velocity_y = self._clamped_linear_velocity(
            recent,
            "relative_y",
            self.max_relative_speed_y_per_s,
        )
        horizon = max(0.0, float(sample_time) - float(last.sample_time))
        if self.max_gap_s > 0.0:
            horizon = min(horizon, self.max_gap_s)
        predicted_x = float(last.relative_x) + velocity_x * horizon
        predicted_y = float(last.relative_y) + velocity_y * horizon

        intervals = [
            float(current.sample_time - previous.sample_time)
            for previous, current in zip(recent, recent[1:])
            if current.sample_time > previous.sample_time
        ]
        normal_interval = float(median(intervals)) if intervals else horizon
        extra_gap = max(0.0, horizon - normal_interval)
        if self.max_gap_s > 0.0:
            gap_fraction = min(1.0, extra_gap / self.max_gap_s)
        else:
            gap_fraction = 0.0
        gate_scale = 1.0 + gap_fraction * (self.max_gap_gate_scale - 1.0)

        return (
            predicted_x,
            predicted_y,
            float(median(item.relative_radius for item in recent)),
            float(median(item.normalized_area for item in recent)),
            gate_scale,
        )

    def _make_observation(
        self,
        candidate: DetectedMole,
        reference_box: ReferenceBox,
        sample_time: float,
    ) -> Optional[_Observation]:
        try:
            center_x = self._finite_float("candidate.center.x", candidate.center[0])
            center_y = self._finite_float("candidate.center.y", candidate.center[1])
            radius = self._finite_float("candidate.radius", candidate.radius)
            area = self._finite_float("candidate.area", candidate.area)
            contrast = self._finite_float("candidate.contrast", candidate.contrast)
        except (AttributeError, IndexError, TypeError):
            raise TypeError("candidates must contain DetectedMole-compatible values")
        if radius <= 0.0 or area <= 0.0 or contrast < 0.0:
            return None

        x, y, width, height = reference_box
        relative_x = (center_x - x) / width
        relative_y = (center_y - y) / height
        margin = self.face_edge_margin
        if not (
            -margin <= relative_x <= 1.0 + margin
            and -margin <= relative_y <= 1.0 + margin
        ):
            return None
        scale = max(1.0, min(width, height))
        return _Observation(
            relative_x=relative_x,
            relative_y=relative_y,
            relative_radius=radius / scale,
            normalized_area=area / (width * height),
            area_px2=area,
            contrast=contrast,
            face_center_x=x + width * 0.5,
            face_width=width,
            sample_time=sample_time,
            frame_index=self._frame_index,
        )

    def _deactivate_expired_tracks(self, sample_time: float) -> None:
        for track in self._tracks:
            if not track.active or not track.observations:
                continue
            last_seen = track.observations[-1].sample_time
            if (
                track.missed_frames > self.max_missed_frames
                or (
                    self.max_gap_s > 0.0
                    and sample_time - last_seen > self.max_gap_s
                )
            ):
                track.active = False

    def _mark_unmatched(self, matched_track_ids, sample_time: float) -> None:
        for track in self._tracks:
            if not track.active or track.track_id in matched_track_ids:
                continue
            track.missed_frames += 1
        self._deactivate_expired_tracks(sample_time)

    def update(
        self,
        candidates: Iterable[DetectedMole],
        reference_box,
        sample_time: float,
    ) -> None:
        """Consume one scan frame without returning a live treatment target."""

        timestamp = self._finite_float("sample_time", sample_time)
        if self._last_sample_time is not None and timestamp <= self._last_sample_time:
            raise ValueError("sample_time must be strictly increasing")
        self._last_sample_time = timestamp
        self._frame_index += 1
        parsed_box = self._parse_reference_box(reference_box)
        parsed_box = self._reference_smoother.update(parsed_box)
        current_candidates = list(candidates)

        self._deactivate_expired_tracks(timestamp)
        if parsed_box is None:
            # A candidate without a face reference cannot be associated safely.
            self._mark_unmatched(set(), timestamp)
            return None

        observations = []
        for candidate in current_candidates:
            observation = self._make_observation(candidate, parsed_box, timestamp)
            if observation is not None:
                observations.append(observation)

        active_tracks = [track for track in self._tracks if track.active]
        possible_matches = []
        for track_index, track in enumerate(active_tracks):
            (
                predicted_x,
                predicted_y,
                _reference_radius,
                reference_area,
                gate_scale,
            ) = self._track_prediction(track, timestamp)
            for observation_index, observation in enumerate(observations):
                area_ratio = self._ratio(
                    observation.normalized_area,
                    reference_area,
                )
                if area_ratio > self.max_area_ratio:
                    continue
                dx = abs(observation.relative_x - predicted_x)
                dy = abs(observation.relative_y - predicted_y)
                cost = math.hypot(
                    dx / (self.max_relative_dx * gate_scale),
                    dy / (self.max_relative_dy * gate_scale),
                )
                if cost <= 1.0:
                    # A small area penalty breaks otherwise identical position
                    # ties without allowing size alone to join remote tracks.
                    cost += 0.08 * min(1.0, (area_ratio - 1.0) / 2.0)
                    possible_matches.append(
                        (cost, track_index, observation_index)
                    )

        by_track = {}
        by_observation = {}
        for match in possible_matches:
            by_track.setdefault(match[1], []).append(match)
            by_observation.setdefault(match[2], []).append(match)
        ambiguous_tracks = set()
        ambiguous_observations = set()
        for track_index, matches in by_track.items():
            ranked = sorted(matches)
            if (
                len(ranked) > 1
                and ranked[1][0] - ranked[0][0] <= self.association_ambiguity
            ):
                ambiguous_tracks.add(track_index)
        for observation_index, matches in by_observation.items():
            ranked = sorted(matches)
            if (
                len(ranked) > 1
                and ranked[1][0] - ranked[0][0] <= self.association_ambiguity
            ):
                ambiguous_observations.add(observation_index)

        matched_track_indices = set()
        matched_observation_indices = set()
        matched_track_ids = set()
        for _cost, track_index, observation_index in sorted(possible_matches):
            if (
                track_index in ambiguous_tracks
                or observation_index in ambiguous_observations
                or track_index in matched_track_indices
                or observation_index in matched_observation_indices
            ):
                continue
            track = active_tracks[track_index]
            track.observations.append(observations[observation_index])
            track.missed_frames = 0
            matched_track_indices.add(track_index)
            matched_observation_indices.add(observation_index)
            matched_track_ids.add(track.track_id)

        self._mark_unmatched(matched_track_ids, timestamp)

        for observation_index, observation in enumerate(observations):
            if observation_index in matched_observation_indices:
                continue
            self._tracks.append(
                _Track(
                    track_id=self._next_track_id,
                    observations=[observation],
                )
            )
            self._next_track_id += 1
        return None

    @staticmethod
    def _clamp01(value: float) -> float:
        return min(1.0, max(0.0, float(value)))

    def _summarize(self, track: _Track) -> ScanCandidateSummary:
        observations = list(track.observations)
        hits = len(observations)
        first_frame = observations[0].frame_index
        last_frame = observations[-1].frame_index
        opportunity_frames = max(1, last_frame - first_frame + 1)
        hit_ratio = min(1.0, float(hits) / float(opportunity_frames))

        relative_x = float(median(item.relative_x for item in observations))
        relative_y = float(median(item.relative_y for item in observations))
        relative_radius = float(
            median(item.relative_radius for item in observations)
        )
        contrast = float(median(item.contrast for item in observations))
        median_area_px2 = float(median(item.area_px2 for item in observations))
        normalized_areas = [item.normalized_area for item in observations]
        median_normalized_area = max(
            1e-12,
            float(median(normalized_areas)),
        )
        relative_area_mad = float(
            median(
                abs(area - median_normalized_area)
                for area in normalized_areas
            )
            / median_normalized_area
        )
        area_consistency = self._clamp01(1.0 - relative_area_mad)

        median_face_width = max(
            1.0,
            float(median(item.face_width for item in observations)),
        )
        face_centers = [item.face_center_x for item in observations]
        view_span = (
            max(face_centers) - min(face_centers)
        ) / median_face_width

        rejection_reasons = []
        if hits < self.min_hits:
            rejection_reasons.append("hits")
        if view_span < self.min_view_span_face_widths:
            rejection_reasons.append("view_span")
        if contrast < self.min_contrast:
            rejection_reasons.append("contrast")
        if area_consistency < self.min_area_consistency:
            rejection_reasons.append("area_consistency")

        support_score = self._clamp01(float(hits) / float(self.min_hits))
        span_denominator = max(1e-6, self.min_view_span_face_widths)
        span_score = self._clamp01(view_span / span_denominator)
        contrast_score = self._clamp01(
            (contrast - self.min_contrast)
            / (self.good_contrast - self.min_contrast)
        )
        confidence = self._clamp01(
            0.30 * support_score
            + 0.15 * hit_ratio
            + 0.20 * span_score
            + 0.18 * contrast_score
            + 0.17 * area_consistency
        )
        if confidence < self.min_confidence:
            rejection_reasons.append("confidence")

        return ScanCandidateSummary(
            track_id=track.track_id,
            relative_center=(relative_x, relative_y),
            relative_radius=relative_radius,
            hits=hits,
            hit_ratio=hit_ratio,
            first_seen_time=observations[0].sample_time,
            last_seen_time=observations[-1].sample_time,
            observation_span_s=max(
                0.0,
                observations[-1].sample_time - observations[0].sample_time,
            ),
            view_span_face_widths=view_span,
            median_contrast=contrast,
            median_area_px2=median_area_px2,
            area_consistency=area_consistency,
            confidence=confidence,
            eligible=not rejection_reasons,
            rejection_reasons=tuple(rejection_reasons),
        )

    @staticmethod
    def _tracklet_signature(track: _Track):
        observations = track.observations
        return (
            float(median(item.relative_x for item in observations)),
            float(median(item.relative_y for item in observations)),
            float(median(item.relative_radius for item in observations)),
            float(median(item.contrast for item in observations)),
        )

    def _stitch_cost(self, earlier: _Track, later: _Track) -> Optional[float]:
        if not earlier.observations or not later.observations:
            return None
        earlier_last = earlier.observations[-1].sample_time
        later_first = later.observations[0].sample_time
        gap_s = later_first - earlier_last
        if gap_s <= 0.0 or gap_s > self.stitch_max_gap_s:
            return None

        ax, ay, ar, ac = self._tracklet_signature(earlier)
        bx, by, br, bc = self._tracklet_signature(later)
        dx = abs(ax - bx)
        dy = abs(ay - by)
        radius_ratio = self._ratio(ar, br)
        contrast_delta = abs(ac - bc)
        if (
            dx > self.stitch_max_relative_dx
            or dy > self.stitch_max_relative_dy
            or radius_ratio > self.stitch_max_radius_ratio
            or contrast_delta > self.stitch_max_contrast_delta
        ):
            return None

        position_cost = math.hypot(
            dx / self.stitch_max_relative_dx,
            dy / self.stitch_max_relative_dy,
        )
        if position_cost > 1.0:
            return None
        radius_cost = (radius_ratio - 1.0) / (
            self.stitch_max_radius_ratio - 1.0
        )
        contrast_cost = 0.0
        if self.stitch_max_contrast_delta > 0.0:
            contrast_cost = contrast_delta / self.stitch_max_contrast_delta
        return position_cost + 0.12 * radius_cost + 0.06 * contrast_cost

    def _stitch_tracks(self) -> List[_Track]:
        if self.stitch_max_gap_s <= 0.0:
            return list(self._tracks)

        ordered = sorted(
            (track for track in self._tracks if track.observations),
            key=lambda item: item.observations[0].sample_time,
        )
        stitched: List[_Track] = []
        for track in ordered:
            matches = []
            for stitched_index, existing in enumerate(stitched):
                cost = self._stitch_cost(existing, track)
                if cost is not None:
                    matches.append((cost, stitched_index))
            matches.sort()
            if (
                not matches
                or (
                    len(matches) > 1
                    and matches[1][0] - matches[0][0]
                    <= self.stitch_ambiguity
                )
            ):
                stitched.append(
                    _Track(
                        track_id=track.track_id,
                        observations=list(track.observations),
                        active=False,
                    )
                )
                continue

            destination = stitched[matches[0][1]]
            destination.observations.extend(track.observations)
            destination.observations.sort(key=lambda item: item.sample_time)
        return stitched

    def finish(self) -> ScanFusionResult:
        """Return discovery summaries and select only an unambiguous winner."""

        summaries = [
            self._summarize(track)
            for track in self._stitch_tracks()
            if track.observations
        ]
        summaries.sort(
            key=lambda item: (item.confidence, item.hits, item.track_id),
            reverse=True,
        )
        candidate_tuple = tuple(summaries)
        if not summaries:
            return ScanFusionResult(
                selected=None,
                candidates=candidate_tuple,
                reason="no_candidates",
                confidence_margin=0.0,
            )

        eligible = [summary for summary in summaries if summary.eligible]
        if not eligible:
            return ScanFusionResult(
                selected=None,
                candidates=candidate_tuple,
                reason="no_eligible_candidate",
                confidence_margin=0.0,
            )

        best = eligible[0]
        margin = 1.0
        if len(eligible) > 1:
            margin = max(0.0, best.confidence - eligible[1].confidence)
            if margin < self.unique_confidence_margin:
                return ScanFusionResult(
                    selected=None,
                    candidates=candidate_tuple,
                    reason="ambiguous_candidates",
                    confidence_margin=margin,
                )

        return ScanFusionResult(
            selected=best,
            candidates=candidate_tuple,
            reason="unique_candidate",
            confidence_margin=margin,
        )


__all__ = [
    "ScanCandidateFusion",
    "ScanCandidateSummary",
    "ScanFusionResult",
    "ScanReferenceBoxSmoother",
]

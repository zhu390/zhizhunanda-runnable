"""Reference-free facial dark-spot candidate detection.

The face detector only limits the search area. Dark-spot candidates are still
found with local image contrast, so this module does not diagnose or classify
skin lesions. A human must select and confirm the final target.
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from .._compat import dataclass
from .mole_detect import DetectParams, DetectedMole, build_skin_mask, detect_moles


@dataclass(frozen=True)
class FaceBox:
    x: int
    y: int
    width: int
    height: int
    confidence: float = 0.0
    source: str = "unknown"

    @property
    def roi(self) -> Tuple[int, int, int, int]:
        return self.x, self.y, self.width, self.height


@dataclass(frozen=True)
class FacialCandidateParams:
    face_confidence: float = 0.35
    face_imgsz: int = 640
    face_edge_margin_ratio: float = 0.035
    max_candidates: int = 12
    face_refresh_frames: int = 2
    face_refresh_misses: int = 2
    partial_face_guard: bool = False
    face_baseline_frames: int = 1
    face_baseline_min_width_ratio: float = 0.35
    face_baseline_min_height_ratio: float = 0.60
    face_baseline_min_area_ratio: float = 0.18
    face_mask_min_width_ratio: float = 0.65
    face_mask_min_height_ratio: float = 0.65
    face_mask_min_area_ratio: float = 0.50
    privacy_blur_sigma: float = 22.0
    eye_pixel_size: int = 12
    eye_blur_sigma: float = 7.0
    background_mode: str = "black"
    # ``anatomical`` preserves the older conservative mask (eyes, eyebrows,
    # nose, mouth and the upper face edge).  Production Qt uses ``eyes_only``:
    # the face silhouette still rejects the background, while only the eyes
    # and a small eye halo are hard safety exclusions for target detection.
    detection_exclusion_mode: str = "anatomical"
    # A soft UVC frame can split one printed mole into two small contours or
    # turn its circular outline into a teardrop.  Once a strict, globally
    # searched candidate has seeded the location, re-check only a small area
    # around the same face-relative point with relaxed shape gates.  Recovery
    # still uses pixels from the current frame and the normal valid_mask; it
    # never republishes a stale coordinate.
    candidate_recovery_enabled: bool = True
    candidate_recovery_radius_px: int = 32
    candidate_recovery_max_misses: int = 6
    candidate_dedup_radius_px: int = 12
    enhanced_candidate_filters: bool = True


@dataclass
class FacialCandidateResult:
    face: Optional[FaceBox]
    candidates: List[DetectedMole]
    face_mask: np.ndarray
    valid_mask: np.ndarray
    excluded_masks: Dict[str, np.ndarray]
    annotated: np.ndarray
    privacy_view: np.ndarray
    message: str


def _load_haar_cascade(filename: str):
    """Load an OpenCV Haar cascade across pip and Jetson system layouts."""
    search_dirs = []
    cv2_data = getattr(cv2, "data", None)
    cv2_haar_dir = getattr(cv2_data, "haarcascades", "")
    if cv2_haar_dir:
        search_dirs.append(Path(cv2_haar_dir))

    cv2_module_path = getattr(cv2, "__file__", "")
    if cv2_module_path:
        search_dirs.append(Path(cv2_module_path).resolve().parent / "data")

    search_dirs.extend(
        (
            Path("/usr/local/share/opencv4/haarcascades"),
            Path("/usr/share/opencv4/haarcascades"),
            Path("/usr/local/share/opencv/haarcascades"),
            Path("/usr/share/opencv/haarcascades"),
        )
    )

    seen = set()
    for directory in search_dirs:
        candidate = directory / filename
        candidate_text = str(candidate)
        if candidate_text in seen or not candidate.is_file():
            continue
        seen.add(candidate_text)
        try:
            classifier = cv2.CascadeClassifier(candidate_text)
        except Exception:
            continue
        if not classifier.empty():
            return classifier
    return cv2.CascadeClassifier()


class FaceLocator:
    """Locate the largest face with the project YOLO model, then Haar fallback."""

    def __init__(
        self,
        model_path: Optional[Path] = None,
        confidence: float = 0.35,
        imgsz: int = 640,
        device: str = "",
    ) -> None:
        self.model_path = Path(model_path) if model_path else None
        self.confidence = float(confidence)
        self.imgsz = max(160, int(imgsz))
        self.device = str(device)
        self._model = None
        self._face_class_id = None
        self._haar = _load_haar_cascade("haarcascade_frontalface_default.xml")
        self._profile_haar = _load_haar_cascade("haarcascade_profileface.xml")
        self.last_error = ""

    def locate(self, frame_bgr: np.ndarray) -> Optional[FaceBox]:
        face = self._locate_haar(frame_bgr)
        if face is not None:
            skin_face = self._locate_skin(frame_bgr)
            if _skin_face_dominates_detector(
                frame_bgr.shape,
                detected_face=face,
                skin_face=skin_face,
            ):
                return skin_face
            return face
        face = self._locate_yolo(frame_bgr)
        if face is not None:
            skin_face = self._locate_skin(frame_bgr)
            if _skin_face_dominates_detector(
                frame_bgr.shape,
                detected_face=face,
                skin_face=skin_face,
            ):
                return skin_face
            return face
        return self._locate_skin(frame_bgr)

    def _load_yolo(self) -> bool:
        if self._model is not None:
            return self._face_class_id is not None
        if self.model_path is None or not self.model_path.exists():
            return False
        try:
            from ultralytics import YOLO

            self._model = YOLO(str(self.model_path))
            names = self._model.names
            items = names.items() if isinstance(names, dict) else enumerate(names)
            for class_id, name in items:
                if str(name).strip().lower() == "face":
                    self._face_class_id = int(class_id)
                    break
            if self._face_class_id is None:
                self.last_error = "YOLO model has no 'face' class"
                return False
            return True
        except Exception as exc:
            self.last_error = "YOLO load failed: {}".format(exc)
            self._model = False
            return False

    def _locate_yolo(self, frame_bgr: np.ndarray) -> Optional[FaceBox]:
        if not self._load_yolo():
            return None
        try:
            kwargs = {
                "source": frame_bgr,
                "conf": self.confidence,
                "imgsz": self.imgsz,
                "classes": [self._face_class_id],
                "verbose": False,
            }
            if self.device:
                kwargs["device"] = self.device
            results = self._model.predict(**kwargs)
            boxes = results[0].boxes if results else None
            if boxes is None or len(boxes) == 0:
                return None
            best = max(
                boxes,
                key=lambda box: float((box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1])),
            )
            x0, y0, x1, y1 = [float(value) for value in best.xyxy[0].tolist()]
            confidence = float(best.conf[0]) if best.conf is not None else 0.0
            return _clamp_face_box(
                frame_bgr.shape,
                int(round(x0)),
                int(round(y0)),
                int(round(x1 - x0)),
                int(round(y1 - y0)),
                confidence,
                "yolo",
            )
        except Exception as exc:
            self.last_error = "YOLO inference failed: {}".format(exc)
            return None

    def _locate_haar(self, frame_bgr: np.ndarray) -> Optional[FaceBox]:
        if self._haar.empty():
            return None
        original_height, original_width = frame_bgr.shape[:2]
        detect_scale = min(
            1.0,
            float(self.imgsz) / max(1.0, float(original_width)),
        )
        if detect_scale < 1.0:
            detect_frame = cv2.resize(
                frame_bgr,
                (
                    max(1, int(round(original_width * detect_scale))),
                    max(1, int(round(original_height * detect_scale))),
                ),
                interpolation=cv2.INTER_AREA,
            )
        else:
            detect_frame = frame_bgr
        gray = cv2.cvtColor(detect_frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self._haar.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(80, 80),
        )
        source = "haar"
        if len(faces) == 0 and not self._profile_haar.empty():
            faces = self._profile_haar.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=4,
                minSize=(70, 70),
            )
            source = "haar-profile"
        if len(faces) == 0 and not self._profile_haar.empty():
            flipped = cv2.flip(gray, 1)
            flipped_faces = self._profile_haar.detectMultiScale(
                flipped,
                scaleFactor=1.1,
                minNeighbors=4,
                minSize=(70, 70),
            )
            if len(flipped_faces):
                faces = [
                    (gray.shape[1] - int(x) - int(w), int(y), int(w), int(h))
                    for x, y, w, h in flipped_faces
                ]
                source = "haar-profile-flipped"
        if len(faces) == 0:
            return None
        x, y, width, height = max(faces, key=lambda item: int(item[2]) * int(item[3]))
        if detect_scale < 1.0:
            inverse_scale = 1.0 / detect_scale
            x = int(round(float(x) * inverse_scale))
            y = int(round(float(y) * inverse_scale))
            width = int(round(float(width) * inverse_scale))
            height = int(round(float(height) * inverse_scale))
        return _clamp_face_box(
            frame_bgr.shape,
            int(x),
            int(y),
            int(width),
            int(height),
            0.0,
            source,
        )

    def _locate_skin(self, frame_bgr: np.ndarray) -> Optional[FaceBox]:
        mask = build_skin_mask(frame_bgr)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if not contours:
            return None
        image_height, image_width = frame_bgr.shape[:2]
        image_area = float(image_height * image_width)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < image_area * 0.03 or area > image_area * 0.85:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            aspect = float(width) / max(1.0, float(height))
            if not 0.45 <= aspect <= 1.65:
                continue
            center_x = x + width * 0.5
            center_y = y + height * 0.5
            center_distance = np.hypot(
                (center_x - image_width * 0.5) / max(1.0, image_width),
                (center_y - image_height * 0.5) / max(1.0, image_height),
            )
            score = area * max(0.1, 1.0 - center_distance)
            candidates.append((score, x, y, width, height))
        if not candidates:
            return None
        _, x, y, width, height = max(candidates, key=lambda item: item[0])
        return _clamp_face_box(
            frame_bgr.shape,
            int(x),
            int(y),
            int(width),
            int(height),
            0.0,
            "skin",
        )


def _candidate_score(candidate: DetectedMole) -> float:
    return float(candidate.area) * max(0.0, float(candidate.contrast))


def _deduplicate_candidates(candidates, radius_px: int):
    """Keep one representative when one dark mark is split into fragments."""

    kept = []
    for candidate in sorted(candidates, key=_candidate_score, reverse=True):
        duplicate = False
        for existing in kept:
            distance = float(
                np.hypot(
                    candidate.center[0] - existing.center[0],
                    candidate.center[1] - existing.center[1],
                )
            )
            merge_radius = max(
                float(max(1, int(radius_px))),
                float(candidate.radius + existing.radius + 3),
            )
            if distance <= merge_radius:
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
    return kept


def _reject_eye_region_candidates(candidates, excluded_masks):
    """Drop dark contours only when they are inside the actual eye masks.

    Earlier versions also rejected eyebrow/eye-guard regions.  In the live
    mannequin close-up this was too aggressive: the broad anatomical guard
    often covered valid cheek/temple moles.  Keep the hard safety exclusion on
    the eye itself, but allow candidates near eyebrows or just below the eye to
    be judged by the normal dark-spot shape/contrast gates.
    """

    if not candidates or not excluded_masks:
        return list(candidates)

    reject_names = (
        "left_eye",
        "right_eye",
    )
    reject_masks = [
        mask
        for name in reject_names
        for mask in (excluded_masks.get(name),)
        if mask is not None
    ]
    if not reject_masks:
        return list(candidates)

    height, width = reject_masks[0].shape[:2]
    kept = []
    for candidate in candidates:
        x, y = candidate.center
        x = int(round(x))
        y = int(round(y))
        if x < 0 or x >= width or y < 0 or y >= height:
            continue
        if any(int(mask[y, x]) > 0 for mask in reject_masks):
            continue
        kept.append(candidate)
    return kept


def _reject_lower_boundary_noise_candidates(candidates, face_mask):
    """Drop tiny weak contours at the chin/neck silhouette boundary."""

    if not candidates:
        return list(candidates)
    mask_box = _face_box_from_mask(face_mask)
    if mask_box is None or mask_box.height <= 1:
        return list(candidates)
    lower_gate_y = float(mask_box.y) + float(mask_box.height) * 0.94
    kept = []
    for candidate in candidates:
        _x, y = candidate.center
        if (
            float(y) >= lower_gate_y
            and int(candidate.radius) <= 5
            and (
                float(candidate.contrast) < 35.0
                or float(candidate.area) < 35.0
            )
        ):
            continue
        kept.append(candidate)
    return kept


def _face_relative_anchor(center, face_box: FaceBox):
    if face_box.width <= 1 or face_box.height <= 1:
        return None
    return (
        (float(center[0]) - float(face_box.x)) / float(face_box.width),
        (float(center[1]) - float(face_box.y)) / float(face_box.height),
    )


def _project_face_anchor(anchor, face_box: FaceBox):
    if anchor is None or face_box.width <= 1 or face_box.height <= 1:
        return None
    try:
        u, v = float(anchor[0]), float(anchor[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not np.isfinite(u) or not np.isfinite(v):
        return None
    return (
        float(face_box.x) + u * float(face_box.width),
        float(face_box.y) + v * float(face_box.height),
    )


class FacialCandidateDetector:
    def __init__(
        self,
        face_locator: FaceLocator,
        candidate_params: Optional[DetectParams] = None,
        params: Optional[FacialCandidateParams] = None,
    ) -> None:
        self.face_locator = face_locator
        base_params = candidate_params or DetectParams()
        self.candidate_params = DetectParams(
            min_area=base_params.min_area,
            max_area=base_params.max_area,
            min_circularity=base_params.min_circularity,
            min_solidity=base_params.min_solidity,
            max_aspect_ratio=base_params.max_aspect_ratio,
            blackhat_ksize=base_params.blackhat_ksize,
            min_contrast=base_params.min_contrast,
            skin_dilate=base_params.skin_dilate,
            use_skin_mask=False,
        )
        self.params = params or FacialCandidateParams()
        self._cached_face = None
        self._face_frames_left = 0
        self._candidate_miss_streak = 0
        self._face_mask_baseline = None
        self._face_mask_pending = None
        self._face_mask_pending_count = 0
        self._candidate_anchor = None
        self._candidate_anchor_misses = 0

    def reset(self) -> None:
        """Start a new view without carrying a stale face or mask baseline."""
        self._cached_face = None
        self._face_frames_left = 0
        self._candidate_miss_streak = 0
        self._face_mask_baseline = None
        self._face_mask_pending = None
        self._face_mask_pending_count = 0
        self._candidate_anchor = None
        self._candidate_anchor_misses = 0

    def detect(
        self,
        frame_bgr: np.ndarray,
        render: bool = True,
    ) -> FacialCandidateResult:
        if frame_bgr is None or frame_bgr.size == 0:
            raise ValueError("empty frame")
        if self._cached_face is None or self._face_frames_left <= 0:
            self._cached_face = self.face_locator.locate(frame_bgr)
            self._face_frames_left = max(
                0,
                int(self.params.face_refresh_frames) - 1,
            )
        else:
            self._face_frames_left -= 1
        face = self._cached_face
        height, width = frame_bgr.shape[:2]
        empty = np.zeros((height, width), dtype=np.uint8)
        if face is None:
            # Do not wait out the normal cache interval after a failed face
            # lookup.  The next frame may already contain a usable face.
            self._face_frames_left = 0
            self._candidate_miss_streak = 0
            original = frame_bgr.copy()
            return FacialCandidateResult(
                face=None,
                candidates=[],
                face_mask=empty.copy(),
                valid_mask=empty.copy(),
                excluded_masks={},
                annotated=original,
                privacy_view=(
                    _blur_all(frame_bgr, self.params.privacy_blur_sigma)
                    if render
                    else original
                ),
                message="NO_FACE",
            )

        face_mask, valid_mask, excluded = build_facial_masks(
            frame_bgr.shape,
            face,
            self.params.face_edge_margin_ratio,
            frame_bgr=frame_bgr,
            detection_exclusion_mode=self.params.detection_exclusion_mode,
        )
        if self.params.partial_face_guard:
            mask_ok, mask_message = self._check_face_mask_quality(
                face_mask,
                frame_bgr.shape,
            )
        else:
            mask_ok, mask_message = True, ""
        if not mask_ok:
            # A one-eye/forehead Haar false positive can produce a plausible
            # skin hull, but it is not a safe coordinate system for a mole.
            # Fail closed for this frame, discard only the bad locator cache,
            # and retain the selected-view baseline for the next re-location.
            baseline_pending = mask_message.startswith("FACE_BASELINE_PENDING")
            if not baseline_pending:
                self._cached_face = None
                self._face_frames_left = 0
                self._candidate_miss_streak = 0
            original = frame_bgr.copy()
            return FacialCandidateResult(
                face=None,
                candidates=[],
                face_mask=empty.copy(),
                valid_mask=empty.copy(),
                excluded_masks={},
                annotated=original,
                privacy_view=(
                    _blur_all(frame_bgr, self.params.privacy_blur_sigma)
                    if render
                    else original
                ),
                message=mask_message,
            )
        detection_roi = _mask_bounding_roi(valid_mask) or face.roi
        candidates = detect_moles(
            frame_bgr,
            params=_candidate_params_for_face(
                self.candidate_params,
                face,
                image_width=width,
            ),
            roi=detection_roi,
            valid_mask=valid_mask,
        )[: max(1, int(self.params.max_candidates))]
        if bool(self.params.enhanced_candidate_filters):
            candidates = _reject_eye_region_candidates(candidates, excluded)
            candidates = _reject_lower_boundary_noise_candidates(candidates, face_mask)
        anchor_exclusion = (
            self._anchor_exclusion_reason(face_mask, valid_mask, excluded)
            if not candidates and self._candidate_anchor is not None
            else None
        )
        candidates = self._recover_candidate_continuity(
            frame_bgr,
            face,
            face_mask,
            valid_mask,
            excluded,
            candidates,
        )[: max(1, int(self.params.max_candidates))]
        if bool(self.params.enhanced_candidate_filters):
            candidates = _reject_eye_region_candidates(candidates, excluded)
            candidates = _reject_lower_boundary_noise_candidates(candidates, face_mask)
        if candidates:
            self._candidate_miss_streak = 0
        else:
            # One motion-blurred frame must still publish no candidate, but it
            # must not immediately discard an otherwise trustworthy full-head
            # ROI.  Re-localizing after every isolated miss allowed Haar to
            # lock onto one eye and cache that small box for several frames.
            # Two consecutive misses still force a prompt refresh when the
            # head or arm genuinely moves out of the cached ROI.
            self._candidate_miss_streak += 1
            refresh_misses = max(1, int(self.params.face_refresh_misses))
            if self._candidate_miss_streak >= refresh_misses:
                self._face_frames_left = 0
                self._candidate_miss_streak = 0
        if render:
            annotated = draw_facial_candidates(frame_bgr, face, candidates, valid_mask)
            privacy_view = build_privacy_view(
                frame_bgr,
                face_mask,
                excluded,
                candidates,
                self.params,
            )
        else:
            # Qt applies temporal candidate filtering before it draws. Avoid
            # rendering raw candidates only to discard and redraw them in the
            # same frame.
            annotated = frame_bgr
            privacy_view = frame_bgr
        return FacialCandidateResult(
            face=face,
            candidates=candidates,
            face_mask=face_mask,
            valid_mask=valid_mask,
            excluded_masks=excluded,
            annotated=annotated,
            privacy_view=privacy_view,
            message=(
                "RAW_EMPTY excluded={} via {}".format(
                    anchor_exclusion,
                    face.source,
                )
                if not candidates and anchor_exclusion is not None
                else "{} candidates via {}".format(len(candidates), face.source)
            ),
        )

    def _recover_candidate_continuity(
        self,
        frame_bgr,
        face,
        face_mask,
        valid_mask,
        excluded_masks,
        strict_candidates,
    ):
        """Normalize a previously seen mole from fresh pixels in a local ROI.

        The global pass remains deliberately strict so triptych capture does
        not gain extra cheek/background candidates.  A strict hit seeds a
        face-relative anchor; subsequent frames may use relaxed *shape* gates
        only near that anchor.  An empty local result remains empty for the
        frame, so the serial layer still publishes ``valid=0`` on real loss.
        """

        dedup_radius = max(1, int(self.params.candidate_dedup_radius_px))
        strict = _deduplicate_candidates(strict_candidates, dedup_radius)
        if not bool(self.params.candidate_recovery_enabled):
            return strict

        mask_box = _face_box_from_mask(face_mask)
        if mask_box is None:
            self._candidate_anchor_misses += 1
            if self._candidate_anchor_misses > max(
                0,
                int(self.params.candidate_recovery_max_misses),
            ):
                self._candidate_anchor = None
            return strict

        had_anchor = self._candidate_anchor is not None
        predicted = _project_face_anchor(self._candidate_anchor, mask_box)
        if predicted is None and strict:
            predicted = strict[0].center

        recovered = None
        recovery_radius = max(12, int(self.params.candidate_recovery_radius_px))
        if predicted is not None:
            px, py = predicted
            height, width = frame_bgr.shape[:2]
            x0 = max(0, int(round(px)) - recovery_radius)
            y0 = max(0, int(round(py)) - recovery_radius)
            x1 = min(width, int(round(px)) + recovery_radius + 1)
            y1 = min(height, int(round(py)) + recovery_radius + 1)
            if x1 > x0 and y1 > y0:
                base_params = _candidate_params_for_face(
                    self.candidate_params,
                    face,
                    image_width=width,
                )
                recovery_params = DetectParams(
                    min_area=base_params.min_area,
                    max_area=base_params.max_area,
                    min_circularity=min(0.32, base_params.min_circularity),
                    min_solidity=min(0.60, base_params.min_solidity),
                    max_aspect_ratio=max(2.8, base_params.max_aspect_ratio),
                    blackhat_ksize=base_params.blackhat_ksize,
                    min_contrast=base_params.min_contrast,
                    skin_dilate=base_params.skin_dilate,
                    use_skin_mask=False,
                )
                local = detect_moles(
                    frame_bgr,
                    params=recovery_params,
                    roi=(x0, y0, x1 - x0, y1 - y0),
                    valid_mask=(
                        _tracked_candidate_recovery_mask(
                            face_mask,
                            valid_mask,
                            excluded_masks,
                            face,
                            self.params.face_edge_margin_ratio,
                        )
                        if had_anchor
                        else valid_mask
                    ),
                )
                match_radius = min(24.0, float(recovery_radius))
                nearby = [
                    item
                    for item in local
                    if float(
                        np.hypot(
                            item.center[0] - px,
                            item.center[1] - py,
                        )
                    )
                    <= match_radius
                ]
                if nearby:
                    recovered = max(nearby, key=_candidate_score)

        if recovered is not None:
            # Replace weak fragments around the same mark with one strongest
            # current-frame contour.  This prevents two pieces of one printed
            # mole from creating competing tracker IDs.
            kept = [
                item
                for item in strict
                if float(
                    np.hypot(
                        item.center[0] - recovered.center[0],
                        item.center[1] - recovered.center[1],
                    )
                )
                > max(
                    float(dedup_radius),
                    float(item.radius + recovered.radius + 3),
                )
            ]
            candidates = _deduplicate_candidates(
                [recovered] + kept,
                dedup_radius,
            )
            self._candidate_anchor = _face_relative_anchor(
                recovered.center,
                mask_box,
            )
            self._candidate_anchor_misses = 0
            return candidates

        if strict:
            self._candidate_anchor = _face_relative_anchor(
                strict[0].center,
                mask_box,
            )
            self._candidate_anchor_misses = 0
            return strict

        self._candidate_anchor_misses += 1
        if self._candidate_anchor_misses > max(
            0,
            int(self.params.candidate_recovery_max_misses),
        ):
            self._candidate_anchor = None
        return []

    def _anchor_exclusion_reason(self, face_mask, valid_mask, excluded_masks):
        """Describe the mask under the face-relative continuity anchor."""

        mask_box = _face_box_from_mask(face_mask)
        predicted = _project_face_anchor(self._candidate_anchor, mask_box)
        if predicted is None:
            return "face_edge"
        height, width = face_mask.shape[:2]
        x = int(round(predicted[0]))
        y = int(round(predicted[1]))
        if x < 0 or x >= width or y < 0 or y >= height:
            return "face_edge"

        # Report anatomical guards before the broad upper-edge band when the
        # masks overlap.  Those guards remain fail-closed during recovery.
        names = tuple(excluded_masks or {})
        priorities = (
            tuple(name for name in names if name.endswith("_eye_guard")),
            tuple(name for name in names if name.endswith("_eyebrow_guard")),
            tuple(name for name in names if name.endswith("_eye")),
            tuple(name for name in names if name.endswith("_eyebrow")),
            tuple(name for name in names if name in ("nostrils", "mouth")),
            tuple(name for name in names if name == "upper_edge_guard"),
        )
        for group in priorities:
            for name in group:
                mask = excluded_masks.get(name)
                if (
                    mask is not None
                    and int(mask[y, x]) > 0
                    and int(valid_mask[y, x]) == 0
                ):
                    if name.endswith("_eye_guard"):
                        return "eye_guard"
                    if name.endswith("_eyebrow_guard"):
                        return "brow_guard"
                    if name.endswith("_eye"):
                        return "eye"
                    if name.endswith("_eyebrow"):
                        return "eyebrow"
                    return name
        if int(face_mask[y, x]) == 0 or int(valid_mask[y, x]) == 0:
            return "face_edge"
        return "none"

    def _check_face_mask_quality(self, face_mask, image_shape):
        """Reject a face silhouette that collapses relative to this view."""
        mask_box = _face_box_from_mask(face_mask)
        image_height, image_width = image_shape[:2]
        if mask_box is None:
            return False, "PARTIAL_FACE empty mask"

        mask_area = int(cv2.countNonZero(face_mask))
        frame_area = float(max(1, image_width * image_height))
        if self._face_mask_baseline is None:
            width_ratio = float(mask_box.width) / max(1.0, float(image_width))
            height_ratio = float(mask_box.height) / max(1.0, float(image_height))
            area_ratio = float(mask_area) / frame_area
            if (
                width_ratio < float(self.params.face_baseline_min_width_ratio)
                or height_ratio < float(self.params.face_baseline_min_height_ratio)
                or area_ratio < float(self.params.face_baseline_min_area_ratio)
            ):
                return (
                    False,
                    "PARTIAL_FACE baseline rejected w={:.2f} h={:.2f} area={:.2f}".format(
                        width_ratio,
                        height_ratio,
                        area_ratio,
                    ),
                )
            current = (
                int(mask_box.width),
                int(mask_box.height),
                int(mask_area),
            )
            required_frames = max(1, int(self.params.face_baseline_frames))
            if self._face_mask_pending is None:
                self._face_mask_pending = current
                self._face_mask_pending_count = 1
            else:
                pending_width, pending_height, pending_area = self._face_mask_pending
                self._face_mask_pending = (
                    max(int(pending_width), current[0]),
                    max(int(pending_height), current[1]),
                    max(int(pending_area), current[2]),
                )
                self._face_mask_pending_count += 1
            if self._face_mask_pending_count < required_frames:
                return (
                    False,
                    "FACE_BASELINE_PENDING {}/{}".format(
                        self._face_mask_pending_count,
                        required_frames,
                    ),
                )
            self._face_mask_baseline = self._face_mask_pending
            self._face_mask_pending = None
            self._face_mask_pending_count = 0
            return True, ""

        baseline_width, baseline_height, baseline_area = self._face_mask_baseline
        width_ratio = float(mask_box.width) / max(1.0, float(baseline_width))
        height_ratio = float(mask_box.height) / max(1.0, float(baseline_height))
        area_ratio = float(mask_area) / max(1.0, float(baseline_area))
        if (
            width_ratio < float(self.params.face_mask_min_width_ratio)
            or height_ratio < float(self.params.face_mask_min_height_ratio)
            or area_ratio < float(self.params.face_mask_min_area_ratio)
        ):
            return (
                False,
                "PARTIAL_FACE collapsed w={:.2f} h={:.2f} area={:.2f}".format(
                    width_ratio,
                    height_ratio,
                    area_ratio,
                ),
            )

        # Freeze the bootstrap baseline.  Updating it from later close-up
        # frames could make the normal selected-view scale look artificially
        # small when the face moves back to its original distance.
        return True, ""

    def render_candidates(
        self,
        frame_bgr: np.ndarray,
        result: FacialCandidateResult,
        candidates: List[DetectedMole],
    ) -> FacialCandidateResult:
        """Replace raw candidates with temporally confirmed candidates and redraw."""
        if result.face is None:
            result.candidates = []
            result.annotated = frame_bgr.copy()
            result.privacy_view = _blur_all(
                frame_bgr,
                self.params.privacy_blur_sigma,
            )
            return result
        result.candidates = list(candidates)
        result.annotated = draw_facial_candidates(
            frame_bgr,
            result.face,
            result.candidates,
            result.valid_mask,
        )
        result.privacy_view = build_privacy_view(
            frame_bgr,
            result.face_mask,
            result.excluded_masks,
            result.candidates,
            self.params,
        )
        result.message = "{} stable candidates via {}".format(
            len(result.candidates),
            result.face.source,
        )
        return result


def _face_edge_valid_mask(
    face_mask: np.ndarray,
    feature_anchor: FaceBox,
    face: FaceBox,
    image_width: int,
    edge_margin_ratio: float,
) -> np.ndarray:
    """Return the normal skin-contour erosion used by every facial pass."""

    effective_edge_margin_ratio = float(edge_margin_ratio)
    side_closeup = _is_profile_face(face) or _is_side_closeup_face(
        face,
        image_width,
    )
    if side_closeup:
        # A cheek mole in a side view can legitimately sit only a few pixels
        # inside the visible skin contour.  The frontal 3.5% erosion removed
        # that real target in all recent left-view captures.
        effective_edge_margin_ratio = min(effective_edge_margin_ratio, 0.012)
    edge_margin = max(
        1,
        int(
            round(
                min(feature_anchor.width, feature_anchor.height)
                * effective_edge_margin_ratio
            )
        ),
    )
    edge_kernel_size = max(3, (edge_margin * 2 + 1) | 1)
    edge_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (edge_kernel_size, edge_kernel_size),
    )
    return cv2.erode(face_mask, edge_kernel, iterations=1)


def _tracked_candidate_recovery_mask(
    face_mask: np.ndarray,
    valid_mask: np.ndarray,
    excluded_masks,
    face: FaceBox,
    edge_margin_ratio: float,
) -> np.ndarray:
    """Restore only the upper-edge pixels near an already tracked target.

    The caller limits detection to the 32-pixel continuity ROI.  Keeping this
    separate from the global valid mask prevents a temple/forehead artifact
    from seeding a new target, while allowing a real cheek mark to survive a
    few pixels of face-mask movement after it has already been selected.
    Anatomical guards and the normal skin-contour erosion stay fail-closed.
    """

    upper_guard = (excluded_masks or {}).get("upper_edge_guard")
    mask_box = _face_box_from_mask(face_mask)
    if upper_guard is None or mask_box is None:
        return valid_mask
    feature_anchor = FaceBox(
        mask_box.x,
        mask_box.y,
        mask_box.width,
        mask_box.height,
        face.confidence,
        face.source,
    )
    base_edge_mask = _face_edge_valid_mask(
        face_mask,
        feature_anchor,
        face,
        face_mask.shape[1],
        edge_margin_ratio,
    )
    recoverable = cv2.bitwise_and(upper_guard, base_edge_mask)
    for name, mask in (excluded_masks or {}).items():
        if name == "upper_edge_guard" or mask is None:
            continue
        recoverable[mask > 0] = 0
    return cv2.bitwise_or(valid_mask, recoverable)


def build_facial_masks(
    image_shape,
    face: FaceBox,
    edge_margin_ratio: float = 0.035,
    frame_bgr: Optional[np.ndarray] = None,
    detection_exclusion_mode: str = "anatomical",
):
    height, width = image_shape[:2]
    face_mask = (
        _skin_face_mask(frame_bgr, face)
        if frame_bgr is not None
        else None
    )
    if face_mask is None:
        face_mask = _fallback_face_mask((height, width), face)
    mask_box = _face_box_from_mask(face_mask)
    feature_anchor = (
        FaceBox(
            mask_box.x,
            mask_box.y,
            mask_box.width,
            mask_box.height,
            face.confidence,
            face.source,
        )
        if mask_box is not None
        else face
    )

    side_closeup = _is_profile_face(face) or _is_side_closeup_face(face, width)
    eyes_only = str(detection_exclusion_mode).strip().lower().replace("-", "_") in (
        "eye",
        "eyes",
        "eye_only",
        "eyes_only",
    )
    valid_mask = _face_edge_valid_mask(
        face_mask,
        feature_anchor,
        face,
        width,
        edge_margin_ratio,
    )

    feature_regions = {
        "left_eyebrow": (0.04, 0.52, 0.22, 0.42),
        "right_eyebrow": (0.48, 0.96, 0.22, 0.42),
        "left_eye": (0.04, 0.52, 0.30, 0.56),
        "right_eye": (0.48, 0.96, 0.30, 0.56),
    }
    fallback_regions = {
        "left_eyebrow": (0.08, 0.50, 0.15, 0.32),
        "right_eyebrow": (0.50, 0.92, 0.15, 0.32),
        "left_eye": (0.08, 0.50, 0.26, 0.46),
        "right_eye": (0.50, 0.92, 0.26, 0.46),
    }
    if _is_skin_face(face):
        # The skin fallback encloses the whole visible head instead of the
        # tighter Haar/YOLO face box.  Distinguish a full head spanning the
        # image height from a cropped side close-up; combining both eye bands
        # into one large fallback would incorrectly hide a cheek mole.
        if _is_full_head_skin(feature_anchor, height):
            feature_regions.update(
                {
                    "left_eyebrow": (0.04, 0.52, 0.26, 0.47),
                    "right_eyebrow": (0.48, 0.96, 0.26, 0.47),
                    # A full-height skin box can start at the cropped scalp.
                    # Its eyes are then around y=0.35 rather than y=0.50.
                    # Search high enough to include the complete eye while
                    # stopping before a cheek mole around y=0.53.
                    "left_eye": (0.04, 0.52, 0.26, 0.52),
                    "right_eye": (0.48, 0.96, 0.26, 0.52),
                }
            )
            fallback_regions.update(
                {
                    "left_eyebrow": (0.08, 0.50, 0.28, 0.44),
                    "right_eyebrow": (0.50, 0.92, 0.28, 0.44),
                    # Keep the fail-closed fallback on the anatomical eye
                    # band.  The previous 0.38..0.56 ellipse and its guard
                    # reached the lower cheek in close-up frames.
                    # Keep the fallback eye mask tight in near full-head
                    # crops.  The live Qt close-up can produce a skin box
                    # covering the whole visible mannequin head; if this
                    # ellipse reaches y~=0.48, its guard swallows a real
                    # upper-cheek mole just below the eye.
                    "left_eye": (0.08, 0.50, 0.28, 0.44),
                    "right_eye": (0.50, 0.92, 0.28, 0.44),
                }
            )
        else:
            feature_regions.update(
                {
                    "left_eyebrow": (0.04, 0.52, 0.06, 0.28),
                    "right_eyebrow": (0.48, 0.96, 0.06, 0.28),
                    "left_eye": (0.04, 0.52, 0.16, 0.42),
                    "right_eye": (0.48, 0.96, 0.16, 0.42),
                }
            )
            fallback_regions.update(
                {
                    "left_eyebrow": (0.08, 0.50, 0.06, 0.24),
                    "right_eyebrow": (0.50, 0.92, 0.06, 0.24),
                    "left_eye": (0.08, 0.50, 0.16, 0.36),
                    "right_eye": (0.50, 0.92, 0.16, 0.36),
                }
            )
    excluded = {}
    if not side_closeup and not eyes_only:
        upper_edge_depth = max(
            5,
            int(round(feature_anchor.width * 0.072)),
        )
        upper_edge_inner = cv2.erode(
            face_mask,
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (upper_edge_depth * 2 + 1, upper_edge_depth * 2 + 1),
            ),
            iterations=1,
        )
        face_edge_band = cv2.subtract(face_mask, upper_edge_inner)
        upper_edge_guard = np.zeros_like(face_mask)
        upper_y0 = max(
            0,
            int(round(feature_anchor.y + feature_anchor.height * 0.20)),
        )
        upper_y1 = min(
            height,
            # Keep the temple/upper-eye edge fail-closed, but stop before the
            # lower cheek.  With the live Haar box, 0.58 intermittently
            # covered the real mark at relative y~=0.56 when the box moved by
            # only a few pixels.
            int(round(feature_anchor.y + feature_anchor.height * 0.52)),
        )
        if upper_y1 > upper_y0:
            upper_edge_guard[upper_y0:upper_y1] = face_edge_band[upper_y0:upper_y1]
        excluded["upper_edge_guard"] = upper_edge_guard
        valid_mask[upper_edge_guard > 0] = 0

    for name, ratios in feature_regions.items():
        reject_mask = None
        if name.endswith("_eye"):
            side = name.split("_", 1)[0]
            reject_mask = excluded.get("{}_eyebrow".format(side))
        zone_mask = (
            _extract_dark_feature_mask(
                frame_bgr,
                feature_anchor,
                ratios,
                name,
                reject_mask=reject_mask,
            )
            if frame_bgr is not None
            else None
        )
        if zone_mask is None:
            zone_mask = _relative_ellipse_mask(
                (height, width),
                face,
                fallback_regions[name],
            )
        if name.endswith("eyebrow"):
            brow_margin_x = max(4, int(round(feature_anchor.width * 0.016)))
            brow_margin_y = max(3, int(round(feature_anchor.height * 0.010)))
            zone_mask = cv2.dilate(
                zone_mask,
                cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE,
                    (brow_margin_x * 2 + 1, brow_margin_y * 2 + 1),
                ),
                iterations=1,
            )
        excluded[name] = zone_mask
        if not eyes_only or name.endswith("_eye"):
            valid_mask[zone_mask > 0] = 0

    for side in ("left", "right"):
        eye_mask = excluded["{}_eye".format(side)]
        eyebrow_mask = excluded["{}_eyebrow".format(side)]
        eye_points = np.where(eye_mask > 0)
        if len(eye_points[0]):
            eyebrow_mask[int(eye_points[0].min()) - 1 :] = 0
            if cv2.countNonZero(eyebrow_mask) < 8:
                eyebrow_mask = _relative_ellipse_mask(
                    (height, width),
                    face,
                    fallback_regions["{}_eyebrow".format(side)],
                )
                eyebrow_mask[int(eye_points[0].min()) - 1 :] = 0
                excluded["{}_eyebrow".format(side)] = eyebrow_mask
        if not eyes_only:
            valid_mask[eyebrow_mask > 0] = 0

        eyebrow_guard_margin_x = max(
            6,
            int(round(feature_anchor.width * 0.040)),
        )
        eyebrow_guard_margin_y = max(
            4,
            int(round(feature_anchor.height * 0.018)),
        )
        eyebrow_guard = cv2.dilate(
            eyebrow_mask,
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (
                    eyebrow_guard_margin_x * 2 + 1,
                    eyebrow_guard_margin_y * 2 + 1,
                ),
            ),
            iterations=1,
        )
        excluded["{}_eyebrow_guard".format(side)] = eyebrow_guard
        if not eyes_only:
            valid_mask[eyebrow_guard > 0] = 0

        if eyes_only:
            eye_guard_margin_x = max(
                5,
                int(round(feature_anchor.width * 0.030)),
            )
            eye_guard_margin_y = max(
                3,
                int(round(feature_anchor.height * 0.012)),
            )
            eye_guard_source = eye_mask
        else:
            eye_guard_margin_x = max(
                8,
                int(round(feature_anchor.width * 0.060)),
            )
            eye_guard_source = cv2.bitwise_or(eye_mask, eyebrow_mask)
        if not eyes_only and side_closeup:
            eye_guard_margin_y = max(
                3,
                int(round(feature_anchor.height * 0.012)),
            )
        elif not eyes_only:
            eye_guard_margin_y = max(
                5,
                int(round(feature_anchor.height * 0.025)),
            )
        eye_guard = cv2.dilate(
            eye_guard_source,
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (
                    eye_guard_margin_x * 2 + 1,
                    eye_guard_margin_y * 2 + 1,
                ),
            ),
            iterations=1,
        )
        if eyes_only:
            under_eye_shift = max(
                2,
                int(round(feature_anchor.height * 0.008)),
            )
        elif side_closeup:
            # Keep the eye itself fail-closed, but do not extend its privacy
            # mask far enough down the profile cheek to swallow the mole.
            under_eye_shift = max(
                1,
                int(round(feature_anchor.height * 0.006)),
            )
        else:
            under_eye_shift = max(
                4,
                int(round(feature_anchor.height * 0.025)),
            )
        shifted_guard = np.zeros_like(eye_guard)
        if under_eye_shift < height:
            shifted_guard[under_eye_shift:] = eye_guard[:-under_eye_shift]
        eye_guard = cv2.bitwise_or(eye_guard, shifted_guard)
        if eyes_only:
            response_halo = max(
                2,
                int(
                    round(
                        min(feature_anchor.width, feature_anchor.height) * 0.005
                    )
                ),
            )
        elif side_closeup:
            response_halo = max(
                2,
                int(
                    round(
                        min(feature_anchor.width, feature_anchor.height) * 0.005
                    )
                ),
            )
        else:
            response_halo = max(
                6,
                int(
                    round(
                        min(feature_anchor.width, feature_anchor.height) * 0.015
                    )
                ),
            )
        eye_guard = cv2.dilate(
            eye_guard,
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (response_halo * 2 + 1, response_halo * 2 + 1),
            ),
            iterations=1,
        )
        excluded["{}_eye_guard".format(side)] = eye_guard
        if not eyes_only:
            valid_mask[eye_guard > 0] = 0

    lower_feature_regions = _lower_feature_regions(feature_anchor, height)
    for name, ratios in lower_feature_regions.items():
        zone_mask = _relative_ellipse_mask((height, width), feature_anchor, ratios)
        excluded[name] = zone_mask
        if not eyes_only:
            valid_mask[zone_mask > 0] = 0

    return face_mask, valid_mask, excluded


def draw_facial_candidates(
    frame_bgr: np.ndarray,
    face: FaceBox,
    candidates: List[DetectedMole],
    valid_mask: np.ndarray,
) -> np.ndarray:
    shown = frame_bgr.copy()
    contours, _ = cv2.findContours(valid_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(shown, contours, -1, (80, 220, 80), 2)
    cv2.rectangle(
        shown,
        (face.x, face.y),
        (face.x + face.width, face.y + face.height),
        (255, 180, 0),
        2,
    )
    _draw_candidate_marks(shown, candidates)
    return shown


def _draw_candidate_marks(
    shown: np.ndarray,
    candidates: List[DetectedMole],
) -> None:
    for index, mole in enumerate(candidates, start=1):
        x, y = mole.center
        radius = max(10, int(mole.radius) + 7)
        color = (0, 70, 255) if index == 1 else (0, 190, 255)
        cv2.circle(shown, (x, y), radius, color, 2)
        cv2.drawMarker(shown, (x, y), color, cv2.MARKER_CROSS, 15, 2)
        cv2.putText(
            shown,
            "#{} X={} Y={}".format(index, x, y),
            (min(x + radius + 4, max(0, shown.shape[1] - 180)), max(22, y - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )


def _fallback_face_mask(image_size, face: FaceBox) -> np.ndarray:
    height, width = image_size
    mask = np.zeros((height, width), dtype=np.uint8)
    center = (
        int(round(face.x + face.width * 0.50)),
        int(round(face.y + face.height * 0.43)),
    )
    axes = (
        max(1, int(round(face.width * 0.52))),
        max(1, int(round(face.height * 0.66))),
    )
    cv2.ellipse(mask, center, axes, 0, 0, 360, 255, thickness=-1)
    return mask


def _skin_face_mask(
    frame_bgr: np.ndarray,
    face: FaceBox,
) -> Optional[np.ndarray]:
    """Extract the connected skin silhouette around a detected face/head."""
    image_height, image_width = frame_bgr.shape[:2]
    x0 = max(0, int(round(face.x - face.width * 0.18)))
    x1 = min(image_width, int(round(face.x + face.width * 1.18)))
    y0 = max(0, int(round(face.y - face.height * 0.38)))
    y1 = min(image_height, int(round(face.y + face.height * 1.10)))
    if x1 <= x0 or y1 <= y0:
        return None

    roi = frame_bgr[y0:y1, x0:x1]
    skin = build_skin_mask(roi)
    base_size = max(3, int(round(min(face.width, face.height) * 0.025)))
    kernel_size = (base_size * 2 + 1) | 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (kernel_size, kernel_size),
    )
    skin = cv2.morphologyEx(skin, cv2.MORPH_CLOSE, kernel, iterations=2)
    skin = cv2.morphologyEx(skin, cv2.MORPH_OPEN, kernel, iterations=1)

    contours = cv2.findContours(skin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return None

    expected_center = (
        float(face.x + face.width * 0.5 - x0),
        float(face.y + face.height * 0.5 - y0),
    )
    min_area = max(100.0, float(face.width * face.height) * 0.20)
    ranked = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        inside = cv2.pointPolygonTest(contour, expected_center, False) >= 0
        bx, by, bw, bh = cv2.boundingRect(contour)
        contour_center = (bx + bw * 0.5, by + bh * 0.5)
        distance = float(
            np.hypot(
                contour_center[0] - expected_center[0],
                contour_center[1] - expected_center[1],
            )
        )
        score = area * (2.0 if inside else 1.0) / (1.0 + distance * 0.01)
        ranked.append((score, contour))
    if not ranked:
        return None

    contour = max(ranked, key=lambda item: item[0])[1]
    hull = cv2.convexHull(contour)
    local_mask = np.zeros(skin.shape, dtype=np.uint8)
    cv2.drawContours(local_mask, [hull], -1, 255, thickness=-1)

    smooth_size = max(3, (kernel_size // 2) | 1)
    smooth_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (smooth_size, smooth_size),
    )
    local_mask = cv2.morphologyEx(
        local_mask,
        cv2.MORPH_CLOSE,
        smooth_kernel,
        iterations=1,
    )

    full_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    full_mask[y0:y1, x0:x1] = local_mask
    return full_mask


def build_privacy_view(
    frame_bgr: np.ndarray,
    face_mask: np.ndarray,
    excluded_masks: Dict[str, np.ndarray],
    candidates: List[DetectedMole],
    params: FacialCandidateParams,
) -> np.ndarray:
    if str(params.background_mode).lower() == "blur":
        shown = _blur_all(frame_bgr, params.privacy_blur_sigma)
    else:
        shown = np.zeros_like(frame_bgr)
    shown[face_mask > 0] = frame_bgr[face_mask > 0]
    for name in ("left_eye", "right_eye"):
        mask = excluded_masks.get(name)
        if mask is not None:
            shown = _blur_masked(shown, mask, params.eye_blur_sigma)
    contours = cv2.findContours(
        face_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]
    cv2.drawContours(shown, contours, -1, (0, 210, 255), 2)
    _draw_candidate_marks(shown, candidates)
    return shown


def _blur_all(frame_bgr: np.ndarray, sigma: float) -> np.ndarray:
    return cv2.GaussianBlur(frame_bgr, (0, 0), sigmaX=max(1.0, float(sigma)))


def _blur_masked(image: np.ndarray, mask: np.ndarray, sigma: float) -> np.ndarray:
    sigma = max(1.0, float(sigma))
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return image

    height, width = image.shape[:2]
    padding = max(3, int(round(sigma * 3.0)))
    x0 = max(0, int(xs.min()) - padding)
    x1 = min(width, int(xs.max()) + 1 + padding)
    y0 = max(0, int(ys.min()) - padding)
    y1 = min(height, int(ys.max()) + 1 + padding)
    if x1 <= x0 or y1 <= y0:
        return image

    patch = image[y0:y1, x0:x1]
    local_mask = mask[y0:y1, x0:x1]
    blurred = cv2.GaussianBlur(patch, (0, 0), sigmaX=sigma)
    feather = cv2.GaussianBlur(
        local_mask,
        (0, 0),
        sigmaX=max(1.0, sigma * 0.35),
    )
    alpha = feather.astype(np.float32)[:, :, None] / 255.0
    blended = np.clip(
        patch.astype(np.float32) * (1.0 - alpha)
        + blurred.astype(np.float32) * alpha,
        0,
        255,
    ).astype(np.uint8)
    result = image.copy()
    result[y0:y1, x0:x1] = blended
    return result


def _pixelate_masked(image: np.ndarray, mask: np.ndarray, block_size: int) -> np.ndarray:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return image
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    patch = image[y0:y1, x0:x1]
    if patch.size == 0:
        return image
    small_width = max(1, patch.shape[1] // block_size)
    small_height = max(1, patch.shape[0] // block_size)
    pixelated = cv2.resize(patch, (small_width, small_height), interpolation=cv2.INTER_LINEAR)
    pixelated = cv2.resize(
        pixelated,
        (patch.shape[1], patch.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )
    result = image.copy()
    local_mask = mask[y0:y1, x0:x1] > 0
    result_patch = result[y0:y1, x0:x1]
    result_patch[local_mask] = pixelated[local_mask]
    return result


def _extract_dark_feature_mask(
    frame_bgr: np.ndarray,
    face: FaceBox,
    ratios,
    feature_name: str,
    reject_mask: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """Extract one eye or eyebrow from a constrained upper-face search region."""
    image_height, image_width = frame_bgr.shape[:2]
    x0r, x1r, y0r, y1r = ratios
    x0 = max(0, int(round(face.x + face.width * x0r)))
    x1 = min(image_width, int(round(face.x + face.width * x1r)))
    y0 = max(0, int(round(face.y + face.height * y0r)))
    y1 = min(image_height, int(round(face.y + face.height * y1r)))
    if x1 - x0 < 8 or y1 - y0 < 5:
        return None

    gray = cv2.cvtColor(frame_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    median = float(np.median(gray))
    percentile = float(np.percentile(gray, 35.0))
    threshold_value = int(round(max(20.0, min(190.0, median - 16.0, percentile))))
    _, dark = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)

    horizontal = max(3, int(round(face.width * 0.018)) | 1)
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (horizontal, 3))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, close_kernel, iterations=1)
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )
    if reject_mask is not None:
        rejected = reject_mask[y0:y1, x0:x1]
        dark[rejected > 0] = 0

    contours = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    min_area = max(12.0, float(face.width * face.height) * 0.00035)
    ranked = []
    is_eye = feature_name.endswith("eye")
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        bx, by, bw, bh = cv2.boundingRect(contour)
        aspect = float(bw) / max(1.0, float(bh))
        if is_eye:
            # Keep dynamic eye masks inside the anatomical eye bands.  The
            # search ROI intentionally has some slack, but without this check
            # a dark, horizontal cheek mark can outrank a blurred eye and be
            # mistaken for the eye itself.  If no contour passes these strict
            # checks, build_facial_masks() falls back to the fixed eye ellipse,
            # so the real eye area remains fail-closed.
            center_x_ratio = (
                float(x0 + bx) + float(bw) * 0.5 - float(face.x)
            ) / max(1.0, float(face.width))
            center_y_ratio = (
                float(y0 + by) + float(bh) * 0.5 - float(face.y)
            ) / max(1.0, float(face.height))
            width_ratio = float(bw) / max(1.0, float(face.width))
            height_ratio = float(bh) / max(1.0, float(face.height))
            horizontal_bounds = (
                (0.12, 0.48)
                if feature_name.startswith("left_")
                else (0.52, 0.88)
            )
            eye_y_bounds = (
                (
                    (0.28, 0.52)
                    if _is_full_head_skin(face, image_height)
                    else (0.16, 0.40)
                )
                if _is_skin_face(face)
                else (0.30, 0.48)
            )
            if not (
                horizontal_bounds[0] <= center_x_ratio <= horizontal_bounds[1]
                and eye_y_bounds[0] <= center_y_ratio <= eye_y_bounds[1]
                and 0.09 <= width_ratio <= 0.38
                and 0.02 <= height_ratio <= 0.12
                and aspect >= 1.40
            ):
                continue
        min_width = face.width * (0.07 if is_eye else 0.10)
        max_width = face.width * 0.43
        max_height = face.height * (0.13 if is_eye else 0.10)
        if not (min_width <= bw <= max_width and 2 <= bh <= max_height):
            continue
        if aspect < (1.15 if is_eye else 1.45):
            continue
        contour_mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
        darkness = median - float(gray[contour_mask > 0].mean())
        if darkness < 10.0:
            continue
        ranked.append((area * (1.0 + min(aspect, 6.0) * 0.12) * darkness, contour))
    if not ranked:
        return None

    contour = max(ranked, key=lambda item: item[0])[1]
    bx, by, bw, bh = cv2.boundingRect(contour)
    mask = np.zeros((image_height, image_width), dtype=np.uint8)
    if is_eye:
        margin_x = max(3, int(round(face.width * 0.018)))
        margin_y = max(2, int(round(face.height * 0.007)))
        ex0 = max(x0, x0 + bx - margin_x)
        ex1 = min(x1, x0 + bx + bw + margin_x)
        ey0 = max(y0, y0 + by - margin_y)
        ey1 = min(y1, y0 + by + bh + margin_y)
        center = ((ex0 + ex1) // 2, (ey0 + ey1) // 2)
        axes = (max(1, (ex1 - ex0) // 2), max(1, (ey1 - ey0) // 2))
        cv2.ellipse(mask, center, axes, 0, 0, 360, 255, thickness=-1)
    else:
        margin_x = max(4, int(round(face.width * 0.018)))
        margin_y = max(2, int(round(face.height * 0.008)))
        ex0 = max(x0, x0 + bx - margin_x)
        ex1 = min(x1, x0 + bx + bw + margin_x)
        ey0 = max(y0, y0 + by - margin_y)
        ey1 = min(y1, y0 + by + bh + margin_y)
        center = ((ex0 + ex1) // 2, (ey0 + ey1) // 2)
        axes = (max(1, (ex1 - ex0) // 2), max(1, (ey1 - ey0) // 2))
        cv2.ellipse(mask, center, axes, 0, 0, 360, 255, thickness=-1)
    return mask


def _relative_ellipse_mask(image_size, face: FaceBox, ratios) -> np.ndarray:
    height, width = image_size
    x0r, x1r, y0r, y1r = ratios
    x0 = int(round(face.x + face.width * x0r))
    x1 = int(round(face.x + face.width * x1r))
    y0 = int(round(face.y + face.height * y0r))
    y1 = int(round(face.y + face.height * y1r))
    center = ((x0 + x1) // 2, (y0 + y1) // 2)
    axes = (max(1, (x1 - x0) // 2), max(1, (y1 - y0) // 2))
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.ellipse(mask, center, axes, 0, 0, 360, 255, thickness=-1)
    return mask


def _clamp_face_box(
    image_shape,
    x: int,
    y: int,
    width: int,
    height: int,
    confidence: float,
    source: str,
) -> Optional[FaceBox]:
    image_height, image_width = image_shape[:2]
    x0 = max(0, min(image_width - 1, x))
    y0 = max(0, min(image_height - 1, y))
    x1 = max(x0 + 1, min(image_width, x + width))
    y1 = max(y0 + 1, min(image_height, y + height))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    return FaceBox(x0, y0, x1 - x0, y1 - y0, confidence, source)


def _face_box_from_mask(mask: np.ndarray) -> Optional[FaceBox]:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return None
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    return FaceBox(x0, y0, x1 - x0, y1 - y0, 0.0, "mask")


def _mask_bounding_roi(mask: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
    box = _face_box_from_mask(mask)
    return box.roi if box is not None else None


def _is_profile_face(face: FaceBox) -> bool:
    return "profile" in str(face.source).lower()


def _is_side_closeup_face(face: FaceBox, image_width: int) -> bool:
    """Recognize an off-centre side view even when Haar labels it frontal."""

    width = max(1.0, float(image_width))
    left_ratio = float(face.x) / width
    right_ratio = float(face.x + face.width) / width
    center_offset = abs(float(face.x + face.width * 0.5) - width * 0.5) / width
    touches_one_side = (left_ratio <= 0.08) != (right_ratio >= 0.92)
    source = str(face.source).strip().lower()
    if (
        source == "skin"
        and center_offset >= 0.08
        and float(face.width) / width >= 0.45
    ):
        return True
    return (
        touches_one_side
        and center_offset >= 0.12
        and float(face.width) / width >= 0.40
    )


def _is_skin_face(face: FaceBox) -> bool:
    return str(face.source).strip().lower() == "skin"


def _is_full_head_skin(face: FaceBox, image_height: int) -> bool:
    return _is_skin_face(face) and face.height >= max(1, int(image_height)) * 0.90


def _skin_face_dominates_detector(
    image_shape,
    detected_face: FaceBox,
    skin_face: Optional[FaceBox],
) -> bool:
    """Reject a small detector eye false positive inside a full skin head.

    OpenCV cascades and the fallback YOLO model can both classify one eye and
    the adjacent cheek as a face on this mannequin, especially in a
    motion-blurred frame.  The skin locator already applies its own area,
    aspect-ratio, and center checks.  Prefer that full-head box only when it is
    substantially larger and contains nearly all of the detector box; a
    normal full-face result and a genuine non-overlapping profile remain
    unchanged.
    """
    source = str(detected_face.source).strip().lower()
    if skin_face is None or not (source.startswith("haar") or source == "yolo"):
        return False

    detected_area = float(detected_face.width * detected_face.height)
    skin_area = float(skin_face.width * skin_face.height)
    image_height, image_width = image_shape[:2]
    image_area = float(max(1, image_height * image_width))
    if detected_area <= 0.0 or skin_area < detected_area * 2.5:
        return False
    if not 0.12 <= skin_area / image_area <= 0.90:
        return False

    intersection_x0 = max(detected_face.x, skin_face.x)
    intersection_y0 = max(detected_face.y, skin_face.y)
    intersection_x1 = min(
        detected_face.x + detected_face.width,
        skin_face.x + skin_face.width,
    )
    intersection_y1 = min(
        detected_face.y + detected_face.height,
        skin_face.y + skin_face.height,
    )
    intersection_area = float(
        max(0, intersection_x1 - intersection_x0)
        * max(0, intersection_y1 - intersection_y0)
    )
    return intersection_area / detected_area >= 0.80


def _skin_face_dominates_haar(
    image_shape,
    haar_face: FaceBox,
    skin_face: Optional[FaceBox],
) -> bool:
    """Backward-compatible wrapper for focused Haar callers/tests."""
    return _skin_face_dominates_detector(
        image_shape,
        detected_face=haar_face,
        skin_face=skin_face,
    )


def _skin_face_dominates_profile(
    image_shape,
    profile_face: FaceBox,
    skin_face: Optional[FaceBox],
) -> bool:
    """Backward-compatible wrapper for older callers and focused tests."""
    return _skin_face_dominates_haar(
        image_shape,
        haar_face=profile_face,
        skin_face=skin_face,
    )


def _candidate_params_for_face(
    params: DetectParams,
    face: FaceBox,
    image_width: Optional[int] = None,
) -> DetectParams:
    is_side_view = _is_profile_face(face) or (
        image_width is not None and _is_side_closeup_face(face, image_width)
    )
    if not is_side_view:
        return params
    return DetectParams(
        min_area=params.min_area,
        max_area=params.max_area,
        min_circularity=params.min_circularity,
        min_solidity=params.min_solidity,
        max_aspect_ratio=max(3.2, params.max_aspect_ratio),
        blackhat_ksize=params.blackhat_ksize,
        min_contrast=params.min_contrast,
        skin_dilate=params.skin_dilate,
        use_skin_mask=params.use_skin_mask,
    )


def _lower_feature_regions(face: FaceBox, image_height: Optional[int] = None):
    """Return pose-aware nose and mouth exclusion zones.

    A frontal nose mask sits in the middle of the face. Reusing that mask for a
    profile moves it onto the cheek and can hide a real dark-spot candidate.
    OpenCV's normal and flipped profile cascades also tell us which side the
    nose points toward, so keep the exclusion close to that side.
    """
    source = str(face.source).lower()
    if source == "haar-profile":
        return {
            "nostrils": (0.05, 0.48, 0.50, 0.64),
            "mouth": (0.08, 0.60, 0.68, 0.84),
        }
    if source == "haar-profile-flipped":
        return {
            "nostrils": (0.52, 0.95, 0.50, 0.64),
            "mouth": (0.40, 0.92, 0.68, 0.84),
        }
    if source == "skin" and image_height is not None and _is_full_head_skin(face, image_height):
        # A skin fallback can enclose the complete bald head, placing the
        # nose and mouth lower than in a tight Haar/YOLO face rectangle.
        return {
            "nostrils": (0.34, 0.66, 0.65, 0.79),
            "mouth": (0.18, 0.82, 0.80, 0.96),
        }
    return {
        "nostrils": (0.37, 0.63, 0.49, 0.66),
        "mouth": (0.24, 0.76, 0.66, 0.84),
    }

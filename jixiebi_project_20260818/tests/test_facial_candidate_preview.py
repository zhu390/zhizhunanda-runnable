import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

from jetson_nano_vision.jetson_vision import facial_candidate_preview as preview
from jetson_nano_vision.jetson_vision.vision.facial_candidate import FaceBox
from jetson_nano_vision.jetson_vision.vision.mole_detect import DetectedMole


class _FakeVideoWriter:
    def __init__(self):
        self.frames = []
        self.released = False

    def isOpened(self):
        return True

    def write(self, frame):
        self.frames.append(frame.copy())

    def release(self):
        self.released = True


class FacialCandidatePreviewDiagnosticTest(unittest.TestCase):
    @staticmethod
    def _result():
        return SimpleNamespace(
            face=FaceBox(12, 23, 210, 320, 0.88, "haar-profile"),
            message="2 candidates via haar-profile",
        )

    def test_record_dir_is_opt_in(self):
        self.assertEqual(preview.parse_args([]).record_dir, "")
        self.assertFalse(preview.parse_args([]).scan_mode)
        self.assertEqual(
            preview.parse_args(["--record-dir", "diagnostics/scan"]).record_dir,
            "diagnostics/scan",
        )
        scan_args = preview.parse_args(
            ["--scan-mode", "--scan-reference-alpha", "0.12"]
        )
        self.assertTrue(scan_args.scan_mode)
        self.assertAlmostEqual(scan_args.scan_reference_alpha, 0.12)

    def test_diagnostic_serializer_keeps_raw_and_track_fields_separate(self):
        raw = DetectedMole((101, 202), 7, 83.5, 0.72, 41.25)
        stable = DetectedMole(
            (103, 204),
            8,
            91.0,
            0.81,
            44.5,
            track_id=9,
            stable_count=6,
            required_count=4,
            track_age_s=0.42,
        )

        payload = preview.serialize_diagnostic_frame(
            frame_index=17,
            monotonic_s=1234.5,
            result=self._result(),
            raw_candidates=[raw],
            stable_candidates=[stable],
            tracking_box=(10, 20, 300, 400),
            detected_tracking_box=(12, 22, 280, 390),
        )

        self.assertEqual(payload["frame_index"], 17)
        self.assertEqual(payload["monotonic_s"], 1234.5)
        self.assertEqual(payload["face"]["source"], "haar-profile")
        self.assertEqual(payload["tracking_box"], [10, 20, 300, 400])
        self.assertEqual(
            payload["detected_tracking_box"],
            [12, 22, 280, 390],
        )
        self.assertEqual(
            set(payload["raw_candidates"][0]),
            {"center", "radius", "area", "circularity", "contrast"},
        )
        self.assertEqual(payload["raw_candidates"][0]["center"], [101, 202])
        self.assertEqual(payload["stable_candidates"][0]["track_id"], 9)
        self.assertEqual(payload["stable_candidates"][0]["stable_count"], 6)
        self.assertEqual(payload["stable_candidates"][0]["required_count"], 4)
        self.assertEqual(payload["stable_candidates"][0]["track_age_s"], 0.42)
        self.assertEqual(payload["message"], "2 candidates via haar-profile")
        json.dumps(payload)

    def test_recorder_uses_actual_frame_size_and_releases_both_outputs(self):
        writer = _FakeVideoWriter()
        writer_calls = []

        def make_writer(path, fourcc, fps, frame_size):
            writer_calls.append((path, fourcc, fps, frame_size))
            return writer

        raw = DetectedMole((11, 22), 4, 20.0, 0.75, 33.0)
        frame = np.full((48, 64, 3), 37, dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            preview.cv2,
            "VideoWriter_fourcc",
            return_value=123,
        ), mock.patch.object(preview.cv2, "VideoWriter", side_effect=make_writer):
            recorder = preview.PreviewDiagnosticRecorder(directory, requested_fps=30)
            json_stream = recorder._jsonl
            recorder.record(
                frame,
                frame_index=0,
                monotonic_s=77.25,
                result=self._result(),
                raw_candidates=[raw],
                stable_candidates=[],
            )
            recorder.close()
            recorder.close()

            self.assertEqual(writer_calls[0][2:], (30.0, (64, 48)))
            self.assertEqual(
                Path(writer_calls[0][0]),
                Path(directory) / preview.PreviewDiagnosticRecorder.VIDEO_FILENAME,
            )
            self.assertEqual(len(writer.frames), 1)
            np.testing.assert_array_equal(writer.frames[0], frame)
            self.assertTrue(writer.released)
            self.assertTrue(json_stream.closed)

            jsonl_path = (
                Path(directory) / preview.PreviewDiagnosticRecorder.JSONL_FILENAME
            )
            lines = jsonl_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 1)
            payload = json.loads(lines[0])
            self.assertEqual(payload["frame_index"], 0)
            self.assertEqual(payload["raw_candidates"][0]["center"], [11, 22])

    def test_main_releases_camera_and_recorder_when_detection_raises(self):
        args = SimpleNamespace(
            stable_frames=4,
            stable_radius=6.0,
            view="algorithm",
            save="",
            image="",
            camera="camera0",
            width=640,
            height=480,
            fps=30,
            record_dir="diagnostics/session",
            flip="none",
        )
        camera = mock.Mock()
        camera.read.return_value = (
            True,
            np.zeros((48, 64, 3), dtype=np.uint8),
        )
        detector = mock.Mock()
        detector.detect.side_effect = RuntimeError("synthetic detector failure")
        recorder = mock.Mock()
        recorder.video_path = Path("diagnostics/session/raw_bgr_mjpg.avi")
        recorder.jsonl_path = Path("diagnostics/session/frames.jsonl")

        with mock.patch.object(preview, "parse_args", return_value=args), mock.patch.object(
            preview,
            "build_detector",
            return_value=detector,
        ), mock.patch.object(preview, "CandidateStabilityTracker"), mock.patch.object(
            preview,
            "open_camera",
            return_value=(camera, "fake camera"),
        ), mock.patch.object(
            preview,
            "PreviewDiagnosticRecorder",
            return_value=recorder,
        ), mock.patch.object(preview.cv2, "destroyAllWindows"):
            with self.assertRaisesRegex(RuntimeError, "synthetic detector failure"):
                preview.main()

        camera.release.assert_called_once_with()
        recorder.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()

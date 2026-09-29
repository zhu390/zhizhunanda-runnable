import os
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtGui import QColor, QImage

from arm_mole_qt.app import main
from jetson_vision.vision.facial_candidate import FaceBox
from jetson_vision.vision.mole_detect import DetectParams, DetectedMole


class TriptychDetectionWorkerTest(unittest.TestCase):
    def test_each_detection_resets_facial_detector_state(self):
        worker = main.TriptychDetectionWorker.__new__(main.TriptychDetectionWorker)
        worker._facial_detector = mock.Mock()
        worker.detector_mode = "facial_candidate"
        worker.spot_params = DetectParams()
        worker.spot_roi = (0.0, 1.0, 0.0, 1.0)
        frame = mock.Mock()

        with mock.patch.object(
            main,
            "_detect_candidates_on_bgr",
            return_value=([], "ok", frame),
        ) as detect:
            worker._detect_one(frame)
            worker._detect_one(frame)

        self.assertEqual(worker._facial_detector.reset.call_count, 2)
        self.assertEqual(detect.call_count, 2)

    def test_each_still_forces_a_fresh_face_location(self):
        image = QImage(640, 480, QImage.Format_RGB888)
        image.fill(QColor(205, 205, 205))
        images = [(1, image), (2, image), (3, image)]
        locator = mock.Mock()
        locator.locate.side_effect = (
            FaceBox(40, 30, 280, 390, 0.0, "skin"),
            FaceBox(160, 20, 320, 430, 0.0, "skin"),
            FaceBox(250, 40, 300, 400, 0.0, "skin"),
        )
        spot = DetectedMole((320, 300), 7, 100.0, 0.8, 50.0)
        worker = main.TriptychDetectionWorker(
            images=images,
            detector_mode="facial_candidate",
            spot_params=DetectParams(),
            spot_roi=(0.0, 1.0, 0.0, 1.0),
            facial_face_model=Path("unused.pt"),
            facial_face_conf=0.35,
            facial_face_device="",
            facial_max_candidates=12,
        )

        with mock.patch.object(main, "FaceLocator", return_value=locator) as locator_factory, mock.patch(
            "jetson_vision.vision.facial_candidate.detect_moles",
            return_value=[spot],
        ):
            worker.run()

        self.assertEqual(locator.locate.call_count, 3)
        locator_factory.assert_called_once_with(
            model_path=Path("unused.pt"),
            confidence=0.35,
            imgsz=416,
            device="",
        )


if __name__ == "__main__":
    unittest.main()

import unittest

import numpy as np

from arm_mole_qt.app.main import AstraRgbWorker


class _RecordingModel:
    def __init__(self):
        self.frame = None
        self.kwargs = None

    def predict(self, frame, **kwargs):
        self.frame = frame
        self.kwargs = kwargs
        return []


class AstraYoloWorkerTest(unittest.TestCase):
    def _worker(self, *, device=""):
        worker = AstraRgbWorker(
            enabled=True,
            camera="/dev/video-test",
            width=640,
            height=480,
            fps=30,
            ui_fps=15.0,
            flip="none",
            yolo_enabled=True,
            yolo_model="unused.pt",
            yolo_conf=0.35,
            yolo_iou=0.45,
            yolo_imgsz=640,
            yolo_device=device,
            yolo_arm_name="arm_tip",
            yolo_face_name="face",
            yolo_interval=0.15,
        )
        worker._yolo_arm_id = 7
        worker._yolo_face_id = 3
        return worker

    def test_predict_filters_required_classes_and_caps_duplicate_candidates(self):
        worker = self._worker(device="0")
        model = _RecordingModel()
        worker._yolo_model = model
        frame = np.zeros((24, 32, 3), dtype=np.uint8)

        _shown, _message, detections = worker._run_yolo(frame)

        self.assertEqual(detections, {})
        self.assertIs(model.frame, frame)
        self.assertEqual(model.kwargs["classes"], [3, 7])
        self.assertEqual(model.kwargs["max_det"], 10)
        self.assertEqual(model.kwargs["imgsz"], 640)
        self.assertEqual(model.kwargs["device"], "0")
        self.assertEqual(model.kwargs["conf"], 0.35)
        self.assertEqual(model.kwargs["iou"], 0.45)

    def test_predict_keeps_automatic_device_selection_when_device_is_empty(self):
        worker = self._worker(device="")
        model = _RecordingModel()
        worker._yolo_model = model

        worker._run_yolo(np.zeros((8, 8, 3), dtype=np.uint8))

        self.assertNotIn("device", model.kwargs)


if __name__ == "__main__":
    unittest.main()

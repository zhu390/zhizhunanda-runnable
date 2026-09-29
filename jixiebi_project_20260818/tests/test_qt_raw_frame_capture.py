import os
import tempfile
import unittest
from pathlib import Path


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtGui import QColor, QImage

from arm_mole_qt.app.main import ImageSample, MainWindow


class RawFrameCaptureTest(unittest.TestCase):
    class _Label:
        def __init__(self):
            self.text = ""

        def setText(self, text):
            self.text = str(text)

    @staticmethod
    def _capture_window(end_image, end_time, preview_image, preview_time):
        window = MainWindow.__new__(MainWindow)
        window._latest_frame_image = end_image
        window._latest_frame_image_time = float(end_time)
        window._latest_rgb_image = preview_image
        window._latest_rgb_image_time = float(preview_time)
        window._capture_images = {1: None, 2: None, 3: None}
        window._capture_session_dir = None
        window._capture_file_paths = {}
        window._capture_persistence_errors = {}
        window.face_image_labels = {}
        window.face_state_labels = {}
        window.capture_progress_value = RawFrameCaptureTest._Label()
        window.capture_states = []
        window.events = []
        window._set_face_capture_state = (
            lambda point_id, text, clear_image: window.capture_states.append(
                (int(point_id), str(text), bool(clear_image))
            )
        )
        window._log_event = window.events.append
        window._set_status = lambda text: setattr(window, "status", str(text))
        return window

    def test_display_uses_rendered_image_but_capture_cache_keeps_raw_image(self):
        displayed = QImage(2, 2, QImage.Format_RGB888)
        displayed.fill(QColor(255, 0, 0))
        raw = QImage(2, 2, QImage.Format_RGB888)
        raw.fill(QColor(0, 255, 0))

        window = MainWindow.__new__(MainWindow)
        window.video_label = object()
        shown_images = []
        capture_times = []
        window._set_label_image = (
            lambda _label, image, _empty_text, **_kwargs: shown_images.append(image)
        )
        window._capture_fresh_frame_if_ready = capture_times.append

        MainWindow._set_frame(
            window,
            ImageSample(
                image=displayed,
                sample_time=12.5,
                raw_image=raw,
            ),
        )

        self.assertEqual(shown_images[0].pixelColor(0, 0).getRgb()[:3], (255, 0, 0))
        self.assertEqual(
            window._latest_frame_image.pixelColor(0, 0).getRgb()[:3],
            (0, 255, 0),
        )
        self.assertEqual(window._latest_frame_image_time, 12.5)
        self.assertEqual(capture_times, [12.5])

    def test_formal_triptych_capture_uses_end_raw_frame_even_if_other_preview_is_newer(self):
        end_raw = QImage(2, 2, QImage.Format_RGB888)
        end_raw.fill(QColor(0, 255, 0))
        other_preview = QImage(2, 2, QImage.Format_RGB888)
        other_preview.fill(QColor(255, 0, 0))
        window = self._capture_window(
            end_image=end_raw,
            end_time=12.5,
            preview_image=other_preview,
            preview_time=99.0,
        )

        stored = MainWindow._store_face_capture(
            window,
            point_id=1,
            source_text="到位后首个新鲜帧",
            min_frame_time=12.0,
        )

        self.assertTrue(stored)
        self.assertEqual(
            window._capture_images[1].pixelColor(0, 0).getRgb()[:3],
            (0, 255, 0),
        )
        self.assertEqual(window.capture_progress_value.text, "已采集 1/3")

    def test_formal_triptych_capture_persists_raw_png_before_success(self):
        end_raw = QImage(2, 2, QImage.Format_RGB888)
        end_raw.fill(QColor(0, 255, 0))
        window = self._capture_window(
            end_image=end_raw,
            end_time=12.5,
            preview_image=None,
            preview_time=0.0,
        )

        with tempfile.TemporaryDirectory() as directory:
            window._capture_session_dir = Path(directory)
            stored = MainWindow._store_face_capture(
                window,
                point_id=3,
                source_text="到位后首个新鲜帧",
                min_frame_time=12.0,
            )

            expected = Path(directory) / "point_3_right_raw.png"
            self.assertTrue(stored)
            self.assertTrue(expected.is_file())
            self.assertEqual(window._capture_file_paths[3], expected)
            persisted = QImage(str(expected))
            self.assertFalse(persisted.isNull())
            self.assertEqual(persisted.pixelColor(0, 0).getRgb()[:3], (0, 255, 0))

    def test_disk_failure_keeps_memory_capture_and_allows_safe_home_sequence(self):
        end_raw = QImage(2, 2, QImage.Format_RGB888)
        end_raw.fill(QColor(0, 255, 0))
        window = self._capture_window(
            end_image=end_raw,
            end_time=12.5,
            preview_image=None,
            preview_time=0.0,
        )

        with tempfile.TemporaryDirectory() as directory:
            invalid_directory = Path(directory) / "not_a_directory"
            invalid_directory.write_text("occupied", encoding="utf-8")
            window._capture_session_dir = invalid_directory
            stored = MainWindow._store_face_capture(
                window,
                point_id=3,
                source_text="到位后首个新鲜帧",
                min_frame_time=12.0,
            )

        self.assertTrue(stored)
        self.assertIsNotNone(window._capture_images[3])
        self.assertNotIn(3, window._capture_file_paths)
        self.assertIn(3, window._capture_persistence_errors)
        self.assertIn("继续完成回 HOME", window.events[-1])

    def test_formal_triptych_capture_rejects_non_new_end_frame_even_if_other_preview_is_newer(self):
        end_raw = QImage(2, 2, QImage.Format_RGB888)
        end_raw.fill(QColor(0, 255, 0))
        other_preview = QImage(2, 2, QImage.Format_RGB888)
        other_preview.fill(QColor(255, 0, 0))

        for end_time in (11.9, 12.0):
            with self.subTest(end_time=end_time):
                window = self._capture_window(
                    end_image=end_raw,
                    end_time=end_time,
                    preview_image=other_preview,
                    preview_time=99.0,
                )

                stored = MainWindow._store_face_capture(
                    window,
                    point_id=1,
                    source_text="到位后首个新鲜帧",
                    min_frame_time=12.0,
                )

                self.assertFalse(stored)
                self.assertIsNone(window._capture_images[1])
                self.assertEqual(
                    window.capture_states,
                    [(1, "无可用画面", False)],
                )
                self.assertEqual(window.events, [])


if __name__ == "__main__":
    unittest.main()

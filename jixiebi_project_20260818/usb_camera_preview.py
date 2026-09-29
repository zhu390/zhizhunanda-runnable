import os
import sys
import time

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2


USB_CAMERA_INDEX = 1


def open_usb_camera():
    attempts = (
        (cv2.CAP_MSMF, "MSMF"),
        (cv2.CAP_DSHOW, "DirectShow"),
    )
    formats = (
        ("MJPG", 1920, 1080, 30),
        ("MJPG", 1280, 720, 30),
        ("MJPG", 1280, 720, 60),
        ("MJPG", 640, 480, 60),
        ("MJPG", 640, 480, 30),
        ("YUYV", 640, 480, 30),
    )

    for backend, backend_name in attempts:
        for fourcc, width, height, fps in formats:
            camera = cv2.VideoCapture(USB_CAMERA_INDEX, backend)
            if not camera.isOpened():
                camera.release()
                continue

            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            camera.set(cv2.CAP_PROP_FPS, fps)
            camera.set(cv2.CAP_PROP_AUTOFOCUS, 1)
            time.sleep(1.0)

            for _ in range(20):
                ok, frame = camera.read()
                if ok and frame is not None:
                    actual_width = frame.shape[1]
                    actual_height = frame.shape[0]
                    if (actual_width, actual_height) != (width, height):
                        print(
                            f"Rejected driver fallback: requested={width}x{height}, "
                            f"actual={actual_width}x{actual_height}"
                        )
                        break
                    actual_fourcc = int(camera.get(cv2.CAP_PROP_FOURCC))
                    actual_fourcc_text = "".join(
                        chr((actual_fourcc >> (8 * i)) & 0xFF) for i in range(4)
                    )
                    actual_fps = camera.get(cv2.CAP_PROP_FPS)
                    print(
                        f"Opened external USB camera index={USB_CAMERA_INDEX} "
                        f"backend={backend_name} requested={fourcc}/{width}x{height}@{fps} "
                        f"actual={actual_fourcc_text}/{frame.shape[1]}x{frame.shape[0]}"
                        f"@{actual_fps:.1f}"
                    )
                    return camera, frame
            camera.release()

    return None, None


def fit_for_preview(frame, max_width=1280, max_height=720):
    """Scale uniformly for display without changing the image aspect ratio."""
    height, width = frame.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    if scale >= 1.0:
        return frame
    preview_width = max(1, int(round(width * scale)))
    preview_height = max(1, int(round(height * scale)))
    return cv2.resize(
        frame,
        (preview_width, preview_height),
        interpolation=cv2.INTER_AREA,
    )


def main() -> int:
    camera, frame = open_usb_camera()
    if camera is None:
        print(
            "External USB Camera was found by Windows as VID_0BDA&PID_5883, "
            "but both MSMF and DirectShow failed to deliver frames.",
            file=sys.stderr,
        )
        return 1

    cv2.namedWindow("External USB Camera - YHTek", cv2.WINDOW_AUTOSIZE)
    while True:
        cv2.imshow("External USB Camera - YHTek", fit_for_preview(frame))
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        ok, next_frame = camera.read()
        if ok and next_frame is not None:
            frame = next_frame

    camera.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

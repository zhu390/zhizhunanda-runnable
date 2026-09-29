import argparse
import cv2


def main():
    parser = argparse.ArgumentParser(description="Preview Astra Pro Plus RGB camera.")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    print("opened:", cap.isOpened())
    print("width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("fps:", cap.get(cv2.CAP_PROP_FPS))

    if not cap.isOpened():
        raise RuntimeError("cannot open camera {}".format(args.camera))

    while True:
        ok, frame = cap.read()
        if not ok:
            print("read failed")
            break

        cv2.imshow("Astra Pro Plus RGB", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

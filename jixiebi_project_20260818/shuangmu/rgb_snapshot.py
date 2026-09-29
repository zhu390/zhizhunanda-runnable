import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

print("opened:", cap.isOpened())

frame = None
ok = False
for _ in range(20):
    ok, frame = cap.read()

cap.release()

if not ok or frame is None:
    raise RuntimeError("read failed")

cv2.imwrite("rgb_snapshot.jpg", frame)
print("saved: rgb_snapshot.jpg")
print("shape:", frame.shape)

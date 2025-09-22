import cv2
from ultralytics import YOLO

model = YOLO("best.pt")

cap = cv2.VideoCapture(0)

# Create a named window with NORMAL flag (helps on Windows)
cv2.namedWindow("YOLOv8 Detection", cv2.WINDOW_NORMAL)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(frame, conf=0.25, verbose=False)
    annotated_frame = results[0].plot()

    # Resize window if needed
    cv2.resizeWindow("YOLOv8 Detection", 800, 600)

    cv2.imshow("YOLOv8 Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

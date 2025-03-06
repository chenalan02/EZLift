import cv2
from picamera2 import Picamera2
from ultralytics import YOLO
import os
os.environ["DISPLAY"] = ":0"

model_path = "/home/pi/EZLift/256_edgetpu.tflite"

model = YOLO(model_path, task='detect')

picam2 = Picamera2()
picam2.preview_configuration.main.size = (3280, 2464)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
picam2.start(show_preview=False)

while True:
    # Capture frame-by-frame
    frame = picam2.capture_array()

    frame = cv2.resize(frame, (256, 256))


    # Run YOLO11 inference on the frame
    results = model.predict(frame, device="tpu:0", imgsz=256)
    # results = model.predict(frame)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the resulting frame
    # cv2.imshow("Camera", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()
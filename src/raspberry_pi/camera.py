import threading
import multiprocessing
import cv2
from picamera2 import Picamera2

def camera_thread(threading.Thread):
    def __init__(self, frame_queue, camera_index=0, sleep_time=0.1):
        threading.Thread.__init__(self)
        self.frame_queue = frame_queue
        self.camera = Picamera2(camera_index)
    camera = Picamera2()
    while True:
        frame = camera.read()
        # print(frame)
        # print(yolo.detect(frame))
        # print(yolo.detect(frame, 0.5))
        # print(yolo.detect(frame, 0.5, 0.5))
        # print(yolo.detect(frame, 0.5, 0.5, 0.5))
        # print(yolo.detect(frame, 0.5, 0.5, 0.5, 0.5))
        # print(yolo.detect(frame, 0.5, 0.5, 0.5, 0.5, 0.5))
        # print(yolo.detect(frame, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
        # print(yolo.detect(frame
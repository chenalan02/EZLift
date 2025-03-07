import threading
import multiprocessing
from multiprocessing.shared_memory import SharedMemory
import cv2
from ultralytics import YOLO
import time
import numpy as np
from collections import deque

class YOLOProcess(multiprocessing.Process):
    def __init__(self, shm_name, shape, dtype, lock, model_path, confidence_threshold=0.5):
        super().__init__()
        self.shared_memory = SharedMemory(name=shm_name)
        self.shared_frame  = np.ndarray(shape, dtype=dtype, buffer=self.shared_memory.buf)
        self.lock = lock
        self.model = YOLO(model_path, task='detect')
        self.confidence_threshold = confidence_threshold
        self.fps = 0
        print("cv_init")

    def run(self):
        fps_queue = deque(maxlen=10)
        while True:
            with self.lock:
                curr_time = time.time()

                frame = np.copy(self.shared_frame)
                results = self.model.predict(frame, device="tpu:0", imgsz=256, verbose=False)

                self.fps = 1.0 / (time.time() - curr_time)
                fps_queue.append(self.fps)
                avg_fps = sum(fps_queue) / len(fps_queue)
                print(f"FPS: {avg_fps:.2f}")
                ### do something with results

            time.sleep(0.02)

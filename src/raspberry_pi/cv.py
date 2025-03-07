import threading
import multiprocessing
from multiprocessing.shared_memory import SharedMemory
import cv2
from ultralytics import YOLO
import time
import numpy as np

class YOLOProcess(multiprocessing.Process):
    def __init__(self, shm_name, shape, dtype, lock, model_path, confidence_threshold=0.5, debug_monitor=False):
        super().__init__()
        self.shared_memory = SharedMemory(name=shm_name)
        self.shared_frame  = np.ndarray(shape, dtype=dtype, buffer=self.shared_memory.buf)
        self.lock = lock
        self.model = YOLO(model_path, task='detect')
        self.confidence_threshold = confidence_threshold
        self.debug_monitor = debug_monitor
        self.fps = 0
        print("cv_init")

    def run(self):
        while True:
            with self.lock:
                start_time = time.time()

                frame = np.copy(self.shared_frame)
                start_time = time.time()
                results = self.model.predict(frame, device="tpu:0", imgsz=256, verbose=False)

                if self.debug_monitor:
                    annotated_frame = results[0].plot()
                    cv2.putText(annotated_frame, f"FPS: {self.fps:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow("Camera", annotated_frame)

                end_time = time.time()
                self.fps = 1.0 / (end_time - start_time)
                # print(f"FPS: {self.fps:.2f}")
                ### do something with results

            time.sleep(0.05)

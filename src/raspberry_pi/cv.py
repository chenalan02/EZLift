import threading
import multiprocessing as mp
from multiprocessing.shared_memory import SharedMemory
import cv2
from ultralytics import YOLO
import time
import numpy as np
from collections import deque

class YOLOProcess(mp.Process):
    def __init__(self, cv_results_queue, shm_name, shape, dtype, lock, model_path, confidence_threshold=0.5):
        super().__init__()
        self.cv_results_queue = cv_results_queue
        self.shared_memory = SharedMemory(name=shm_name)
        self.shared_frame  = np.ndarray(shape, dtype=dtype, buffer=self.shared_memory.buf)
        self.lock = lock
        self.model = YOLO(model_path, task='detect')
        self.confidence_threshold = confidence_threshold
        self.fps = 0
        print("cv_init")

    def run(self):
        # fps_queue = deque(maxlen=10)
        while True:
            with self.lock:
                
                frame = np.copy(self.shared_frame)
                results = self.model.predict(frame, device="tpu:0", imgsz=256, verbose=False)[0]
                bboxes = results.boxes.xyxy.cpu().numpy()
                conf = results.conf.cpu().numpy()
                cls_id = results.boxes.cls.cpu().numpy()
                orig_img = results.orig_img
                self.cv_results_queue.put((bboxes, orig_img, conf, cls_id))

                # curr_time = time.time()
                # self.fps = 1.0 / (time.time() - curr_time)
                # fps_queue.append(self.fps)
                # avg_fps = sum(fps_queue) / len(fps_queue)
                # print(f"FPS: {avg_fps:.2f}")

            time.sleep(0.02)

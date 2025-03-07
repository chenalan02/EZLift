import threading
import multiprocessing as mp
from multiprocessing.shared_memory import SharedMemory
from picamera2 import Picamera2
import time
import os
import numpy as np

class CameraThread(threading.Thread):
    def __init__(self, shm_name, shape, dtype, lock):
        super().__init__()

        os.environ["DISPLAY"] = ":0"
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (3280, 2464)
        self.picam2.preview_configuration.main.format = "RGB888"
        self.picam2.preview_configuration.align()
        self.picam2.configure("preview")
        self.picam2.start(show_preview=False)

        self.shared_memory = SharedMemory(name=shm_name)
        self.shared_frame  = np.ndarray(shape, dtype=dtype, buffer=self.shared_memory.buf)
        self.lock = lock
        print("cam_init")

    def run(self):
        while True:
            frame = self.picam2.capture_array()
            with self.lock:
                np.copyto(self.shared_frame, frame)
                print(1)
            time.sleep(0.02)


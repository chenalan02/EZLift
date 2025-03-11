import numpy as np
import cv2
import math
import multiprocessing
import time
import queue

SHELF_HEIGHT = 0

class ControlsProcess(multiprocessing.Process):
    def __init__(self, cv_results_queue, cmd_queue):
        super().__init__()
        self.cv_results_queue = cv_results_queue
        self.command_queue = cmd_queue
        self.idle = True
        self.command = None

        self.serial_send_queue = queue.queue()
        self.serial_recv_queue = queue.queue()
        
        print("controls_init")

    def _get_bbox_error(self, bbox, frame_shape):
        frame_center = (frame_shape[1]//2, frame_shape[0]//2)
        bbox_center = ((bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2)
        return frame_center[0] - bbox_center[0], frame_center[1] - bbox_center[1]
    
    def _get_angle_error(self, bbox, frame_shape):
        frame_center = (frame_shape[1]//2, frame_shape[0]//2)
        bbox_center = ((bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2)
        return math.atan2(bbox_center[1] - frame_center[1], bbox_center[0] - frame_center[0])

    def run(self):
        while True:
            if self.idle:
                if not self.command_queue.empty():
                    self.command = self.command_queue.get()
                    self.idle = False
                time.sleep(0.05)

            elif self.command == "pickup":
                self.pickup()

            elif self.command == "drop":
                self.drop()
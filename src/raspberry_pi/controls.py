import numpy as np
import cv2
import math
import multiprocessing as mp
import threading
import time
import queue
import serial

import RPi.GPIO as GPIO
from queue import Empty
from collections import deque

ESTOP_BUTTON = 17

PALLET_CLS_ID = 0
ANGLE_THRESH = 3 # error in degrees
DIST_THRESH = 0.1 # error in ratio (pallet width compared to image width)
CENTER_THRESH = 10 # error in horizontal pixel s
INIT_DIST_TARGET = 0.6 # ratio to approach box on initial search loop
DIST_TARGET = 0.8 # ratio to approach box on subsequent search loops
ERROR_QUEUE_MAXLEN = 3
MIN_TURN_VOLTAGE = 50
MIN_FORWARD_VOLTAGE = 50
MIN_SIDEWAY_VOLTAGE = 50
CONF_THRESH = 0.45

class ControlsProcess(mp.Process):
    def __init__(self, cv_results_queue, cmd_queue):
        super().__init__()
        self.cv_results_queue = cv_results_queue
        self.command_queue = cmd_queue
        self.shutdown_event = mp.Event()
        
        self.initial_search_loop = True
        self.center_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
        self.dist_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
        self.angle_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)

        self.idle = True
        self.command = None
        self.action_phase = 'Turn' # Angle, Forwards, Sideways, Lift

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ESTOP_BUTTON, GPIO.IN)

        self.serial_send_queue = queue.Queue(maxsize=50)
        # self.serial_read_queue = queue.Queue()
        self.serial_port = serial.Serial("/dev/ttyACM0", 9600)
        
        print("controls_init")


    def _hough_line_detection(self, img, canny_low=150, canny_high=450, hough_thresh=45, min_len=20, max_gap=7):
        # Get edges using Canny
        edges = edges = cv2.Canny(img, canny_low, canny_high)

        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 
                                rho=1,            # Distance resolution of accumulator in pixels
                                theta=np.pi / 180, # Angle resolution of accumulator in radians
                                threshold=hough_thresh, # Minimum number of votes (intersections in Hough grid cell)
                                minLineLength=min_len,  # Minimum length of line (pixels)
                                maxLineGap=max_gap)     # Maximum allowed gap between points on the same line
        return lines

    # Get the angle of the shelf relative to the camera
    def _get_angle_from_lines(self, lines, angle_thresh=2):

        lines_polar = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx = x2 - x1
                dy = y2 - y1
                angle_rad = math.atan2(-dy, dx)
                angle_deg = math.degrees(angle_rad)
                angle_deg = (angle_deg + 360) % 360
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                lines_polar.append((angle_deg, length))
        
        else:
            return 0

        lines_polar = sorted(lines_polar, key=lambda x: x[0])
        groups = []
        current_group = [lines_polar[0]]

        for angle, length in lines_polar[1:]:
            if abs(angle - current_group[-1][0]) < angle_thresh:
                current_group.append((angle, length))
            else:
                groups.append(current_group)
                current_group = [(angle, length)]
        groups.append(current_group)

        best_metric = 0
        best_angle = None
        for group in groups:
            avg_length = np.mean([length for _, length in group])
            if len(group) * avg_length**1.5 > best_metric:
                avg_angle = np.mean([angle for angle, _ in group])
                if not (avg_angle < 90 + angle_thresh and avg_angle > 90 - angle_thresh):
                    best_metric = len(group) * avg_length**1.5
                    best_angle = np.mean([angle for angle, _ in group])

        return best_angle

    def _get_angle_error(self, bbox, img):

        img_cropped = img[int(bbox[1]):int(bbox[3]), int(bbox[0]):int(bbox[2])]
        lines = self._hough_line_detection(img_cropped)
        angle = self._get_angle_from_lines(lines, angle_thresh=2)
        if angle > 180:
            angle -= 180
        if angle > 90:
            error = angle - 180
        else:
            error = angle
        return error

    def _get_distance_error(self, bbox, img, desired_bbox_ratio=0.5):

        bbox_ratio = abs(bbox[2] - bbox[0]) / img.shape[1]
        print(bbox_ratio)
        error = desired_bbox_ratio - bbox_ratio
        print(error)
        return error

    def _get_center_error(self, bbox, img):

        bbox_center = (bbox[0] + bbox[2]) / 2
        img_center = img.shape[1] / 2
        error = bbox_center - img_center
        return error

    def _PD_controller(self, errors, kp=5, kd=0.1):
        error = errors[0]
        # initial case
        derivative = errors[-1] - errors[-2] if len(errors) >= 2 else 0
        output = kp * error + kd * derivative
        return output

    def _get_most_conf_box(self, bbox, conf, cls_id):
        best_i = 0
        best_conf = 0
        for i in range(len(conf)):
            if cls_id[i] == PALLET_CLS_ID:
                if conf[i] > best_conf:
                    best_conf = conf[i]
                    best_i = i
        return bbox[best_i]


    def run(self):

        # serial_read_thread = threading.Thread(target=self.serial_read_queue)
        serial_write_thread = threading.Thread(target=self.serial_writer)
        # serial_read_thread.start()
        serial_write_thread.start()

        try:
            while True:
                # get cv results from queue
                if not self.cv_results_queue.empty():
                    bboxes, frame, conf, cls_id = self.cv_results_queue.get()
                    if len(bboxes) > 0:
                        if any([conf > CONF_THRESH for conf in conf]):
                            bbox = self._get_most_conf_box(bboxes, conf, cls_id)
                            empty_frame = False
                        else:
                            empty_frame = True
                    else:
                        empty_frame = True
                else:
                    empty_frame = True

                # check if estop pressed
                if GPIO.input(ESTOP_BUTTON) == 0:
                    print("Emergency stop button pressed")
                    self.idle = True
                    while not self.serial_send_queue.empty():
                        try:
                            self.serial_send_queue.get_nowait()
                        except queue.Empty:
                            break
                    self.serial_send_queue.put(f"stop\n")
                    time.sleep(1)

                # check command queue when idle
                if self.idle:
                    if not self.command_queue.empty():
                        print("recieved command")
                        self.command = self.command_queue.get()
                        if self.command.intent != None:
                            print(self.command.intent)
                            self.action_phase = 'Turn'
                            self.idle = False
                            self.initial_search_loop = True
                            # initialize errors to small values
                            self.center_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            self.dist_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            self.angle_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            self.center_errors.append(50)
                            self.dist_errors.append(10)
                            self.angle_errors.append(10)
                            time.sleep(1)
                        else:
                            print("Command Not Recognized")
                    
                # pickup box
                elif self.command.intent == "pickup":
                    if self.action_phase == 'Turn':
                        # get bbox error
                        if not empty_frame:
                            self.center_errors.append(self._get_center_error(bbox, frame))
                        # get PD controller output
                        turn_voltage = self._PD_controller(self.center_errors, kp=1, kd=0.1)
                        if turn_voltage > 0 and turn_voltage < MIN_TURN_VOLTAGE:
                            turn_voltage = MIN_TURN_VOLTAGE
                        if turn_voltage < 0 and turn_voltage > -MIN_TURN_VOLTAGE:
                            turn_voltage = -MIN_TURN_VOLTAGE
                        # send turn command
                        self.serial_send_queue.put(f"turn:{turn_voltage}\n")
                        # if not empty_frame:
                        #     self.serial_send_queue.put(f"turn:{turn_voltage}\n")
                        # else:
                        #     self.serial_send_queue.put(f"stop\n")
                        # check turn end condition
                        if all([abs(e) < CENTER_THRESH for e in self.center_errors]):
                            # self.center_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            self.serial_send_queue.put(f"stop\n")
                            self.action_phase = 'Forwards'
 
                    elif self.action_phase == 'Forwards':
                        if not empty_frame:
                            # error = self._get_distance_error(bbox, frame, desired_bbox_ratio=INIT_DIST_TARGET)
                            # self.dist_errors.append(error)
                            # different distance for first search loop
                            if self.initial_search_loop:
                                self.dist_errors.append(self._get_distance_error(bbox, frame, desired_bbox_ratio=INIT_DIST_TARGET))
                            else:
                                self.dist_errors.append(self._get_distance_error(bbox, frame, desired_bbox_ratio=DIST_TARGET))
                        # get PD controller output
                        forwards_voltage = self._PD_controller(self.dist_errors, kp=10, kd=1)
                        if forwards_voltage > 0 and forwards_voltage < MIN_FORWARD_VOLTAGE:
                            forwards_voltage = MIN_FORWARD_VOLTAGE
                        if forwards_voltage < 0 and forwards_voltage > -MIN_FORWARD_VOLTAGE:
                            forwards_voltage = -MIN_FORWARD_VOLTAGE
                        # send move command
                        self.serial_send_queue.put(f"forwards:{forwards_voltage}\n")
                        # check forward end condition
                        if all([abs(e) < DIST_THRESH for e in self.dist_errors]):
                            # self.dist_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            self.serial_send_queue.put(f"stop\n")
                            self.action_phase = 'Sideways'

                    elif self.action_phase == 'Sideways':
                        if not empty_frame:
                            self.angle_errors.append(self._get_angle_error(bbox, frame))
                        # get PD controller output
                        side_voltage = self._PD_controller(self.angle_errors, kp=1, kd=0.1)
                        if side_voltage > 0 and side_voltage < MIN_SIDEWAY_VOLTAGE:
                            side_voltage = MIN_SIDEWAY_VOLTAGE
                        if side_voltage < 0 and side_voltage > -MIN_SIDEWAY_VOLTAGE:
                            side_voltage = -MIN_SIDEWAY_VOLTAGE
                        # send move command
                        self.serial_send_queue.put(f"side:{side_voltage}\n")
                        # # check search end condition
                        if all([abs(e) < ANGLE_THRESH for e in self.angle_errors]) and all([abs(e) < CENTER_THRESH for e in self.center_errors]) and all([abs(e) < DIST_THRESH for e in self.dist_errors]):
                            # self.angle_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            # self.center_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            # self.dist_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            self.serial_send_queue.put(f"stop\n")
                            self.action_phase = 'Lift'
                        # check side end condition
                        elif all([abs(e) < ANGLE_THRESH for e in self.angle_errors]) or bbox[0] < 0 or bbox[2] > frame.shape[1]:
                            # self.angle_errors = deque(maxlen=ERROR_QUEUE_MAXLEN)
                            self.serial_send_queue.put(f"stop\n")
                            self.action_phase = 'Turn'
                            self.initial_search_loop = False
                    
                    elif self.action_phase == 'Lift':
                        print("Lifting box")
                        self.serial_send_queue.put(f"stop\n")
                        time.sleep(1)

                elif self.command == "drop":
                    pass

                time.sleep(0.05)
        
        except KeyboardInterrupt:
            print("Controls Process Interrupted by user, shutting down.")
        
        finally:
            print("[ControlsProcess] Shutting down...")
            self.shutdown_event.set()

            # Wait for threads to finish
            # serial_read_thread.join()
            serial_write_thread.join()

            # Close serial port
            self.serial_port.close()
            print("[ControlsProcess] Process exiting cleanly.")

    def serial_writer(self):
        """Reads commands from send queue and writes them to serial port."""
        while not self.shutdown_event.is_set():
            try:
                cmd = self.serial_send_queue.get(timeout=1)
                self.serial_port.write(cmd.encode())
                print(f"[Serial Writer] Sent: {cmd.strip()}")
                # print(self.serial_send_queue.qsize())
            except Empty:
                pass
            except Exception as e:
                print(f"[Serial Writer] Error: {e}")
                time.sleep(0.1)
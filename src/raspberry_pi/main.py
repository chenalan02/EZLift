import threading
import time
import multiprocessing as mp
import pickle
import cv2

from multiprocessing.shared_memory import SharedMemory
from queue import Empty

from camera import CameraThread
from controls import ControlsProcess
from cv import YOLOProcess
from voice import VoiceThread


if __name__ == "__main__":
    
    image_path = "/home/pi/EZLift/src/raspberry_pi/frame.pkl"
    with open(image_path, "rb") as f:
        frame = pickle.load(f)
    frame = cv2.resize(frame, (256, 256))

    shm = SharedMemory(create=True, size=frame.nbytes)
    lock = mp.Lock()
    cmd_queue = mp.Queue()
    cv_results_queue = mp.Queue()

    wake_model_path = "/home/pi/EZLift/src/raspberry_pi/Easy-Lift_en_raspberry-pi_v3_0_0.ppn"
    intent_model_path="/home/pi/EZLift/src/raspberry_pi/ezlift_en_raspberry-pi_v3_0_0.rhn"
    access_key="Dwi/yCZhuIbXGkKPNbK/vgoCuRjzFs9XrqZpEmD6mDLGVyYrFv4MuQ=="
    yolo_path = "/home/pi/EZLift/src/raspberry_pi/256_edgetpu.tflite"
    
    controls_proc= ControlsProcess(cv_results_queue, cmd_queue)
    camera_thread = CameraThread(shm.name, frame.shape, frame.dtype, lock)
    yolo_proc = YOLOProcess(cv_results_queue, shm.name, frame.shape, frame.dtype, lock, yolo_path)
    voice_thread = VoiceThread(cmd_queue, access_key, wake_model_path, intent_model_path)

    controls_proc.start()
    camera_thread.start()
    yolo_proc.start()
    voice_thread.start()

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")

    finally:
        print("Cleaning up resources...")
        if camera_thread.is_alive():
            camera_thread.join(timeout=1)

        # Terminate YOLO process
        if yolo_proc.is_alive():
            yolo_proc.terminate()
            yolo_proc.join(timeout=1)

        # Terminate Voice process
        if voice_thread.is_alive():
            voice_thread.terminate()
            voice_thread.join(timeout=1)

        # Terminate Controls process
        if controls_proc.is_alive():
            controls_proc.terminate()
            controls_proc.join(timeout=1)

        print("Closing shared memory resources...")
        shm.close()
        shm.unlink()
        print("Resources cleaned up.")

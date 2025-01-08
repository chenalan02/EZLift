import cv2
import numpy as np
import time
from ultralytics import YOLO

if __name__ == '__main__':
    model = YOLO("yolo11n.pt")
    results = model.train(data="model_training/datasets/combined_dataset/data.yaml", epochs=5, batch=16)
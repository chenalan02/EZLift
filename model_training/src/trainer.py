from ultralytics import YOLO

if __name__ == '__main__':
    # https://docs.ultralytics.com/modes/train/#train-settings

    model = YOLO("yolov8n.pt")
    data = "model_training/datasets/pallets_dataset_with_negatives/data.yaml"
    device = 0

    imgsz = 640
    epochs = 1
    batch = 16
    patience = 5
    lr0 = 0.01
    dropout = 0.0

    resume = False # resume training
    multi_scale = True # vary img-size 
    close_mosaic = 5 # disables mosaic in last N epochs
    pretrained = True
    exist_ok = False # True to overwrite run dir
    profile = True # enables profiling of ONNX and TensorRT
    plots = True

    results = model.train(data=data, imgsz=imgsz, batch=batch, epochs=epochs, device=device, lr0=lr0, dropout=dropout, resume=resume, exist_ok=exist_ok, multi_scale=multi_scale, close_mosaic=close_mosaic, pretrained=pretrained, profile=profile, plots=plots)
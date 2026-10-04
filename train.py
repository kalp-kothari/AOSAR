
from ultralytics import YOLO

def main():
    # Load a pretrained lightweight YOLOv8 model
    model = YOLO("yolov8n.pt")

    # Train the model
    model.train(
        data="data.yaml",
        epochs=50,
        imgsz=640,
        batch=8,
        patience=10,
        device="cpu",
        workers=2,
        project="runs",
        name="scroll_detection",
        save=True,
        plots=True
    )

if __name__ == "__main__":
    main()

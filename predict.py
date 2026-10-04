from ultralytics import YOLO

model = YOLO("runs/scroll_detection/weights/best.pt")

results = model.predict(
    source="test_image.jpg",
    conf=0.5,
    save=True
)
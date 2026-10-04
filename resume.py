
from ultralytics import YOLO

def main():
    model = YOLO(r"runs\detect\runs\scroll_detection\weights\last.pt")
    model.train(resume=True)

if __name__ == "__main__":
    main()

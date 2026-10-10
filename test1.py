import cv2
import os
import glob
from ultralytics import YOLO

# 1. Define paths
INPUT_DIR = "/Users/admin/Downloads/frames"      # Folder with your extracted .png frames
OUTPUT_DIR = "/Users/admin/Desktop/interdomain/codes/output_boxes" # Folder to save annotated images
os.makedirs(OUTPUT_DIR, exist_ok=True)

# 2. Load model
model = YOLO("yolov8n.pt")

# Target class keywords (COCO labels that match box/container types)
TARGET_CLASSES = ["box", "suitcase", "pack", "container"]

# 3. Get all .png images from the folder (sorted numerically/alphabetically)
image_paths = sorted(glob.glob(os.path.join(INPUT_DIR, "*.jpg")))

if not image_paths:
    print(f"No .jpg files found in {INPUT_DIR}")
    exit()

print(f"Found {len(image_paths)} images. Processing...")

# 4. Loop through each PNG image frame
for img_path in image_paths:
    frame = cv2.imread(img_path)
    if frame is None:
        continue

    filename = os.path.basename(img_path)
    results = model(frame)[0]

    # Filter detections to ONLY target box classes
    for box in results.boxes:
        class_id = int(box.cls[0])
        class_name = model.names[class_id]

        # Check if the detected object is a box
        if class_name.lower() in TARGET_CLASSES or "box" in class_name.lower():
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])

            # Draw green bounding box around the detected box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame, 
                f"Box {conf:.2f}", 
                (x1, max(y1 - 10, 20)), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.6, 
                (0, 255, 0), 
                2
            )

    # Save the annotated PNG image
    output_path = os.path.join(OUTPUT_DIR, filename)
    cv2.imwrite(output_path, frame)

print(f"Finished! Annotated images saved to: {OUTPUT_DIR}")
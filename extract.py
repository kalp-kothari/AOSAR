
import cv2
import os

video_folder = "videos"
output_folder = "frames"

os.makedirs(output_folder, exist_ok=True)

for filename in os.listdir(video_folder):
    if not filename.lower().endswith(".mp4"):
        continue

    video_path = os.path.join(video_folder, filename)
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Could not open:", filename)
        continue

    frame_no = 0
    saved = 0

    video_name = os.path.splitext(filename)[0]

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        output_path = os.path.join(
            output_folder,
            f"{video_name}_{saved:05d}.jpg"
        )

        cv2.imwrite(output_path, frame)
        saved += 1
        frame_no += 1

    cap.release()
    print(f"{filename}: extracted {saved} frames")

print("Done!")

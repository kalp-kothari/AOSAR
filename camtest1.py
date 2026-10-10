import cv2

stream_url = "http://admin:robocon@10.9.19.75:8081/video"

print(f"Connecting to: {stream_url} ...")
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Failed on /video endpoint, trying /mjpeg endpoint...")
    stream_url = f"http://admin:robocon@10.9.19.75:8081/mjpeg"
    cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("ERROR: Connection refused or password incorrect.")
    print("Please check the Username and Password inside your IP Camera Lite app settings.")
else:
    print("SUCCESS: Stream connected! Opening video window...")
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        cv2.imshow("MacBook OpenCV Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
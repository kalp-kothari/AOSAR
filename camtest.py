import cv2
import threading
import time

class RealtimeStream:
    def __init__(self, url):
        print(f"Connecting to stream: {url} ...")
        self.cap = cv2.VideoCapture(url)
        
        # Test if stream opens successfully
        if not self.cap.isOpened():
            print("ERROR: Could not open video stream. Check IP, Port, and network connection!")
            self.running = False
            self.ret = False
            self.frame = None
            return

        print("Stream connected successfully! Reading first frame...")
        self.ret, self.frame = self.cap.read()
        self.running = True

        # Start background thread to flush buffer continuously
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.ret, self.frame = ret, frame
            else:
                print("Warning: Lost frame from stream...")
                time.sleep(0.01)

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.running = False
        if hasattr(self, 'cap'):
            self.cap.release()

# -------------------------------------------------------------
# REPLACE THIS WITH YOUR STREAM URL
# -------------------------------------------------------------
stream_url = "http://10.9.19.75:8081/video"  # Paste your updated URL here

cam = RealtimeStream(stream_url)

if not cam.running:
    print("Exiting script because connection failed.")
else:
    print("Opening display window... Press 'q' to quit.")
    while True:
        ret, frame = cam.read()
        if not ret or frame is None:
            print("Waiting for valid frame data...")
            time.sleep(0.1)
            continue

        cv2.imshow("Realtime OpenCV Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.stop()
    cv2.destroyAllWindows()
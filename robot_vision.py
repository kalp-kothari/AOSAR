import cv2
import math
import socket
import time
import threading
from collections import deque
from pathlib import Path
from datetime import datetime
from ultralytics import YOLO

# ---------------- CONFIGURATION ----------------

STREAM_URLS = [
    "http://admin:robocon@10.80.160.217:8081/video",
    "http://admin:robocon@10.80.160.217:8081/mjpeg"
]

MODEL_PATH = r"runs\detect\runs\blue_red_scratch\weights\best.pt"

CONFIDENCE = 0.60
IMAGE_SIZE = 416          # Lower than 640 for faster inference
SAVE_DIR = Path("captured_frames")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# ---------------- ROBOT LINK ----------------
# "tcp"    -> WiFi to the ESP32 telnet server (robot can drive freely)
# "serial" -> USB cable to the ESP32 (needs: pip install pyserial)
# "none"   -> vision only, nothing is sent
ROBOT_LINK = "tcp"
ROBOT_HOST = "robot.local"        # or the IP printed by the ESP32, e.g. "192.168.1.50"
ROBOT_PORT = 23
SERIAL_PORT = "COM5"
SERIAL_BAUD = 115200

# Pixel x where a box centre counts as "straight ahead".
# None = middle of the frame. If the phone is not mounted on the robot's axis,
# measure the box centre x when the robot is truly aligned and put it here.
ALIGNED_CX_PX = None

# ---------------- DISTANCE CALIBRATION ----------------
# True bounding-box areas (px^2) measured at the settle distance, per class.
# They were measured on 1024 x 768 frames; live frames are rescaled to match.
REF_W, REF_H = 1024, 768
REFERENCE = {
    "RED":  {"area": 133955, "dist_cm": 40.0},
    "BLUE": {"area": 88500,  "dist_cm": 60.0},
}

TOLERANCE = 0.08          # +/-8% area band counts as "ON TARGET"
SMOOTH_N = 5              # median over the last N frames to remove jitter

# Colors (BGR)
GREEN = (0, 200, 0)
ORANGE = (0, 140, 255)
RED_C = (0, 0, 220)


class RobotLink:
    """Sends 'D,<R|B>,<dist>,<cx_err>' and 'H' lines to the ESP32 and prints its replies."""

    def __init__(self):
        self.sock = None
        self.ser = None
        self.connected = False
        self.last_try = 0.0

    def _connect(self):
        if ROBOT_LINK == "none" or time.time() - self.last_try < 2.0:
            return
        self.last_try = time.time()
        try:
            if ROBOT_LINK == "tcp":
                s = socket.create_connection((ROBOT_HOST, ROBOT_PORT), timeout=3)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(None)
                self.sock = s
                threading.Thread(target=self._read_tcp, args=(s,), daemon=True).start()
            else:
                import serial
                self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.2)
                threading.Thread(target=self._read_serial, args=(self.ser,), daemon=True).start()
            self.connected = True
            print(f"\n[link] connected ({ROBOT_LINK})")
        except Exception as e:
            self.connected = False
            print(f"\n[link] connect failed: {e}")

    def _read_tcp(self, s):
        buf = b""
        try:
            while True:
                data = s.recv(512)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    print(f"\n[robot] {line.decode(errors='ignore').strip()}")
        except Exception:
            pass
        self._close()

    def _read_serial(self, ser):
        try:
            while ser.is_open:
                line = ser.readline()
                if line:
                    print(f"\n[robot] {line.decode(errors='ignore').strip()}")
        except Exception:
            pass
        self._close()

    def _close(self):
        self.connected = False
        for obj in (self.sock, self.ser):
            try:
                if obj is not None:
                    obj.close()
            except Exception:
                pass
        self.sock = None
        self.ser = None

    def send(self, line):
        if ROBOT_LINK == "none":
            return
        if not self.connected:
            self._connect()
        if not self.connected:
            return
        data = (line + "\n").encode()
        try:
            if self.sock is not None:
                self.sock.sendall(data)
            elif self.ser is not None:
                self.ser.write(data)
        except Exception:
            self._close()

    def close(self):
        self._close()


def class_key(name):
    """Map a model class name such as 'RED BOX' to 'RED' or 'BLUE'."""
    n = name.upper()
    if "RED" in n:
        return "RED"
    if "BLUE" in n:
        return "BLUE"
    return None


def estimate_distance(key, area_px, frame_w, frame_h):
    """Distance from box area: area is proportional to 1 / distance^2."""
    scale = (REF_W * REF_H) / float(frame_w * frame_h)
    area = area_px * scale
    ref = REFERENCE[key]
    dist = ref["dist_cm"] * math.sqrt(ref["area"] / area)
    area_err = area / ref["area"] - 1.0
    return dist, area_err


def status_from_error(area_err):
    if area_err > TOLERANCE:
        return "TOO CLOSE", RED_C
    if area_err < -TOLERANCE:
        return "TOO FAR", ORANGE
    return "ON TARGET", GREEN


def draw_label(img, text, x, y, color):
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    h, w = img.shape[:2]
    x = max(0, min(x, w - tw - 6))
    y = max(th + 6, min(y, h - base - 4))
    cv2.rectangle(img, (x, y - th - 6), (x + tw + 6, y + base), color, -1)
    cv2.putText(img, text, (x + 3, y - 3), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 2)


# ---------------- LOAD MODEL ----------------

model = YOLO(MODEL_PATH)
print("Classes:", model.names)

link = RobotLink()

# ---------------- CONNECT TO STREAM ----------------

cap = None

for url in STREAM_URLS:
    print("Connecting to IP camera...")

    candidate = cv2.VideoCapture(url)

    # May be ignored by some IP camera backends
    candidate.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if candidate.isOpened():
        ret, frame = candidate.read()

        if ret and frame is not None:
            cap = candidate
            break

        candidate.release()
    else:
        candidate.release()

if cap is None:
    raise SystemExit("Could not connect to IP camera.")

print("Camera connected.")

# ---------------- LATEST-FRAME CAPTURE THREAD ----------------

latest_frame = None
frame_id = 0
frame_lock = threading.Lock()
running = True


def capture_frames():
    global latest_frame, frame_id, running

    while running:
        ret, frame = cap.read()

        if not ret or frame is None:
            time.sleep(0.01)
            continue

        # Replace the old frame instead of building a queue
        with frame_lock:
            latest_frame = frame
            frame_id += 1


capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

# ---------------- LIVE DETECTION ----------------

print("Detection running. Press S to save, Q to quit.")

last_id = -1
last_frame_time = 0
fps = 0.0
area_history = {k: deque(maxlen=SMOOTH_N) for k in REFERENCE}

try:
    while True:
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None
            current_id = frame_id

        if frame is None or current_id == last_id:
            time.sleep(0.002)
            continue
        last_id = current_id

        frame_h, frame_w = frame.shape[:2]
        aligned_cx = ALIGNED_CX_PX if ALIGNED_CX_PX is not None else frame_w / 2.0

        results = model.predict(
            source=frame,
            conf=CONFIDENCE,
            imgsz=IMAGE_SIZE,
            max_det=10,
            verbose=False
        )

        result = results[0]
        display = result.plot()

        # Processing FPS
        now = time.perf_counter()
        elapsed = now - last_frame_time if last_frame_time else 0
        last_frame_time = now

        if elapsed > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / elapsed) if fps else 1.0 / elapsed

        cv2.putText(display, f"Inference loop: {fps:.1f} FPS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # aligned-centre guide line
        cv2.line(display, (int(aligned_cx), 0), (int(aligned_cx), frame_h), (255, 255, 0), 1)

        # ---- best (highest-confidence) box per class ----
        best = {}
        if result.boxes is not None:
            for box in result.boxes:
                key = class_key(model.names[int(box.cls[0])])
                if key is None:
                    continue
                conf = float(box.conf[0])
                if key not in best or conf > best[key][0]:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    best[key] = (conf, x1, y1, x2, y2)

        # Forget history for classes not seen this frame
        for key in area_history:
            if key not in best:
                area_history[key].clear()

        # ---- distance for each class, and send it to the robot ----
        overall = None  # (conf, key, dist, status, color)
        for key, (conf, x1, y1, x2, y2) in best.items():
            area_history[key].append((x2 - x1) * (y2 - y1))
            hist = sorted(area_history[key])
            area_med = hist[len(hist) // 2]

            dist, area_err = estimate_distance(key, area_med, frame_w, frame_h)
            status, color = status_from_error(area_err)

            cx_err = ((x1 + x2) / 2.0 - aligned_cx) / frame_w    # + = box to the right
            link.send(f"D,{key[0]},{dist:.1f},{cx_err:.3f}")

            draw_label(display,
                       f"{dist:.1f} cm | {status} | A={area_med} ({area_err:+.0%}) | x={cx_err:+.2f}",
                       x1, y2 + 28, color)

            if overall is None or conf > overall[0]:
                overall = (conf, key, dist, status, color)

        link.send("H")      # heartbeat: one per processed frame (also when nothing is seen)

        # ---- banner for the most confident detection ----
        if overall is not None:
            _, key, dist, status, color = overall
            draw_label(display,
                       f"{key}: {dist:.1f} cm (target {REFERENCE[key]['dist_cm']:.0f} cm) - {status}",
                       10, 70, color)

        link_txt = "LINK: OK" if link.connected else ("LINK: OFF" if ROBOT_LINK == "none" else "LINK: DOWN")
        draw_label(display, link_txt, 10, 110, GREEN if link.connected else RED_C)

        cv2.imshow("Low-Latency IP Camera Detection", display)

        k = cv2.waitKey(1) & 0xFF

        if k == ord("s"):
            filename = SAVE_DIR / f"frame_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
            cv2.imwrite(str(filename), frame)
            print(f"\nSaved: {filename}")

        elif k == ord("q"):
            break

finally:
    running = False
    link.close()
    cap.release()
    cv2.destroyAllWindows()

print("\nDetection stopped.")

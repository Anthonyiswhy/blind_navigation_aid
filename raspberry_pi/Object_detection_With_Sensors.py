#!/usr/bin/env python3
# pi/main.py
"""
Combined Pi script:
- Picamera2 capture + OpenCV display
- MobileNet-SSD (optional)
- Read TF-Luna via /dev/serial0 (GPIO UART)
- Read ESP32 (ToF + ultrasonic) via /dev/ttyUSB0
- Directional visual overlay and buzzer (GPIO)
- CSV logging
"""

import os
import time
import csv
import math
import threading
from datetime import datetime

import cv2
import serial
from picamera2 import Picamera2
import RPi.GPIO as GPIO

# --- CONFIG ---
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200
LIDAR_PORT = "/dev/serial0"
LIDAR_BAUD = 115200

BUZZER_PIN = 18           # BCM pin for PWM
CSV_FOLDER = "./logs"
FRAME_SIZE = (640, 480)
DETECTION_INTERVAL = 6
DETECTION_CONF = 0.5

# Sound range (Hz)
BUZZER_MIN_FREQ = 300
BUZZER_MAX_FREQ = 1500

# MobileNet-SSD model paths (optional)
MODEL_DIR = os.path.expanduser("~/models")
PROTO = os.path.join(MODEL_DIR, "MobileNetSSD_deploy.prototxt")
MODEL = os.path.join(MODEL_DIR, "MobileNetSSD_deploy.caffemodel")

# CLASSES for MobileNet SSD (if used)
CLASSES = ["background","aeroplane","bicycle","bird","boat","bottle",
           "bus","car","cat","chair","cow","diningtable","dog","horse",
           "motorbike","person","pottedplant","sheep","sofa","train","tvmonitor"]

# --- SENSOR GEOMETRY (relative to camera center) ---
# Using values you provided (mm). angle_deg = sensor pointing angle relative to camera centerline.
SENSOR_POS = {
    "tof1":  {"x": 11.24, "y": 12.25, "z": -11.12, "angle_deg": 22.5},   # right
    "tof2":  {"x": -11.24, "y": 12.25, "z": -11.12, "angle_deg": -22.5}, # left
    "ultra": {"x": 0.0, "y": 19.37, "z": -9.0, "angle_deg": 0.0},
    "lidar": {"x": 0.0, "y": 38.75, "z": -2.5, "angle_deg": 0.0},
}

# --- GLOBALS ---
run_flag = True
esp32_data = {"tof1": -1, "tof2": -1, "ultrasonic": -1.0}
esp_lock = threading.Lock()

# --- GPIO / buzzer setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
buzzer_pwm = GPIO.PWM(BUZZER_PIN, 440)
buzzer_pwm.start(0)

def set_buzzer_freq_for_distance_and_dir(cm, direction):
    """Map distance (cm) + direction to frequency + duty cycle."""
    if cm is None or cm <= 0:
        buzzer_pwm.ChangeDutyCycle(0)
        return
    # clamp to 0..200 cm for mapping
    d = max(0.0, min(float(cm), 200.0))
    t = max(0.0, 1.0 - (d / 200.0))
    base_freq = int(BUZZER_MIN_FREQ + (BUZZER_MAX_FREQ - BUZZER_MIN_FREQ) * t)
    duty = min(80, 20 + int(t * 70))
    if direction == "left":
        base_freq = max(100, base_freq - 120)
    elif direction == "right":
        base_freq = max(100, base_freq + 120)
    buzzer_pwm.ChangeFrequency(base_freq)
    buzzer_pwm.ChangeDutyCycle(duty)

def get_direction_label(sensor_name):
    ang = SENSOR_POS.get(sensor_name, {}).get("angle_deg", 0.0)
    if ang > 10:
        return "right"
    elif ang < -10:
        return "left"
    else:
        return "center"

# --- Serial readers ---
def esp32_reader_thread():
    global esp32_data, run_flag
    try:
        ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
        time.sleep(2)
    except Exception as e:
        print("ESP32 serial open error:", e)
        return
    print("ESP32 reader started on", ESP32_PORT)
    while run_flag:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("#"): line = line[1:]
            parts = line.split(",")
            if len(parts) >= 3:
                with esp_lock:
                    try: esp32_data["tof1"] = int(parts[0])
                    except: esp32_data["tof1"] = -1
                    try: esp32_data["tof2"] = int(parts[1])
                    except: esp32_data["tof2"] = -1
                    try: esp32_data["ultrasonic"] = float(parts[2])
                    except: esp32_data["ultrasonic"] = -1.0
        except Exception:
            time.sleep(0.05)
    try: ser.close()
    except: pass
    print("ESP32 reader exiting")

def read_lidar_packet(ser):
    """Return TF-Luna distance in cm or None."""
    try:
        if ser.in_waiting < 9:
            return None
        b = ser.read(9)
        if len(b) < 9: return None
        # realign if header not at 0
        if b[0] != 0x59 or b[1] != 0x59:
            idx = b.find(b'\x59\x59')
            if idx == -1: return None
            if idx + 9 > len(b):
                more = ser.read((idx+9)-len(b))
                b = b[idx:idx+9] + more
            else:
                b = b[idx:idx+9]
        checksum = sum(b[0:8]) & 0xFF
        if checksum != b[8]: return None
        dist = b[2] + (b[3] << 8)
        return int(dist)
    except Exception:
        return None

# --- Detection model loader (optional) ---
def load_dnn():
    if os.path.exists(PROTO) and os.path.exists(MODEL):
        net = cv2.dnn.readNetFromCaffe(PROTO, MODEL)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        print("DNN model loaded")
        return net
    print("DNN model files not found; detection disabled")
    return None

# --- Visual overlay helpers ---
def compute_projection_x_px(angle_deg, image_w, hfov_deg=102.0):
    """
    Approximate horizontal pixel offset for a direction 'angle_deg' from center.
    hfov_deg: horizontal FOV of camera in degrees (IMX708 ≈ 102° H)
    returns x pixel coordinate (int)
    """
    hfov_rad = math.radians(hfov_deg)
    fx = (image_w / 2.0) / math.tan(hfov_rad / 2.0)   # focal length approx in pixels
    # projected x offset from center in pixels:
    x_offset = fx * math.tan(math.radians(angle_deg))
    center_x = image_w // 2
    return int(center_x + x_offset)

def draw_sensor_overlay(frame, w, h):
    """Draw arrows and labels for each sensor direction on the frame."""
    # draw center vertical
    cx = w//2
    cv2.line(frame, (cx, 0), (cx, h), (60,60,60), 1, cv2.LINE_AA)
    for name, info in SENSOR_POS.items():
        px = compute_projection_x_px(info["angle_deg"], w)
        color = (0,255,255) if "tof" in name else (200,200,0)
        # draw vertical marker
        cv2.line(frame, (px, h-40), (px, h-10), color, 2)
        cv2.putText(frame, name, (px-20, h-45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    # a small legend
    cv2.putText(frame, "Overlay: sensor direction projection (approx)", (10, h-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180,180,180), 1)

# --- CSV logging ---
os.makedirs(CSV_FOLDER, exist_ok=True)
csv_filename = os.path.join(CSV_FOLDER, datetime.now().strftime("sensor_log_%Y-%m-%d_%H-%M-%S.csv"))
csv_file = open(csv_filename, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["time_iso","time_sec","lidar_cm","tof1_cm","tof2_cm","ultrasonic_cm","detections"])

# --- Main ---
def main():
    global run_flag
    # start ESP32 thread
    t = threading.Thread(target=esp32_reader_thread, daemon=True)
    t.start()

    # open lidar serial
    try:
        lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.05)
        time.sleep(0.2)
        print("Lidar serial open", LIDAR_PORT)
    except Exception as e:
        print("LIDAR serial open error:", e)
        lidar_ser = None

    net = load_dnn()
    detection_enabled = (net is not None)

    # camera init
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": FRAME_SIZE})
    picam2.configure(config)
    picam2.start()
    print("Camera started; detection:", detection_enabled)

    frame_idx = 0
    try:
        while True:
            frame = picam2.capture_array()      # RGB
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = frame_bgr.shape[:2]

            # read sensors
            with esp_lock:
                tof1 = esp32_data["tof1"]
                tof2 = esp32_data["tof2"]
                ultra = esp32_data["ultrasonic"]

            lidar_cm = None
            if lidar_ser:
                val = read_lidar_packet(lidar_ser)
                if val is not None:
                    lidar_cm = val

            # choose closest sensor + direction
            distances = { "lidar": lidar_cm, "tof1": (tof1 if tof1>0 else None),
                          "tof2": (tof2 if tof2>0 else None), "ultra": (ultra if ultra>0 else None) }
            valid = {k:v for k,v in distances.items() if v is not None}
            if valid:
                closest_sensor = min(valid, key=valid.get)
                closest_val = valid[closest_sensor]
                direction = get_direction_label(closest_sensor)
                set_buzzer_freq_for_distance_and_dir(closest_val, direction)
            else:
                closest_sensor = None
                closest_val = None
                direction = None
                buzzer_pwm.ChangeDutyCycle(0)

            # overlay sensor directions
            draw_sensor_overlay(frame_bgr, w, h)

            # show sensor readings
            y = 25
            cv2.putText(frame_bgr, f"LIDAR: {lidar_cm if lidar_cm is not None else 'N/A'} cm", (10,y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            y += 26
            cv2.putText(frame_bgr, f"TOF1: {tof1}", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            y += 26
            cv2.putText(frame_bgr, f"TOF2: {tof2}", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            y += 26
            cv2.putText(frame_bgr, f"ULTRA: {ultra:.1f} cm" if isinstance(ultra,float) else f"ULTRA: {ultra}",
                        (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            y += 26
            if closest_sensor:
                cv2.putText(frame_bgr, f"Closest: {closest_sensor} ({closest_val} cm) dir={direction}",
                            (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,200,200), 2)

            # detection (reduced rate)
            detections_summary = ""
            if detection_enabled and (frame_idx % DETECTION_INTERVAL == 0):
                blob = cv2.dnn.blobFromImage(frame_bgr, 0.007843, (300,300), 127.5)
                net.setInput(blob)
                dets = net.forward()
                items = []
                for i in range(dets.shape[2]):
                    conf = float(dets[0,0,i,2])
                    if conf > DETECTION_CONF:
                        idx = int(dets[0,0,i,1])
                        label = CLASSES[idx] if idx < len(CLASSES) else str(idx)
                        box = (dets[0,0,i,3:7] * [w,h,w,h]).astype(int)
                        (sx, sy, ex, ey) = box
                        cv2.rectangle(frame_bgr, (sx,sy), (ex,ey), (0,255,255), 2)
                        cv2.putText(frame_bgr, f"{label} {conf:.2f}", (sx, sy-6),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                        items.append(f"{label}:{conf:.2f}")
                detections_summary = ";".join(items)

            cv2.imshow("Blind Navigation Aid", frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # logging
            t_iso = datetime.utcnow().isoformat()
            t_sec = time.time()
            csv_writer.writerow([t_iso, f"{t_sec:.3f}",
                                 (lidar_cm if lidar_cm is not None else 0),
                                 (tof1 if tof1 is not None else 0),
                                 (tof2 if tof2 is not None else 0),
                                 (ultra if ultra is not None else 0.0),
                                 detections_summary])
            csv_file.flush()
            frame_idx += 1

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        print("Cleanup")
        run_flag = False
        try: picam2.stop()
        except: pass
        try: csv_file.close()
        except: pass
        try: lidar_ser.close()
        except: pass
        GPIO.cleanup()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

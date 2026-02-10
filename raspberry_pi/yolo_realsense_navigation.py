#!/usr/bin/env python3
"""
Intelligent Navigation Assistant v3.4 FAST
- YOLO object detection
- RealSense depth sensing (OPTIMIZED)
- Smart clustering (only when needed)
- Velocity tracking with sanity checks
- Voice output (espeak)

KEY CHANGES IN v3.4:
- Use fast median by default (not expensive clustering every frame)
- Only cluster when distance seems ambiguous
- Pick LARGEST cluster (most pixels), not closest
- Filter distance jumps to prevent false velocity
- Stricter NMS to reduce duplicates
- TARGET: 12-18 FPS on Raspberry Pi 4
"""
import os
import time
import csv
import threading
import math
import subprocess
from datetime import datetime

import numpy as np
import cv2
import pyrealsense2 as rs
import onnxruntime as ort

# ============= CONFIGURATION =============
MODEL_PATH = os.path.expanduser("~/yolo_models/yolo11n.onnx")
IMG_SIZE = 320
DETECTION_INTERVAL = 1
CONF_THRESH = 0.30
NMS_THRESH = 0.40  # Lower is stricter for suppression (reduces duplicates)

# THREAT THRESHOLDS (cm)
CRITICAL_DISTANCE = 50
WARNING_DISTANCE = 150
SAFE_DISTANCE = 300

# VELOCITY LIMITS (cm/s)
MAX_HUMAN_VELOCITY = 300
MAX_OBJECT_VELOCITY = 500

# DISTANCE FILTERING
MAX_DISTANCE_JUMP = 100  # cm - reject jumps larger than this
TRACK_IOU_THRESH = 0.45
DEPTH_CENTER_CROP_RATIO = 0.6  # use center area of bbox to reduce background contamination
DEPTH_STRIDE = 2  # sample every N pixels for speed on Pi

# VOICE SETTINGS
VOICE_ENABLED = True
VOICE_COOLDOWN = 5.0

# LOGGING
AUTO_SAVE_DETECTIONS = False
SNAPSHOT_FOLDER = os.path.expanduser("~/blindnav_snapshots")
os.makedirs(SNAPSHOT_FOLDER, exist_ok=True)

CSV_FOLDER = os.path.expanduser("~/blindnav_logs")
os.makedirs(CSV_FOLDER, exist_ok=True)
CSV_FILE = os.path.join(CSV_FOLDER, datetime.utcnow().strftime("log_%Y%m%d_%H%M%S.csv"))

# ============= YOLO CLASS NAMES =============
CLASS_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

# THREAT WEIGHTS
OBJECT_THREAT_WEIGHTS = {
    "person": 1.5,
    "bicycle": 2.0,
    "car": 3.0,
    "motorcycle": 2.5,
    "truck": 3.5,
    "bus": 3.5,
    "train": 4.0,
    "dog": 1.3,
    "chair": 0.8,
    "bench": 0.9,
    "potted plant": 0.7,
    "bottle": 0.3,
    "cup": 0.3,
}

# DISPLAY COLORS
CLASS_COLORS = {
    "person": (0, 255, 0),
    "car": (255, 0, 0),
    "bicycle": (255, 128, 0),
    "motorcycle": (255, 64, 0),
    "truck": (200, 0, 0),
    "chair": (0, 255, 255),
    "bottle": (255, 255, 0),
}
DEFAULT_COLOR = (0, 255, 255)

# ============= VOICE ASSISTANT CLASS =============
class VoiceAssistant:
    def __init__(self):
        self.is_speaking = False
        self.last_announcement = {}
        self.cooldown = VOICE_COOLDOWN

        try:
            result = subprocess.run(
                ["espeak", "--version"],
                capture_output=True,
                timeout=2,
                check=False,
            )
            self.available = result.returncode == 0
            if self.available:
                print("[OK] Voice Assistant initialized (espeak)")
        except Exception:
            self.available = False
            print("[WARN] espeak not available")

    def speak(self, text, key=None):
        if not VOICE_ENABLED or not self.available or self.is_speaking:
            return

        if key:
            current_time = time.time()
            if key in self.last_announcement:
                if current_time - self.last_announcement[key] < self.cooldown:
                    return
            self.last_announcement[key] = current_time

        thread = threading.Thread(target=self._speak_thread, args=(text,))
        thread.daemon = True
        thread.start()

    def _speak_thread(self, text):
        self.is_speaking = True
        try:
            subprocess.run(
                ["espeak", "-s", "150", "-p", "40", "-g", "5", "-a", "200", text],
                stderr=subprocess.DEVNULL,
                timeout=10,
                check=False,
            )
        except Exception:
            pass
        finally:
            self.is_speaking = False

# ============= OBJECT TRACKER CLASS =============
class ObjectTracker:
    def __init__(self, max_age=1.0):
        self.tracks = {}
        self.next_id = 0
        self.max_age = max_age

    def update(self, detections, current_time):
        # Remove expired tracks
        expired = [
            tid for tid, track in self.tracks.items()
            if current_time - track.last_seen > self.max_age
        ]
        for tid in expired:
            del self.tracks[tid]

        if not detections:
            return list(self.tracks.values())

        # Match detections to existing tracks
        matched = []
        for det in detections:
            best_match = None
            best_iou = TRACK_IOU_THRESH

            for tid, track in self.tracks.items():
                if tid in matched:
                    continue
                if track.class_name != det["class_name"]:
                    continue

                iou = self._calculate_iou(track.box, det["box"])
                if iou > best_iou:
                    best_iou = iou
                    best_match = tid

            if best_match is not None:
                self.tracks[best_match].update(det, current_time)
                matched.append(best_match)
            else:
                new_track = Track(self.next_id, det, current_time)
                self.tracks[self.next_id] = new_track
                self.next_id += 1

        return list(self.tracks.values())

    def _calculate_iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        if x2 < x1 or y2 < y1:
            return 0.0

        inter = (x2 - x1) * (y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter

        return inter / union if union > 0 else 0.0

class Track:
    def __init__(self, track_id, detection, timestamp):
        self.id = track_id
        self.class_name = detection["class_name"]
        self.class_id = detection["class_id"]
        self.box = detection["box"]
        self.score = detection["score"]
        self.last_seen = timestamp

        self.distance = None
        self.distance_history = []
        self.velocity = 0.0
        self.velocity_valid = True
        self.num_clusters = 0  # For logging only

    def update(self, detection, timestamp):
        self.box = detection["box"]
        self.score = detection["score"]
        self.last_seen = timestamp

    def update_distance(self, new_distance, timestamp):
        if new_distance is None:
            return

        # FILTER: Reject impossible distance jumps
        if self.distance is not None:
            distance_jump = abs(new_distance - self.distance)
            if distance_jump > MAX_DISTANCE_JUMP:
                # Huge jump detected - probably clustering picked wrong cluster
                # Keep old distance, mark velocity invalid
                self.velocity_valid = False
                self.velocity = 0.0
                return

        self.distance = new_distance
        self.distance_history.append((timestamp, new_distance))
        self.distance_history = self.distance_history[-5:]

        # Calculate velocity
        if len(self.distance_history) >= 3:
            old_time, old_dist = self.distance_history[0]
            time_delta = timestamp - old_time

            if time_delta > 0.3:
                raw_velocity = (new_distance - old_dist) / time_delta

                # Sanity check
                max_vel = (
                    MAX_HUMAN_VELOCITY
                    if self.class_name == "person"
                    else MAX_OBJECT_VELOCITY
                )
                if abs(raw_velocity) > max_vel:
                    self.velocity_valid = False
                    self.velocity = 0.0
                    return

                self.velocity_valid = True

                # Noise filtering
                if abs(raw_velocity) < 5:
                    raw_velocity = 0

                # Smoothing
                alpha = 0.3
                self.velocity = alpha * raw_velocity + (1 - alpha) * self.velocity

# ============= THREAT ASSESSMENT CLASS =============
class ThreatAssessment:
    @staticmethod
    def calculate_threat_score(track):
        if track.distance is None:
            return 0.0

        if not track.velocity_valid:
            if track.distance < CRITICAL_DISTANCE:
                return 50.0
            if track.distance < WARNING_DISTANCE:
                return 20.0
            return 0.0

        score = 0.0
        distance = track.distance
        velocity = track.velocity

        # Distance component
        if distance < CRITICAL_DISTANCE:
            score += 50 + 50 * (CRITICAL_DISTANCE - distance) / CRITICAL_DISTANCE
        elif distance < WARNING_DISTANCE:
            score += 50 * (WARNING_DISTANCE - distance) / WARNING_DISTANCE
        elif distance < SAFE_DISTANCE:
            score += 10 * (SAFE_DISTANCE - distance) / SAFE_DISTANCE

        # Velocity component
        if velocity < -5:
            vel_score = min(200, abs(velocity) * 10)
            score += vel_score
            if velocity < -20:
                score += min(100, abs(velocity) * 2)
        elif velocity > 5:
            score *= 0.2

        # Object weight
        weight = OBJECT_THREAT_WEIGHTS.get(track.class_name, 1.0)
        score *= weight

        # Confidence
        if track.score > 0.7:
            score *= 1.1
        elif track.score < 0.3:
            score *= 0.9

        return min(500, score)

    @staticmethod
    def get_threat_level(score):
        if score > 80:
            return "CRITICAL"
        if score > 40:
            return "WARNING"
        if score > 10:
            return "CAUTION"
        return "SAFE"

    @staticmethod
    def prioritize_threats(tracks):
        scored = [(ThreatAssessment.calculate_threat_score(t), t) for t in tracks]
        scored.sort(reverse=True, key=lambda x: x[0])
        return scored

# ============= HELPER FUNCTIONS =============

def letterbox_resize(img, target_size):
    h, w = img.shape[:2]
    scale = min(target_size / h, target_size / w)
    new_h, new_w = int(h * scale), int(w * scale)

    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    padded = np.full((target_size, target_size, 3), 114, dtype=np.uint8)

    top = (target_size - new_h) // 2
    left = (target_size - new_w) // 2
    padded[top:top + new_h, left:left + new_w] = resized

    return padded, scale, (left, top)


def _normalize_output(output):
    preds = output
    if preds is None:
        return None
    if preds.ndim == 3:
        preds = preds[0]
    if preds.ndim != 2:
        return None
    if preds.shape[0] < preds.shape[1]:
        preds = preds.T
    return preds


def yolo_postprocess(output, orig_w, orig_h, scale, pad_left, pad_top):
    preds = _normalize_output(output)
    if preds is None:
        return []

    detections = []
    num_classes = len(CLASS_NAMES)

    for i in range(preds.shape[0]):
        row = preds[i]
        if row.shape[0] < 4 + num_classes:
            continue

        x_center = float(row[0])
        y_center = float(row[1])
        box_width = float(row[2])
        box_height = float(row[3])

        if row.shape[0] >= 5 + num_classes:
            obj_conf = float(row[4])
            class_scores = row[5:5 + num_classes]
            cls_id = int(np.argmax(class_scores))
            confidence = obj_conf * float(class_scores[cls_id])
        else:
            class_scores = row[4:4 + num_classes]
            cls_id = int(np.argmax(class_scores))
            confidence = float(class_scores[cls_id])

        if confidence < CONF_THRESH:
            continue

        # Convert to original coordinates
        x_center = (x_center - pad_left) / scale
        y_center = (y_center - pad_top) / scale
        box_width = box_width / scale
        box_height = box_height / scale

        x1 = int(x_center - box_width / 2)
        y1 = int(y_center - box_height / 2)
        x2 = int(x_center + box_width / 2)
        y2 = int(y_center + box_height / 2)

        x1 = max(0, min(x1, orig_w - 1))
        y1 = max(0, min(y1, orig_h - 1))
        x2 = max(0, min(x2, orig_w - 1))
        y2 = max(0, min(y2, orig_h - 1))

        if x2 <= x1 or y2 <= y1 or (x2 - x1) < 10 or (y2 - y1) < 10:
            continue

        class_name = (
            CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"class_{cls_id}"
        )

        detections.append({
            "class_id": cls_id,
            "class_name": class_name,
            "score": confidence,
            "box": [x1, y1, x2, y2],
        })

    # NMS
    if len(detections) > 1:
        detections.sort(key=lambda x: x["score"], reverse=True)
        keep = []

        for det in detections:
            keep_flag = True
            box_i = det["box"]
            area_i = (box_i[2] - box_i[0]) * (box_i[3] - box_i[1])

            for kept in keep:
                box_j = kept["box"]
                area_j = (box_j[2] - box_j[0]) * (box_j[3] - box_j[1])

                inter_x1 = max(box_i[0], box_j[0])
                inter_y1 = max(box_i[1], box_j[1])
                inter_x2 = min(box_i[2], box_j[2])
                inter_y2 = min(box_i[3], box_j[3])

                if inter_x2 > inter_x1 and inter_y2 > inter_y1:
                    inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
                    union_area = area_i + area_j - inter_area
                    iou = inter_area / union_area if union_area > 0 else 0.0

                    if iou > NMS_THRESH:
                        keep_flag = False
                        break

            if keep_flag:
                keep.append(det)

        return keep

    return detections


def get_smart_distance(depth_data, bbox):
    """
    OPTIMIZED: Fast median with optional clustering

    Strategy:
    1. Use center crop to reduce box-edge/background contamination
    2. Always calculate robust median (trim tails)
    3. Only cluster when distribution is ambiguous
    4. Pick LARGEST cluster (most pixels), not closest
    """
    x1, y1, x2, y2 = bbox

    # Keep center region of the box to reduce depth bleed from background
    box_w = x2 - x1
    box_h = y2 - y1
    if box_w < 8 or box_h < 8:
        return None, 0

    crop_w = int(box_w * DEPTH_CENTER_CROP_RATIO)
    crop_h = int(box_h * DEPTH_CENTER_CROP_RATIO)
    cx = x1 + box_w // 2
    cy = y1 + box_h // 2

    sx1 = max(x1, cx - crop_w // 2)
    sy1 = max(y1, cy - crop_h // 2)
    sx2 = min(x2, cx + crop_w // 2)
    sy2 = min(y2, cy + crop_h // 2)

    # Extract depth region (strided sampling for speed)
    depth_region = depth_data[sy1:sy2:DEPTH_STRIDE, sx1:sx2:DEPTH_STRIDE]
    valid_depths = depth_region[(depth_region > 0) & (depth_region < 8000)]

    if len(valid_depths) < 100:
        return None, 0

    # Step 1: Robust median (trim out tails)
    p10 = np.percentile(valid_depths, 10)
    p90 = np.percentile(valid_depths, 90)
    trimmed = valid_depths[(valid_depths >= p10) & (valid_depths <= p90)]
    if len(trimmed) < 60:
        trimmed = valid_depths

    median_dist_mm = np.median(trimmed)
    median_dist_cm = int(median_dist_mm / 10)

    # Step 2: Check if clustering is needed (IQR is robust)
    iqr = np.percentile(trimmed, 75) - np.percentile(trimmed, 25)

    # Low spread => median is reliable, skip clustering
    if iqr < 450:
        return median_dist_cm, 1

    # Step 3: High variance - do clustering
    try:
        hist, bin_edges = np.histogram(trimmed, bins=range(0, 8000, 300))

        # Find all peaks
        clusters = []
        for i in range(1, len(hist) - 1):
            if hist[i] > hist[i - 1] and hist[i] > hist[i + 1]:
                if hist[i] >= len(trimmed) * 0.18:
                    distance_mm = (bin_edges[i] + bin_edges[i + 1]) / 2
                    distance_cm = int(distance_mm / 10)
                    pixel_count = hist[i]
                    clusters.append((distance_cm, pixel_count))

        if len(clusters) > 1:
            largest_cluster = max(clusters, key=lambda c: c[1])
            return largest_cluster[0], len(clusters)

        return median_dist_cm, len(clusters)

    except Exception:
        return median_dist_cm, 0

# ============= MAIN =============

def main():
    global VOICE_ENABLED

    print("=" * 70)
    print("INTELLIGENT NAVIGATION ASSISTANT v3.4 FAST")
    print("RealSense D435 + YOLOv11n + Smart Distance + Voice")
    print("=" * 70)

    # CSV logging
    csv_file = open(CSV_FILE, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp",
        "fps",
        "tracked_objects",
        "top_threat",
        "threat_score",
        "distance_cm",
        "velocity_cm_s",
        "velocity_valid",
        "threat_level",
        "clusters",
    ])
    print(f"[LOG] CSV file: {CSV_FILE}")

    # Voice
    voice = VoiceAssistant()

    # YOLO
    print(f"\n[YOLO] Loading model: {MODEL_PATH}")
    try:
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess = ort.InferenceSession(
            MODEL_PATH, sess_options, providers=["CPUExecutionProvider"]
        )
        inp_name = sess.get_inputs()[0].name
        print("[OK] YOLO loaded")
    except Exception as e:
        print(f"[ERROR] YOLO failed: {e}")
        return

    # RealSense
    print("\n[REALSENSE] Initializing D435...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
        print("[OK] RealSense started")
    except Exception as e:
        print(f"[ERROR] RealSense failed: {e}")
        return

    align = rs.align(rs.stream.color)

    print("[INFO] Warming up (30 frames)...")
    for _ in range(30):
        pipeline.wait_for_frames()

    tracker = ObjectTracker()

    print("\n[READY] System active!")
    print("[CONTROLS] q = quit, s = snapshot, v = toggle voice, j = save threat")
    print(f"[CONFIG] AUTO_SAVE={AUTO_SAVE_DETECTIONS}, VOICE_COOLDOWN={VOICE_COOLDOWN}s")
    print("[INFO] Press Ctrl+C or 'q' to exit\n")

    frame_count = 0
    fps_list = []
    last_threat_print = time.time()
    detect_ms_list = []
    depth_ms_list = []

    try:
        while True:
            loop_start = time.time()

            # Get frames
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_data = np.asanyarray(depth_frame.get_data())
            h, w = color_image.shape[:2]

            # YOLO detection
            detections = []
            if frame_count % DETECTION_INTERVAL == 0:
                detect_start = time.time()
                try:
                    color_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                    img_letterbox, scale, (pad_left, pad_top) = letterbox_resize(
                        color_rgb, IMG_SIZE
                    )
                    img_in = img_letterbox.astype(np.float32) / 255.0
                    img_in = np.transpose(img_in, (2, 0, 1))[None]

                    outputs = sess.run(None, {inp_name: img_in})
                    detections = yolo_postprocess(
                        outputs[0], w, h, scale, pad_left, pad_top
                    )
                except Exception as e:
                    print(f"[ERROR] Detection failed: {e}")
                finally:
                    detect_ms_list.append((time.time() - detect_start) * 1000)
                    if len(detect_ms_list) > 60:
                        detect_ms_list.pop(0)

            # Update tracker
            current_time = time.time()
            tracks = tracker.update(detections, current_time)

            # FAST distance measurement
            depth_start = time.time()
            for track in tracks:
                distance, num_clusters = get_smart_distance(depth_data, track.box)
                track.num_clusters = num_clusters
                track.update_distance(distance, current_time)
            depth_ms_list.append((time.time() - depth_start) * 1000)
            if len(depth_ms_list) > 60:
                depth_ms_list.pop(0)

            # Threat assessment
            prioritized_threats = ThreatAssessment.prioritize_threats(tracks)
            top_threat = prioritized_threats[0] if prioritized_threats else None

            # Voice
            if top_threat:
                score, track = top_threat
                level = ThreatAssessment.get_threat_level(score)
                distance_m = track.distance / 100 if track.distance else 0

                if track.velocity_valid or abs(track.velocity) < 5:
                    if level == "CRITICAL" and score > 80:
                        voice.speak(
                            f"Warning! {track.class_name} {distance_m:.1f} meters",
                            key=f"critical_{track.class_name}",
                        )
                    elif level == "WARNING" and score > 40:
                        voice.speak(
                            f"{track.class_name} ahead {distance_m:.1f} meters",
                            key=f"warning_{track.class_name}",
                        )

            # Print
            if prioritized_threats and time.time() - last_threat_print > 2.0:
                print(f"\n[THREATS] Detected: {len(prioritized_threats)}")
                for score, track in prioritized_threats[:3]:
                    level = ThreatAssessment.get_threat_level(score)

                    vel_str = (
                        "INVALID!"
                        if not track.velocity_valid
                        else "<<FAST"
                        if track.velocity < -10
                        else "<-APP"
                        if track.velocity < -5
                        else "FAST>>"
                        if track.velocity > 10
                        else "AWAY->"
                        if track.velocity > 5
                        else "STATIC"
                    )

                    cluster_str = (
                        f", clusters={track.num_clusters}" if track.num_clusters > 1 else ""
                    )

                    print(
                        f"  [{level}] {track.class_name}: {track.distance}cm, "
                        f"{vel_str} {abs(track.velocity):.1f}cm/s (score: {score:.1f}){cluster_str}"
                    )

                last_threat_print = time.time()

            # Display
            frame_display = color_image.copy()

            for track in tracks:
                if track.distance is None:
                    continue

                x1, y1, x2, y2 = map(int, track.box)

                if top_threat and track == top_threat[1]:
                    score = top_threat[0]
                    level = ThreatAssessment.get_threat_level(score)
                    color = (
                        (0, 0, 255)
                        if level == "CRITICAL"
                        else (0, 165, 255)
                        if level == "WARNING"
                        else (0, 255, 255)
                    )
                    thickness = 3
                else:
                    color = CLASS_COLORS.get(track.class_name, DEFAULT_COLOR)
                    thickness = 2

                cv2.rectangle(frame_display, (x1, y1), (x2, y2), color, thickness)

                vel_indicator = (
                    "ERR"
                    if not track.velocity_valid
                    else "<<"
                    if track.velocity < -10
                    else "<"
                    if track.velocity < -5
                    else ">>"
                    if track.velocity > 10
                    else ">"
                    if track.velocity > 5
                    else "o"
                )

                label = f"{track.class_name} {track.distance}cm {vel_indicator}"
                if track.num_clusters > 1:
                    label += f" [{track.num_clusters}]"

                (text_w, text_h), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                )
                cv2.rectangle(
                    frame_display,
                    (x1, y1 - text_h - 8),
                    (x1 + text_w + 6, y1),
                    color,
                    -1,
                )
                cv2.putText(
                    frame_display,
                    label,
                    (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),
                    1,
                )

            # FPS
            loop_time = time.time() - loop_start
            fps = 1.0 / loop_time if loop_time > 0 else 0
            fps_list.append(fps)
            if len(fps_list) > 30:
                fps_list.pop(0)
            avg_fps = sum(fps_list) / len(fps_list)

            # Overlay
            cv2.putText(
                frame_display,
                f"Tracked: {len(tracks)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            if top_threat:
                score, track = top_threat
                level = ThreatAssessment.get_threat_level(score)
                color = (
                    (0, 0, 255)
                    if level == "CRITICAL"
                    else (0, 165, 255)
                    if level == "WARNING"
                    else (0, 255, 255)
                )

                cv2.putText(
                    frame_display,
                    f"THREAT: {track.class_name} {track.distance}cm [{level}]",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

            cv2.putText(
                frame_display,
                f"FPS: {avg_fps:.1f}",
                (w - 150, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.imshow("Navigation Assistant", frame_display)

            # Lightweight perf diagnostics for field debugging
            if frame_count % 60 == 0 and detect_ms_list and depth_ms_list:
                print(
                    f"[PERF] detect={np.mean(detect_ms_list):.1f}ms "
                    f"depth={np.mean(depth_ms_list):.1f}ms total={(1000.0 / avg_fps) if avg_fps > 0 else 0:.1f}ms"
                )

            # CSV
            if frame_count % 30 == 0:
                csv_writer.writerow([
                    datetime.utcnow().isoformat(),
                    f"{avg_fps:.1f}",
                    len(tracks),
                    top_threat[1].class_name if top_threat else "",
                    f"{top_threat[0]:.1f}" if top_threat else "0",
                    top_threat[1].distance
                    if (top_threat and top_threat[1].distance)
                    else -1,
                    f"{top_threat[1].velocity:.1f}" if top_threat else "0",
                    top_threat[1].velocity_valid if top_threat else True,
                    ThreatAssessment.get_threat_level(top_threat[0])
                    if top_threat
                    else "NONE",
                    top_threat[1].num_clusters if top_threat else 0,
                ])
                csv_file.flush()

            # Controls
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                print("\n[EXIT] Quit")
                break
            if key == ord("v"):
                VOICE_ENABLED = not VOICE_ENABLED
                print(f"[VOICE] {'ON' if VOICE_ENABLED else 'OFF'}")

            frame_count += 1

    except KeyboardInterrupt:
        print("\n[EXIT] Interrupted")
    finally:
        print("\n[CLEANUP] Shutting down...")
        pipeline.stop()
        cv2.destroyAllWindows()
        csv_file.close()
        print("[OK] Done")
        print("=" * 70)


if __name__ == "__main__":
    main()

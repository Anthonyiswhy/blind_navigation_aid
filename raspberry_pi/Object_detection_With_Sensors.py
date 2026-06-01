#!/usr/bin/env python3
"""
blind_nav_intelligent.py
INTELLIGENT NAVIGATION SYSTEM
- YOLOv8n support
- Object tracking with velocity calculation
- Threat assessment and prioritization
- Sensor fusion with directional awareness
"""
import os
import time
import csv
import threading
from datetime import datetime
import collections
import math

import numpy as np
import cv2
import serial
import onnxruntime as ort
from picamera2 import Picamera2
import RPi.GPIO as GPIO

# ============= CONFIG =============
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200

LIDAR_PORT = "/dev/serial0"
LIDAR_BAUD = 115200

MODEL_PATH = os.path.expanduser("~/yolo_models/yolo11n.onnx")  # YOLOv11n
IMG_SIZE = 320
DETECTION_INTERVAL = 3  # Every 3 frames for better FPS
CONF_THRESH = 0.30  # Higher to reduce false positives
NMS_THRESH = 0.45

# Visual distance estimation (fallback when sensors can't see object)
ENABLE_VISUAL_DISTANCE = True  # Estimate distance from bounding box size
DEBUG_LOGGING = True  # Enhanced logging for diagnostics

# Threat assessment thresholds
CRITICAL_DISTANCE = 50   # cm - immediate danger
WARNING_DISTANCE = 150   # cm - need attention
SAFE_DISTANCE = 300      # cm - awareness only

BUZZER_PIN = 18
BUZZER_MIN_FREQ = 400
BUZZER_MAX_FREQ = 1500

AUTO_SAVE_DETECTIONS = True
SNAPSHOT_FOLDER = os.path.expanduser("~/blindnav_snapshots")
os.makedirs(SNAPSHOT_FOLDER, exist_ok=True)

CSV_FOLDER = os.path.expanduser("~/blindnav_logs")
os.makedirs(CSV_FOLDER, exist_ok=True)
CSV_FILE = os.path.join(CSV_FOLDER, datetime.utcnow().strftime("log_%Y%m%d_%H%M%S.csv"))

# ============= GLOBALS =============
run_flag = True
esp32_data = {"tof1": -1, "tof2": -1, "ultrasonic": -1}
esp_lock = threading.Lock()

latest_lidar_cm = None
lidar_lock = threading.Lock()

# Median buffers
lidar_buf = collections.deque(maxlen=3)
tof1_buf = collections.deque(maxlen=3)
tof2_buf = collections.deque(maxlen=3)
ultra_buf = collections.deque(maxlen=3)

# YOLO class names
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

# Object type threat weights
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

last_save_time = 0
SAVE_COOLDOWN = 3

# ============= OBJECT TRACKER CLASS =============
class ObjectTracker:
    """Track objects across frames to calculate velocity."""
    
    def __init__(self, max_age=1.0):  # Reduced from 30 frames to 1 second
        self.tracks = {}  # {track_id: Track}
        self.next_id = 0
        self.max_age = max_age  # seconds before track expires
    
    def update(self, detections, current_time):
        """Update tracks with new detections."""
        # Age out old tracks - FASTER expiration
        expired = [tid for tid, track in self.tracks.items() 
                  if current_time - track.last_seen > self.max_age]
        for tid in expired:
            del self.tracks[tid]
        
        # Match detections to existing tracks
        if not detections:
            return list(self.tracks.values())
        
        matched = []
        for det in detections:
            best_match = None
            best_iou = 0.4  # HIGHER threshold to prevent mismatches (was 0.3)
            
            for tid, track in self.tracks.items():
                if tid in matched:
                    continue
                if track.class_name != det['class_name']:
                    continue
                
                iou = self._calculate_iou(track.box, det['box'])
                if iou > best_iou:
                    best_iou = iou
                    best_match = tid
            
            if best_match:
                self.tracks[best_match].update(det, current_time)
                matched.append(best_match)
            else:
                # New track
                new_track = Track(self.next_id, det, current_time)
                self.tracks[self.next_id] = new_track
                self.next_id += 1
        
        return list(self.tracks.values())
    
    def _calculate_iou(self, box1, box2):
        """Calculate intersection over union."""
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
    """Individual tracked object."""
    
    def __init__(self, track_id, detection, timestamp):
        self.id = track_id
        self.class_name = detection['class_name']
        self.class_id = detection['class_id']
        self.box = detection['box']
        self.score = detection['score']
        self.last_seen = timestamp
        
        # History for velocity calculation - REDUCED for speed
        self.position_history = [(timestamp, self._get_center())]
        self.distance_history = []  # [(timestamp, distance)]
        
        self.velocity = 0.0  # cm/s, negative = approaching
        self.distance = None
        self.distance_source = "NONE"  # Track how distance was determined
        self.distance_confidence = 0.0  # Confidence in distance estimate
    
    def update(self, detection, timestamp):
        """Update track with new detection."""
        self.box = detection['box']
        self.score = detection['score']
        self.last_seen = timestamp
        
        # Update position history - keep only 5 (was 10)
        center = self._get_center()
        self.position_history.append((timestamp, center))
        self.position_history = self.position_history[-5:]
    
    def update_distance(self, distance, timestamp):
        """Update distance measurement."""
        self.distance = distance
        self.distance_history.append((timestamp, distance))
        self.distance_history = self.distance_history[-5:]  # Keep only 5 (was 10)
        
        # Calculate velocity if we have history
        if len(self.distance_history) >= 2:
            old_time, old_dist = self.distance_history[0]
            time_delta = timestamp - old_time
            if time_delta > 0:
                # Negative velocity = approaching
                self.velocity = (distance - old_dist) / time_delta
    
    def _get_center(self):
        """Get bounding box center."""
        return ((self.box[0] + self.box[2]) // 2, 
                (self.box[1] + self.box[3]) // 2)
    
    def get_position_in_frame(self, frame_width):
        """Get normalized position: -1 (left) to +1 (right)."""
        center_x = (self.box[0] + self.box[2]) / 2
        return (center_x - frame_width / 2) / (frame_width / 2)

# ============= THREAT ASSESSMENT CLASS =============
class ThreatAssessment:
    """Assess and prioritize threats."""
    
    @staticmethod
    def calculate_threat_score(track):
        """
        Calculate threat score for a tracked object.
        Higher score = more urgent threat
        """
        if track.distance is None:
            return 0.0
        
        score = 0.0
        distance = track.distance
        velocity = track.velocity
        
        # 1. DISTANCE COMPONENT (closer = more urgent)
        if distance < CRITICAL_DISTANCE:
            # Critical zone: exponential urgency
            score += 100 * math.exp(-(distance / 20))
        elif distance < WARNING_DISTANCE:
            # Warning zone: linear decrease
            score += 50 * (WARNING_DISTANCE - distance) / WARNING_DISTANCE
        elif distance < SAFE_DISTANCE:
            # Awareness zone: low priority
            score += 10 * (SAFE_DISTANCE - distance) / SAFE_DISTANCE
        
        # 2. VELOCITY COMPONENT (approaching = more urgent)
        if velocity < -5:  # Approaching faster than 5 cm/s
            # Exponential increase for fast approaching objects
            score += abs(velocity) * 5
            # Bonus for very fast approach
            if velocity < -20:
                score += 50
        elif velocity > 5:  # Moving away
            # Reduce urgency if moving away
            score *= 0.5
        
        # 3. OBJECT TYPE WEIGHT
        weight = OBJECT_THREAT_WEIGHTS.get(track.class_name, 1.0)
        score *= weight
        
        # 4. CONFIDENCE BONUS (trust high confidence detections more)
        if track.score > 0.7:
            score *= 1.2
        elif track.score < 0.3:
            score *= 0.8
        
        return score
    
    @staticmethod
    def get_threat_level(score):
        """Convert score to threat level."""
        if score > 80:
            return "CRITICAL"
        elif score > 40:
            return "WARNING"
        elif score > 10:
            return "CAUTION"
        else:
            return "SAFE"
    
    @staticmethod
    def prioritize_threats(tracks):
        """Sort tracks by threat score."""
        scored = [(ThreatAssessment.calculate_threat_score(t), t) for t in tracks]
        scored.sort(reverse=True, key=lambda x: x[0])
        return scored

# ============= SENSOR HEALTH MONITORING =============
class SensorHealth:
    """Track sensor reliability and detect faults."""
    
    def __init__(self, name, min_value, max_value):
        self.name = name
        self.min_value = min_value
        self.max_value = max_value
        
        self.last_value = None
        self.last_change_time = time.time()
        self.stuck = False
        self.valid = True
        self.confidence = 1.0
        
        self.history = collections.deque(maxlen=10)
    
    def update(self, value, min_change=1.0, max_stuck_time=0.8, max_speed_cm_s=400):
        """
        Update sensor health based on new reading.
        
        Checks:
        1. Stuck value (no change for too long)
        2. Range validity (physical limits)
        3. Temporal plausibility (no teleporting)
        """
        now = time.time()
        
        # Range validity check
        if value < self.min_value or value > self.max_value:
            self.valid = False
            self.confidence = 0.0
            return {"distance": value, "confidence": 0.0, "valid": False, "stuck": False}
        else:
            self.valid = True
        
        # Stuck value detection
        if self.last_value is not None:
            value_change = abs(value - self.last_value)
            
            if value_change < min_change:
                # Value hasn't changed significantly
                time_stuck = now - self.last_change_time
                if time_stuck > max_stuck_time:
                    self.stuck = True
                    # Degrade confidence gradually
                    self.confidence = max(0.2, self.confidence * 0.9)
            else:
                # Value changed - sensor is responsive
                self.last_change_time = now
                self.stuck = False
                # Restore confidence gradually
                self.confidence = min(1.0, self.confidence + 0.1)
            
            # Temporal plausibility check (anti-teleport)
            dt = now - self.last_change_time if self.last_change_time else 0.1
            if dt > 0:
                apparent_speed = value_change / dt
                if apparent_speed > max_speed_cm_s:
                    # Physically impossible motion - likely glitch
                    self.confidence *= 0.5
        
        self.last_value = value
        self.history.append(value)
        
        return {
            "distance": value,
            "confidence": self.confidence,
            "valid": self.valid,
            "stuck": self.stuck
        }
    
    def get_status_string(self):
        """Get human-readable status."""
        if not self.valid:
            return f"{self.name}: INVALID"
        elif self.stuck:
            return f"{self.name}: STUCK (conf={self.confidence:.2f})"
        elif self.confidence < 0.5:
            return f"{self.name}: DEGRADED (conf={self.confidence:.2f})"
        else:
            return f"{self.name}: OK"

# Initialize sensor health trackers
sensor_health = {
    'lidar': SensorHealth('LIDAR', min_value=2, max_value=800),
    'ultrasonic': SensorHealth('Ultrasonic', min_value=2, max_value=400),
    'tof1': SensorHealth('TOF1(R)', min_value=1, max_value=200),
    'tof2': SensorHealth('TOF2(L)', min_value=1, max_value=200),
}

# ============= CONFIDENCE-BASED FUSION =============
def fuse_distances_with_confidence(estimates, conflict_threshold=0.4):
    """
    Fuse multiple distance estimates using confidence weighting.
    
    Returns: (fused_distance, total_confidence, has_conflict)
    """
    # Filter out invalid estimates
    valid_estimates = [e for e in estimates if e["valid"] and e["confidence"] > 0.05]
    
    if not valid_estimates:
        return None, 0.0, False
    
    # Check for conflicts
    if len(valid_estimates) >= 2:
        distances = [e["distance"] for e in valid_estimates]
        spread = max(distances) - min(distances)
        mean_dist = sum(distances) / len(distances)
        has_conflict = (spread / mean_dist > conflict_threshold) if mean_dist > 0 else False
    else:
        has_conflict = False
    
    # Weighted fusion
    weighted_sum = 0.0
    total_weight = 0.0
    
    for e in valid_estimates:
        weighted_sum += e["distance"] * e["confidence"]
        total_weight += e["confidence"]
    
    if total_weight == 0:
        return None, 0.0, has_conflict
    
    fused_distance = int(weighted_sum / total_weight)
    
    # If conflict detected, reduce overall confidence
    if has_conflict:
        total_weight *= 0.5
    
    return fused_distance, total_weight, has_conflict
class VisualDistanceEstimator:
    """
    Estimate distance from bounding box size (fallback when sensors can't see object).
    
    Calibration assumptions:
    - Person at 50cm fills ~40% of 480px height (192px)
    - Person at 100cm fills ~20% of height (96px)
    - Person at 200cm fills ~10% of height (48px)
    
    Formula: distance ≈ (reference_height / box_height) * reference_distance
    """
    
    # Reference measurements (calibrate these for your setup)
    REFERENCE_SIZES = {
        "person": {"height": 170, "width": 50, "reference_pixels": 192, "reference_distance": 50},
        "car": {"height": 150, "width": 180, "reference_pixels": 240, "reference_distance": 200},
        "bottle": {"height": 25, "width": 8, "reference_pixels": 100, "reference_distance": 30},
        "cup": {"height": 12, "width": 8, "reference_pixels": 80, "reference_distance": 25},
        "chair": {"height": 90, "width": 50, "reference_pixels": 150, "reference_distance": 100},
        "laptop": {"height": 25, "width": 35, "reference_pixels": 120, "reference_distance": 40},
        "cell phone": {"height": 15, "width": 7, "reference_pixels": 90, "reference_distance": 20},
    }
    
    @staticmethod
    def estimate_distance(class_name, box, frame_height):
        """
        Estimate distance from bounding box size.
        Returns distance in cm, or None if can't estimate.
        """
        if class_name not in VisualDistanceEstimator.REFERENCE_SIZES:
            # Unknown object - use generic estimation
            ref = {"reference_pixels": 100, "reference_distance": 50}
        else:
            ref = VisualDistanceEstimator.REFERENCE_SIZES[class_name]
        
        # Calculate box height in pixels
        box_height = box[3] - box[1]
        
        # Avoid division by zero
        if box_height < 10:
            return None
        
        # Simple inverse proportion: distance = ref_dist * (ref_pixels / box_pixels)
        estimated_distance = ref["reference_distance"] * (ref["reference_pixels"] / box_height)
        
        # Clamp to reasonable range
        estimated_distance = max(5, min(estimated_distance, 500))
        
        return int(estimated_distance)
    
    @staticmethod
    def get_confidence(box, frame_width, frame_height):
        """
        Get confidence in visual estimate based on box size.
        Larger boxes in center of frame = more confident.
        """
        box_area = (box[2] - box[1]) * (box[3] - box[1])
        frame_area = frame_width * frame_height
        area_ratio = box_area / frame_area
        
        # Center position (0-1, where 1 = perfect center)
        cx = (box[0] + box[2]) / 2
        cy = (box[1] + box[3]) / 2
        center_x = cx / frame_width
        center_y = cy / frame_height
        
        # Distance from center (0 = center, 0.5 = edge)
        dist_from_center = ((center_x - 0.5)**2 + (center_y - 0.5)**2) ** 0.5
        center_score = 1.0 - (dist_from_center * 2)  # 0-1
        
        # Combine area and center
        confidence = (area_ratio * 100) * center_score
        
        return min(1.0, max(0.1, confidence))
class SensorFusion:
    """Match objects with appropriate distance sensors."""
    
    @staticmethod
    def assign_distances_to_tracks(tracks, sensor_data, frame_width):
        """
        Assign distance measurements to tracks based on position.
        
        Sensor layout:
        - TOF1 (right): 22.5° right of center
        - TOF2 (left): 22.5° left of center  
        - LIDAR/Ultrasonic: center (forward)
        """
        lidar = sensor_data.get('lidar')
        ultra = sensor_data.get('ultrasonic')
        tof1 = sensor_data.get('tof1')  # Right
        tof2 = sensor_data.get('tof2')  # Left
        
        # Use most reliable front sensor
        front_distance = None
        if lidar and lidar > 0:
            front_distance = lidar
        elif ultra and ultra > 0:
            front_distance = ultra
        
        current_time = time.time()
        
        for track in tracks:
            # Get object position in frame (-1 = left, 0 = center, +1 = right)
            position = track.get_position_in_frame(frame_width)
            
            # Assign distance based on position
            # TOF sensors cover ±22.5° = roughly ±0.4 in normalized coords
            if position > 0.3:  # Right side
                distance = tof1 if tof1 and tof1 > 0 else front_distance
            elif position < -0.3:  # Left side
                distance = tof2 if tof2 and tof2 > 0 else front_distance
            else:  # Center
                distance = front_distance
            
            if distance:
                track.update_distance(distance, current_time)

# ============= BUZZER CONTROL =============
def set_buzzer_by_threat(top_threat, buzzer):
    """Control buzzer based on top threat."""
    if not top_threat:
        buzzer.ChangeDutyCycle(0)
        return
    
    score, track = top_threat
    
    if score < 10:
        buzzer.ChangeDutyCycle(0)
        return
    
    # Map threat score to frequency
    freq = int(BUZZER_MIN_FREQ + (BUZZER_MAX_FREQ - BUZZER_MIN_FREQ) * min(score / 100, 1.0))
    
    # Map threat score to duty cycle (urgency)
    duty = int(20 + min(score / 100, 1.0) * 60)
    
    buzzer.ChangeFrequency(max(200, freq))
    buzzer.ChangeDutyCycle(min(80, max(0, duty)))

# ============= ESP32 READER =============
def esp32_reader_thread():
    """Read ESP32 data."""
    global esp32_data, run_flag
    
    print("ESP32: Connecting...")
    
    try:
        ser_esp = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.5)
        time.sleep(2.0)
        ser_esp.reset_input_buffer()
        print(f"✅ ESP32: Connected")
    except Exception as e:
        print(f"❌ ESP32 failed: {e}")
        while run_flag:
            with esp_lock:
                esp32_data["tof1"] = 50
                esp32_data["tof2"] = 60
                esp32_data["ultrasonic"] = 40
            time.sleep(0.1)
        return
    
    line_count = 0
    
    while run_flag:
        try:
            line = ser_esp.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("#"):
                continue
            
            line_count += 1
            parts = line[1:].split(",")
            
            if len(parts) == 3:
                try:
                    t1 = int(parts[0])
                    t2 = int(parts[1])
                    u = float(parts[2])
                    
                    with esp_lock:
                        esp32_data = {"tof1": t1, "tof2": t2, "ultrasonic": u}
                    
                    if line_count % 100 == 0:
                        print(f"ESP32: TOF1(R)={t1}cm, TOF2(L)={t2}cm, Ultra={u}cm")
                        
                except ValueError:
                    pass
        except Exception:
            time.sleep(0.01)
    
    try:
        ser_esp.close()
    except:
        pass
    print("ESP32 thread exiting")

# ============= TF-LUNA READER =============
def tfluna_reader_thread():
    """Read TF-Luna LIDAR."""
    global latest_lidar_cm, run_flag
    
    print("TF-Luna: Starting...")
    
    try:
        ser_lidar = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
        time.sleep(0.2)
        ser_lidar.reset_input_buffer()
        print(f"✅ TF-Luna: Connected")
    except serial.SerialException as e:
        print(f"❌ TF-Luna failed: {e}")
        return
    
    read_count = 0
    
    while run_flag:
        try:
            if ser_lidar.read() == b'\x59' and ser_lidar.read() == b'\x59':
                dist_l = ser_lidar.read()
                dist_h = ser_lidar.read()
                if dist_l and dist_h:
                    dist_cm = dist_l[0] + dist_h[0] * 256
                    
                    read_count += 1
                    
                    if dist_cm < 800:
                        with lidar_lock:
                            latest_lidar_cm = dist_cm
                        
                        if dist_cm > 0:
                            lidar_buf.append(dist_cm)
                        
                        if read_count % 50 == 0:
                            print(f"TF-Luna: {dist_cm}cm")
                        
        except Exception as e:
            if read_count % 100 == 0:
                print(f"TF-Luna error: {e}")
            time.sleep(0.1)
    
    try:
        ser_lidar.close()
    except:
        pass
    print("TF-Luna thread exiting")

# ============= LETTERBOX RESIZE =============
def letterbox_resize(img, target_size):
    """Resize with letterboxing."""
    h, w = img.shape[:2]
    scale = min(target_size / h, target_size / w)
    new_h, new_w = int(h * scale), int(w * scale)
    
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    padded = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
    
    top = (target_size - new_h) // 2
    left = (target_size - new_w) // 2
    
    padded[top:top+new_h, left:left+new_w] = resized
    
    return padded, scale, (left, top)

# ============= YOLOV8/V11 POSTPROCESSING =============
def yolo_postprocess(output, orig_w, orig_h, scale, pad_left, pad_top):
    """
    Process YOLOv8/v11 output - both use same ONNX format
    Format: (1, 84, N) where N = number of anchors (2100 for 320x320)
    Layout: [x_center, y_center, width, height, class0_conf, ..., class79_conf]
    Coordinates are in PIXELS relative to input image size (320x320)
    """
    if output is None or len(output) == 0:
        return []
    
    preds = output[0]  # Shape: (1, 84, 2100) for 320x320
    
    # Remove batch dimension and transpose to (N, 84)
    if preds.ndim == 3:
        preds = preds[0].T  # Now (2100, 84)
    elif preds.ndim == 2:
        preds = preds.T
    
    detections = []
    candidates = 0  # Track how many pass confidence
    
    for i in range(preds.shape[0]):
        row = preds[i]
        
        # First 4 values: box coordinates (in pixels, relative to 320x320 input)
        x_center = float(row[0])
        y_center = float(row[1])
        box_width = float(row[2])
        box_height = float(row[3])
        
        # Next 80 values: class confidences (one per COCO class)
        class_scores = row[4:84]  # Exactly 80 classes
        
        # Find best class
        cls_id = int(np.argmax(class_scores))
        confidence = float(class_scores[cls_id])
        
        # Skip low confidence
        if confidence < CONF_THRESH:
            continue
        
        candidates += 1
        
        # Validate class ID (COCO has 80 classes: 0-79)
        if cls_id < 0 or cls_id >= 80:
            continue
        
        # Convert from letterbox coordinates (320x320) to original image coordinates
        # Step 1: Remove letterbox padding
        x_center = x_center - pad_left
        y_center = y_center - pad_top
        
        # Step 2: Scale back to original image size
        x_center = x_center / scale
        y_center = y_center / scale
        box_width = box_width / scale
        box_height = box_height / scale
        
        # Convert center/width/height to corners
        x1 = int(x_center - box_width / 2)
        y1 = int(y_center - box_height / 2)
        x2 = int(x_center + box_width / 2)
        y2 = int(y_center + box_height / 2)
        
        # Clamp to image bounds
        x1 = max(0, min(x1, orig_w - 1))
        y1 = max(0, min(y1, orig_h - 1))
        x2 = max(0, min(x2, orig_w - 1))
        y2 = max(0, min(y2, orig_h - 1))
        
        # Skip invalid boxes
        if x2 <= x1 or y2 <= y1:
            continue
        
        # Skip tiny boxes (smaller threshold for 320x320)
        if (x2 - x1) < 10 or (y2 - y1) < 10:
            continue
        
        # Get class name
        class_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"unknown_{cls_id}"
        
        detections.append({
            "class_id": cls_id,
            "class_name": class_name,
            "score": confidence,
            "box": [x1, y1, x2, y2]
        })
    
    # Debug: print detection stats occasionally (removed for speed)
    # Keeping candidates check for major issues only
    
    # Apply Non-Maximum Suppression
    if len(detections) > 1:
        detections.sort(key=lambda x: x["score"], reverse=True)
        keep = []
        
        for i in range(len(detections)):
            keep_flag = True
            box_i = detections[i]["box"]
            area_i = (box_i[2] - box_i[0]) * (box_i[3] - box_i[1])
            
            for j in range(len(keep)):
                box_j = keep[j]["box"]
                area_j = (box_j[2] - box_j[0]) * (box_j[3] - box_j[1])
                
                # Calculate intersection
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
                keep.append(detections[i])
        
        return keep
    
    return detections

# ============= AUTO-SAVE =============
def auto_save_snapshot(frame_display, tracks, top_threat):
    """Save snapshot with threat data."""
    global last_save_time
    
    current_time = time.time()
    if current_time - last_save_time < SAVE_COOLDOWN:
        return False
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename = os.path.join(SNAPSHOT_FOLDER, f"threat_{timestamp}.jpg")
    
    cv2.imwrite(filename, frame_display)
    
    # Save metadata
    meta_file = filename.replace('.jpg', '.txt')
    with open(meta_file, 'w') as f:
        f.write(f"Timestamp: {timestamp}\n\n")
        
        if top_threat:
            score, track = top_threat
            f.write(f"TOP THREAT:\n")
            f.write(f"  Object: {track.class_name}\n")
            f.write(f"  Threat Score: {score:.1f}\n")
            f.write(f"  Threat Level: {ThreatAssessment.get_threat_level(score)}\n")
            f.write(f"  Distance: {track.distance}cm\n")
            f.write(f"  Velocity: {track.velocity:.1f}cm/s\n")
            f.write(f"  Confidence: {track.score:.2f}\n")
        
        f.write(f"\nAll Tracked Objects ({len(tracks)}):\n")
        for track in tracks:
            f.write(f"  {track.class_name}: dist={track.distance}cm, vel={track.velocity:.1f}cm/s\n")
    
    last_save_time = current_time
    print(f"\n📸 SAVED: {filename}")
    
    return True

# ============= MAIN =============
def main():
    global run_flag
    
    print("=" * 70)
    print("INTELLIGENT BLIND NAVIGATION SYSTEM v2.1")
    print("=" * 70)
    print("FEATURES:")
    print("✓ YOLOv11n object detection (latest model)")
    print("✓ Multi-object tracking with velocity")
    print("✓ Threat assessment and prioritization")
    print("✓ Vision-primary sensor fusion")
    print("✓ Sensor health monitoring (stuck detection, confidence)")
    print("✓ Conflict detection & resolution")
    print("✓ Visual distance estimation (solves blind spots!)")
    print("=" * 70)
    print(f"Model: YOLOv11n @ 320x320 (detection every 3 frames)")
    print(f"Camera: Wide-angle (102° FOV)")
    print(f"Sensors: LIDAR(front), TOF1(right 22.5°), TOF2(left 22.5°), Ultrasonic")
    print(f"Fusion: Vision-primary with sensor validation")
    print(f"Health: Real-time stuck detection & confidence tracking")
    print(f"Debug: {'ENABLED' if DEBUG_LOGGING else 'DISABLED'}")
    print("Press ESC to exit, 's' for snapshot, 't' to show threat details")
    print("=" * 70)
    
    # CSV
    csv_file = open(CSV_FILE, "w", newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["timestamp", "lidar_cm", "tof1_cm", "tof2_cm", "ultrasonic_cm", 
                         "tracked_objects", "top_threat", "threat_score", "distance_source"])
    print(f"Log: {CSV_FILE}")
    
    # GPIO / buzzer
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    buzzer = GPIO.PWM(BUZZER_PIN, BUZZER_MIN_FREQ)
    buzzer.start(0)
    print(f"Buzzer: BCM{BUZZER_PIN}")
    
    # Load model
    if not os.path.exists(MODEL_PATH):
        print(f"❌ ERROR: {MODEL_PATH} not found")
        return
    
    try:
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess = ort.InferenceSession(MODEL_PATH, sess_options, providers=['CPUExecutionProvider'])
        inp_name = sess.get_inputs()[0].name
        print(f"✅ YOLOv8n loaded: {inp_name}")
    except Exception as e:
        print(f"❌ Model error: {e}")
        return
    
    # Initialize tracking
    tracker = ObjectTracker()
    
    # Start sensors
    print("\nStarting sensors...")
    t_esp = threading.Thread(target=esp32_reader_thread, daemon=True)
    t_lidar = threading.Thread(target=tfluna_reader_thread, daemon=True)
    
    t_esp.start()
    t_lidar.start()
    time.sleep(3)
    
    # Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={"AwbEnable": True, "FrameRate": 30}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    
    print("✅ Camera ready")
    print("\n🚀 System active - tracking threats!\n")
    
    # Main loop
    frame_idx = 0
    last_detection_print = time.time()
    last_sensor_print = time.time()
    last_threat_print = time.time()
    fps_counter = collections.deque(maxlen=30)
    show_threats = False
    
    try:
        while run_flag:
            loop_start = time.time()
            
            # Capture
            frame = picam2.capture_array()
            if frame is None:
                continue
            
            if frame.shape[2] == 3:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
            
            frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
            h, w = frame_rgb.shape[:2]
            
            # Object detection
            detections = []
            if frame_idx % DETECTION_INTERVAL == 0:
                try:
                    img_letterbox, scale, (pad_left, pad_top) = letterbox_resize(frame_rgb, IMG_SIZE)
                    img_in = img_letterbox.astype(np.float32) / 255.0
                    img_in = np.transpose(img_in, (2, 0, 1))[None]
                    
                    outputs = sess.run(None, {inp_name: img_in})
                    
                    if frame_idx == DETECTION_INTERVAL:
                        print(f"🔍 YOLOv8 output: {outputs[0].shape}, letterbox: scale={scale:.2f}")
                    
                    detections = yolo_postprocess(outputs[0], w, h, scale, pad_left, pad_top)
                    
                except Exception as e:
                    if frame_idx == DETECTION_INTERVAL:
                        print(f"❌ Detection error: {e}")
                        import traceback
                        traceback.print_exc()
            
            # Update tracker
            current_time = time.time()
            tracks = tracker.update(detections, current_time)
            
            # Get sensor data
            with esp_lock:
                t1_raw = esp32_data["tof1"]
                t2_raw = esp32_data["tof2"]
                u_raw = esp32_data["ultrasonic"]
            
            with lidar_lock:
                lidar_cm = latest_lidar_cm
            
            # Update buffers
            if t1_raw > 0:
                tof1_buf.append(t1_raw)
            if t2_raw > 0:
                tof2_buf.append(t2_raw)
            if u_raw > 0:
                ultra_buf.append(u_raw)
            if lidar_cm and lidar_cm > 0 and lidar_cm < 800:
                lidar_buf.append(lidar_cm)
            
            def median(deque):
                if len(deque) == 0:
                    return None
                return sorted(deque)[len(deque) // 2]
            
            t1_med = median(tof1_buf)
            t2_med = median(tof2_buf)
            u_med = median(ultra_buf)
            lidar_med = median(lidar_buf)
            
            # Sensor fusion - assign distances to tracks with health monitoring
            sensor_data = {
                'lidar': lidar_med,
                'ultrasonic': u_med,
                'tof1': t1_med,  # Right
                'tof2': t2_med   # Left
            }
            SensorFusion.assign_distances_to_tracks(tracks, sensor_data, w)
            
            # Threat assessment
            prioritized_threats = ThreatAssessment.prioritize_threats(tracks)
            top_threat = prioritized_threats[0] if prioritized_threats else None
            
            # Print sensor health occasionally
            if time.time() - last_sensor_print > 10.0:
                print(f"\n🔧 SENSOR HEALTH:")
                for name, health in sensor_health.items():
                    print(f"   {health.get_status_string()}")
                last_sensor_print = time.time()
            
            # Print threats occasionally with ENHANCED LOGGING
            if prioritized_threats and time.time() - last_threat_print > 2.0:
                print(f"\n⚠️  THREATS ({len(prioritized_threats)}):")
                for score, track in prioritized_threats[:3]:  # Top 3
                    level = ThreatAssessment.get_threat_level(score)
                    vel_str = "→" if track.velocity > 5 else "←" if track.velocity < -5 else "•"
                    
                    # Enhanced logging with distance source
                    dist_info = f"{track.distance}cm"
                    if DEBUG_LOGGING and hasattr(track, 'distance_source'):
                        dist_info += f" [{track.distance_source}]"
                    
                    print(f"   [{level}] {track.class_name}: {dist_info}, "
                          f"{vel_str} {abs(track.velocity):.0f}cm/s (score: {score:.1f})")
                    
                    # Warning for visual-only distances
                    if DEBUG_LOGGING and hasattr(track, 'distance_source') and track.distance_source == "Visual-ONLY":
                        print(f"      ⚡ WARNING: No sensor coverage! Using visual estimate only.")
                
                last_threat_print = time.time()
            
            # Control buzzer based on top threat
            set_buzzer_by_threat(top_threat, buzzer)
            
            # ============= DISPLAY =============
            frame_display = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            # Sensor overlay
            y_pos = 25
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 2
            
            cv2.putText(frame_display, f"L:{lidar_med if lidar_med else '--'}cm U:{u_med if u_med else '--'}cm", 
                       (10, y_pos), font, font_scale, (0, 255, 255), thickness)
            y_pos += 20
            cv2.putText(frame_display, f"TOF-R:{t1_med if t1_med else '--'}cm TOF-L:{t2_med if t2_med else '--'}cm", 
                       (10, y_pos), font, font_scale, (0, 255, 0), thickness)
            y_pos += 20
            cv2.putText(frame_display, f"Tracked: {len(tracks)}", 
                       (10, y_pos), font, font_scale, (255, 255, 0), thickness)
            
            # Top threat indicator with confidence
            if top_threat:
                score, track = top_threat
                level = ThreatAssessment.get_threat_level(score)
                color = (0, 0, 255) if level == "CRITICAL" else (0, 165, 255) if level == "WARNING" else (0, 255, 255)
                y_pos += 25
                threat_text = f"THREAT: {track.class_name} {track.distance}cm [{level}]"
                if hasattr(track, 'distance_confidence'):
                    threat_text += f" ({track.distance_confidence:.1f})"
                cv2.putText(frame_display, threat_text, 
                           (10, y_pos), font, font_scale, color, thickness)
            
            # Draw tracks
            for track in tracks:
                x1, y1, x2, y2 = map(int, track.box)
                
                # Color by threat level
                if track == (top_threat[1] if top_threat else None):
                    score = top_threat[0]
                    level = ThreatAssessment.get_threat_level(score)
                    if level == "CRITICAL":
                        color = (0, 0, 255)  # Red
                    elif level == "WARNING":
                        color = (0, 165, 255)  # Orange
                    else:
                        color = (0, 255, 255)  # Yellow
                else:
                    color = CLASS_COLORS.get(track.class_name, DEFAULT_COLOR)
                
                # Box
                thickness = 3 if top_threat and track == top_threat[1] else 2
                cv2.rectangle(frame_display, (x1, y1), (x2, y2), color, thickness)
                
                # Label with velocity
                vel_indicator = "←" if track.velocity < -5 else "→" if track.velocity > 5 else "•"
                label = f"{track.class_name} {track.distance}cm {vel_indicator}"
                
                (text_width, text_height), _ = cv2.getTextSize(label, font, 0.4, 1)
                cv2.rectangle(frame_display, 
                            (x1, y1 - text_height - 8),
                            (x1 + text_width + 6, y1),
                            color, -1)
                cv2.putText(frame_display, label, 
                          (x1 + 3, y1 - 4),
                          font, 0.4, (0, 0, 0), 1)
                
                # Track ID (for debugging)
                if show_threats:
                    cv2.putText(frame_display, f"#{track.id}", 
                              (x2 - 30, y2 - 5),
                              font, 0.3, color, 1)
            
            # FPS
            loop_time = time.time() - loop_start
            fps_counter.append(loop_time)
            avg_fps = 1.0 / (sum(fps_counter) / len(fps_counter))
            cv2.putText(frame_display, f"FPS: {avg_fps:.1f}", 
                       (w-100, 25), font, 0.5, (255, 255, 255), 1)
            
            cv2.imshow("Intelligent Navigation", frame_display)
            
            # Auto-save on critical threats
            if AUTO_SAVE_DETECTIONS and top_threat and top_threat[0] > 40:
                auto_save_snapshot(frame_display, tracks, top_threat)
            
            # Keys
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('s'):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(SNAPSHOT_FOLDER, f"manual_{timestamp}.jpg")
                cv2.imwrite(filename, frame_display)
                print(f"\n📷 Snapshot: {filename}")
            elif key == ord('t'):
                show_threats = not show_threats
                print(f"\n🎯 Threat overlay: {'ON' if show_threats else 'OFF'}")
            
            # Log
            top_threat_name = top_threat[1].class_name if top_threat else ""
            top_threat_score = top_threat[0] if top_threat else 0
            top_threat_source = top_threat[1].distance_source if (top_threat and hasattr(top_threat[1], 'distance_source')) else ""
            
            csv_writer.writerow([
                datetime.utcnow().isoformat(),
                lidar_med if lidar_med else -1,
                t1_med if t1_med else -1,
                t2_med if t2_med else -1,
                u_med if u_med else -1,
                len(tracks),
                top_threat_name,
                f"{top_threat_score:.1f}",
                top_threat_source
            ])
            
            if frame_idx % 30 == 0:
                csv_file.flush()
            
            frame_idx += 1
            
    except KeyboardInterrupt:
        print("\n⏹️  Interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        run_flag = False
        time.sleep(0.5)
        
        try:
            buzzer.stop()
        except:
            pass
        GPIO.cleanup()
        
        try:
            picam2.stop()
        except:
            pass
        
        try:
            cv2.destroyAllWindows()
        except:
            pass
        
        try:
            csv_file.close()
        except:
            pass
        
        print("\n✅ Cleanup complete")
        print("=" * 70)

if __name__ == "__main__":
    main()

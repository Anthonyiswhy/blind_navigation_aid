# System Architecture

This document provides a detailed technical overview of the blind navigation aid system architecture.

## Table of Contents

1. [System Overview](#system-overview)
2. [Pipeline Flow](#pipeline-flow)
3. [Component Details](#component-details)
4. [Sensor Fusion Strategy](#sensor-fusion-strategy)
5. [Visual Distance Estimation](#visual-distance-estimation)
6. [Performance Optimizations](#performance-optimizations)

---

## System Overview

The system follows a vision-primary architecture where computer vision provides the primary awareness layer, validated and refined by physical distance sensors.

```
┌─────────────────────────────────────────────────────────┐
│                    INPUT LAYER                          │
│  Camera (102° FOV) + LIDAR + TOF×2 + Ultrasonic        │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│                  DETECTION LAYER                        │
│  YOLOv11n (320×320) → Bounding boxes + Classes         │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│                  TRACKING LAYER                         │
│  IoU matching → Track IDs + Velocity calculation        │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│              DISTANCE ESTIMATION LAYER                  │
│  Visual (bbox size) + Sensor validation → Distance      │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│                 ASSESSMENT LAYER                        │
│  Threat scoring → Priority queue                        │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│                   OUTPUT LAYER                          │
│  Buzzer frequency + Visual overlay + CSV logging        │
└─────────────────────────────────────────────────────────┘
```

---

## Pipeline Flow

### Frame Processing (Every Frame)

```python
1. Capture frame from Picamera2 (640×480 RGB)
2. Rotate 180° (camera mounting)
3. Read sensor data:
   - LIDAR (via I²C serial)
   - TOF1, TOF2, Ultrasonic (via ESP32 serial)
4. Apply median filtering (3-sample buffer)
5. Update sensor health monitors
```

### Object Detection (Every 3rd Frame)

```python
6. Letterbox resize to 320×320
7. Normalize to [0,1] and transpose to CHW
8. YOLOv11n inference
9. Postprocess:
   - Parse (1, 84, 2100) output
   - Filter by confidence (>0.30)
   - Convert to original image coordinates
   - Apply NMS (IoU threshold 0.45)
```

### Tracking & Distance (Every Frame)

```python
10. Update tracker with new detections
11. Match existing tracks using IoU (>0.4)
12. Expire old tracks (>1.0s)
13. For each track:
    a. Estimate visual distance from bbox size
    b. Assign appropriate sensor based on position
    c. Fuse visual + sensor estimates
    d. Calculate velocity from distance history
```

### Threat Assessment (Every Frame)

```python
14. Calculate threat score for each track
15. Sort by score (descending)
16. Update buzzer based on top threat
17. Log to CSV
18. Auto-save snapshot if critical threat
```

---

## Component Details

### 1. YOLOv11n Detection

**Model:** `yolo11n.onnx` (320×320 input)

**Input Shape:** `(1, 3, 320, 320)` - NCHW format

**Output Shape:** `(1, 84, 2100)` where:
- 84 = 4 box coords + 80 class confidences
- 2100 = number of anchor points (for 320×320: 80×80/4 + 40×40/4 + 20×20/4 = 2100)

**Coordinate Format:**
- `[x_center, y_center, width, height]` in pixels relative to 320×320 input
- Coordinates must be transformed back to original image space

**Postprocessing Steps:**
1. Transpose to `(2100, 84)`
2. Parse box coords (first 4 values)
3. Find best class (argmax of next 80 values)
4. Filter by confidence threshold
5. Remove letterbox padding effects
6. Scale back to original resolution
7. Apply NMS

### 2. Object Tracker

**Algorithm:** IoU-based matching

**Key Parameters:**
- `max_age`: 1.0s - Track expiry time
- `iou_threshold`: 0.4 - Minimum IoU for match
- `max_tracks`: Unlimited

**Track State:**
```python
Track {
    id: int
    class_name: str
    box: [x1, y1, x2, y2]
    score: float
    distance: int (cm)
    distance_source: str
    distance_confidence: float
    velocity: float (cm/s, negative = approaching)
    position_history: deque(5)
    distance_history: deque(5)
}
```

**Velocity Calculation:**
```python
if len(distance_history) >= 2:
    old_time, old_dist = distance_history[0]
    time_delta = current_time - old_time
    velocity = (current_dist - old_dist) / time_delta  # cm/s
```

### 3. Visual Distance Estimator

**Principle:** Inverse proportion - object size in image inversely proportional to distance

**Formula:**
```python
distance = reference_distance × (reference_pixels / actual_pixels)
```

**Calibration Data:**

| Object | Real Height (cm) | Ref Distance (cm) | Ref Pixels @ Distance |
|--------|------------------|-------------------|-----------------------|
| Person | 170 | 50 | 192 |
| Car | 150 | 200 | 240 |
| Bottle | 25 | 30 | 100 |
| Cup | 12 | 25 | 80 |
| Chair | 90 | 100 | 150 |
| Laptop | 25 | 40 | 120 |
| Phone | 15 | 20 | 90 |

**Confidence Calculation:**
```python
box_area_ratio = (box_area / frame_area) × 100
center_distance = distance_from_frame_center × 2
center_score = 1.0 - center_distance

confidence = box_area_ratio × center_score
# Large boxes near center = high confidence
# Small boxes at edges = low confidence
```

### 4. Sensor Fusion

**Sensor Assignment by Position:**

```
          TOF2 (Left)     LIDAR/Ultra     TOF1 (Right)
               ↖              ↑              ↗
                 \            |            /
            -0.4  \           0          /  +0.4
                   \          |        /
                    \         |       /
                     \_______FOV_____/
                           102°
```

**Position Mapping:**
```python
position = (box_center_x - frame_width/2) / (frame_width/2)
# -1.0 = left edge, 0.0 = center, +1.0 = right edge

if position > 0.3:    # Right side
    use TOF1
elif position < -0.3: # Left side
    use TOF2
else:                 # Center
    use LIDAR or Ultrasonic
```

**Confidence-Weighted Fusion:**
```python
estimates = [
    {"distance": visual_dist, "confidence": visual_conf, "valid": True},
    {"distance": sensor_dist, "confidence": sensor_conf, "valid": sensor_ok}
]

weighted_sum = Σ(distance × confidence)
total_weight = Σ(confidence)
fused_distance = weighted_sum / total_weight
```

### 5. Sensor Health Monitor

**Health Metrics:**

```python
SensorHealth {
    last_value: float
    last_change_time: float
    stuck: bool
    valid: bool
    confidence: float (0-1)
    history: deque(10)
}
```

**Checks Performed:**

1. **Stuck Detection:**
   ```python
   if abs(current - last_value) < 1.0:  # No change
       time_stuck = now - last_change_time
       if time_stuck > 0.8:  # Stuck for 0.8s
           stuck = True
           confidence *= 0.9  # Degrade gradually
   ```

2. **Range Validation:**
   ```python
   if value < min_value or value > max_value:
       valid = False
       confidence = 0.0
   ```

3. **Temporal Plausibility:**
   ```python
   apparent_speed = abs(current - last_value) / time_delta
   if apparent_speed > 400 cm/s:  # Faster than reasonable
       confidence *= 0.5  # Likely glitch
   ```

### 6. Threat Assessment

**Scoring Function:**

```python
def calculate_threat_score(track):
    score = 0.0
    
    # COMPONENT 1: Distance urgency
    if distance < 50:
        # Exponential urgency in critical zone
        score += 100 × exp(-(distance / 20))
    elif distance < 150:
        # Linear urgency in warning zone
        score += 50 × (150 - distance) / 150
    elif distance < 300:
        # Low urgency in awareness zone
        score += 10 × (300 - distance) / 300
    
    # COMPONENT 2: Velocity factor
    if velocity < -5:  # Approaching
        score += abs(velocity) × 5
        if velocity < -20:  # Fast approach
            score += 50  # Danger bonus
    elif velocity > 5:  # Moving away
        score × 0.5  # Reduce urgency
    
    # COMPONENT 3: Object type weight
    weights = {
        "train": 4.0, "truck": 3.5, "bus": 3.5,
        "car": 3.0, "motorcycle": 2.5, "bicycle": 2.0,
        "person": 1.5, "dog": 1.3,
        "chair": 0.8, "bench": 0.9, "plant": 0.7,
        "bottle": 0.3, "cup": 0.3
    }
    score × weights.get(type, 1.0)
    
    # COMPONENT 4: Confidence adjustment
    if track.score > 0.7:
        score × 1.2  # High confidence detection
    elif track.score < 0.3:
        score × 0.8  # Low confidence detection
    
    return score
```

**Threat Levels:**

| Level | Score Range | Buzzer Frequency | Action |
|-------|-------------|------------------|--------|
| CRITICAL | >80 | 1200-1500 Hz | Immediate alert |
| WARNING | 40-80 | 800-1200 Hz | Strong warning |
| CAUTION | 10-40 | 400-800 Hz | Awareness |
| SAFE | <10 | Off | No action |

---

## Sensor Fusion Strategy

### Why Vision-Primary?

Traditional assistive devices use **sensor-primary** fusion:
- Sensors detect distance
- Camera identifies what's at that distance
- **Problem:** Blind spots where camera sees objects but sensors don't

Our **vision-primary** approach:
- Camera detects all visible objects (102° FOV)
- Visual estimator provides distance for ALL objects
- Sensors validate and refine when available
- **Advantage:** No blind spots, graceful degradation

### Fusion Logic Flow

```
1. Object detected by YOLO
   ↓
2. Visual distance estimated from bbox size
   ↓
3. Check object position in frame
   ↓
4. Assign appropriate sensor(s)
   ↓
5. Get sensor reading with health check
   ↓
6. Fuse visual + sensor:
   - If sensor valid & agrees: weighted average
   - If sensor valid & conflicts: log warning, use weighted
   - If sensor invalid: use visual only
   ↓
7. Update track with fused distance
```

### Example Fusion Scenarios

**Scenario A: Agreement (Ideal)**
```
Visual: 45cm (conf=0.8)
TOF1:   42cm (conf=1.0, healthy)
→ Fused: 43cm, high confidence
```

**Scenario B: Conflict**
```
Visual: 25cm (conf=0.7)
TOF2:    6cm (conf=0.2, STUCK)
→ Fused: 23cm, medium confidence (TOF downweighted)
→ Log: "⚠️ CONFLICT detected"
```

**Scenario C: Visual-Only**
```
Visual: 60cm (conf=0.6)
Sensor: None (object outside sensor FOV)
→ Fused: 60cm, medium confidence
→ Source: "Visual-ONLY"
```

---

## Performance Optimizations

### 1. Model Selection

**Comparison:**

| Model | Input Size | FPS | Accuracy | Choice |
|-------|------------|-----|----------|--------|
| YOLOv8m | 640×640 | 1.2 | 92% | ❌ Too slow |
| YOLOv8n | 640×640 | 3.4 | 87% | ❌ Still slow |
| YOLOv8n | 320×320 | 7.2 | 83% | ⚠️ Better |
| **YOLOv11n** | **320×320** | **8-10** | **85%** | ✅ **Best** |

### 2. Detection Frequency

**Strategy:** Detect every 3rd frame, track on all frames

```python
if frame_idx % 3 == 0:
    detections = yolo_inference(frame)
tracks = tracker.update(detections, current_time)
```

**Impact:**
- 3× fewer YOLO calls
- Tracking maintains continuity
- Effective FPS: 24-30 Hz (display + tracking)
- Detection FPS: 8-10 Hz (YOLO)

### 3. Track Expiry Tuning

**Problem:** Long expiry (2s) caused lag - objects left "ghost boxes"

**Solution:** Aggressive expiry (1s)
```python
expired = [t for t in tracks if current_time - t.last_seen > 1.0]
```

**Impact:**
- Eliminated lag effect
- Trade-off: Brief flicker if detection drops
- Balanced by higher IoU threshold (0.4)

### 4. Letterbox vs. Resize

**Why Letterbox?**
```python
# Simple resize distorts aspect ratio
resized = cv2.resize(img, (320, 320))  # ❌ Distorts objects

# Letterbox maintains aspect ratio
padded = letterbox_resize(img, 320)     # ✅ Preserves shapes
```

**Impact:**
- Better detection accuracy (+3%)
- Minimal performance cost (<5ms)

### 5. Median Filtering

**Why Not Mean?**
```python
# Mean is sensitive to outliers
lidar_values = [65, 64, 200, 66]  # One bad reading
mean = 99 cm  # ❌ Heavily affected

# Median is robust
median = 65 cm  # ✅ Ignores outlier
```

**Buffer Size:** 3 samples
- Smaller = responsive
- Larger = smoother but lagged

---

## Data Flow Example

**Input:**
```
Frame: 640×480 RGB
LIDAR: 65cm
TOF1: 58cm
TOF2: 6cm (STUCK)
Ultra: 62cm
```

**Detection:**
```
YOLOv11n → [person @ (100,150,250,400), conf=0.82]
```

**Visual Estimation:**
```
bbox_height = 250px
reference_height = 192px (person @ 50cm)
visual_distance = 50 × (192/250) = 38cm
visual_confidence = 0.75 (large box, near center)
```

**Sensor Assignment:**
```
position = (175 - 320) / 320 = -0.45 (left side)
→ Use TOF2 + LIDAR
```

**Sensor Health:**
```
TOF2: 6cm, STUCK, confidence=0.2
LIDAR: 65cm, OK, confidence=1.0
```

**Fusion:**
```
estimates = [
    Visual: 38cm × 0.75 = 28.5
    TOF2:    6cm × 0.2  =  1.2  (downweighted due to stuck)
    LIDAR:  65cm × 1.0  = 65.0
]
sum_weights = 0.75 + 0.2 + 1.0 = 1.95
fused = (28.5 + 1.2 + 65.0) / 1.95 = 49cm
```

**Velocity:**
```
Last distance: 60cm @ t=0
Current: 49cm @ t=0.3s
velocity = (49-60)/0.3 = -37 cm/s (approaching!)
```

**Threat:**
```
distance_score = 50 × (150-49)/150 = 34
velocity_score = 37 × 5 + 50 = 235 (fast approach bonus!)
type_weight = 1.5 (person)
final_score = (34 + 235) × 1.5 = 404 × 1.5 = 606
→ CRITICAL threat!
```

**Output:**
```
Buzzer: 1450 Hz (high urgency)
Display: RED box around person
CSV: ...,"person",606,"Visual+LIDAR"
```

---

## System Specifications

| Component | Specification |
|-----------|--------------|
| **Processing** | Raspberry Pi 4B, 4GB RAM, quad-core ARM Cortex-A72 |
| **Detection** | YOLOv11n ONNX, 320×320, 8-10 FPS |
| **Camera** | Arducam IMX708, 102° FOV, 640×480 @ 30 FPS |
| **Sensors** | LIDAR (0-800cm), TOF×2 (0-200cm), Ultrasonic (0-400cm) |
| **Latency** | <100ms end-to-end (capture → decision) |
| **Power** | ~3W total system draw |
| **Accuracy** | ±3cm (sensor), ±15cm (visual-only) |
| **Coverage** | 102° horizontal (camera), 45° (TOF), 8m forward (LIDAR) |

---

**Last Updated:** December 2024  
**Version:** 2.1
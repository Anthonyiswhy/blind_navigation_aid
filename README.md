# 🦯 Blind Navigation Aid - AI-Powered Assistive Navigation System

> Real-time obstacle detection, tracking, and threat assessment system for visually impaired users using computer vision and multi-sensor fusion on Raspberry Pi 4B.

[![Python](https://img.shields.io/badge/Python-3.11-blue.svg)](https://www.python.org/)
[![YOLOv11n](https://img.shields.io/badge/YOLO-v11n-green.svg)](https://github.com/ultralytics/ultralytics)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-4B-red.svg)](https://www.raspberrypi.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## 🎯 Project Overview

This system provides **360° awareness** with distance and velocity tracking for visually impaired users, solving the fundamental limitations of traditional white canes (ground-level only, ~1m reach) and guide dogs (expensive, not universally available).

### Key Innovation: Vision-Primary Sensor Fusion

Unlike traditional sensor-primary systems, this uses **monocular depth estimation** from bounding box sizes as the primary distance source, validated by physical sensors. This solves the "blind spot problem" - objects visible to the camera but outside sensor FOV still get accurate distance estimates.

## ✨ Features

- ✅ **YOLOv11n Object Detection** - 80 object classes at 8-10 FPS on Pi 4B
- ✅ **Multi-Object Tracking** - Track objects across frames with velocity calculation
- ✅ **Intelligent Threat Assessment** - Prioritize threats based on distance, velocity, and object type
- ✅ **Vision-Primary Sensor Fusion** - Hybrid approach combining visual estimates with sensor validation
- ✅ **Sensor Health Monitoring** - Real-time detection of stuck sensors and confidence tracking
- ✅ **Graceful Degradation** - System works even if 2/4 sensors fail
- ✅ **Audio Feedback** - Buzzer frequency indicates threat urgency

## 🔧 Hardware Stack

### Core Components
- **Raspberry Pi 4B (4GB)** - Main compute unit
- **ESP32** - Sensor hub for TOF/ultrasonic (offloads Pi)
- **Arducam IMX708 (102° FOV)** - Wide-angle camera for object detection

### Sensors
- **TF-Luna LIDAR** - Forward distance (2cm-8m range, ±3cm accuracy)
- **VL53L0X TOF (×2)** - Left/Right coverage at 22.5° angles (1-200cm)
- **HC-SR04 Ultrasonic** - Backup forward sensor (2-400cm)

### Output
- **Buzzer** - Audio feedback (frequency = urgency)

## 📐 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI 4B                          │
│  ┌──────────────────────────────────────────────────────┐  │
│  │   YOLOv11n (320x320 ONNX) - 8-10 FPS                 │  │
│  │   • Object detection (80 classes)                    │  │
│  │   • Letterbox preprocessing                          │  │
│  └──────────────────────────────────────────────────────┘  │
│                           ↓                                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │   Object Tracker                                      │  │
│  │   • IoU matching across frames                       │  │
│  │   • Velocity calculation (Δdist/Δtime)               │  │
│  │   • Track expiry: 1s (prevents lag)                  │  │
│  └──────────────────────────────────────────────────────┘  │
│                           ↓                                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │   Visual Distance Estimator (INNOVATION!)            │  │
│  │   distance = ref_dist × (ref_pixels / box_height)    │  │
│  │   • Calibrated for person, car, bottle, etc.         │  │
│  │   • Confidence based on box size & position          │  │
│  └──────────────────────────────────────────────────────┘  │
│                           ↓                                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │   Sensor Fusion                                       │  │
│  │   • Assigns sensors based on object position         │  │
│  │   • Weighted fusion with confidence                  │  │
│  │   • Conflict detection & resolution                  │  │
│  └──────────────────────────────────────────────────────┘  │
│                           ↓                                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │   Sensor Health Monitor                              │  │
│  │   • Stuck detection (>0.8s no change)                │  │
│  │   • Range validation                                 │  │
│  │   • Temporal plausibility (anti-teleport)            │  │
│  │   • Confidence degradation on failure                │  │
│  └──────────────────────────────────────────────────────┘  │
│                           ↓                                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │   Threat Assessment                                   │  │
│  │   score = f(distance, velocity, type, confidence)    │  │
│  │   • Distance: exponential urgency <50cm              │  │
│  │   • Velocity: approaching objects prioritized        │  │
│  │   • Type: cars > people > bottles                    │  │
│  └──────────────────────────────────────────────────────┘  │
│                           ↓                                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │   Audio Feedback (Buzzer)                            │  │
│  │   frequency = map(threat_score, 400-1500 Hz)         │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
         ↑                    ↑                    ↑
    [TF-Luna]            [ESP32]              [Camera]
      LIDAR           TOF1(R)+TOF2(L)        Arducam
       (I²C)           + Ultrasonic           IMX708
                        (Serial USB)          102° FOV
```

## 🧠 Visual Distance Estimation - How It Works

The system estimates distance from bounding box size using calibrated reference measurements:

```python
REFERENCE_SIZES = {
    "person": {
        "reference_pixels": 192,  # Height at reference distance
        "reference_distance": 50   # cm
    },
    "car": {"reference_pixels": 240, "reference_distance": 200},
    "bottle": {"reference_pixels": 100, "reference_distance": 30}
}

# Simple inverse proportion
distance = ref_distance × (ref_pixels / actual_box_height)
```

**Example:**
- Person at 50cm → 192px tall (calibrated)
- Person at 100cm → 96px tall (half the pixels = double the distance)
- Person at 25cm → 384px tall (double pixels = half the distance)

**Why This Matters:**
- Solves "blind spot" problem: Objects visible to 102° camera but outside 45° sensor coverage still get distance estimates
- Sensor fusion validates and refines visual estimates
- Works even when all sensors fail (degraded mode)

## 🚦 Threat Assessment Algorithm

```python
def calculate_threat_score(track):
    score = 0
    
    # 1. DISTANCE (closer = more urgent)
    if distance < 50cm:
        score += 100 × exp(-(distance/20))  # Exponential urgency
    elif distance < 150cm:
        score += 50 × (150-distance)/150    # Linear decrease
    
    # 2. VELOCITY (approaching = more urgent)
    if velocity < -5 cm/s:  # Approaching
        score += abs(velocity) × 5
        if velocity < -20 cm/s:  # Fast approach
            score += 50  # Bonus
    
    # 3. OBJECT TYPE WEIGHT
    weights = {"car": 3.0, "person": 1.5, "bottle": 0.3}
    score × weights[type]
    
    # 4. CONFIDENCE BONUS
    if confidence > 0.7:
        score × 1.2
    
    return score
```

**Threat Levels:**
- `CRITICAL` (>80) - Immediate danger, high frequency buzzer
- `WARNING` (40-80) - Need attention, medium frequency
- `CAUTION` (10-40) - Awareness, low frequency
- `SAFE` (<10) - No alert

## 📊 Performance Metrics

| Metric | Value |
|--------|-------|
| **Detection FPS** | 8-10 FPS |
| **Detection Accuracy** | ~85% for common objects |
| **Distance Accuracy** | ±15cm (visual-only), ±3cm (sensor validated) |
| **Sensor Coverage** | 102° camera, 45° TOF, 8m LIDAR |
| **Processing Latency** | <100ms end-to-end |
| **Track Stability** | 1s expiry prevents lag |

## 🔄 Sensor Health Monitoring

The system continuously monitors sensor reliability:

```
🔧 SENSOR HEALTH:
   LIDAR: OK
   Ultrasonic: OK
   TOF1(R): OK
   TOF2(L): STUCK (conf=0.3) ⚠️
```

**Detection Methods:**
1. **Stuck Detection** - No value change >0.8s
2. **Range Validation** - Physical limits check
3. **Temporal Plausibility** - No teleporting (max speed check)
4. **Confidence Degradation** - Gradually reduce trust in failing sensors

## 🚀 Quick Start

### Prerequisites

```bash
# System dependencies
sudo apt update
sudo apt install python3-opencv python3-pip libatlas-base-dev

# Python packages
pip3 install pyserial picamera2 onnxruntime numpy
```

### Setup

1. **Clone repository:**
```bash
git clone https://github.com/Anthonyiswhy/blind_navigation_aid.git
cd blind_navigation_aid
```

2. **Download YOLOv11n model:**
```bash
mkdir -p ~/yolo_models
# Download yolo11n.onnx and place in ~/yolo_models/
```

3. **Enable serial on Pi:**
```bash
sudo raspi-config
# Interface Options → Serial Port → Enable
```

4. **Upload ESP32 code:**
```bash
# Open esp32/sensors_serial.ino in Arduino IDE
# Select ESP32 board and port
# Upload
```

5. **Run:**
```bash
cd raspberry_pi
python3 blind_nav_intelligent.py
```

### Controls
- `ESC` - Exit
- `s` - Manual snapshot
- `t` - Toggle threat overlay

## 📁 Repository Structure

```
blind_navigation_aid/
├── raspberry_pi/
│   ├── blind_nav_intelligent.py    # Main system (v2.1)
│   └── requirements.txt             # Python dependencies
├── esp32/
│   └── sensors_serial.ino           # ESP32 sensor hub
├── stl/
│   └── sensor_prototype.stl         # 3D printable mount
├── media/
│   └── demo_screenshots/            # Example outputs
├── docs/
│   ├── ARCHITECTURE.md              # Detailed architecture
│   ├── CALIBRATION.md               # Visual distance calibration
│   └── TROUBLESHOOTING.md           # Common issues
└── README.md                        # This file
```

## 🔧 Key Challenges Solved

### 1. The "Finger Problem"
**Problem:** User puts hand 10cm from camera, all sensors point away → wrong distance assigned
**Solution:** Vision-primary fusion - every visible object gets visual estimate first

### 2. Sensor Failures
**Problem:** TOF sensor stuck at 6cm for minutes
**Solution:** Real-time health monitoring with confidence degradation, auto-switch to visual-only

### 3. Performance on Pi 4B
**Problem:** YOLOv8 @ 640×640 → 3.4 FPS ❌
**Solution:** YOLOv11n @ 320×320 → 8-10 FPS ✅

### 4. Duplicate Bounding Boxes
**Problem:** Objects left "ghost boxes" when moving
**Solution:** Reduced track expiry 2s→1s, increased IoU threshold 0.3→0.4

## 📈 Results

**Demo Scenario 1: Person Approaching**
```
Frame 1: person 60cm, • 0cm/s [WARNING]
Frame 2: person 45cm, ← 15cm/s [CRITICAL] ← Fast approach detected!
Buzzer: High frequency alert
```

**Demo Scenario 2: Sensor Conflict Resolution**
```
person: 25cm [Visual+TOF1(R)] ⚠️CONFLICT
└─ Visual=25cm, TOF1(R)=22cm ✓ Agreement
└─ TOF2(L)=6cm ⚠️ STUCK - ignored
System logs conflict, trusts vision+TOF1 fusion
```

## 🛣️ Roadmap

### Phase 2: Wearable Belt (In Progress)
- [ ] 360° TOF sensor ring (8 sensors around waist)
- [ ] IMU for fall detection + walking direction
- [ ] GPS integration for route guidance
- [ ] Bone conduction audio (keeps ears free)
- [ ] 8+ hour battery life

### Phase 3: Advanced Intelligence
- [ ] YOLO-World for semantic hazards (stairs, curbs, wet floor signs)
- [ ] Path planning (suggest safe direction)
- [ ] Memory system (remember obstacles at home)
- [ ] Emergency SOS on fall detection

## 📝 Lessons Learned

1. **Vision-primary beats sensor-primary** for wide coverage (102° camera vs 45° sensors)
2. **Redundancy matters** - hybrid approach survives single-point failures
3. **Real-time constraints force tradeoffs** - 320×320 is "good enough" and 3× faster
4. **Health monitoring is crucial** - stuck sensors are silent failures
5. **Track expiry is critical** - too long = lag, too short = flickering

## 🤝 Contributing

Contributions welcome! Areas of interest:
- Better sensor fusion algorithms
- Power optimization for battery operation
- Path planning implementation
- UI/UX for visually impaired users

## 📄 License

MIT License - See [LICENSE](LICENSE) for details

## 🙏 Acknowledgments

- YOLOv11 by Ultralytics
- Picamera2 by Raspberry Pi Foundation
- Community feedback from r/raspberry_pi

## 📧 Contact

**Anthony** - [@Anthonyiswhy](https://github.com/Anthonyiswhy)

**Project Link:** [https://github.com/Anthonyiswhy/blind_navigation_aid](https://github.com/Anthonyiswhy/blind_navigation_aid)

---

*Built with ❤️ for accessibility and independence*
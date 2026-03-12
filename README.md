# BlindNav — Intelligent Navigation Assistant for Blind Users

A wearable real-time navigation assistant that runs on a Raspberry Pi 4 and uses a depth camera to detect obstacles, estimate how fast they are approaching, and speak audio warnings to the user.

> **Current version:** v3.20 | **Primary tester:** Ricardo Salazar

---

## What It Does

The system continuously scans the environment with an Intel RealSense D435 depth camera, detects objects using YOLO, and announces warnings through Bluetooth headphones. It is designed to be worn on the chest.

**Example outputs:**
- *"Person ahead, 1.2 meters, approaching fast"*
- *"Chair on your left, 80 centimeters"*
- *"Path clear"*

Warnings are prioritized into three tiers based on distance and approach speed:
- **URGENT** — object under 40cm or closing in under 2 seconds
- **WARNING** — object under 70cm or closing in under 5 seconds  
- **AWARENESS** — object nearby but not an immediate threat

---

## Hardware

| Component | Purpose |
|-----------|---------|
| Raspberry Pi 4B (4GB) | Main compute |
| Intel RealSense D435 | RGB-D depth camera |
| ICM-20948 IMU | Detects whether user is walking or stationary |
| Piper TTS + Bluetooth headphones | Audio output |
| Chest harness mount | Wearable form factor |
| USB-C battery bank (10,000mAh) | ~4–6 hours runtime |

> The ESP32, LiDAR, and ToF sensors in the original repo are no longer part of this system. The project pivoted to a RealSense depth camera approach in early 2026.

---

## How It Works

```
RealSense D435
  ├── Color frame  ──▶  YOLO26n (224×224 ONNX)  ──▶  Object detections
  └── Depth frame  ──▶  Distance per object  ──▶  Velocity calculation
                                                         │
                                              Ego-motion compensation
                                              (subtracts camera's own
                                               movement from readings)
                                                         │
                                              Threat scoring + TTS
                                              (Piper neural TTS → aplay)
```

**Key design decisions:**
- **No NMS post-processing** — YOLO26n uses a one-to-one head that outputs pre-filtered detections, removing an entire processing stage
- **Pipelined capture** — frame capture and YOLO inference run in parallel threads, improving throughput ~30%
- **Ego-motion compensation** — Lucas-Kanade optical flow on background pixels estimates camera movement and subtracts it from object velocity, preventing false alarms when the user walks forward
- **IMU gate** — stationary users don't get warned about static objects; only moving/approaching objects trigger alerts

---

## Repository Structure

```
blind_navigation_aid/
├── raspberry_pi/
│   └── yolo_realsense_navigation.py   # Main script (v3.20)
├── tests/
│   └── test_blindnav.py               # Unit tests (37 tests, no hardware needed)
├── esp32/                             # Legacy ESP32 code (not used in current system)
├── stl/                               # 3D models for mount
├── media/                             # Photos and demo videos
├── SETUP.md                           # Installation and hardware wiring guide
├── STATUS.md                          # What works, what's untested, what's missing
└── README.md
```

---

## Quick Start

See [SETUP.md](SETUP.md) for full installation instructions.

```bash
# Activate environment and run
source ~/blindnav-venv/bin/activate
python3 raspberry_pi/yolo_realsense_navigation.py
```

Press `d` during runtime to trigger a scene description (Claude Vision).  
Press `Ctrl+C` to quit.

---

## Running Tests

No camera or model file needed:

```bash
pip install pytest
pytest tests/test_blindnav.py -v
```

---

## Performance (Raspberry Pi 4, headless)

| Metric | Value |
|--------|-------|
| FPS (YOLO11n) | 12–14 |
| FPS (YOLO26n, projected) | 18–20 |
| YOLO inference time | ~40ms (YOLO26n) |
| Detection resolution | 640×480 → 224×224 |
| ONNX threads | 4 |

---

## Configuration

Key settings at the top of `yolo_realsense_navigation.py`:

```python
CONF_THRESH       = 0.38   # Detection confidence threshold
DETECTION_INTERVAL = 2     # Run YOLO every N frames
RECORD_TO_FILE    = ""     # Set to a .bag path to record a session
PLAYBACK_FILE     = ""     # Set to a .bag path to replay a recording
```

---

## Version History

| Version | Key Change |
|---------|-----------|
| v3.7 | Performance baseline — 4-thread ONNX, pre-allocated buffers, 13 FPS |
| v3.12 | Three-tier alert system (URGENT/WARNING/AWARENESS) + ghost filter |
| v3.14 | Two-layer distance/TTC threat system |
| v3.15 | IMU integration (ICM-20948) + Piper neural TTS |
| v3.16 | Pending-voice queue, ThreatTransitionTracker, adaptive depth stride |
| v3.17 | Ego-motion compensation (Lucas-Kanade + background depth) |
| v3.18 | Bag file recording and playback |
| v3.19 | YOLO26n, pipelined capture, bounding-box EMA smoothing |
| v3.20 | Removed legacy NMS fallback, datetime fix, unit tests added |
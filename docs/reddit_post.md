# Built an AI-Powered Navigation Aid for the Blind on Raspberry Pi 4B

**TL;DR:** Created a real-time assistive navigation system using YOLOv11n, multiple distance sensors, and computer vision to help visually impaired people detect obstacles, calculate approach velocity, and prioritize threats. Runs at 8-10 FPS on Pi 4B with intelligent sensor fusion and automatic failure recovery.

---

## The Problem

Blind navigation is hard. White canes only detect ground-level obstacles within arm's reach (~1m). Guide dogs are expensive ($50k+) and not everyone can have one. I wanted to build something that provides **360° awareness** with distance and velocity information in real-time.

---

## Hardware Stack

- **Raspberry Pi 4B (4GB)** - main compute
- **Arducam IMX708 (102° FOV)** - wide-angle camera for object detection
- **TF-Luna LIDAR** - forward distance (2cm-8m range, ±3cm accuracy)
- **VL53L0X TOF sensors (×2)** - left/right coverage at 22.5° angles (1-200cm)
- **HC-SR04 Ultrasonic** - backup forward sensor (2-400cm)
- **ESP32** - handles TOF/ultrasonic via serial (offloads Pi)
- **Buzzer** - audio feedback (frequency = urgency)

---

## Software Architecture

### Core Components:

**1. YOLOv11n Object Detection (320×320 ONNX)**
- Detects 80 object classes in real-time
- Optimized for edge devices - runs at **8-10 FPS** on Pi 4B
- Letterbox preprocessing maintains aspect ratio

```python
# Efficient letterbox resizing for YOLOv11n
def letterbox_resize(img, target_size=320):
    scale = min(target_size / h, target_size / w)
    # Pad with gray borders to maintain aspect ratio
    padded = np.full((320, 320, 3), 114, dtype=np.uint8)
```

**2. Multi-Object Tracking with Velocity**
- Tracks objects across frames using IoU matching
- Calculates velocity from distance changes: `velocity = Δdistance / Δtime`
- Negative velocity = approaching (dangerous!)

**3. Vision-Primary Sensor Fusion (The Innovation!)**
- **Monocular depth estimation** from bounding box size
- Solves "blind spot" problem: objects visible to 102° camera but outside 45° sensor FOV
- Uses calibrated reference sizes for different object types

```python
# Visual distance estimation
REFERENCE_SIZES = {
    "person": {"reference_pixels": 192, "reference_distance": 50},
    "car": {"reference_pixels": 240, "reference_distance": 200}
}

distance = ref_distance * (ref_pixels / box_height_pixels)
```

**Why this matters:** If you put your hand 10cm from the camera but all physical sensors point away, traditional systems fail. My system estimates distance from the hand's size in the image, then validates with sensors when available.

**4. Sensor Health Monitoring**
- Detects stuck sensors (no value change for >0.8s)
- Range validation (physical limits)
- Temporal plausibility check (no teleporting)
- Auto-degrades confidence on failures

Example output:
```
🔧 SENSOR HEALTH:
   LIDAR: OK
   Ultrasonic: OK
   TOF1(R): OK
   TOF2(L): STUCK (conf=0.3) ⚠️
```

**5. Threat Assessment System**

```python
threat_score = f(distance, velocity, object_type, confidence)

# Distance: exponential urgency under 50cm
if distance < 50:
    score += 100 * exp(-(distance / 20))

# Velocity: fast approach = high priority
if velocity < -5:  # Approaching
    score += abs(velocity) * 5

# Object type: cars more dangerous than bottles
score *= {"car": 3.0, "person": 1.5, "bottle": 0.3}[type]
```

---

## Key Challenges Solved

**Challenge 1: "The Finger Problem"**
- User puts hand 10cm from camera
- All sensors point away → assign wrong distance (60cm from LIDAR)
- **Solution:** Vision-primary fusion - every object in camera view gets visual distance estimate first, sensors validate when they can see it

**Challenge 2: Sensor Failures**
- TOF sensor stuck at 6cm for minutes (happened during testing!)
- **Solution:** Real-time health monitoring with confidence degradation
- System automatically switches to visual-only when sensor fails

**Challenge 3: Performance on Pi 4B**
- Initial YOLOv8 @ 640×640: **3.4 FPS** ❌
- Optimized YOLOv11n @ 320×320: **8-10 FPS** ✅
- Techniques: Letterbox preprocessing, detection every 3 frames, aggressive track expiry

**Challenge 4: Duplicate Bounding Boxes (Lag Effect)**
- Objects left "ghost boxes" when moving
- **Solution:** Reduced track expiry from 2s → 1s, increased IoU threshold 0.3 → 0.4

---

## Results

- ✅ Detection accuracy: ~85% for common objects
- ✅ Distance accuracy: ±15cm (visual-only), ±3cm (with sensor validation)
- ✅ Velocity tracking: Real-time cm/s measurements
- ✅ FPS: 8-10 (suitable for navigation)
- ✅ Graceful degradation: Works even if 2/4 sensors fail

---

## Demo Scenarios

**Scenario 1: Person approaching**
```
Frame 1: person 60cm, • 0cm/s [WARNING]
Frame 2: person 45cm, ← 15cm/s [CRITICAL] ← Detected fast approach!
Buzzer: High frequency alert
```

**Scenario 2: Sensor conflict detected**
```
person: 25cm [Visual+TOF1(R)] ⚠️CONFLICT
└─ Visual=25cm, TOF1(R)=22cm ✓ Agreement
└─ TOF2(L)=6cm ⚠️ STUCK - ignored
System logs conflict, trusts vision+TOF1 fusion
```

---

## What's Next?

**Desk Prototype → Wearable Belt v2:**
- 360° TOF sensor ring (8 sensors around waist)
- IMU for fall detection + walking direction
- GPS integration for route guidance
- Bone conduction audio (keeps ears free)
- 8+ hour battery life target

**Software Improvements:**
- Add YOLO-World for semantic hazards (stairs, curbs, wet floor signs)
- Implement path planning (suggest safe direction to turn)
- Memory system (remember obstacles at home)
- Emergency SOS on fall detection

---

## Code & Architecture

**Full code on GitHub:** [github.com/Anthonyiswhy/blind_navigation_aid](https://github.com/Anthonyiswhy/blind_navigation_aid)

Includes:
- Complete Python implementation
- Sensor health logs with conflict detection
- Distance calibration data
- Architecture diagrams

---

## Lessons Learned

1. **Vision-primary beats sensor-primary** for wide coverage (102° camera vs 45° sensors)
2. **Redundancy matters** - hybrid approach survives single-point failures
3. **Real-time constraints force tradeoffs** - 320×320 YOLO is "good enough" and 3× faster than 640×640
4. **Health monitoring is crucial** - stuck sensors are silent failures without detection
5. **Track expiry is critical** - too long = lag, too short = flickering

---

## Questions for the Community

1. **Sensor fusion:** Anyone experienced with robotics sensor fusion? Better alternatives to weighted averaging?
2. **360° coverage:** Best approach for wearable? Considering LiDAR360 vs ring of 8 TOFs
3. **Power optimization:** Tips for 8+ hour battery life on Pi 4B? Currently draws ~3W
4. **Audio feedback:** Experience with bone conduction for navigation? Concerned about ear safety with constant buzzer

---

## Tech Stack Summary

- **Hardware:** Pi 4B (4GB), Arducam IMX708, TF-Luna LIDAR, 2× VL53L0X TOF, HC-SR04, ESP32
- **Software:** Python 3.11, YOLOv11n ONNX (ONNXRuntime), OpenCV, NumPy, Picamera2
- **Detection:** 320×320 input, 8-10 FPS, 80 COCO classes
- **Sensors:** I²C (LIDAR), Serial (ESP32→Pi), GPIO (Buzzer)

---

This project taught me more about real-time systems, sensor fusion, and edge AI than any tutorial could. The moment it detected my hand approaching and ramped up the buzzer frequency was incredible - it felt like giving someone back a sense!

Happy to answer questions or discuss alternative approaches. Would love feedback from the community on improving the design! 🚀

**Update:** Added sensor health monitoring after a TOF got stuck during testing and went unnoticed for 10 minutes. Now the system automatically detects and reports sensor failures with confidence degradation.

---

*PS: If you're working on assistive tech or accessibility projects, I'd love to chat! Always looking to learn from others in this space.*
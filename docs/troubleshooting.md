# Troubleshooting Guide

Common issues and solutions for the Blind Navigation Aid system.

## Table of Contents

1. [Installation Issues](#installation-issues)
2. [Camera Problems](#camera-problems)
3. [Sensor Issues](#sensor-issues)
4. [Detection Problems](#detection-problems)
5. [Performance Issues](#performance-issues)
6. [System Errors](#system-errors)

---

## Installation Issues

### Python Dependencies

**Problem:** `ModuleNotFoundError: No module named 'cv2'`

**Solution:**
```bash
# Install OpenCV
sudo apt install python3-opencv

# Or via pip
pip3 install opencv-python
```

**Problem:** `ImportError: libonnxruntime.so: cannot open shared object file`

**Solution:**
```bash
# Install ONNX Runtime
pip3 install onnxruntime

# For better performance, install optimized version
pip3 install onnxruntime-armv7l  # For Pi 3/4 32-bit
# or
pip3 install onnxruntime-aarch64  # For Pi 64-bit OS
```

### Serial Port Access

**Problem:** `serial.serialutil.SerialException: [Errno 13] Permission denied: '/dev/ttyUSB0'`

**Solution:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Logout and login, or reboot
sudo reboot
```

**Problem:** ESP32 not detected at `/dev/ttyUSB0`

**Solution:**
```bash
# Check connected devices
ls -l /dev/tty*

# Common alternatives:
# /dev/ttyUSB1
# /dev/ttyACM0
# /dev/serial0

# Update ESP32_PORT in code
ESP32_PORT = "/dev/ttyACM0"  # Adjust as needed
```

---

## Camera Problems

### Camera Not Detected

**Problem:** `RuntimeError: Failed to open camera`

**Solution:**
```bash
# Check camera connection
vcgencmd get_camera

# Should show: supported=1 detected=1

# Enable camera interface
sudo raspi-config
# Interface Options → Camera → Enable

# Reboot
sudo reboot
```

### Poor Image Quality

**Problem:** Blurry or dark images

**Solution:**
```python
# Adjust camera controls in code
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    controls={
        "AwbEnable": True,
        "ExposureTime": 10000,  # Adjust for lighting
        "AnalogueGain": 2.0      # Increase for low light
    }
)
```

### Camera Orientation

**Problem:** Image is upside down

**Solution:**
```python
# Change rotation in code
frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)  # Current
# or
frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_90_CLOCKWISE)
```

---

## Sensor Issues

### LIDAR Not Responding

**Problem:** `latest_lidar_cm = None` always

**Solution:**
```bash
# Check serial connection
ls -l /dev/serial0

# Enable UART
sudo raspi-config
# Interface Options → Serial Port
# Login shell: NO
# Serial hardware: YES

# Check Raspberry Pi config
sudo nano /boot/config.txt
# Should have: enable_uart=1
```

**Problem:** LIDAR readings stuck at 0 or 800

**Solution:**
- Check wiring (TX→RX, RX→TX, GND, 5V)
- Verify baud rate matches (115200)
- Try different UART port
- Check for interference (keep away from motors)

### TOF Sensors Stuck

**Problem:** `TOF1(R): STUCK (conf=0.3)` warning

**Diagnosis:**
```
🔧 SENSOR HEALTH:
   TOF1(R): STUCK (conf=0.3) ⚠️
   # Sensor reporting same value for >0.8s
```

**Solutions:**

1. **Physical obstruction**
   ```bash
   # Check if something is blocking sensor
   # Remove any objects within 5cm
   ```

2. **Wiring issue**
   ```bash
   # Check I²C connections on ESP32
   # Verify SDA/SCL pins
   # Check for loose connections
   ```

3. **I²C address conflict**
   ```cpp
   // In ESP32 code, verify addresses
   VL53L0X sensor1;  // Address 0x29 (default)
   VL53L0X sensor2;  // Address 0x30 (after change)
   ```

4. **Sensor failure**
   - System will automatically downweight stuck sensor
   - Replace sensor if stuck persists

### Ultrasonic Erratic Readings

**Problem:** Ultrasonic jumps between values

**Solution:**
```python
# Increase median buffer size
ultra_buf = collections.deque(maxlen=5)  # Was 3

# Or add additional filtering
def smooth_ultrasonic(value, last_value):
    if abs(value - last_value) > 50:  # Spike
        return last_value  # Ignore
    return value
```

---

## Detection Problems

### No Objects Detected

**Problem:** `Tracked: 0` always

**Solution:**

1. **Check model path**
   ```python
   MODEL_PATH = os.path.expanduser("~/yolo_models/yolo11n.onnx")
   # Verify file exists
   print(f"Model exists: {os.path.exists(MODEL_PATH)}")
   ```

2. **Lower confidence threshold**
   ```python
   CONF_THRESH = 0.20  # Was 0.30
   ```

3. **Check camera view**
   ```python
   # Save raw frame to verify camera works
   cv2.imwrite("debug_frame.jpg", frame_display)
   ```

### Too Many False Positives

**Problem:** Detecting objects that aren't there

**Solution:**

1. **Increase confidence threshold**
   ```python
   CONF_THRESH = 0.40  # Was 0.30
   ```

2. **Increase NMS threshold**
   ```python
   NMS_THRESH = 0.50  # Was 0.45
   ```

3. **Filter by size**
   ```python
   # In yolo_postprocess, add minimum box size
   if (x2 - x1) < 20 or (y2 - y1) < 20:  # Was 10
       continue  # Skip tiny boxes
   ```

### Duplicate Detections

**Problem:** Multiple boxes on same object

**Solution:**

1. **Increase NMS threshold**
   ```python
   NMS_THRESH = 0.55  # Higher = more aggressive merging
   ```

2. **Increase tracking IoU**
   ```python
   best_iou = 0.5  # Was 0.4 in ObjectTracker
   ```

### Ghost Boxes (Lag)

**Problem:** Boxes remain after object moves

**Solution:**

1. **Reduce track expiry**
   ```python
   tracker = ObjectTracker(max_age=0.8)  # Was 1.0
   ```

2. **Increase IoU threshold**
   ```python
   best_iou = 0.45  # Higher = stricter matching
   ```

---

## Performance Issues

### Low FPS (<5)

**Problem:** System running slowly

**Solutions:**

1. **Reduce input size**
   ```python
   IMG_SIZE = 256  # Was 320
   ```

2. **Increase detection interval**
   ```python
   DETECTION_INTERVAL = 5  # Was 3 (detect every 5th frame)
   ```

3. **Disable display**
   ```python
   # Comment out imshow for headless operation
   # cv2.imshow("Intelligent Navigation", frame_display)
   ```

4. **Close other applications**
   ```bash
   # Check CPU usage
   htop
   
   # Kill unnecessary processes
   sudo systemctl stop bluetooth
   ```

5. **Overclock Pi (careful!)**
   ```bash
   sudo nano /boot/config.txt
   # Add:
   over_voltage=6
   arm_freq=2000
   gpu_freq=750
   ```

### High CPU Usage

**Problem:** CPU at 100% constantly

**Solution:**

1. **Enable camera hardware acceleration**
   ```python
   # Ensure using Picamera2 (hardware accelerated)
   # Not regular OpenCV VideoCapture
   ```

2. **Reduce camera framerate**
   ```python
   controls={"FrameRate": 15}  # Was 30
   ```

### Memory Issues

**Problem:** `MemoryError` or system freezing

**Solution:**

1. **Increase swap**
   ```bash
   sudo dphys-swapfile swapoff
   sudo nano /etc/dphys-swapfile
   # CONF_SWAPSIZE=2048  # Was 100
   sudo dphys-swapfile setup
   sudo dphys-swapfile swapon
   ```

2. **Reduce track history**
   ```python
   self.position_history = deque(maxlen=3)  # Was 5
   self.distance_history = deque(maxlen=3)  # Was 5
   ```

---

## System Errors

### ONNX Runtime Errors

**Problem:** `ONNXRuntimeError: The parameter is incorrect`

**Solution:**

1. **Check input shape**
   ```python
   print(f"Input shape: {img_in.shape}")
   # Should be (1, 3, 320, 320)
   ```

2. **Verify model compatibility**
   ```bash
   # Re-export model with correct opset
   # yolo export model=yolo11n.pt format=onnx opset=12
   ```

### Buzzer Not Working

**Problem:** No audio feedback

**Solution:**

1. **Check GPIO pin**
   ```python
   BUZZER_PIN = 18  # BCM numbering
   # Try different pin if needed
   ```

2. **Test buzzer manually**
   ```python
   import RPi.GPIO as GPIO
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(18, GPIO.OUT)
   pwm = GPIO.PWM(18, 1000)
   pwm.start(50)
   # Should hear tone
   ```

3. **Check permissions**
   ```bash
   # Add user to gpio group
   sudo usermod -a -G gpio $USER
   ```

### CSV Logging Errors

**Problem:** `PermissionError: [Errno 13] Permission denied`

**Solution:**

1. **Check folder permissions**
   ```bash
   ls -ld ~/blindnav_logs
   # Should show drwxr-xr-x
   
   # Fix if needed
   chmod 755 ~/blindnav_logs
   ```

2. **Check disk space**
   ```bash
   df -h
   # Ensure sufficient space on /home
   ```

---

## Diagnostic Commands

### System Health Check

```bash
#!/bin/bash
echo "=== System Health Check ==="

echo "Camera:"
vcgencmd get_camera

echo "Temperature:"
vcgencmd measure_temp

echo "CPU freq:"
vcgencmd measure_clock arm

echo "Memory:"
free -h

echo "Disk:"
df -h /home

echo "USB devices:"
lsusb

echo "Serial ports:"
ls -l /dev/tty*

echo "Python packages:"
pip3 list | grep -E "opencv|onnx|picamera|serial"
```

### Run with Debug Output

```bash
# Enable verbose logging
python3 blind_nav_intelligent.py 2>&1 | tee debug.log

# Check logs
tail -f debug.log
```

### Monitor Resource Usage

```bash
# CPU and memory in real-time
htop

# GPU memory (camera)
vcgencmd get_mem gpu

# Temperature monitoring
watch -n 1 vcgencmd measure_temp
```

---

## Getting Help

If you're still having issues:

1. **Check logs**
   ```bash
   # System logs
   journalctl -xe
   
   # Application logs
   cat ~/blindnav_logs/log_*.csv
   ```

2. **Create issue on GitHub**
   - Include system info (`uname -a`)
   - Include Python version (`python3 --version`)
   - Include error messages
   - Include relevant code sections

3. **Test sensors individually**
   ```python
   # Create minimal test scripts
   # test_lidar.py, test_esp32.py, test_camera.py
   ```

---

## Common Error Messages

| Error | Likely Cause | Solution |
|-------|-------------|----------|
| `cv2.error: (-215:Assertion failed)` | Invalid image/frame | Check camera capture |
| `SerialException: device reports readiness` | Bad USB connection | Replug ESP32, check cable |
| `MemoryError` | Insufficient RAM | Reduce buffer sizes, increase swap |
| `ONNXRuntimeError` | Model/input mismatch | Verify model format and input shape |
| `GPIO.error` | Permission issue | Add user to gpio group |
| `Failed to open camera` | Camera not enabled | Run raspi-config |

---

**Last Updated:** December 2024  
**Version:** 2.1

For more help, open an issue at: https://github.com/Anthonyiswhy/blind_navigation_aid/issues
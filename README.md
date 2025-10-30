# Blind Navigation Aid

A wearable prototype that helps visually impaired users navigate using LiDAR, multiple ToF distance sensors, OpenCV object detection, and haptic + audio feedback.

## Hardware
- Raspberry Pi 4B
- ESP32
- LiDAR module
- 4x VL53L0X Time-of-Flight sensors
- Ultrasonic sensor
- Pi camera + YOLO object detection
- Buzzer + vibration motor

## Features
- Real-time obstacle detection
- Haptic and audio feedback
- Object recognition with YOLO + OpenCV
- Wireless communication between ESP32 and Pi
## Structure
- `raspberry_pi/` - Raspberry Pi scripts (Picamera2 + OpenCV + LiDAR + ESP32 + buzzer + CSV logging)
- `esp32/` - Arduino sketch to read ToF sensors, ultrasonic and stream serial to Pi
- `stl/` - 3D Model for Stand

## Quick start (Pi)
1. Copy `pi/` folder on your Pi.
2. Install system deps (you already installed libcamera/picamera2):
   - `sudo apt update && sudo apt install python3-opencv python3-pip libatlas-base-dev`
   - `pip3 install pyserial`
   - If using the DNN detection, download MobileNetSSD files into `~/models/`.
3. Enable serial:
   - `sudo raspi-config` -> Interface Options -> Serial Port. Make sure the Pi UART is available as `/dev/serial0`.
4. Run:
   - `python3 main.py`
   - Press `q` to quit.

## Quick start (ESP32)
1. Open `esp32/sensors_serial.ino` in Arduino IDE.
2. Select your ESP32 board & port, upload.
3. Connect ESP32 to Pi via USB; Pi should see `/dev/ttyUSB0`.

## Files to add
- Put your `sensor_prototype.stl` into `/stl/`.
- The SCAD file for modeling is in `/cad/sensor_mount.scad`.

## Notes & troubleshooting
- Camera FOV used ~102° horizontal — if your camera differs, edit `compute_projection_x_px` in `pi/main.py`.
- If the DNN model isn't present, detection is disabled and script still runs sensors + overlay.
- If ToF libs differ (Adafruit vs Pololu) update the ESP32 sketch accordingly.


## Goal
Improve safe navigation and independence for visually impaired users.

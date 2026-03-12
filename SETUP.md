# Setup Guide

## Requirements

- Raspberry Pi 4B (4GB recommended)
- Intel RealSense D435 camera
- ICM-20948 IMU breakout board
- Bluetooth headphones (A2DP)
- USB-C battery bank

---

## 1. OS and System Dependencies

Start with Raspberry Pi OS Bookworm (64-bit, headless).

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-venv python3-opencv \
    libatlas-base-dev libasound2-dev portaudio19-dev \
    espeak ffmpeg i2c-tools
```

Enable I2C for the IMU:
```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

---

## 2. Python Environment

```bash
python3 -m venv ~/blindnav-venv
source ~/blindnav-venv/bin/activate

pip install pyrealsense2 onnxruntime numpy opencv-python \
    anthropic smbus2
```

> **Note:** `pyrealsense2` may need to be built from source on Pi. Pre-built wheels are available at https://github.com/IntelRealSense/librealsense/releases — look for the ARM64 `.whl` file matching your Python version.

---

## 3. YOLO26n Model

On any machine with `ultralytics` installed:

```bash
pip install ultralytics
python3 -c "
from ultralytics import YOLO
model = YOLO('yolo26n.pt')
model.export(format='onnx', imgsz=224, opset=13, simplify=True)
"
```

Copy the output `yolo26n.onnx` to the Pi:
```bash
scp yolo26n.onnx pi@raspberrypi:~/yolo_models/yolo26n.onnx
```

Verify the output shape is correct:
```bash
python3 -c "
import onnxruntime as ort
sess = ort.InferenceSession('~/yolo_models/yolo26n.onnx', providers=['CPUExecutionProvider'])
print(sess.get_outputs()[0].shape)  # expect [1, 300, 6]
"
```

---

## 4. Piper TTS

```bash
# Download binary
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/piper_arm64.tar.gz
tar -xzf piper_arm64.tar.gz -C ~/piper

# Download voice model
mkdir -p ~/piper_voices
cd ~/piper_voices
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx.json
```

Test:
```bash
echo "Navigation active" | ~/piper/piper --model ~/piper_voices/en_US-amy-medium.onnx --output_raw | aplay -r 22050 -f S16_LE -t raw -
```

---

## 5. IMU Wiring (ICM-20948)

Connect the ICM-20948 breakout board to the Pi GPIO header:

| IMU Pin | Pi Pin | GPIO |
|---------|--------|------|
| VCC | Pin 1 | 3.3V |
| GND | Pin 6 | GND |
| SDA | Pin 3 | GPIO 2 |
| SCL | Pin 5 | GPIO 3 |

Verify it appears on the I2C bus:
```bash
i2cdetect -y 1
# Should show 0x68 or 0x69
```

---

## 6. Camera Mount

The camera is mounted on a chest harness (TEKCAM J-hook style) using a GoPro-to-1/4"-20 adapter. The RealSense D435 has a standard 1/4"-20 tripod thread on the bottom.

Angle the camera slightly downward (~10–15°) to cover ground-level obstacles. USB cable routes over the shoulder to the Pi in a jacket pocket.

---

## 7. Running

```bash
source ~/blindnav-venv/bin/activate
python3 raspberry_pi/yolo_realsense_navigation.py
```

**Runtime keys:**
- `d` — trigger scene description (uses Claude Vision API)
- `Ctrl+C` — quit

**Logs** are saved automatically to `~/blindnav_logs/` as CSV files.

---

## 8. Recording Test Scenarios

To record a bag file for regression testing:

```python
# In yolo_realsense_navigation.py, set:
RECORD_TO_FILE = "/home/anthonyiswhy/bags/test_walking.bag"
```

To replay:
```python
PLAYBACK_FILE = "/home/anthonyiswhy/bags/test_walking.bag"
```

Recommended scenarios to record:
- Person walking toward camera at normal pace
- User walking toward a stationary chair
- Person standing still at 40cm (close range)
- Plain white wall corridor (optical flow fallback)
- Person approaching then turning away

---

## 9. Running Tests

```bash
pip install pytest
pytest tests/test_blindnav.py -v
# Expected: 37 passed
```
# Setup Guide

## Hardware

- Raspberry Pi 4
- Intel RealSense D435
- ICM-20948 IMU on I2C address `0x68`
- Bluetooth headphones using A2DP
- Chest harness with the camera angled slightly downward

## OS and System Packages

Start with Raspberry Pi OS Bookworm 64-bit.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-venv python3-opencv \
    libatlas-base-dev libasound2-dev espeak ffmpeg i2c-tools
```

Enable I2C:

```bash
sudo raspi-config
# Interface Options -> I2C -> Enable
```

## Python Environment

```bash
python3 -m venv ~/blindnav-venv
source ~/blindnav-venv/bin/activate
pip install numpy==1.26.4 onnxruntime opencv-python anthropic smbus2 pytest
```

Important notes:

- Do not install `numpy` 2.0+.
- Do not rely on PyPI `pyrealsense2` ARM wheels. Build or install the working
  ARM64 package already used on the Pi environment.

## YOLO26n Export

Export from a machine with `ultralytics` installed:

```bash
python3 - <<'PY'
from ultralytics import YOLO
model = YOLO("yolo26n.pt")
model.export(format="onnx", imgsz=224, opset=13, simplify=True)
PY
```

Copy to the Pi:

```bash
mkdir -p ~/yolo_models
scp yolo26n.onnx pi@raspberrypi:~/yolo_models/yolo26n.onnx
```

Verify the output shape:

```bash
python3 - <<'PY'
import onnxruntime as ort
sess = ort.InferenceSession("/home/pi/yolo_models/yolo26n.onnx",
                            providers=["CPUExecutionProvider"])
print(sess.get_outputs()[0].shape)
PY
```

Expected output: `(1, 300, 6)`

## Piper TTS

Install Piper and download the `en_US-amy-medium` voice model. The runtime
expects:

- `~/piper_voices/en_US-amy-medium.onnx`
- `~/piper_voices/en_US-amy-medium.onnx.json`

## IMU Wiring

| IMU Pin | Pi Pin |
|--------|--------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 2 |
| SCL | GPIO 3 |

Check the bus:

```bash
i2cdetect -y 1
```

## Run

```bash
source ~/blindnav-venv/bin/activate
export ANTHROPIC_API_KEY="sk-..."
python3 raspberry_pi/yolo_realsense_navigation.py
```

## Display Version

Do not use `cv2.imshow` on the Pi. For visual debugging, use the Flask MJPEG
display script and open `http://<pi-ip>:5000` from another device.

## Tests

These tests require no hardware:

```bash
pytest tests/test_blindnav.py tests/test_blindnav_v326.py -v
```

Expected current result: `150 passed`

## Health Checks Before Field Use

```bash
vcgencmd measure_temp
i2cdetect -y 1
hostname -I
```

Field-use rule: keep the Pi below 70 C before a walking session.

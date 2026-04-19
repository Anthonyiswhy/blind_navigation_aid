# BlindNav

Wearable navigation assistant for blind users built for Raspberry Pi 4 + Intel
RealSense D435. The system detects obstacles, estimates threat from distance
and time-to-collision, and speaks warnings through Bluetooth headphones using
Piper neural TTS.

Current production version: `v3.26b HEADLESS`

- Production script: `raspberry_pi/yolo_realsense_navigation.py`
- Foundational regression suite: `tests/test_blindnav.py`
- Advanced voice/latency regression suite: `tests/test_blindnav_v326.py`
- Verified locally on April 18, 2026: `150 passed`

## What It Does

- Runs YOLO26n ONNX inference on Pi 4 CPU.
- Samples depth per tracked object using adaptive stride + clustering.
- Compensates apparent approach speed using background-depth ego-motion.
- Suppresses static-object chatter when the user is standing still.
- Speaks left/right/ahead warnings with distance-aware cooldown buckets.
- Logs per-alert latency timestamps to `events.log`.
- Provides on-demand scene description with the `d` key.

## Recent Changes

### v3.25

- Reduced ONNX Runtime to 3 threads so Piper synthesis gets CPU time.
- Added ego-Z clamp and confidence gating to block impossible velocity spikes.
- Switched to zone-based voice cooldown keys so tracker ID churn does not
  retrigger the same warning.
- Added per-alert latency logging.

### v3.26b

- Extracted `_select_voice_message()` so alert wording is unit-testable.
- Fixed neutral wording leakage in close-distance branches when ego-motion is
  unreliable.
- Added safe urgent supersession: an urgent alert can cancel a lower-priority
  phrase only while that phrase is still synthesizing.
- Replaced terminate-style preemption with BT-safe skip-ahead before playback.
- Preserved the hard rule that active `aplay` playback is never terminated.

## Hard Rules

- Never send SIGTERM to `aplay`.
- Never use `numpy` 2.0+; pin `numpy==1.26.4`.
- Never use `cv2.imshow` on the Pi; use the Flask MJPEG stream for display work.
- Never put the ghost filter inside `ObjectTracker`.
- Never announce a threat as cleared while its compensated velocity is still
  negative enough to indicate approach.

## Repository Layout

```text
blind_navigation_aid/
|-- AGENTS.md
|-- README.md
|-- SETUP.md
|-- STATUS.md
|-- raspberry_pi/
|   `-- yolo_realsense_navigation.py
|-- tests/
|   |-- test_blindnav.py
|   `-- test_blindnav_v326.py
`-- .github/workflows/tests.yml
```

## Quick Start

```bash
source ~/blindnav-venv/bin/activate
export ANTHROPIC_API_KEY="sk-..."
python3 raspberry_pi/yolo_realsense_navigation.py
```

Press `d` for a scene description. Use `Ctrl+C` to exit.

## Tests

All tests run without camera hardware, a RealSense device, Piper, or an IMU.
Hardware modules are stubbed at import time.

```bash
pytest tests/test_blindnav.py tests/test_blindnav_v326.py -v
```

Current collected totals:

- `tests/test_blindnav.py`: 37 tests
- `tests/test_blindnav_v326.py`: 113 tests
- Combined: 150 tests

## Performance Notes

- Expected field FPS: roughly 8-14 depending on thermals.
- YOLO export must produce output shape `(1, 300, 6)`.
- ONNX Runtime stays on float32. INT8 was slower on Pi 4 ARM in project tests.
- Thermal throttling is still the main real-world performance limiter.

## Current Priorities

- Add a heatsink before field sessions.
- Push the v3.26b repo state and field-test it with Ricardo Salazar.
- Record bag-file scenarios for regression playback.
- Add traffic-light color classification after the base obstacle system is
  stable.

## Design Notes

- Urgent audio is optimized for freshness, but active playback is not forcibly
  interrupted because Bluetooth stream renegotiation is worse than waiting for a
  short phrase to finish.
- Queueing, cooldown, latency, wording, and ego-motion regressions are all
  testable without hardware and are now covered in the advanced suite.

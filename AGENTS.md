# AGENTS.md — BlindNav Navigation Assistant
**Drop this file in the root of `blind_navigation_aid/` so Codex reads it every session.**

Wearable AI navigation assistant for blind users. Raspberry Pi 4 + Intel RealSense D435 depth
camera on a chest harness. Detects obstacles, scores threats, speaks warnings through Bluetooth
headphones using Piper neural TTS. On-demand scene description via Codex Vision API.

**Current production version:** v3.28 HEADLESS
**Production file:** `raspberry_pi/yolo_realsense_navigation.py`
**Test files:** `tests/test_blindnav.py` + `tests/test_blindnav_v326.py` (172 collected tests, no hardware required)

---

## Commands

```bash
# Run (on Pi)
source ~/blindnav-venv/bin/activate
export ANTHROPIC_API_KEY="sk-..."
python3 raspberry_pi/yolo_realsense_navigation.py

# Run display version (browser stream)
python3 ~/blindnav_code/yolo_realsense_navigation_vX.XX_DISPLAY.py
# Then open: http://$(hostname -I | awk '{print $1}'):5000

# Tests (no camera, no hardware, runs anywhere)
cd ~/blind_navigation_aid
pytest tests/test_blindnav.py tests/test_blindnav_v326.py -v

# Health check (Pi)
vcgencmd measure_temp   # must be < 70°C before field use
i2cdetect -y 1          # 0x68 = IMU present
hostname -I             # Pi IP for browser stream

# Git push workflow
cp ~/blindnav_code/yolo_realsense_navigation_vX.XX_HEADLESS.py \
   raspberry_pi/yolo_realsense_navigation.py
git add -A && git commit -m "vX.XX: description" && git push
# If push rejected: git pull --rebase first
```

---

## Hard Rules — Never Break These

**NEVER send SIGTERM to aplay.**
Killing aplay on Bluetooth A2DP drops the codec stream and forces 1–2s re-negotiation on every
subsequent phrase. The whole voice system uses a pending-queue pattern (`allow_pending=True`)
precisely to avoid this. If voice seems broken, fix the queue, not the process.

**NEVER use numpy 2.0+.**
This codebase requires `numpy==1.26.4`. The `np.float` attribute was removed in 2.0 and ONNX
inference code breaks silently. If you see `AttributeError: module 'numpy' has no attribute 'float'`,
the fix is always `pip install numpy==1.26.4`.

**NEVER use cv2.imshow on the Pi.**
Pi OS Bookworm defaults to Wayland. OpenCV's Qt backend in a pip venv can't find the Wayland
plugin — you get a black window with no error. Use the Flask MJPEG stream (port 5000) instead.
The DISPLAY version of the script already does this.

**NEVER use INT8 quantization for YOLO on Pi 4 ARM.**
Tested in v3.22: INT8 was 252ms vs 130ms float32. ONNX Runtime dynamic quantization doesn't speed
up this op set on ARM Cortex-A72. Always use float32 yolo26n.onnx.

**NEVER put the ghost filter inside ObjectTracker.**
`seen_frames < 3` filtering happens in the main loop, after `tracker.update()` returns. The tracker
itself returns ALL tracks. Several unit tests (specifically `test_returns_all_tracks`) verify this
separation. Breaking it will silently skip voice alerts for newly appearing objects.

**NEVER announce "cleared" while velocity is still negative.**
`ThreatTransitionTracker._announce_cleared()` has a velocity guard: if `vel < -10 cm/s` the
object is still approaching and the cleared announcement is blocked. `MIN_THREAT_FRAMES=12` alone
is not enough — we burned this in v3.21 when the system said "person stopped" at -35 cm/s.

---

## Architecture Map

```
_capture_worker (daemon thread)
  └─ pipeline.wait_for_frames() → align → queue.Queue(maxsize=2) [drops stale]

Main loop (every frame)
  ├─ YOLO26n inference (every 2 frames) → yolo_postprocess() → detections list
  ├─ ObjectTracker.update() → all tracks (ghost filter NOT here)
  ├─ get_smart_distance() → adaptive stride depth sampling per track
  ├─ EgoMotionCompensator.update() → camera_z_velocity (every 2 frames)
  ├─ Track.update_distance(dist, now, camera_z_velocity) → compensated velocity
  ├─ MotionDetector.update() + is_moving() → user_moving (with hysteresis)
  ├─ SceneDescriber.capture_and_describe() → no-op unless 'd' key pressed
  ├─ ThreatAssessment.prioritize_threats() → sorted [(score, track)]
  ├─ ThreatTransitionTracker.update() → "moved away" / "path clear" announcements
  ├─ Wall fallback (depth-only, center 30% of frame) → catches glass/walls YOLO misses
  ├─ Busy area (≥4 confirmed tracks) → "Busy area, slow down"
  └─ Voice announcement logic → ghost filter + IMU gate + two-layer alerts
```

**Two-layer alert system (do not collapse into one):**
- Layer 1: Distance override — dist <40cm → URGENT always; dist <70cm → WARNING always
- Layer 2: TTC — TTC <2s → URGENT; TTC 2–4s → WARNING; TTC 4–8s → AWARENESS

**Voice cooldown keys are zone-based and include distance bucket:** `_voice_key(pos, class_name, tier, int(dist_cm//30))`
This survives tracker ID churn while still re-firing every 30cm so distance in the message stays accurate as an object approaches.

---

## YOLO26n Facts

- Released January 14, 2026. One-to-one head (default export) = NMS-free.
- ONNX output shape: `(1, 300, 6)` = `[x1, y1, x2, y2, conf, cls_id]` in letterbox space.
- If you see overlapping boxes or shape `(1, nc+4, 8400)`, the wrong head was exported.
- Export command: `model.export(format='onnx', imgsz=224, opset=13, simplify=True)`
- Verify on Pi: `sess.get_outputs()[0].shape` must be `(1, 300, 6)`.
- ~40ms inference at 224px on Pi 4 (vs ~69ms for YOLO11n).
- Do NOT use `end2end=False` — that gives the slow one-to-many head requiring NMS.

---

## Ego-Motion Compensation

When the user walks at 80 cm/s forward, every stationary object appears to approach at 80 cm/s.
`EgoMotionCompensator` removes this:

```
compensated_velocity = raw_velocity - camera_z_velocity
```

Two methods run in parallel:
1. **Background depth** — median depth of pixels outside all bboxes. Rate of change = camera Z velocity. Most reliable, handles walking.
2. **Lucas-Kanade optical flow** — tracks ~80 Shi-Tomasi background feature points. Captures lateral sway. CLAHE preprocessing for dim corridors.

Falls back to zero compensation if <150 valid background pixels (doorway, all-occluded scene).

**v3.23 added TTC gate:** if `TTC > 12s`, velocity contribution is zeroed before scoring.
This prevents false CRITICAL alerts for people 3m away whose apparent velocity is just depth sensor noise
(`noise_floor = 3 + dist * 0.02 cm/s`).

**v3.25/v3.26 added ego confidence gating:** raw background-depth Z is clamped to `+/-160 cm/s`; if the rolling
window standard deviation is too high, ego compensation is zeroed and wording falls back to neutral.

---

## Unit Test Design Notes

Tests mock all hardware at import time via `sys.modules` stubs (pyrealsense2, onnxruntime, cv2,
anthropic, smbus2, icm20948). Module loaded with `importlib.util.spec_from_file_location`.
Test file imports from `raspberry_pi/yolo_realsense_navigation.py` — that exact path.

**Critical test quirks — do not change these:**
- Confidence threshold test uses `CONF_THRESH + 0.001`, not `CONF_THRESH`. float32 stores
  0.38 as 0.37999..., which is strictly less than float64 0.38. Using exactly 0.38 fails.
- Two-cluster distance test asserts `abs(dist-100)<=20 OR abs(dist-300)<=20`. Does NOT assume
  which cluster wins — either is valid.
- Track expiry test documents that the condition is `>` not `>=`. A track at exactly `max_age`
  is still alive.
- Ghost filter tests verify the tracker returns ALL tracks (seen_frames=1 included).

Run CI: `pytest tests/test_blindnav.py tests/test_blindnav_v326.py -v` — no camera, no model, no Pi needed.

---

## Version Bump Checklist

When incrementing to vX.XX, update ALL of these:
1. Docstring at top of file (version + changes)
2. Print statement in `main()` startup banner
3. `MASTER_UPDATED.md` — version, status, version history table
4. Git commit: `git commit -m "vX.XX: brief description"`
5. Copy to repo: `cp ~/blindnav_code/...HEADLESS.py raspberry_pi/yolo_realsense_navigation.py`

---

## Decided Against — Do Not Re-Suggest

| Idea | Why not |
|------|---------|
| Sidewalk vs road detection | Route planning, not obstacle avoidance. Wrong problem for this system. |
| SAM segmentation masks | Too slow for Pi 4. 10× slower than YOLO. |
| Reduce resolution to 424×240 | Uneven YOLO scaling — width fits but height gets distorted, risks missing small objects. |
| INT8 quantization | Slower on Pi 4 ARM than float32. Tested in v3.22. |
| Dense optical flow | Too slow. LK sparse flow (~80 points) is enough for ego-motion. |
| DETECTION_INTERVAL=1 | Kills FPS. At interval=2, YOLO and capture overlap; at 1, they serialize. |
| cv2.imshow on Pi | Wayland breaks it. Flask MJPEG stream is the answer. |

---

## Pending Work (as of v3.28)

- [ ] **Heatsink** — #1 hardware priority. Pi throttles to 6–8 FPS above ~65°C.
- [ ] Merge/push v3.28 to GitHub main
- [ ] Field test with Ricardo Salazar (blind user, primary tester) — not yet scheduled
- [ ] Record bag file scenarios for regression testing (5 scenarios: person approach, chair, close-range, white wall, person turning away)
- [ ] Traffic light detection — crop YOLO's `traffic light` box, classify red/green pixels
- [ ] Voice input via Whisper API

---

## Debugging Quick Reference

| Symptom | First check |
|---------|-------------|
| No camera detected | `lsusb \| grep Intel` → camera present? `groups` → user in `video` group? |
| FPS < 8 | `vcgencmd measure_temp` → thermal throttle is cause #1 |
| Black browser window | You're using localhost. Use `hostname -I` and open `http://<Pi-IP>:5000` |
| IMU not found | `i2cdetect -y 1` → 0x68 present? System runs fine without IMU (all objects treated as user moving) |
| Voice announces stale distance | Distance bucket in cooldown key missing — check key format `{id}_tier_{int(dist//30)}` |
| IMU flip-flops MOVING/STILL | `IMU_HYSTERESIS_FRAMES` should be 8 — requires 8 consecutive frames in new state |
| BT audio cuts after urgent alert | Something called SIGTERM on aplay. Find it and remove it. Use pending-queue. |
| ONNX shape error | Run `sess.get_outputs()[0].shape` → must be `(1, 300, 6)`. Re-export if not. |
| numpy AttributeError | `pip install numpy==1.26.4` — 2.0+ removed `np.float` |
| "person stopped" fires while approaching | Velocity guard in `_announce_cleared()` — check `vel < -10` early return |

---

## Hardware Quick Reference

| Device | Interface | Notes |
|--------|-----------|-------|
| RealSense D435 | USB 3.0 | Scratched RGB lens — depth (IR) unaffected. 640×480 @ 30fps. |
| ICM-20948 IMU | I2C, 0x68 | `icm20948` library. System works without it — all objects treated as user moving. |
| GoPro chest harness | 1/4"-20 adapter | Camera angled 10–15° downward. Pi in pocket, USB cable over shoulder. |
| Bluetooth headphones | A2DP | Never SIGTERM aplay. A2DP codec re-negotiates on every new aplay process if stream dropped. |

**pyrealsense2==2.54.2 was built from source on ARM64.** Do not `pip install pyrealsense2` — the
PyPI package doesn't have working ARM64 wheels. The built version lives in the venv on the Pi.

# Project Status

Last updated: March 2026 — v3.20

---

## What Works (Confirmed on Hardware)

- **Object detection** — YOLO26n ONNX running at ~40ms inference on Pi 4 CPU
- **Depth measurement** — RealSense D435 stereo depth, center-crop sampling, IQR clustering for multi-plane scenes
- **Three-tier voice alerts** — URGENT / WARNING / AWARENESS with per-tier cooldowns
- **Piper neural TTS** — low-latency speech via pending-queue pattern, Bluetooth stream stays warm between phrases
- **IMU gate** — ICM-20948 suppresses static object alerts when user is standing still
- **ThreatTransitionTracker** — announces "path clear" / "person moved away" when threats resolve
- **Adaptive depth stride** — prevents FPS drop at close range (was 3.6 FPS, now ~13 FPS at 40cm)
- **Ghost filter** — new detections invisible until seen for 3 consecutive frames (prevents single-frame false alarms)
- **Bounding box EMA smoothing** — stabilises depth sampling region across frames
- **Pipelined capture** — capture+align thread overlaps with YOLO inference (~30% FPS gain)
- **Bag file recording and playback** — full session recording via RealSense SDK for regression testing
- **Scene description** — on-demand Claude Vision description triggered by `d` key
- **CSV logging** — timestamped detection/distance/velocity log saved per session
- **Unit tests** — 37 tests covering postprocess, letterbox, Track EMA, distance, and tracker logic; run without hardware

---

## Not Yet Field Tested

- **Ego-motion compensation (v3.17)** — implemented and code-reviewed by Biped.ai, but not yet tested while actually walking. The chest harness mount is on order. This is the highest priority item.
- **YOLO26n in production** — model is exported and verified (shape confirmed `[1, 300, 6]`), but has not been used in a real walking session yet. Baseline bag files with YOLO11n don't exist yet to compare against.
- **Bag file regression suite** — planned 5 scenarios have not been recorded yet (depends on harness mount arriving).

---

## Known Limitations

- **Texture-poor environments** — Lucas-Kanade optical flow needs trackable corner features. Plain white walls or featureless corridors reduce lateral ego-motion compensation quality. Depth-based forward compensation still works.
- **Full-frame occlusion** — if detected objects fill the entire frame and no background pixels are available, ego-motion compensation falls back to zero (uncorrected velocity).
- **Fast head rotation** — sharp turns blur the image enough that Lucas-Kanade tracking temporarily fails. Feature points refresh each frame so recovery is fast.
- **Bluetooth A2DP latency** — first phrase after a gap has ~100–200ms codec warm-up delay. The pending-queue pattern keeps subsequent phrases immediate.
- **No display output** — headless mode only. There is no visual feedback for debugging in the field; all information comes from the printed `[PERF]` and `[VOICE]` log lines.
- **640×480 color resolution kept** — GPT suggested reducing to 424×240 for performance. Rejected: YOLO would be upsampling height (240→224) and downsampling width (424→224) unevenly, risking missed detections on small/distant objects. Kept at 640×480.

---

## What's Missing / Future Work

### Near-term (before or during first field test)
- [ ] Record bag file test scenarios (needs chest harness mount)
- [ ] Field test ego-motion compensation with Ricardo walking toward static objects
- [ ] Confirm YOLO26n FPS improvement vs YOLO11n using same bag file
- [ ] Update GitHub with bag files folder structure once recordings exist

### Medium-term
- [ ] Lateral distance estimation — currently only forward distance is measured. Could use bounding box X position to say "on your left" vs "ahead" more precisely.
- [ ] Confidence decay for tracked objects that leave frame — currently a tracked object keeps its last known distance until `max_age` expires (1 second).
- [ ] Investigate bimodal depth gap as occlusion signal (low priority, flagged as potentially interesting experiment).

### Open questions from Biped.ai conversation
- Their system is rule-based with hard distance thresholds. TTC-based approach here is more informative but harder to tune reliably. Need field data to evaluate false alarm rate.
- Biped confirmed lateral sway is the dominant noise source. The current implementation handles it via Lucas-Kanade lateral flow, but this hasn't been validated with actual walking data yet.

### Infrastructure
- [ ] GitHub Actions CI — run `pytest tests/test_blindnav.py` automatically on every push
- [ ] `.gitignore` entries for `*.bag`, `*.onnx`, `*.pt`, `~/blindnav_logs/`

---

## External References

- **Motor Focus (2024)** — Kuzdeuov et al., Nazarbayev University. Ego-motion compensation for blind pedestrian navigation without camera calibration. Directly informed the LK optical flow approach used here. Authors contacted for discussion.
- **Biped.ai** — Commercial blind navigation device (France). Contact: Mael. Provided production field insights in March 2026.
- **YOLO26 bug (GitHub #23397)** — Reported export issue affecting yolo26l ONNX where NMS-free behavior is lost. Not confirmed for yolo26n. Symptom: multiple overlapping boxes per object.
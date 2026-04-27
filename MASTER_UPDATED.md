# BlindNav Master Status

Current production version: `v3.29 HEADLESS`

Status:
- Production script: `raspberry_pi/yolo_realsense_navigation.py`
- Hardware-free validation: `192 passed`
- Verified on: April 27, 2026

Version history:

| Version | Status | Notes |
| --- | --- | --- |
| v3.29 | Current branch target | Push-to-talk voice commands with OpenAI STT, deterministic command routing, nonblocking snapshots |
| v3.28 | Previous branch target | Bucketed speech, side-pass person alerts, bad-ego TTC clamp, richer voice diagnostics |
| v3.27 | Previous branch target | Piper default alerts, cached alert clips, filtered motion, wide-angle position hysteresis, clean shutdown |
| v3.26b | Previous production baseline | BT-safe skip-ahead and neutral-wording fixes |
| v3.25 | Earlier baseline | Ego confidence gating, latency logging, zone keys |

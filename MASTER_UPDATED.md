# BlindNav Master Status

Current production version: `v3.30 HEADLESS`

Status:
- Production script: `raspberry_pi/yolo_realsense_navigation.py`
- Hardware-free validation: `195 passed`
- Verified on: April 27, 2026

Version history:

| Version | Status | Notes |
| --- | --- | --- |
| v3.30 | Current branch target | Optional OpenAI alert TTS field-test mode with local fallback and side-by-side run scripts |
| v3.29 | Previous branch target | Push-to-talk voice commands with OpenAI STT, deterministic command routing, nonblocking snapshots |
| v3.28 | Previous branch target | Bucketed speech, side-pass person alerts, bad-ego TTC clamp, richer voice diagnostics |
| v3.27 | Previous branch target | Piper default alerts, cached alert clips, filtered motion, wide-angle position hysteresis, clean shutdown |
| v3.26b | Previous production baseline | BT-safe skip-ahead and neutral-wording fixes |
| v3.25 | Earlier baseline | Ego confidence gating, latency logging, zone keys |

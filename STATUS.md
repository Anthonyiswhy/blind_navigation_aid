# Project Status

Last updated: April 27, 2026
Current repo target: `v3.29 HEADLESS`

## Verified in Code and Tests

- YOLO26n ONNX pipeline is configured for Pi-friendly CPU inference.
- Voice arbitration includes:
  - 3-slot priority queue
  - pre-synthesis
  - zone-based cooldown keys
  - per-alert latency timestamps
  - safe urgent supersession before playback
  - Piper as the default urgent/warning alert voice
  - prewarmed cached alert clips for common short safety phrases
  - bucketed spoken distances for cache reuse
  - explicit queue/synth/cache latency diagnostics in `events.log`
- Side-pass people now get awareness/warning promotion while the user is moving.
- Bad-ego TTC is now trusted only at close range while the user is still.
- Motion filtering now suppresses large one-frame depth jumps and small
  far-range drift before threat scoring/logging.
- Position labeling now uses wide-angle-aware angle mapping with hysteresis.
- Repeated same-frame `get_position()` reads now return a cached label instead
  of advancing hysteresis multiple times inside one video frame.
- Optional push-to-talk voice input is implemented with:
  - `arecord` command capture
  - OpenAI speech-to-text transcription
  - deterministic command routing for describe, nearest, people, status,
    repeat, and cancel
  - thread-safe navigation snapshots so STT never blocks detection
- Ego-motion compensation includes:
  - background-depth velocity estimation
  - hard clamp at `+/-160 cm/s`
  - confidence gating via rolling-window standard deviation
- Threat wording is routed through `_select_voice_message()`, which is directly
  covered by truth-table tests.
- Hardware-free validation currently passes:
  - `tests/test_blindnav.py`
  - `tests/test_blindnav_v326.py`
  - combined result: `192 passed`

## Confirmed Design Invariants

- Active `aplay` playback is never terminated.
- Lower-priority synthesized WAVs are skipped before playback if a higher-priority
  alert becomes pending.
- Ghost filtering stays outside `ObjectTracker`.
- Distance bucket cooldowns keep distance wording fresh as threats approach.
- Ego-motion compensation is zeroed when confidence is poor because bad
  compensation is worse than no compensation.
- Voice commands use `voice.speak_info()` only; urgent/warning obstacle alerts
  remain owned by the main detection loop.

## What Is Hardware-Validated vs Simulated

### Strongly covered without hardware

- queue eviction and TTL expiry
- cooldown behavior
- zone-key dedup under tracker ID churn
- cold-start silence injection
- latency timestamp logging
- threat score truth tables
- message wording selection
- ego-motion clamp and confidence behavior
- voice command parsing, routing, failure handling, and nonblocking listener
  behavior

### Still requires on-device validation

- RealSense depth noise in real corridors and doorways
- actual Bluetooth wake timing under field conditions
- walking sessions with harness sway
- thermal behavior above 65 C
- tracker stability with crowded scenes and occlusions
- real microphone quality, OpenAI STT latency, and push-to-talk ergonomics on
  the Pi

## Current Known Limitations

- Urgent cannot cut off a phrase that is already playing. This is intentional:
  killing `aplay` breaks Bluetooth smoothness and violates the repo rule.
- The system is still thermally sensitive on Pi 4 and can fall to low FPS when
  hot.
- Full-frame occlusion can still remove enough background pixels to disable
  ego-motion compensation temporarily.
- The advanced test suite is strong, but field walking data is still needed for
  final tuning confidence.
- Voice input requires `OPENAI_API_KEY`, `openai`, working `arecord`, and a
  microphone; without those it remains disabled or fails gracefully.

## Pending Work

- Heatsink: highest hardware priority
- Review and merge v3.29 to GitHub main
- Schedule a field test with Ricardo Salazar
- Record five bag-file regression scenarios
- Add traffic-light color classification
- Field-test push-to-talk voice input and tune command phrasing

## CI State

GitHub Actions should run both test files:

```bash
pytest tests/test_blindnav.py tests/test_blindnav_v326.py -v
```

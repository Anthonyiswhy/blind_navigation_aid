# BlindNav

Wearable navigation assistant for blind users built for Raspberry Pi 4 + Intel
RealSense D435. The system detects obstacles, estimates threat from distance
and time-to-collision, and speaks warnings through Bluetooth headphones using
local audio/TTS paths.

Current production version: `v3.30 HEADLESS`

- Production script: `raspberry_pi/yolo_realsense_navigation.py`
- Foundational regression suite: `tests/test_blindnav.py`
- Advanced voice/latency regression suite: `tests/test_blindnav_v326.py`
- Verified locally on May 17, 2026: `200 passed`

## What It Does

- Runs YOLO26n ONNX inference on Pi 4 CPU.
- Samples depth per tracked object using adaptive stride + clustering.
- Compensates apparent approach speed using background-depth ego-motion.
- Suppresses static-object chatter when the user is standing still.
- Speaks left/right/ahead warnings with distance-aware cooldown buckets.
- Logs per-alert latency timestamps to `events.log`.
- Provides on-demand scene description with the `d` key.
- Provides optional push-to-talk voice commands with OpenAI speech-to-text.
- Provides an optional OpenAI alert TTS field-test mode.
- Provides a local generated-alert-clip mode for high-quality safety speech
  without live cloud/Piper synthesis during walking.

## Recent Changes

### v3.30

- Added `BLINDNAV_ALERT_TTS=openai` as a field-test mode for urgent, warning,
  and cleared alert speech.
- Added `BLINDNAV_ALERT_TTS=clips` for local pre-generated alert WAV playback
  with `espeak-ng` fallback when a phrase is missing.
- In clip mode, live Piper fallback is disabled by default for queued speech so
  awareness/info messages do not unexpectedly add Pi-side synthesis latency.
- Keeps detection, threat scoring, voice queueing, and `aplay` playback policy
  unchanged so the field test compares TTS output only.
- Uses OpenAI WAV speech output with local fallback enabled by default.
- Added `tools/run_tts_local.sh`, `tools/run_tts_openai.sh`, and
  `tools/FIELD_TEST_TTS_COMPARE.md` for repeatable side-by-side field tests.

### v3.29

- Added optional push-to-talk voice command input with `BLINDNAV_VOICE_INPUT=1`.
- Records short commands with `arecord`, transcribes through OpenAI STT, and
  routes deterministic commands: describe, nearest, people, status, repeat, and
  cancel.
- Keeps safety alerts on the existing local Piper/VoiceAssistant path by
  default; OpenAI STT is used only for command input.
- Adds a thread-safe navigation snapshot so command responses can report the
  nearest object, people count, and runtime status without blocking detection.
- Adds `tools/test_transcribe.py` for command-line STT smoke testing.

### v3.28

- Bucketed spoken distances to the same 30 cm voice buckets already used by
  cooldown keys so repeat warnings reuse the same Piper phrases instead of
  synthesizing slightly different decimals.
- Added richer voice diagnostics to `events.log`, including queue wait, synth
  time, launch wait, cache hit vs miss, and synthesis mode.
- Made left/right/ahead hysteresis frame-stable so repeated same-frame position
  reads no longer consume the switch threshold early.
- Promoted nearby side-pass people on the left/right while the user is moving,
  so a person walking by no longer depends entirely on radial TTC.
- Clamped bad-ego TTC usage to close range when the user is still, blocking
  far nonsense alerts such as `person ahead, 6.4 meters`.
- Switched non-person urgent/warning phrasing to cached `obstacle` wording so
  close-object warnings stay fast even when the classifier label changes.

### v3.27

- Switched urgent/warning alerts to Piper by default while keeping `espeak`
  available as an override through `BLINDNAV_ALERT_TTS=espeak`.
- Added a prewarmed Piper alert-clip cache for common short safety phrases so
  urgent/warning speech stays natural without paying full synthesis cost every
  time.
- Kept `en_US-amy-medium` as the default Piper voice and added env-based voice
  overrides so `lessac-medium` can be tested without editing the script.
- Added large-jump confirmation and far-noise suppression so static people at
  roughly 2-3m do not accumulate fake approach velocity.
- Replaced the thirds-based left/right/ahead split with wide-angle-aware angle
  mapping plus per-track hysteresis.
- Unified filtered motion across threat scoring, TTC logging, console output,
  and CSV logging.
- Hardened shutdown so the voice queue drains cleanly and the capture thread is
  joined before process exit.

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
export OPENAI_API_KEY="sk-..."            # optional, for voice commands
export BLINDNAV_VOICE_INPUT=1             # optional, press v to speak command
python3 raspberry_pi/yolo_realsense_navigation.py
```

Press `d` for a scene description, or `v` for a voice command when enabled.
Use `Ctrl+C` to exit.

To compare alert TTS output only:

```bash
bash tools/run_tts_local.sh
OPENAI_API_KEY="sk-..." bash tools/run_tts_openai.sh
```

To use high-quality pre-generated local alert clips:

```bash
source ~/.config/blindnav/secrets.env
python3 tools/generate_alert_clips.py
bash tools/run_tts_clips.sh
```

Clip mode uses `~/blindnav_alert_clips` by default and falls back to
`espeak-ng` if a phrase clip is missing.

To test clip mode with optional non-speech proximity tones:

```bash
export BLINDNAV_LOG_UPLOAD=1
bash tools/run_tts_clips_tones.sh
```

This keeps speech for action-level alerts and adds local stereo pulses for
nearby hazards. Set `BLINDNAV_AUDIO_MODE=quiet|balanced|training` and
`BLINDNAV_TONE_VOLUME=0.0..1.0` to tune it.

To upload each completed run's CSV and event log to GitHub automatically:

```bash
export BLINDNAV_LOG_UPLOAD=1
python3 raspberry_pi/yolo_realsense_navigation.py
```

Logs are pushed to the `blindnav-field-logs` branch by default. Set
`BLINDNAV_LOG_UPLOAD_BRANCH` or `BLINDNAV_LOG_UPLOAD_REMOTE` to override that.
The uploader writes its own `upload_*.log` file under `~/blindnav_logs`.
BlindNav keeps the newest 10 navigation runs by default and prunes older
`log_*.csv` / `events_*.log` pairs on startup and shutdown. Set
`BLINDNAV_LOG_RETENTION_RUNS` to change that. When upload is enabled, startup
also retries upload for the newest previous completed run so logs from a prior
failed shutdown are not stranded locally.

## Tests

All tests run without camera hardware, a RealSense device, Piper, or an IMU.
Hardware modules are stubbed at import time.

```bash
pytest tests/test_blindnav.py tests/test_blindnav_v326.py -v
```

Current collected totals:

- `tests/test_blindnav.py`: 37 tests
- `tests/test_blindnav_v326.py`: 163 tests
- Combined: 200 tests

## Performance Notes

- Expected field FPS: roughly 8-14 depending on thermals.
- YOLO export must produce output shape `(1, 300, 6)`.
- ONNX Runtime stays on float32. INT8 was slower on Pi 4 ARM in project tests.
- Thermal throttling is still the main real-world performance limiter.

## Current Priorities

- Add a heatsink before field sessions.
- Review and merge the v3.30 repo state, then field-test it with Ricardo Salazar.
- Record bag-file scenarios for regression playback.
- Field-test push-to-talk voice input on the Pi with the real microphone.
- Field-test local Piper, OpenAI alert TTS, and generated local clips using the
  same walking route and compare `[LATENCY] play_start` in `events.log`.
- Add traffic-light color classification after the base obstacle system is
  stable.

## Design Notes

- Urgent audio is optimized for freshness, but active playback is not forcibly
  interrupted because Bluetooth stream renegotiation is worse than waiting for a
  short phrase to finish.
- Queueing, cooldown, latency, wording, and ego-motion regressions are all
  testable without hardware and are now covered in the advanced suite.

# BlindNav v3.30 Voice, Log Upload, and Clip Mode Handoff

Branch: `codex/v330-clip-infra-cleanup`

Base PR: https://github.com/Anthonyiswhy/blind_navigation_aid/pull/14

Latest follow-up: local alert clip mode and repo infrastructure cleanup.

## What Changed

- Added optional OpenAI alert TTS field-test mode from the v3.30 branch.
- Added optional push-to-talk OpenAI speech-to-text command input.
- Added `BLINDNAV_LOG_UPLOAD=1` to upload completed run logs to GitHub.
- Added automatic pruning for `~/blindnav_logs`.
- Added startup recovery upload for the newest previous completed run when log upload is enabled.
- Added generated local alert clip mode with `BLINDNAV_ALERT_TTS=clips`.
- Expanded the clip generator to cover observed 2.3m phrases and common
  awareness/info replies.
- In clip mode, live Piper fallback is disabled by default so a desk/field test
  does not silently reintroduce Pi-side synthesis latency.
- Added a GitHub pull request template with BlindNav-specific safety checks.
- Pinned CI and Pi requirements to `numpy==1.26.4`.
- Made `tools/run_tts_local.sh` and `tools/run_tts_openai.sh` executable.

## Runtime Defaults

- Voice input is off by default: `BLINDNAV_VOICE_INPUT=0`.
- Alert TTS defaults to local Piper unless a helper script overrides it.
- Clip mode is explicit: `BLINDNAV_ALERT_TTS=clips`.
- Clip mode does not use live Piper fallback unless
  `BLINDNAV_CLIP_MODE_ALLOW_LIVE_PIPER=1` is set.
- Log upload is off by default: `BLINDNAV_LOG_UPLOAD=0`.
- Log retention keeps the newest 10 navigation runs by default.

## Field Test Commands

Local Piper baseline:

```bash
cd ~/blind_navigation_aid
source ~/blindnav-venv/bin/activate
export BLINDNAV_LOG_UPLOAD=1
export BLINDNAV_VOICE_INPUT=0
./tools/run_tts_local.sh
```

OpenAI alert TTS comparison:

```bash
cd ~/blind_navigation_aid
source ~/blindnav-venv/bin/activate
export OPENAI_API_KEY="sk-..."
export BLINDNAV_LOG_UPLOAD=1
export BLINDNAV_VOICE_INPUT=0
./tools/run_tts_openai.sh
```

Generate high-quality local alert clips:

```bash
cd ~/blind_navigation_aid
source ~/blindnav-venv/bin/activate
source ~/.config/blindnav/secrets.env
python3 tools/generate_alert_clips.py
```

The default generator now covers distance buckets from 0.2m through 3.2m,
including the observed field phrase `Stop! person on your left, 2.3 meters`.

Run local clip mode:

```bash
cd ~/blind_navigation_aid
source ~/blindnav-venv/bin/activate
export BLINDNAV_LOG_UPLOAD=1
export BLINDNAV_VOICE_INPUT=0
./tools/run_tts_clips.sh
```

Voice command input test:

```bash
cd ~/blind_navigation_aid
source ~/blindnav-venv/bin/activate
export OPENAI_API_KEY="sk-..."
export BLINDNAV_LOG_UPLOAD=1
export BLINDNAV_VOICE_INPUT=1
python3 raspberry_pi/yolo_realsense_navigation.py
```

Press `v` for a command when voice input is enabled.

## Log Behavior

- Completed runs create a CSV log and matching event log in `~/blindnav_logs`.
- If `BLINDNAV_LOG_UPLOAD=1`, cleanup starts an async uploader after both logs are closed.
- On the next startup, if upload is enabled, the newest previous completed run is retried.
- The newest 10 navigation runs are kept by default.
- Override retention with:

```bash
export BLINDNAV_LOG_RETENTION_RUNS=10
export BLINDNAV_UPLOAD_LOG_RETENTION=10
export BLINDNAV_LOG_RECOVERY_UPLOAD_RUNS=1
```

## Validation

- `python -m pytest tests/test_blindnav.py tests/test_blindnav_v326.py -q`
  - Current result: `200 passed`.
- `python -m py_compile raspberry_pi/yolo_realsense_navigation.py tools/generate_alert_clips.py tools/upload_run_logs_to_github.py tools/test_transcribe.py`
- `python tools/generate_alert_clips.py --list`
  - Current result: `294 phrase(s)`.

## Open Questions

- Do not replace safety alerts with cloud-only speech.
- Keep voice input disabled during TTS latency comparisons.
- If OpenAI TTS is slow, expensive, or unreliable, use cloud TTS only to generate local alert clips ahead of time.
- `BLINDNAV_ALERT_TTS=clips` is the preferred production candidate for high-quality fast safety speech.
- In clip-mode field logs, investigate every `mode=espeak_alert cache=fallback`
  line and add the missing phrase to `tools/generate_alert_clips.py`.
- JAX Whisper should not replace OpenAI STT on the Pi unless it is benchmarked under live YOLO/RealSense load and shown not to reduce detection FPS.

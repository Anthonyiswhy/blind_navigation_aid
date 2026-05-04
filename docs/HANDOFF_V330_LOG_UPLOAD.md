# BlindNav v3.30 Log Upload Handoff

Branch: `codex/v330-log-upload`

PR: https://github.com/Anthonyiswhy/blind_navigation_aid/pull/14

## What Changed

- Added optional OpenAI alert TTS field-test mode from the v3.30 branch.
- Added optional push-to-talk OpenAI speech-to-text command input.
- Added `BLINDNAV_LOG_UPLOAD=1` to upload completed run logs to GitHub.
- Added automatic pruning for `~/blindnav_logs`.
- Added startup recovery upload for the newest previous completed run when log upload is enabled.
- Made `tools/run_tts_local.sh` and `tools/run_tts_openai.sh` executable.

## Runtime Defaults

- Voice input is off by default: `BLINDNAV_VOICE_INPUT=0`.
- Alert TTS defaults to local Piper unless a helper script overrides it.
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
  - First run hit an unrelated random-noise ego-motion edge.
  - Rerun passed: `195 passed`.
- `python -m py_compile raspberry_pi/yolo_realsense_navigation.py tools/upload_run_logs_to_github.py`

## Open Questions

- Do not replace safety alerts with cloud-only speech.
- Keep voice input disabled during TTS latency comparisons.
- If OpenAI TTS is slow, expensive, or unreliable, benchmark another TTS provider as a third candidate instead of replacing the local safety fallback.
- JAX Whisper should not replace OpenAI STT on the Pi unless it is benchmarked under live YOLO/RealSense load and shown not to reduce detection FPS.

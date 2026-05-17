# Codex Handoff - 2026-05-17

## Session

- Date: 2026-05-17
- Branch: `codex/v330-clip-infra-cleanup`
- Commit: pending
- Pull request: pending
- Related logs: `origin/blindnav-field-logs`, run `20260513_180243`

## Summary

Reviewed the newest uploaded v3.30 clip-mode code and the handoff docs, then
patched the largest field-test risks found in the May 13 desk log.

## Why It Matters

The uploaded log showed local clip hits were fast, but it also exposed two
gaps: one 2.3m safety phrase fell back to `espeak-ng`, and one awareness phrase
still used live Piper. This follow-up keeps clip-mode field tests focused on
local playback latency instead of accidentally measuring Pi-side synthesis.

## Files Or Logs Used

- `raspberry_pi/yolo_realsense_navigation.py`
- `tools/generate_alert_clips.py`
- `tools/FIELD_TEST_TTS_COMPARE.md`
- `docs/HANDOFF_V330_LOG_UPLOAD.md`
- `origin/blindnav-field-logs:field_logs/20260513/20260513_180243/events_20260513_180243.log`
- `origin/blindnav-field-logs:field_logs/20260513/20260513_180243/log_20260513_180243.csv`

## Evidence

The May 13 run showed clip mode working for most safety phrases:

```text
mode=clip_alert cache=hit play_start=0.030s
mode=clip_alert cache=hit play_start=0.004s
mode=clip_alert cache=hit play_start=0.028s
```

It also showed a missing generated clip:

```text
mode=espeak_alert cache=fallback
text="Stop! person on your left, 2.3 meters"
```

And one non-safety phrase still using live Piper:

```text
label=AWARE text="Busy area, 4 objects"
mode=piper_live synth=1.833s
```

## Validation

```bash
python -m pytest tests/test_blindnav.py tests/test_blindnav_v326.py -q
python -m py_compile raspberry_pi/yolo_realsense_navigation.py tools/generate_alert_clips.py tools/upload_run_logs_to_github.py tools/test_transcribe.py
python tools/generate_alert_clips.py --list
```

Result:

```text
200 passed
294 phrase(s)
```

## Caveats

- This was code review plus hardware-free validation. It did not run the Pi,
  RealSense, Bluetooth headphones, or ElevenLabs generation live.
- `BLINDNAV_ALERT_TTS=clips` requires clips to be generated before field use.
- Missing phrases still fall back to `espeak-ng`, which is fast but lower
  quality than generated clips.

## Next Steps

1. Pull this branch on the Pi.
2. Regenerate clips with `python3 tools/generate_alert_clips.py`.
3. Walk the same route using `tools/run_tts_local.sh`, `tools/run_tts_openai.sh`,
   and `tools/run_tts_clips.sh`.
4. Compare `[LATENCY] play_start`, `mode`, and `cache` in `events_*.log`.

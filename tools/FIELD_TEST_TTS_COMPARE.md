# BlindNav Alert TTS Field Test

Use these runs to compare only alert speech output. Detection, threat scoring,
queue policy, and `aplay` playback stay the same.

## Local baseline

```bash
source ~/blindnav-venv/bin/activate
./tools/run_tts_local.sh
```

Expected startup line:

```text
alert_tts=piper
```

## OpenAI alert TTS

```bash
source ~/blindnav-venv/bin/activate
export OPENAI_API_KEY="sk-..."
./tools/run_tts_openai.sh
```

Expected startup line:

```text
alert_tts=openai
```

## Local generated clips

Generate clips once before walking:

```bash
source ~/blindnav-venv/bin/activate
source ~/.config/blindnav/secrets.env
python3 tools/generate_alert_clips.py
```

Then run:

```bash
source ~/blindnav-venv/bin/activate
./tools/run_tts_clips.sh
```

Expected startup line:

```text
alert_tts=clips
clip_live_piper=off
```

## Compare

For each run, walk the same short course and save `~/blindnav_logs/events_*.log`.

Focus on `[LATENCY]` lines:

- `play_start`: user-perceived alert delay.
- `synth`: local Piper or OpenAI TTS generation time.
- `mode`: `piper_alert`, `openai_alert`, `clip_alert`, `espeak_alert`, or fallback.
- `cache`: `hit`, `miss`, `n/a`, or `fallback`.

Prefer clip mode if median and worst-case `play_start` improve without missed
phrases. Any `mode=espeak_alert cache=fallback` line means a phrase is missing
from the generated clip set and should be added before production use.

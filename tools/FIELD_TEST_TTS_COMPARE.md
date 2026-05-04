# BlindNav Alert TTS Field Test

Use these two runs to compare only alert speech output. Detection, threat scoring, queue policy, and `aplay` playback stay the same.

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

## Compare

For each run, walk the same short course and save `~/blindnav_logs/events_*.log`.

Focus on `[LATENCY]` lines:

- `play_start`: user-perceived alert delay.
- `synth`: local Piper or OpenAI TTS generation time.
- `mode`: `piper_alert`, `openai_alert`, or fallback.
- `cache`: `hit`, `miss`, `n/a`, or `fallback`.

Keep OpenAI mode only if median and worst-case `play_start` improve without network stalls or missed urgent alerts.

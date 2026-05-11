# Weekly Progress Check - 2026-05-11

## Summary

This week focused on two things: making field logs easier to recover through
GitHub, and testing whether OpenAI alert speech is a realistic alternative to
local Piper speech on the Raspberry Pi.

Only the two May 11 GitHub log uploads are used for this progress check:

```text
field_logs/20260511/20260511_174454/
field_logs/20260511/20260511_174941/
```

Older uploaded logs from May 4, May 6, and May 8 are intentionally excluded
from this report.

Main result:

- The automatic GitHub log upload flow is working.
- The Piper run still showed multi-second delay on cache misses.
- The OpenAI alert TTS run was not a clean win because the first OpenAI speech
  request took about `40.399s`, likely affected by network quality.
- The Piper run also showed weak ego/motion confidence, which needs its own
  follow-up check.

## Monday - OpenAI Alert Speech And Log Upload Setup

### What Changed

The codebase now has a log-upload path merged into `main` through PR #14. That
matters because the Pi can push run logs to GitHub after a run, so later
diagnosis can happen from a PC instead of needing direct access to the Pi.

The relevant branch history now includes:

```text
8c92e60 Merge pull request #14 from Anthonyiswhy/codex/v330-log-upload
431043e docs: add v3.30 handoff
a97578a v3.30: prune and recover run logs
227c8e0 v3.30: add optional GitHub log upload
```

### Log Upload Config

The production script now has an environment-controlled upload mode:

```python
LOG_UPLOAD_ENABLED = os.environ.get(
    "BLINDNAV_LOG_UPLOAD", "0"
).strip().lower() in {"1", "true", "yes", "on"}

LOG_UPLOAD_BRANCH = os.environ.get(
    "BLINDNAV_LOG_UPLOAD_BRANCH", "blindnav-field-logs"
).strip() or "blindnav-field-logs"
```

Why this matters:

- It is off by default, so normal runs do not push logs unless enabled.
- When enabled, logs go to a separate branch instead of cluttering `main`.
- The default branch is `blindnav-field-logs`.

### Uploader Script

The uploader writes logs under a run-specific directory:

```python
def copy_logs(log_paths, workdir, run_id):
    dest_dir = Path(workdir) / "field_logs" / run_id[:8] / run_id
    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = []
    for log_path in log_paths:
        src = Path(log_path).expanduser()
        if not src.exists() or not src.is_file():
            print(f"skip missing log: {src}", flush=True)
            continue
        dest = dest_dir / src.name
        shutil.copy2(src, dest)
        copied.append(dest)
    return copied
```

Then it commits and pushes the logs:

```python
run(["git", "add", "field_logs"], cwd=workdir)
run(["git", "commit", "-m", f"Upload BlindNav logs {run_id}"], cwd=workdir)
run(["git", "push", "origin", f"HEAD:{branch}"], cwd=workdir)
```

### What Happened

This worked. The remote branch `origin/blindnav-field-logs` now contains May 11
field logs:

```text
field_logs/20260511/20260511_174454/events_20260511_174454.log
field_logs/20260511/20260511_174454/log_20260511_174454.csv
field_logs/20260511/20260511_174941/events_20260511_174941.log
field_logs/20260511/20260511_174941/log_20260511_174941.csv
```

That is real workflow progress because logs are now available from GitHub for
diagnosis on another computer.

## Tuesday - AP Testing

Tuesday was reserved for AP testing, so no major code implementation was done
that day. The useful progress for the week came from the log upload workflow
and the May 11 field runs.

## Wednesday-Friday - Field Log Testing And Bug Review

### Test 1: Piper Alert TTS Run

The first May 11 run was:

```text
field_logs/20260511/20260511_174454/
```

The run started with voice input disabled:

```text
[17:45:07.249] [VOICE_CMD] Disabled (set BLINDNAV_VOICE_INPUT=1 to enable)
```

This means the run was focused on normal navigation alerts, not push-to-talk
commands.

### Piper Latency Evidence

The Piper alert path produced 13 latency lines. Cache hits were usually fast,
but cache misses were still slow:

```text
piper_alert cache=miss synth=2.528s play_start=2.533s
piper_alert cache=hit  synth=0.035s play_start=1.595s
piper_alert cache=miss synth=3.158s play_start=4.983s
piper_alert cache=hit  synth=0.055s play_start=0.103s
piper_alert cache=hit  synth=0.005s play_start=0.012s
piper_alert cache=miss synth=3.240s play_start=3.277s
piper_alert cache=hit  synth=0.101s play_start=0.103s
piper_alert cache=hit  synth=0.117s play_start=0.123s
piper_alert cache=hit  synth=0.030s play_start=0.054s
piper_alert cache=hit  synth=0.005s play_start=0.009s
piper_alert cache=hit  synth=0.002s play_start=2.317s
piper_alert cache=hit  synth=0.003s play_start=0.396s
piper_alert cache=miss synth=0.270s play_start=0.460s
```

What this means:

- Cached Piper phrases are usually usable.
- First-use or uncached phrases can still take multiple seconds.
- One cache hit still had `play_start=2.317s`, which suggests queue/playback
  timing can still matter even when synthesis is fast.

### Piper Run CSV Summary

The CSV had 15 rows:

```text
rows=15
avg_fps=3.09
moving_true=15
moving_false=0
ego_confident=true on about 1 row
ego_confident=false on about 14 rows
threat_counts=CAUTION:2, NONE:3, SAFE:9, WARNING:1
```

One important observation is that ego confidence was mostly bad, and `ego_z_cm_s`
was often `0.0`. That matches the concern that the motion/ego signal may not be
giving useful movement context during this run.

Example CSV rows:

```text
2026-05-11T17:45:31Z, fps=2.4, tracked_objects=0, threat_level=NONE, user_moving=True, ego_z_cm_s=0.0, ego_confident=False
2026-05-11T17:46:13Z, fps=3.3, tracked_objects=2, top_threat=person, distance_cm=660, threat_level=SAFE, user_moving=True, ego_z_cm_s=0.0, ego_confident=False
2026-05-11T17:47:40Z, fps=2.5, tracked_objects=3, top_threat=keyboard, distance_cm=42, threat_level=WARNING, user_moving=True, ego_z_cm_s=0.0, ego_confident=False
```

### Interpretation

This was a useful baseline run. It confirms the earlier finding that Piper can
still be slow on uncached phrases. It also shows a separate motion/ego issue:
the run was marked as moving, but ego confidence was mostly bad and ego Z was
usually flat.

That should be investigated separately from TTS.

## OpenAI Alert TTS Run

The second May 11 run was:

```text
field_logs/20260511/20260511_174941/
```

This run used OpenAI alert TTS for safety alert speech:

```text
mode=openai_alert
```

### OpenAI Latency Evidence

The OpenAI alert speech path produced these latency lines:

```text
openai_alert cache=n/a synth=40.399s play_start=40.400s
openai_alert cache=n/a synth=3.450s  play_start=4.764s
openai_alert cache=n/a synth=3.872s  play_start=4.567s
openai_alert cache=n/a synth=1.821s  play_start=4.372s
piper_live   cache=n/a synth=4.008s  play_start=4.056s
```

### What Happened

The OpenAI run was not a clean success. The first OpenAI speech request took
about `40.399s` before playback could start:

```text
[LATENCY] label=URGENT text="Obstacle, 0.8 meters"
synth=40.399s
play_start=40.400s
mode=openai_alert
```

That is too slow for safety alerts. Since Wi-Fi was weak during the test, this
run should not be treated as final proof that OpenAI alert TTS is bad. It does
show that network quality can make cloud speech unsafe if the system waits on
it for urgent alerts.

### OpenAI Run CSV Summary

The CSV had 16 rows:

```text
rows=16
avg_fps=2.58
moving_true=0
moving_false=16
ego_confident=true on about 5 rows
ego_confident=false on about 11 rows
threat_counts=WARNING:3, CRITICAL:1, SAFE:3, NONE:9
```

Example rows:

```text
2026-05-11T17:50:29Z, fps=1.4, tracked_objects=4, top_threat=person, distance_cm=29, threat_level=CRITICAL, user_moving=False, ego_confident=True
2026-05-11T17:50:42Z, fps=3.6, tracked_objects=2, top_threat=keyboard, distance_cm=45, threat_level=WARNING, user_moving=False, ego_confident=False
2026-05-11T17:52:17Z, fps=2.3, tracked_objects=5, top_threat=chair, distance_cm=215, threat_level=SAFE, user_moving=False, ego_confident=False
```

### Interpretation

OpenAI TTS needs another test under better network conditions before deciding
whether it should replace Piper for alerts. The first request was much too slow,
but the later OpenAI requests were closer to the same range as slow Piper misses.

The main risk is that cloud speech adds a new failure mode:

```text
bad Wi-Fi -> slow OpenAI synthesis -> delayed urgent alert
```

That risk does not exist with local cached Piper phrases.

## Code Progress This Week

### GitHub Log Upload Is Now Working

The biggest practical code progress was the log upload workflow. It created a
remote branch with field logs:

```text
origin/blindnav-field-logs
```

The latest commits on that branch are:

```text
08d7417 Upload BlindNav logs 20260511_174941
6b66637 Upload BlindNav logs 20260511_174454
```

This directly supports future debugging because logs from the Pi can be read on
the PC.

### OpenAI Alert TTS Still Needs Better Testing

The current OpenAI branch remains a field-test branch, not a production choice.
The key config is:

```bash
export BLINDNAV_ALERT_TTS=openai
```

The decision rule should stay:

```text
Only keep OpenAI alert TTS if median and worst-case play_start improve without network stalls.
```

Today's OpenAI run failed that rule because the first request took about
`40.400s` to start playback.

## What I Learned

1. GitHub log upload is working and is useful.
2. Piper cache misses are still a latency problem.
3. Piper cache hits are usually fast enough.
4. OpenAI alert TTS is very sensitive to Wi-Fi quality.
5. The Piper run showed weak ego/motion confidence, which needs a separate IMU
   or ego-motion check.
6. FPS was low in both May 11 runs, around `2.58-3.09 FPS`, so performance is
   still a field concern.

## What Still Needs Work

### 1. Retest OpenAI TTS With Better Wi-Fi

The current OpenAI test should be repeated before making a final decision.

Run:

```bash
export OPENAI_API_KEY="sk-..."
export BLINDNAV_ALERT_TTS=openai
python3 raspberry_pi/yolo_realsense_navigation.py
```

Then compare:

```text
[LATENCY] synth=...
[LATENCY] play_start=...
mode=openai_alert
```

### 2. Compare Against Piper In The Same Location

Run the same route with Piper:

```bash
export BLINDNAV_ALERT_TTS=piper
python3 raspberry_pi/yolo_realsense_navigation.py
```

Compare:

```text
mode=piper_alert cache=hit
mode=piper_alert cache=miss
mode=openai_alert
```

### 3. Investigate Motion/Ego Signal

The Piper run had `user_moving=True` on all CSV rows, but ego confidence was
mostly false and ego Z was usually `0.0`. That should be checked before relying
too much on movement-aware suppression or TTC logic.

Useful fields:

```text
user_moving
ego_z_cm_s
ego_confident
```

### 4. Keep Log Upload Enabled

For future tests:

```bash
export BLINDNAV_LOG_UPLOAD=1
```

That way each field run can be diagnosed later from GitHub.

## Next Step

The next real test should be a controlled A/B run:

1. Same hallway.
2. Same lighting.
3. Same camera angle.
4. Same starting position.
5. One run with Piper.
6. One run with OpenAI alert TTS.
7. Good Wi-Fi for the OpenAI run.

The report should compare:

```text
average FPS
confirmed person tracks
threat levels
synth latency
play_start latency
cache hit/miss rate
ego_confident true/false counts
```

If OpenAI still shows multi-second or unstable `play_start`, then the better
near-term safety path is probably more aggressive Piper phrase caching or a
local fast fallback for urgent alerts, not cloud speech as the default.

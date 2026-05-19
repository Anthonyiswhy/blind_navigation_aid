#!/usr/bin/env bash
set -euo pipefail

# Follow-on field test: pre-generated speech clips plus local proximity tones.
# Use this only after testing PR #17's plain clip mode.
cd "$(dirname "$0")/.."

export BLINDNAV_ALERT_TTS=clips
export BLINDNAV_ALERT_CLIP_DIR="${BLINDNAV_ALERT_CLIP_DIR:-$HOME/blindnav_alert_clips}"
export BLINDNAV_CLIP_MODE_ALLOW_LIVE_PIPER="${BLINDNAV_CLIP_MODE_ALLOW_LIVE_PIPER:-0}"
export BLINDNAV_AUDIO_MODE="${BLINDNAV_AUDIO_MODE:-balanced}"
export BLINDNAV_PROXIMITY_TONES=1
export BLINDNAV_TONE_VOLUME="${BLINDNAV_TONE_VOLUME:-0.35}"

python3 raspberry_pi/yolo_realsense_navigation.py

#!/usr/bin/env bash
set -euo pipefail

# Production candidate: play pre-generated local WAV clips for safety alerts,
# with espeak fallback if a phrase clip is missing.
cd "$(dirname "$0")/.."

export BLINDNAV_ALERT_TTS=clips
export BLINDNAV_ALERT_CLIP_DIR="${BLINDNAV_ALERT_CLIP_DIR:-$HOME/blindnav_alert_clips}"
export BLINDNAV_CLIP_MODE_ALLOW_LIVE_PIPER="${BLINDNAV_CLIP_MODE_ALLOW_LIVE_PIPER:-0}"

python3 raspberry_pi/yolo_realsense_navigation.py

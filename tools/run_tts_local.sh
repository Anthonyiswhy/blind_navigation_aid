#!/usr/bin/env bash
set -euo pipefail

# Field-test baseline: local Piper alert speech with cached urgent/warning clips.
# Detection, scoring, queueing, and playback policy are unchanged.
cd "$(dirname "$0")/.."

export BLINDNAV_ALERT_TTS=piper
export BLINDNAV_ALERT_CACHE_PREWARM="${BLINDNAV_ALERT_CACHE_PREWARM:-1}"

python3 raspberry_pi/yolo_realsense_navigation.py

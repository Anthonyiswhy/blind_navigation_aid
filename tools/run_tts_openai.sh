#!/usr/bin/env bash
set -euo pipefail

# Field-test variant: OpenAI TTS for urgent/warning/cleared alert WAV output.
# Detection, scoring, queueing, and aplay playback policy are unchanged.
cd "$(dirname "$0")/.."

: "${OPENAI_API_KEY:?Set OPENAI_API_KEY before running OpenAI alert TTS mode}"

export BLINDNAV_ALERT_TTS=openai
export BLINDNAV_OPENAI_TTS_MODEL="${BLINDNAV_OPENAI_TTS_MODEL:-gpt-4o-mini-tts}"
export BLINDNAV_OPENAI_TTS_VOICE="${BLINDNAV_OPENAI_TTS_VOICE:-coral}"
export BLINDNAV_OPENAI_TTS_TIMEOUT_S="${BLINDNAV_OPENAI_TTS_TIMEOUT_S:-4.0}"
export BLINDNAV_OPENAI_TTS_FALLBACK="${BLINDNAV_OPENAI_TTS_FALLBACK:-1}"

python3 raspberry_pi/yolo_realsense_navigation.py

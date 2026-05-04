#!/usr/bin/env python3
"""Smoke-test BlindNav command transcription from a WAV file or arecord."""
import argparse
import os
import subprocess
import sys
import tempfile
import time


def record_wav(seconds, device):
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        path = f.name
    cmd = [
        "arecord", "-q",
        "-f", "S16_LE",
        "-r", "16000",
        "-c", "1",
        "-d", str(int(round(seconds))),
        path,
    ]
    if device:
        cmd[1:1] = ["-D", device]
    result = subprocess.run(cmd, capture_output=True, check=False)
    if result.returncode != 0:
        try:
            os.unlink(path)
        except OSError:
            pass
        err = result.stderr.decode("utf-8", errors="ignore").strip()
        raise RuntimeError(err or "arecord failed")
    return path


def transcribe(path, model, timeout):
    api_key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY is not set")
    try:
        from openai import OpenAI
    except ImportError as exc:
        raise RuntimeError("Install the OpenAI Python package first: pip install openai") from exc
    client = OpenAI(api_key=api_key, timeout=timeout)
    with open(path, "rb") as audio_file:
        result = client.audio.transcriptions.create(
            model=model,
            file=audio_file,
            response_format="text",
            prompt="BlindNav voice commands: describe, nearest, people, status, repeat, cancel.",
        )
    if isinstance(result, str):
        return result.strip()
    return getattr(result, "text", str(result)).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="Existing wav/mp3/m4a/webm audio file")
    parser.add_argument("--record-seconds", type=float, default=4.0)
    parser.add_argument("--device", default=os.environ.get("BLINDNAV_ARECORD_DEVICE", ""))
    parser.add_argument("--model", default=os.environ.get("BLINDNAV_STT_MODEL", "gpt-4o-mini-transcribe"))
    parser.add_argument("--timeout", type=float, default=float(os.environ.get("BLINDNAV_STT_TIMEOUT_S", "8")))
    args = parser.parse_args()

    owned_file = False
    audio_path = args.file
    try:
        if not audio_path:
            print(f"Recording {args.record_seconds:.1f}s command with arecord...")
            audio_path = record_wav(args.record_seconds, args.device)
            owned_file = True

        start = time.time()
        text = transcribe(audio_path, args.model, args.timeout)
        elapsed = time.time() - start
        print(f"model={args.model}")
        print(f"audio={audio_path}")
        print(f"latency_s={elapsed:.3f}")
        print(f"text={text}")
    finally:
        if owned_file and audio_path:
            try:
                os.unlink(audio_path)
            except OSError:
                pass


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)

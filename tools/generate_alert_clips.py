#!/usr/bin/env python3
"""Generate local WAV alert clips for BlindNav safety speech.

The runtime safety path should play local clips instead of live cloud/Piper TTS.
This tool generates those clips ahead of field testing.
"""
import argparse
import hashlib
import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import wave
from pathlib import Path


ALERT_DISTANCE_BUCKET_CM = 30
ALERT_SPOKEN_OFFSET_CM = 20
ALERT_CACHE_MAX_BUCKET = 5
ALERT_CACHE_POSITIONS = ("ahead", "on your left", "on your right")
DEFAULT_CLIP_DIR = "~/blindnav_alert_clips"
DEFAULT_ELEVEN_VOICE_ID = "JBFqnCBsd6RMkjVDRZzb"
DEFAULT_ELEVEN_MODEL = "eleven_flash_v2_5"
DEFAULT_ELEVEN_OUTPUT_FORMAT = "pcm_22050"


def clip_key(text):
    normalized = " ".join((text or "").strip().lower().split())
    return hashlib.sha1(normalized.encode("utf-8")).hexdigest()


def clip_phrases(max_bucket=ALERT_CACHE_MAX_BUCKET):
    distances_m = tuple(
        ((bucket * ALERT_DISTANCE_BUCKET_CM) + ALERT_SPOKEN_OFFSET_CM) / 100.0
        for bucket in range(max_bucket + 1)
    )
    phrases = []
    for dist_m in distances_m:
        phrases.append(f"Obstacle, {dist_m:.1f} meters")
        for pos in ALERT_CACHE_POSITIONS:
            phrases.extend([
                f"obstacle {pos}, {dist_m:.1f} meters",
                f"Stop! obstacle {pos}, {dist_m:.1f} meters",
                f"Watch out, obstacle {pos}, {dist_m:.1f} meters",
                f"person {pos}, {dist_m:.1f} meters",
                f"person {pos} approaching, {dist_m:.1f} meters",
                f"Stop! person {pos}, {dist_m:.1f} meters",
                f"Watch out, person {pos}, {dist_m:.1f} meters",
                f"Heads up, person {pos}, {dist_m:.1f} meters",
            ])
    phrases.extend([
        "path clear",
        "Command not recognized",
        "Voice command failed",
        "Scene description unavailable",
    ])
    return sorted(set(phrases), key=str.lower)


def parse_pcm_format(output_format):
    if not output_format.startswith("pcm_"):
        raise ValueError(f"Only pcm output formats are supported for WAV wrapping: {output_format}")
    sample_rate = int(output_format.split("_", 1)[1])
    return sample_rate, 1, 2


def write_pcm_wav(path, pcm_bytes, output_format):
    sample_rate, channels, sample_width = parse_pcm_format(output_format)
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(sample_width)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm_bytes)


def generate_elevenlabs(text, api_key, voice_id, model_id, output_format, timeout):
    query = urllib.parse.urlencode({"output_format": output_format})
    url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}?{query}"
    payload = json.dumps({
        "text": text,
        "model_id": model_id,
        "voice_settings": {
            "stability": 0.55,
            "similarity_boost": 0.75,
            "style": 0.0,
            "use_speaker_boost": True,
        },
    }).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=payload,
        headers={
            "Content-Type": "application/json",
            "Accept": "audio/wav",
            "xi-api-key": api_key,
        },
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def load_manifest(path):
    if not path.exists():
        return {"version": 1, "clips": {}}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_manifest(path, manifest):
    tmp = path.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--provider", default=os.environ.get("BLINDNAV_CLIP_PROVIDER", "elevenlabs"))
    parser.add_argument("--clip-dir", default=os.environ.get("BLINDNAV_ALERT_CLIP_DIR", DEFAULT_CLIP_DIR))
    parser.add_argument("--voice-id", default=os.environ.get("ELEVENLABS_VOICE_ID", DEFAULT_ELEVEN_VOICE_ID))
    parser.add_argument("--model", default=os.environ.get("ELEVENLABS_TTS_MODEL", DEFAULT_ELEVEN_MODEL))
    parser.add_argument("--output-format", default=os.environ.get("ELEVENLABS_OUTPUT_FORMAT", DEFAULT_ELEVEN_OUTPUT_FORMAT))
    parser.add_argument("--timeout", type=float, default=float(os.environ.get("ELEVENLABS_TIMEOUT_S", "30")))
    parser.add_argument("--max-bucket", type=int, default=ALERT_CACHE_MAX_BUCKET)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--list", action="store_true", help="Print phrases without generating clips")
    parser.add_argument("--limit", type=int, default=0, help="Generate only the first N phrases, for smoke tests")
    args = parser.parse_args()

    phrases = clip_phrases(max_bucket=args.max_bucket)
    if args.limit > 0:
        phrases = phrases[:args.limit]

    if args.list:
        for phrase in phrases:
            print(phrase)
        print(f"{len(phrases)} phrase(s)")
        return 0

    if args.provider != "elevenlabs":
        raise SystemExit(f"Unsupported provider: {args.provider}")

    api_key = os.environ.get("ELEVENLABS_API_KEY", "").strip()
    if not api_key:
        raise SystemExit("ELEVENLABS_API_KEY is not set")

    clip_dir = Path(args.clip_dir).expanduser()
    clip_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = clip_dir / "manifest.json"
    manifest = load_manifest(manifest_path)
    manifest.update({
        "version": 1,
        "provider": args.provider,
        "voice_id": args.voice_id,
        "model": args.model,
        "output_format": args.output_format,
        "clip_dir": str(clip_dir),
        "updated_at": int(time.time()),
    })
    manifest.setdefault("clips", {})

    generated = 0
    skipped = 0
    failed = 0
    for i, phrase in enumerate(phrases, 1):
        key = clip_key(phrase)
        wav_name = f"{key}.wav"
        wav_path = clip_dir / wav_name
        if wav_path.exists() and not args.force:
            skipped += 1
            manifest["clips"][phrase] = {"file": wav_name, "key": key}
            print(f"[{i}/{len(phrases)}] skip {phrase}")
            continue
        try:
            audio = generate_elevenlabs(
                phrase,
                api_key=api_key,
                voice_id=args.voice_id,
                model_id=args.model,
                output_format=args.output_format,
                timeout=args.timeout,
            )
            write_pcm_wav(wav_path, audio, args.output_format)
            manifest["clips"][phrase] = {"file": wav_name, "key": key}
            generated += 1
            print(f"[{i}/{len(phrases)}] generated {phrase}")
        except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError, Exception) as exc:
            failed += 1
            print(f"[{i}/{len(phrases)}] FAILED {phrase}: {exc}", file=sys.stderr)
        if i % 10 == 0:
            save_manifest(manifest_path, manifest)

    save_manifest(manifest_path, manifest)
    print(
        f"done: generated={generated} skipped={skipped} failed={failed} "
        f"clips={clip_dir} manifest={manifest_path}"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

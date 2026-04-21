#!/usr/bin/env python3
"""
Intelligent Navigation Assistant v3.27 HEADLESS
Builds on v3.26b with:

  FIX 10 - Piper alerts by default, with cached urgent/warning clips.
    Amy-medium remains the default Piper voice, urgent/warning phrases use
    Piper instead of espeak, and common short safety phrases are prewarmed into
    a local clip cache to cut alert latency without giving up voice quality.

  FIX 11 - Shared filtered motion across scoring, TTC, logs, and CSV.
    Large depth jumps now require confirmation, small far-range drift is
    zeroed before it becomes fake approach velocity, and left/right/ahead uses
    wide-angle-aware hysteresis instead of a naive thirds split.

  FIX 12 - Shutdown is cleaner.
    Voice shutdown blocks new enqueues, clears pending speech safely, and the
    capture thread is joined during cleanup to avoid the post-exit abort path.

  FIX 8 — Urgent speech can supersede lower-priority synthesis safely.
    The v3.25 queue still allowed a bad case: a P2/P1 phrase could be in the
    middle of Piper synthesis when a P0 URGENT arrived, forcing urgent to wait
    for that lower-priority phrase to finish synthesizing and start playback.
    This version drops the lower-priority WAV only if a higher-priority alert
    becomes pending before aplay starts. Once audio playback has started, it is
    never terminated. This preserves the hard rule: NEVER send SIGTERM to
    aplay.

  FIX 9 — Partial neutral-wording fix in close-distance branches (Codex review).
    v3.25 gated "approaching" wording on ego_reliable in TTC branches but not
    in the very_close (<40cm) and close (<70cm) distance-override branches.
    Those branches use the `approaching` flag which derives from raw uncorrected
    velocity when ego is unreliable — wrong-perspective wording could still leak.
    Fix: gate `approaching` wording on `ego_reliable` in all three distance
    branches. When ego is unreliable -> neutral "{obj} {pos}, {dist}m" always.

INHERITED FROM v3.25:
  - YOLO 3 threads + SEQUENTIAL (Piper gets 1 core)
  - Ego-Z clamped to +/-160cm/s + confidence gating
  - Zone-based cooldown keys (survives track-ID churn)
  - Presyn semaphore (≤1 concurrent synthesis thread)
  - Per-alert latency timestamps in events.log
  - BT cold-start silence 150ms (was 300ms)
INHERITED FROM v3.24:
  - 3-slot priority queue, pre-synthesis, stream-cold silence
INHERITED FROM v3.23:
  - Velocity noise floor + TTC gate (>12s -> static)
  - Left/right/ahead position, richer CSV, events.log
INHERITED FROM v3.22:
  - Distance bucket cooldown keys, ThreatTransitionTracker velocity guard
  - IMU hysteresis (8 frames)
INHERITED FROM v3.21:
  - Wall/obstacle depth fallback, busy area detection
INHERITED FROM v3.17–v3.20:
  - Ego-motion compensation (LK + background depth)
  - YOLO26n NMS-free postprocess, pipelined capture, Piper TTS
"""
import os
import sys
import io
import time
import csv
import wave
import threading
import math
import hashlib
import shutil
import subprocess
import tempfile
import base64
from datetime import datetime, timezone

import queue
import numpy as np
import cv2
import pyrealsense2 as rs
import onnxruntime as ort

# ============= CONFIGURATION =============
MODEL_PATH = os.path.expanduser("~/yolo_models/yolo26n.onnx")
IMG_SIZE = 224
DETECTION_INTERVAL = 2
CONF_THRESH = 0.38

CRITICAL_DISTANCE = 50
WARNING_DISTANCE = 150
SAFE_DISTANCE = 300

MAX_HUMAN_VELOCITY = 800
MAX_OBJECT_VELOCITY = 2000

MAX_DISTANCE_JUMP = 150
TRACK_IOU_THRESH = 0.30
DEPTH_CENTER_CROP_RATIO = 0.6
DEPTH_STRIDE = 2
MAX_DEPTH_SAMPLE_PIXELS = 4000

IGNORED_CLASSES = {
    "kite", "airplane", "boat", "train", "surfboard", "skis",
    "snowboard", "frisbee", "sports ball", "baseball bat",
    "baseball glove", "skateboard", "bird",
}

VOICE_ENABLED = True

WALL_DISTANCE_CM  = 80
WALL_CENTER_CROP  = 0.30
WALL_MIN_VALID_PX = 200
WALL_COOLDOWN_S   = 3.0

BUSY_TRACK_THRESHOLD = 4
BUSY_COOLDOWN_S      = 10.0

VOICE_COOLDOWN = 5.0

PIPER_VOICE_NAME = os.environ.get("BLINDNAV_PIPER_VOICE", "amy-medium").strip() or "amy-medium"
PIPER_VOICE_DIR = os.path.expanduser(
    os.environ.get("BLINDNAV_PIPER_DIR", "~/piper_voices")
)
PIPER_MODEL  = os.path.expanduser(
    os.environ.get(
        "BLINDNAV_PIPER_MODEL",
        os.path.join(PIPER_VOICE_DIR, f"en_US-{PIPER_VOICE_NAME}.onnx"),
    )
)
PIPER_CONFIG = os.path.expanduser(
    os.environ.get("BLINDNAV_PIPER_CONFIG", PIPER_MODEL + ".json")
)
PIPER_SPEED  = 0.85
PIPER_SILENCE_MS = 150
ALERT_TTS_MODE = os.environ.get("BLINDNAV_ALERT_TTS", "piper").strip().lower()
if ALERT_TTS_MODE not in {"piper", "espeak"}:
    ALERT_TTS_MODE = "piper"
ALERT_CACHE_DIR = os.path.expanduser(
    os.environ.get("BLINDNAV_ALERT_CACHE_DIR",
                   os.path.join(tempfile.gettempdir(), "blindnav_alert_cache"))
)
ALERT_CACHE_PREWARM = os.environ.get(
    "BLINDNAV_ALERT_CACHE_PREWARM", "1"
).strip().lower() not in {"0", "false", "no"}
ALERT_CACHE_COMMON_DISTANCES_M = (0.6, 1.2, 1.8, 2.4)
FAST_ALERT_SILENCE_MS = 220
ESPEAK_ALERT_SPEED = 175
ESPEAK_ALERT_GAP = 8
ESPEAK_ALERT_AMPLITUDE = 180

POSITION_CAMERA_HFOV_DEG = 69.0
POSITION_SIDE_ENTER_DEG  = 13.1
POSITION_SIDE_EXIT_DEG   = 10.0
POSITION_SWITCH_FRAMES   = 2
POSITION_FORCE_SWITCH_DEG = 18.0

IMU_I2C_ADDR        = 0x68
IMU_MOTION_THRESHOLD = 0.05
IMU_WINDOW_SIZE     = 20
IMU_READ_INTERVAL   = 0.05
IMU_HYSTERESIS_FRAMES = 8
IMU_MAX_IO_ERRORS   = 3
IMU_REINIT_BACKOFF_S = 1.0

SCENE_MODEL       = "claude-haiku-4-5-20251001"
SCENE_MAX_TOKENS  = 150
SCENE_JPEG_QUALITY = 60
SCENE_COOLDOWN    = 8.0
SCENE_MAX_OBJECTS = 5

SNAPSHOT_FOLDER = os.path.expanduser("~/blindnav_snapshots")
os.makedirs(SNAPSHOT_FOLDER, exist_ok=True)

RECORD_TO_FILE = ""
PLAYBACK_FILE  = ""
PLAYBACK_REPEAT = True

BAG_FOLDER = os.path.expanduser("~/bags")
CSV_FOLDER = os.path.expanduser("~/blindnav_logs")
os.makedirs(BAG_FOLDER, exist_ok=True)
os.makedirs(CSV_FOLDER, exist_ok=True)
CSV_FILE = os.path.join(CSV_FOLDER, datetime.now(timezone.utc).strftime("log_%Y%m%d_%H%M%S.csv"))

# ============= YOLO CLASS NAMES =============
CLASS_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

OBJECT_THREAT_WEIGHTS = {
    "person": 1.5, "bicycle": 2.0, "car": 3.0, "motorcycle": 2.5,
    "truck": 3.5, "bus": 3.5, "train": 4.0, "dog": 1.3,
    "horse": 2.0, "bear": 4.0, "skateboard": 1.5,
    "chair": 0.8, "bench": 0.9, "potted plant": 0.7,
    "bottle": 0.3, "cup": 0.3, "couch": 0.6, "bed": 0.5,
    "dining table": 0.7,
}

# ============= ZONE-BASED VOICE KEY =============
_FURNITURE_CLASSES   = {"chair","couch","bed","bench","dining table","potted plant"}
_ELECTRONICS_CLASSES = {"tv","laptop","keyboard","mouse","remote","cell phone",
                        "microwave","oven","toaster"}
_VEHICLE_CLASSES     = {"car","truck","bus","motorcycle","bicycle"}

def _voice_key(pos: str, class_name: str, tier: str, dbucket: int = 0) -> str:
    if class_name in _FURNITURE_CLASSES:    family = "furniture"
    elif class_name in _ELECTRONICS_CLASSES: family = "electronics"
    elif class_name in _VEHICLE_CLASSES:    family = "vehicle"
    elif class_name == "person":            family = "person"
    else:                                   family = "object"
    zone = "left" if "left" in pos else "right" if "right" in pos else "center"
    return f"{zone}_{family}_{tier}_{dbucket}"


def _select_voice_message(obj, pos, dist_m, dist_cm, vel,
                          user_moving, ego_reliable,
                          approaching, very_close, close, fast_approach, ttc):
    """
    Pure function: given threat state -> return (tier, message) or (None, None).

    tier: 'urgent' | 'warning' | 'awareness' | None
    Extracted from main() so it can be unit-tested without hardware.

    ego_reliable gates ALL directional ("approaching", "closer") wording.
    When unreliable -> neutral "{obj} {pos}, {dist}m" at appropriate tier.
    """
    if very_close:
        # FIX 9: gate approaching wording on ego_reliable
        if approaching and ego_reliable and not user_moving:
            msg = f"{obj} {pos} approaching, {dist_m:.1f} meters"
        elif approaching and ego_reliable and user_moving:
            msg = f"Stop! {obj} {pos}, {dist_m:.1f} meters"
        else:
            msg = f"{obj} {pos}, {dist_m:.1f} meters"
        return "urgent", msg

    if close:
        # FIX 9: gate approaching wording on ego_reliable
        if approaching and ego_reliable and user_moving:
            msg = f"Watch out, {obj} {pos}, {dist_m:.1f} meters"
        elif approaching and ego_reliable and not user_moving:
            msg = f"{obj} {pos} closer, {dist_m:.1f} meters"
        else:
            msg = f"{obj} {pos}, {dist_m:.1f} meters"
        return "warning", msg

    if ttc < 2:
        if ego_reliable and fast_approach:
            msg = f"Stop! {obj} {pos}, {dist_m:.1f} meters"
        elif ego_reliable and approaching:
            msg = f"{obj} {pos} approaching, {dist_m:.1f} meters"
        else:
            msg = f"{obj} {pos}, {dist_m:.1f} meters"
        return "urgent", msg

    if ttc < 4:
        msg = (f"Watch out, {obj} {pos}, {dist_m:.1f} meters"
               if ego_reliable else f"{obj} {pos}, {dist_m:.1f} meters")
        return "warning", msg

    if ttc < 8:
        return "awareness", f"Heads up, {obj} {pos}, {dist_m:.1f} meters"

    return None, None


# ============= PIPER VOICE =============
class PiperVoice:
    def __init__(self):
        self.available = False
        self._voice = None
        self._syn_config = None
        self._sample_rate = 22050
        self._n_channels = 1
        self._sampwidth = 2
        self._use_piper = False
        self._espeak_cmd = self._detect_espeak()
        self._voice_label = os.path.splitext(os.path.basename(PIPER_MODEL))[0]
        self._alert_cache_dir = ALERT_CACHE_DIR
        self._alert_cache_enabled = False
        try:
            from piper.voice import PiperVoice as _PiperVoice
            from piper.config import SynthesisConfig
            if not os.path.exists(PIPER_MODEL):
                self._init_espeak(); return
            self._voice = _PiperVoice.load(PIPER_MODEL, config_path=PIPER_CONFIG)
            self._syn_config = SynthesisConfig(length_scale=PIPER_SPEED)
            print("[VOICE] Warming up Piper TTS...")
            buf = io.BytesIO()
            with wave.open(buf, "wb") as wav:
                self._voice.synthesize_wav("ready", wav, syn_config=self._syn_config)
            buf.seek(0)
            with wave.open(buf, "rb") as wr:
                self._sample_rate = wr.getframerate()
                self._n_channels  = wr.getnchannels()
                self._sampwidth   = wr.getsampwidth()
            self.available = True
            self._use_piper = True
            self._alert_cache_enabled = ALERT_TTS_MODE == "piper"
            if self._alert_cache_enabled:
                os.makedirs(self._alert_cache_dir, exist_ok=True)
                if ALERT_CACHE_PREWARM:
                    phrases = self._prime_alert_cache()
                    print(f"[VOICE] Piper alert cache primed ({phrases} clips)")
            print(
                f"[OK] Piper TTS ready ({self._sample_rate}Hz, "
                f"voice={self._voice_label}, speed={PIPER_SPEED})"
            )
        except ImportError:
            print("[VOICE] piper-tts not installed — falling back to espeak")
            self._init_espeak()
        except Exception as e:
            print(f"[VOICE] Piper init failed: {e} — falling back to espeak")
            self._init_espeak()

    def _init_espeak(self):
        self._use_piper = False
        self.available = self._espeak_cmd is not None
        if self.available:
            print(f"[OK] Voice fallback: {self._espeak_cmd}")
        else:
            print("[WARN] No TTS available")

    @staticmethod
    def _detect_espeak():
        for cmd in ("espeak-ng", "espeak"):
            try:
                result = subprocess.run([cmd, "--version"], capture_output=True,
                                        timeout=2, check=False)
                if result.returncode == 0:
                    return cmd
            except Exception:
                pass
        return None

    def _alert_cache_key(self, text):
        digest = hashlib.sha1(
            f"{self._voice_label}|{PIPER_SPEED}|{text}".encode("utf-8")
        ).hexdigest()
        return os.path.join(self._alert_cache_dir, f"{digest}.wav")

    def _copy_to_temp(self, wav_path):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            tmpfile = f.name
        shutil.copyfile(wav_path, tmpfile)
        return tmpfile

    def _materialize_cached_alert(self, text):
        cache_path = self._alert_cache_key(text)
        if os.path.exists(cache_path):
            return self._copy_to_temp(cache_path)

        tmpfile = self.synthesize_to_file(text, silence_ms=0)
        if tmpfile is None:
            return None
        try:
            if not os.path.exists(cache_path):
                shutil.copyfile(tmpfile, cache_path)
        except Exception:
            pass
        return tmpfile

    def _prime_alert_cache(self):
        phrases = []
        for dist_m in ALERT_CACHE_COMMON_DISTANCES_M:
            phrases.append(f"Obstacle, {dist_m:.1f} meters")
            for pos in ("ahead", "on your left", "on your right"):
                phrases.append(f"Stop! person {pos}, {dist_m:.1f} meters")
                phrases.append(f"Watch out, person {pos}, {dist_m:.1f} meters")
        for phrase in phrases:
            tmpfile = self._materialize_cached_alert(phrase)
            if tmpfile:
                try:
                    os.unlink(tmpfile)
                except Exception:
                    pass
        return len(phrases)

    def synthesize_to_file(self, text, silence_ms=0):
        try:
            speech_buf = io.BytesIO()
            with wave.open(speech_buf, "wb") as wav:
                self._voice.synthesize_wav(text, wav, syn_config=self._syn_config)
            speech_buf.seek(0)
            with wave.open(speech_buf, "rb") as wr:
                params = wr.getparams()
                speech_frames = wr.readframes(wr.getnframes())
            if silence_ms > 0:
                silence_samples = int(params.framerate * params.nchannels * silence_ms / 1000)
                silence_frames  = b"\x00" * (silence_samples * params.sampwidth)
            else:
                silence_frames = b""
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                tmpfile = f.name
            with wave.open(tmpfile, "wb") as out:
                out.setparams(params)
                out.writeframes(silence_frames + speech_frames)
            return tmpfile
        except Exception as e:
            print(f"[VOICE] Synthesis error: {e}")
            return None

    def prepend_silence(self, wav_path, silence_ms):
        try:
            with wave.open(wav_path, "rb") as wr:
                params = wr.getparams()
                speech_frames = wr.readframes(wr.getnframes())
            silence_samples = int(params.framerate * params.nchannels * silence_ms / 1000)
            silence_frames  = b"\x00" * (silence_samples * params.sampwidth)
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                newpath = f.name
            with wave.open(newpath, "wb") as out:
                out.setparams(params)
                out.writeframes(silence_frames + speech_frames)
            try: os.unlink(wav_path)
            except Exception: pass
            return newpath
        except Exception as e:
            print(f"[VOICE] Silence prepend error: {e}")
            return wav_path

    def synthesize_alert_to_file(self, text, silence_ms=0):
        if self._use_piper and ALERT_TTS_MODE == "piper":
            tmpfile = self._materialize_cached_alert(text)
            if tmpfile is None:
                return None
            if silence_ms > 0:
                return self.prepend_silence(tmpfile, silence_ms)
            return tmpfile

        if self._espeak_cmd is None:
            return self.synthesize_to_file(text, silence_ms=silence_ms)
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                tmpfile = f.name
            result = subprocess.run(
                [
                    self._espeak_cmd,
                    "-s", str(ESPEAK_ALERT_SPEED),
                    "-p", "40",
                    "-g", str(ESPEAK_ALERT_GAP),
                    "-a", str(ESPEAK_ALERT_AMPLITUDE),
                    "-w", tmpfile,
                    text,
                ],
                capture_output=True,
                timeout=5,
                check=False,
            )
            if result.returncode != 0:
                try:
                    os.unlink(tmpfile)
                except Exception:
                    pass
                return self.synthesize_to_file(text, silence_ms=silence_ms)
            if silence_ms > 0:
                return self.prepend_silence(tmpfile, silence_ms)
            return tmpfile
        except Exception:
            return self.synthesize_to_file(text, silence_ms=silence_ms)




# ============= VOICE ASSISTANT (v3.27) =============
class VoiceAssistant:
    """
    3-slot priority queue with pre-synthesis, BT-safe skip-ahead (FIX 8),
    per-alert latency logging, presyn semaphore, and injectable TTS/player
    for testing.

    BT-SAFE SKIP-AHEAD (FIX 8):
      When a higher-priority alert becomes pending while a lower-priority
      phrase is still synthesizing, the lower-priority WAV is dropped before
      playback begins and the higher-priority phrase is drained next.
      Once playback has started, it is never interrupted.

    Constructor params:
      event_logger:  callable(str) for events.log
      _tts_override: inject fake TTS for unit tests
      _player_fn:    inject fake player for unit tests
                     Signature: (wav_path) -> object with .wait()
    """

    PRIO_URGENT  = 0
    PRIO_WARNING = 1
    PRIO_AWARE   = 2

    COOLDOWN_URGENT    = 1.5
    COOLDOWN_WARNING   = 3.0
    COOLDOWN_AWARENESS = 8.0
    COOLDOWN_CLEARED   = 4.0

    STREAM_COLD_S    = 1.5

    _TTL = {0: 3.0, 1: 5.0, 2: 8.0}

    def __init__(self, event_logger=None, _tts_override=None, _player_fn=None):
        self._event_logger = event_logger
        self._player_fn    = _player_fn
        self.is_speaking   = False
        self._lock         = threading.Lock()

        self._pending  = {0: None, 1: None, 2: None}
        self._prefetch = {0: None, 1: None, 2: None}

        self._presyn_sem = threading.Semaphore(1)

        self._current_proc    = None
        self._last_announcement = {}
        self._last_speech_end   = 0.0
        self._workers          = set()
        self._shutting_down    = False

        self._tts = _tts_override if _tts_override is not None else PiperVoice()
        self.available = self._tts.available
        self._alert_tts_mode = ALERT_TTS_MODE
        self._alert_synth_available = hasattr(self._tts, "synthesize_alert_to_file")

    def _use_alert_synth(self, priority):
        return (
            priority is not None
            and priority <= self.PRIO_WARNING
            and self._alert_synth_available
        )

    def _alert_silence_ms(self, need_silence):
        if not need_silence:
            return 0
        if self._alert_tts_mode == "espeak":
            return FAST_ALERT_SILENCE_MS
        return PIPER_SILENCE_MS

    # ---- PUBLIC API ----

    def speak_urgent(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_URGENT,    "URGENT", self.PRIO_URGENT)

    def speak_warning(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_WARNING,   "WARN",   self.PRIO_WARNING)

    def speak_awareness(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_AWARENESS, "AWARE",  self.PRIO_AWARE)

    def speak_cleared(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_CLEARED,   "CLEAR",  self.PRIO_WARNING)

    def speak_info(self, text):
        self._enqueue(text, None, 0, "INFO", self.PRIO_AWARE)

    def cleanup_old_keys(self, active_ids):
        with self._lock:
            stale = [k for k in self._last_announcement if k not in active_ids]
            for k in stale:
                del self._last_announcement[k]

    # ---- INTERNAL ----

    def _enqueue(self, text, key, cooldown, label, priority):
        if not VOICE_ENABLED or not self.available:
            return
        event_created_ts = time.time()
        with self._lock:
            if self._shutting_down:
                return
            if key and cooldown > 0:
                if (event_created_ts - self._last_announcement.get(key, 0)) < cooldown:
                    return

            enqueued_ts = time.time()
            if not self.is_speaking:
                self._start_locked(text, key, cooldown, label, priority,
                                   event_created_ts, enqueued_ts)
                return

            # Currently speaking — store in priority slot, evict lower-priority slots
            for lower in range(priority + 1, 3):
                if self._pending[lower] is not None:
                    self._pending[lower] = None
                    self._cancel_prefetch_locked(lower)

            old = self._pending[priority]
            if old is not None and old[0] != text:
                self._cancel_prefetch_locked(priority)

            self._pending[priority] = (text, key, cooldown, label,
                                       enqueued_ts, event_created_ts)

            # Start presyn if semaphore free
            if self._tts._use_piper and not self._use_alert_synth(priority):
                existing = self._prefetch[priority]
                if existing is None or existing[1] != text:
                    if existing is not None:
                        self._cancel_prefetch_locked(priority)
                    threading.Thread(
                        target=self._presyn_worker,
                        args=(text, priority), daemon=True).start()

    def _cancel_prefetch_locked(self, priority):
        pf = self._prefetch[priority]
        self._prefetch[priority] = None
        if pf:
            threading.Thread(target=self._rm_wav, args=(pf[0],), daemon=True).start()

    @staticmethod
    def _rm_wav(path):
        try:
            if path: os.unlink(path)
        except Exception:
            pass

    def _presyn_worker(self, text, priority):
        # FIX 4: bail immediately if another synthesis is already running
        if not self._presyn_sem.acquire(blocking=False):
            return
        try:
            wav = self._tts.synthesize_to_file(text, silence_ms=0)
        finally:
            self._presyn_sem.release()
        if wav is None:
            return
        with self._lock:
            pend = self._pending[priority]
            if pend is not None and pend[0] == text:
                self._prefetch[priority] = (wav, text)
            else:
                threading.Thread(target=self._rm_wav, args=(wav,), daemon=True).start()

    def _start_locked(self, text, key, cooldown, label, priority,
                      event_created_ts=None, enqueued_ts=None):
        """Start speech. Must hold self._lock."""
        if key and cooldown > 0:
            self._last_announcement[key] = time.time()
        self.is_speaking   = True

        log_msg = f"[VOICE] {label}: \"{text}\""
        print(log_msg)
        if self._event_logger:
            self._event_logger(log_msg)

        # Claim pre-synthesized WAV if text matches
        prefetched = None
        if (self._tts._use_piper
                and not self._use_alert_synth(priority)
                and self._prefetch[priority] is not None):
            pf = self._prefetch[priority]
            if pf[1] == text:
                prefetched = pf[0]
                self._prefetch[priority] = None

        worker = threading.Thread(
            target=self._speak_worker,
            args=(text, prefetched, label, priority, event_created_ts, enqueued_ts),
            daemon=True)
        self._workers.add(worker)
        worker.start()

    def _speak_worker(self, *args):
        worker = threading.current_thread()
        try:
            self._speak_thread(*args)
        finally:
            with self._lock:
                self._workers.discard(worker)

    def _speak_thread(self, text, prefetched_wav=None, label="",
                      priority=None, event_created_ts=None, enqueued_ts=None):
        """Play audio. Logs per-alert latency timestamps."""
        tts_start_ts  = None
        play_start_ts = None
        play_end_ts   = None
        skipped_before_play = False
        try:
            if self._tts._use_piper:
                with self._lock:
                    gap = time.time() - self._last_speech_end
                need_silence = gap > self.STREAM_COLD_S
                use_alert_synth = self._use_alert_synth(priority)

                tts_start_ts = time.time()

                if prefetched_wav:
                    if need_silence:
                        tmpfile = self._tts.prepend_silence(prefetched_wav, PIPER_SILENCE_MS)
                    else:
                        tmpfile = prefetched_wav
                else:
                    silence_ms = self._alert_silence_ms(need_silence) if use_alert_synth else (
                        PIPER_SILENCE_MS if need_silence else 0
                    )
                    if use_alert_synth:
                        tmpfile = self._tts.synthesize_alert_to_file(text, silence_ms=silence_ms)
                    else:
                        # FIX: use semaphore for direct synthesis too — prevents
                        # speak_thread and a stale presyn_worker from both calling
                        # synthesize_to_file at the same time (blocking acquire is
                        # fine here since _speak_thread runs on its own daemon thread)
                        self._presyn_sem.acquire()
                        try:
                            tmpfile = self._tts.synthesize_to_file(text, silence_ms=silence_ms)
                        finally:
                            self._presyn_sem.release()

                if tmpfile:
                    with self._lock:
                        if priority is not None and priority > self.PRIO_URGENT:
                            skipped_before_play = any(
                                self._pending[higher] is not None
                                for higher in range(priority)
                            )
                    if skipped_before_play:
                        try: os.unlink(tmpfile)
                        except Exception: pass
                    else:
                        play_start_ts = time.time()
                        if self._player_fn is not None:
                            proc = self._player_fn(tmpfile)
                        else:
                            proc = subprocess.Popen(["aplay", tmpfile],
                                                    stderr=subprocess.DEVNULL)
                        with self._lock:
                            self._current_proc = proc
                        proc.wait()
                        play_end_ts = time.time()
                        with self._lock:
                            self._current_proc    = None
                            self._last_speech_end = play_end_ts
                        try: os.unlink(tmpfile)
                        except Exception: pass
            else:
                tts_start_ts  = time.time()
                play_start_ts = time.time()
                if self._player_fn is not None:
                    proc = self._player_fn(text)
                else:
                    proc = subprocess.Popen(
                        ["espeak", "-s", "150", "-p", "40", "-g", "5", "-a", "200", text],
                        stderr=subprocess.DEVNULL)
                with self._lock:
                    self._current_proc = proc
                proc.wait()
                play_end_ts = time.time()
                with self._lock:
                    self._current_proc    = None
                    self._last_speech_end = play_end_ts

        except Exception as e:
            print(f"[VOICE] Thread error: {e}")

        finally:
            # FIX 6: Log per-alert latency timestamps to events.log
            if self._event_logger and event_created_ts is not None:
                t0 = event_created_ts
                lat_line = (
                    f"[LATENCY] label={label} text=\"{text[:40]}\""
                    f" event_created={t0:.3f}"
                    f" enqueued={enqueued_ts - t0:.3f}s"
                    f" tts_start={((tts_start_ts  or t0) - t0):.3f}s"
                    f" play_start={((play_start_ts or t0) - t0):.3f}s"
                    f" play_end={((play_end_ts    or t0) - t0):.3f}s"
                )
                self._event_logger(lat_line)
            if skipped_before_play:
                msg = f"[VOICE] Skipped before playback due to higher-priority pending: \"{text}\""
                print(msg)
                if self._event_logger:
                    self._event_logger(msg)

            with self._lock:
                self.is_speaking   = False
                if self._shutting_down:
                    return
                # Drain highest-priority non-None pending slot
                for prio in range(3):
                    pend = self._pending[prio]
                    if pend is None:
                        continue
                    ptext, pkey, pcooldown, plabel, penqueued, pev_created = pend
                    self._pending[prio] = None

                    age = time.time() - penqueued
                    if age > self._TTL[prio]:
                        msg = f"[VOICE] Expired ({age:.1f}s old, TTL={self._TTL[prio]}s): \"{ptext}\""
                        print(msg)
                        if self._event_logger:
                            self._event_logger(msg)
                        self._cancel_prefetch_locked(prio)
                        continue

                    if pkey and pcooldown > 0:
                        if (time.time() - self._last_announcement.get(pkey, 0)) < pcooldown:
                            self._cancel_prefetch_locked(prio)
                            continue

                    self._start_locked(ptext, pkey, pcooldown, plabel, prio,
                                       pev_created, penqueued)
                    break

    def shutdown(self, timeout=6.0):
        with self._lock:
            self._shutting_down = True
            self._pending = {0: None, 1: None, 2: None}
            for prio in range(3):
                self._cancel_prefetch_locked(prio)
            workers = [
                worker for worker in self._workers
                if worker is not threading.current_thread()
            ]

        deadline = time.time() + max(0.0, timeout)
        for worker in workers:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            worker.join(timeout=remaining)


# ============= THREAT TRANSITION TRACKER =============
class ThreatTransitionTracker:
    MIN_THREAT_FRAMES = 12

    def __init__(self):
        self._state = {}

    def update(self, prioritized_threats, voice):
        current_ids = set()
        for score, track in prioritized_threats:
            tid = track.id
            current_ids.add(tid)
            level = ThreatAssessment.get_threat_level(score)
            prev = self._state.get(tid, {"level":"SAFE","threat_frames":0,
                                         "cleared_announced":False,"last_dist":999})
            if level in ("CRITICAL", "WARNING"):
                self._state[tid] = {"level":level,
                                    "threat_frames":prev["threat_frames"]+1,
                                    "cleared_announced":False,
                                    "last_dist":track.distance or prev["last_dist"]}
            else:
                was_threatening = prev["level"] in ("CRITICAL","WARNING")
                enough_frames   = prev["threat_frames"] >= self.MIN_THREAT_FRAMES
                already         = prev["cleared_announced"]
                if was_threatening and enough_frames and not already:
                    self._announce_cleared(track, prev, voice)
                    self._state[tid] = {"level":level,"threat_frames":0,
                                        "cleared_announced":True,
                                        "last_dist":track.distance or prev["last_dist"]}
                else:
                    self._state[tid] = {"level":level,
                                        "threat_frames":prev["threat_frames"],
                                        "cleared_announced":prev["cleared_announced"],
                                        "last_dist":track.distance or prev["last_dist"]}
        for tid in set(self._state.keys()) - current_ids:
            prev = self._state.pop(tid)
            if (prev["level"] in ("CRITICAL","WARNING")
                    and prev["threat_frames"] >= self.MIN_THREAT_FRAMES
                    and not prev["cleared_announced"]):
                voice.speak_cleared("path clear", key="path_clear")

    def _announce_cleared(self, track, prev_state, voice):
        vel    = track.velocity if hasattr(track, "velocity") else 0
        dist_m = (track.distance / 100.0) if track.distance else 0
        if track.velocity_valid and vel < -10:
            return
        if vel > 30:   msg = f"{track.class_name} moved away"
        elif vel > 10: msg = f"{track.class_name} moving away"
        elif vel > 3:  msg = f"{track.class_name} moved aside"
        else:          msg = f"{track.class_name} stopped, {dist_m:.1f} meters away"
        voice.speak_cleared(msg, key=f"cleared_{track.id}")


# ============= SCENE DESCRIBER =============
class SceneDescriber:
    SYSTEM_PROMPT = (
        "You are a navigation assistant for a blind user wearing a camera. "
        "Describe what you see in 2 to 3 short sentences optimized for spoken audio. "
        "Be spatial and direct: use left, right, ahead, with distances. "
        "IMPORTANT: Always use meters for distances, never feet or inches. "
        "Focus only on what matters for safe navigation. Never say 'I see'. Speak directly to the user."
    )

    def __init__(self, voice):
        self.voice = voice
        self.available = False
        self._pending = threading.Event()
        self._processing = False
        self._stop = False
        self._last_request_time = 0
        api_key = os.environ.get("ANTHROPIC_API_KEY", "").strip()
        if not api_key:
            print("[SCENE] WARN: ANTHROPIC_API_KEY not set — scene description disabled")
            self._start_keyboard_thread(); return
        try:
            import anthropic
            self._client = anthropic.Anthropic(api_key=api_key)
            self.available = True
            print(f"[OK] Scene Describer ready ({SCENE_MODEL})")
            print("[SCENE] Press 'd' anytime for scene description")
        except Exception as e:
            print(f"[SCENE] WARN: {e}")
        self._start_keyboard_thread()

    def _start_keyboard_thread(self):
        threading.Thread(target=self._keyboard_worker, daemon=True).start()

    def _keyboard_worker(self):
        try:
            import tty, termios, select as sel
        except ImportError: return
        fd = sys.stdin.fileno()
        try: old = termios.tcgetattr(fd)
        except termios.error: return
        try:
            tty.setcbreak(fd)
            while not self._stop:
                ready, _, _ = sel.select([sys.stdin], [], [], 0.1)
                if ready and sys.stdin.read(1).lower() == "d":
                    self._trigger()
        except Exception: pass
        finally:
            try: termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception: pass

    def _trigger(self):
        if not self.available: print("[SCENE] Not available"); return
        if self._processing: print("[SCENE] Still processing"); return
        elapsed = time.time() - self._last_request_time
        if elapsed < SCENE_COOLDOWN:
            print(f"[SCENE] Cooldown — wait {SCENE_COOLDOWN-elapsed:.1f}s"); return
        print("[SCENE] Capturing frame...")
        self._pending.set()

    def capture_and_describe(self, color_frame, tracks):
        if not self._pending.is_set(): return
        self._pending.clear()
        self._last_request_time = time.time()
        frame_copy = color_frame.copy()
        snap = [{"class_name":t.class_name,"distance":t.distance,
                 "velocity":t.velocity,"box":list(t.box)}
                for t in tracks if t.distance is not None]
        threading.Thread(target=self._describe_thread, args=(frame_copy, snap),
                         daemon=True).start()

    def _describe_thread(self, frame, tracks_snapshot):
        self._processing = True
        try:
            self.voice.speak_info("Analyzing scene")
            depth_ctx = self._build_depth_context(tracks_snapshot, frame.shape[1])
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, SCENE_JPEG_QUALITY])
            if not ok: raise RuntimeError("JPEG encoding failed")
            img_b64 = base64.standard_b64encode(buf.tobytes()).decode("utf-8")
            user_text = (f"Depth: {depth_ctx}. Describe for blind user, max 3 sentences, meters only."
                         if depth_ctx else "No depth data. Describe for blind user, max 3 sentences, meters only.")
            resp = self._client.messages.create(
                model=SCENE_MODEL, max_tokens=SCENE_MAX_TOKENS,
                system=self.SYSTEM_PROMPT,
                messages=[{"role":"user","content":[
                    {"type":"image","source":{"type":"base64","media_type":"image/jpeg","data":img_b64}},
                    {"type":"text","text":user_text}]}])
            desc = resp.content[0].text.strip()
            print(f"\n[SCENE]\n  {desc}\n")
            self.voice.speak_info(desc)
        except Exception as e:
            print(f"[SCENE] Error: {e}")
            self.voice.speak_info("Scene description failed")
        finally:
            self._processing = False

    def _build_depth_context(self, tracks_snapshot, frame_width):
        valid = sorted([t for t in tracks_snapshot if t["distance"] is not None],
                       key=lambda t: t["distance"])[:SCENE_MAX_OBJECTS]
        parts = []
        for t in valid:
            pos = get_position(type("SceneTrack", (), {"box": t["box"]})(), frame_width)
            v = t["velocity"]
            motion = ("approaching fast" if v<-15 else "approaching" if v<-5
                      else "moving away fast" if v>15 else "moving away" if v>5 else "stationary")
            parts.append(f"{t['class_name']} {pos} at {t['distance']/100:.1f}m ({motion})")
        return ", ".join(parts)

    def shutdown(self):
        self._stop = True


# ============= IMU MOTION DETECTOR =============
class MotionDetector:
    def __init__(self):
        self.imu = None
        self.available = False
        self._io_error_count = 0
        self._last_reinit_attempt = 0.0
        self._imu_factory = None
        self._init_imu(initial=True)
        self.accel_history = []
        self.last_read_time = 0
        self._last_moving_state = True
        self._was_moving = True
        self._candidate_state = True
        self._candidate_count = 0

    def _init_imu(self, initial=False):
        try:
            from icm20948 import ICM20948
            self._imu_factory = ICM20948
            self.imu = self._imu_factory(i2c_addr=IMU_I2C_ADDR)
            self.available = True
            self._io_error_count = 0
            if initial:
                print(f"[OK] IMU initialized (ICM-20948 at 0x{IMU_I2C_ADDR:02x})")
            else:
                print("[OK] IMU reinitialized after I/O error")
            return True
        except Exception as e:
            if initial:
                print(f"[WARN] IMU unavailable at startup: {e}. Running without IMU.")
            else:
                print(f"[WARN] IMU reinit failed: {e}. Retrying in background.")
            self.available = False
            self.imu = None
            return False

    def _set_imu_offline(self, reason):
        if self.available or self.imu is not None:
            print(f"[WARN] IMU offline: {reason}. Will retry initialization.")
        self.available = False
        self.imu = None

    def _maybe_reinit(self, now):
        if now - self._last_reinit_attempt < IMU_REINIT_BACKOFF_S:
            return
        self._last_reinit_attempt = now
        self._init_imu(initial=False)

    def update(self):
        now = time.time()
        if not self.available:
            self._maybe_reinit(now)
            return
        if now - self.last_read_time < IMU_READ_INTERVAL: return
        self.last_read_time = now
        try:
            ax, ay, az, _, _, _ = self.imu.read_accelerometer_gyro_data()
            self._io_error_count = 0
            self.accel_history.append(math.sqrt(ax**2 + ay**2 + az**2))
            if len(self.accel_history) > IMU_WINDOW_SIZE:
                self.accel_history.pop(0)
        except OSError as e:
            self._io_error_count += 1
            if getattr(e, "errno", None) == 5 and self._io_error_count >= IMU_MAX_IO_ERRORS:
                self._set_imu_offline(f"I2C read failed with errno 5 ({e})")
                self._maybe_reinit(now)
        except Exception:
            self._io_error_count += 1
            if self._io_error_count >= IMU_MAX_IO_ERRORS:
                self._set_imu_offline("repeated IMU read errors")
                self._maybe_reinit(now)

    def is_moving(self):
        if not self.available or len(self.accel_history) < 10:
            return True
        mean = sum(self.accel_history) / len(self.accel_history)
        variance = sum((x-mean)**2 for x in self.accel_history) / len(self.accel_history)
        raw_moving = math.sqrt(variance) > IMU_MOTION_THRESHOLD
        if raw_moving == self._candidate_state:
            self._candidate_count += 1
        else:
            self._candidate_state = raw_moving
            self._candidate_count = 1
        if self._candidate_count >= IMU_HYSTERESIS_FRAMES:
            self._last_moving_state = raw_moving
        return self._last_moving_state

    def state_changed(self):
        current = self._last_moving_state
        changed = current != self._was_moving
        self._was_moving = current
        return changed

    @property
    def std_dev(self):
        if len(self.accel_history) < 2: return 0.0
        mean = sum(self.accel_history) / len(self.accel_history)
        return math.sqrt(sum((x-mean)**2 for x in self.accel_history) / len(self.accel_history))


# ============= OBJECT TRACKER =============
class ObjectTracker:
    def __init__(self, max_age=1.0):
        self.tracks = {}
        self.next_id = 0
        self.max_age = max_age

    def update(self, detections, current_time):
        expired = [tid for tid, t in self.tracks.items()
                   if current_time - t.last_seen > self.max_age]
        for tid in expired: del self.tracks[tid]
        if not detections: return list(self.tracks.values())
        detections_sorted = sorted(detections, key=lambda d: d["score"], reverse=True)
        matched_ids = set()
        for det in detections_sorted:
            best_match, best_iou = None, TRACK_IOU_THRESH
            for tid, track in self.tracks.items():
                if tid in matched_ids or track.class_name != det["class_name"]: continue
                iou = self._iou(track.box, det["box"])
                if iou > best_iou: best_iou = iou; best_match = tid
            if best_match is not None:
                self.tracks[best_match].update(det, current_time)
                matched_ids.add(best_match)
            else:
                self.tracks[self.next_id] = Track(self.next_id, det, current_time)
                self.next_id += 1
        return list(self.tracks.values())

    def _iou(self, b1, b2):
        x1, y1 = max(b1[0],b2[0]), max(b1[1],b2[1])
        x2, y2 = min(b1[2],b2[2]), min(b1[3],b2[3])
        if x2<=x1 or y2<=y1: return 0.0
        inter = (x2-x1)*(y2-y1)
        a1 = (b1[2]-b1[0])*(b1[3]-b1[1])
        a2 = (b2[2]-b2[0])*(b2[3]-b2[1])
        union = a1+a2-inter
        return inter/union if union>0 else 0.0

class Track:
    def __init__(self, track_id, detection, timestamp):
        self.id = track_id
        self.class_name = detection["class_name"]
        self.class_id   = detection["class_id"]
        self.box        = detection["box"]
        self.score      = detection["score"]
        self.last_seen  = timestamp
        self.distance   = None
        self.distance_history = []
        self.velocity   = 0.0
        self.velocity_valid = True
        self.num_clusters   = 0
        self._prev_velocity_valid = True
        self.seen_frames = 1
        self._pending_distance = None
        self._pending_distance_count = 0

    def update(self, detection, timestamp):
        alpha = 0.6 if self.seen_frames > 1 else 1.0
        nb = detection["box"]
        self.box = [int(alpha*nb[i] + (1-alpha)*self.box[i]) for i in range(4)]
        self.score = detection["score"]
        self.last_seen = timestamp
        self.seen_frames += 1

    def update_distance(self, new_distance, timestamp, camera_z_velocity=0.0):
        if new_distance is None: return
        if self.distance is not None:
            if abs(new_distance - self.distance) > MAX_DISTANCE_JUMP:
                if (self._pending_distance is not None
                        and abs(new_distance - self._pending_distance) <= 40):
                    self._pending_distance_count += 1
                else:
                    self._pending_distance = new_distance
                    self._pending_distance_count = 1
                self.velocity_valid = False
                self.velocity = 0.0
                if self._pending_distance_count >= 2:
                    self.distance = new_distance
                    self.distance_history = [(timestamp, new_distance)]
                    self._pending_distance = None
                    self._pending_distance_count = 0
                    self._prev_velocity_valid = False
                return
        self._pending_distance = None
        self._pending_distance_count = 0
        self.distance = new_distance
        self.distance_history.append((timestamp, new_distance))
        self.distance_history = self.distance_history[-5:]
        if len(self.distance_history) >= 3:
            times = np.array([t for t, _ in self.distance_history], dtype=np.float64)
            dists = np.array([d for _, d in self.distance_history], dtype=np.float64)
            dt = times[-1] - times[0]
            if dt > 0.15:
                delta_dist = float(dists[-1] - dists[0])
                min_motion_delta = max(8.0, max(dists[0], dists[-1]) * 0.03)
                if abs(delta_dist) < min_motion_delta:
                    raw_v = 0.0
                else:
                    centered_t = times - times.mean()
                    centered_d = dists - dists.mean()
                    denom = float(np.dot(centered_t, centered_t))
                    raw_v = (delta_dist / dt if denom <= 1e-6
                             else float(np.dot(centered_t, centered_d) / denom))
                max_v = MAX_HUMAN_VELOCITY if self.class_name == "person" else MAX_OBJECT_VELOCITY
                if abs(raw_v) > max_v:
                    self.velocity_valid = False; self.velocity = 0.0
                    self._prev_velocity_valid = False; return
                if abs(raw_v) < 3: raw_v = 0.0
                compensated_v = raw_v - camera_z_velocity
                if abs(compensated_v) < 3: compensated_v = 0.0
                alpha = 1.0 if not self._prev_velocity_valid else 0.5
                self.velocity = alpha * compensated_v + (1-alpha) * self.velocity
                self.velocity_valid = True
                self._prev_velocity_valid = True


# ============= THREAT ASSESSMENT =============
class ThreatAssessment:
    @staticmethod
    def calculate_threat_score(track, user_moving=False, ego_reliable=True):
        if track.distance is None: return 0.0
        d = track.distance
        motion = evaluate_track_motion(track, user_moving=user_moving,
                                       ego_reliable=ego_reliable)
        v = motion["effective_velocity"]

        score = 0.0
        if d < CRITICAL_DISTANCE:
            score += 50 + 50 * (CRITICAL_DISTANCE - d) / CRITICAL_DISTANCE
        elif d < WARNING_DISTANCE:
            score += 50 * (WARNING_DISTANCE - d) / WARNING_DISTANCE
        elif d < SAFE_DISTANCE:
            score += 10 * (SAFE_DISTANCE - d) / SAFE_DISTANCE

        if v < -5:
            score += min(400, abs(v) * 10)
            if v < -50: score += min(200, abs(v) * 2)
        elif v > 5:
            score *= 0.2

        score *= OBJECT_THREAT_WEIGHTS.get(track.class_name, 1.0)
        if track.score > 0.7:   score *= 1.1
        elif track.score < 0.3: score *= 0.9
        return min(600, score)

    @staticmethod
    def get_threat_level(score):
        if score > 80:  return "CRITICAL"
        if score > 40:  return "WARNING"
        if score > 10:  return "CAUTION"
        return "SAFE"

    @staticmethod
    def prioritize_threats(tracks, user_moving=False, ego_reliable=True):
        scored = [(ThreatAssessment.calculate_threat_score(
                    t, user_moving=user_moving, ego_reliable=ego_reliable), t)
                  for t in tracks]
        scored.sort(reverse=True, key=lambda x: x[0])
        return scored


# ============= HELPER FUNCTIONS =============

def get_position(track, frame_width=640):
    if track.box is None: return "ahead"
    angle_deg = _position_angle_deg(track.box, frame_width)
    prev_zone = getattr(track, "_position_zone", None)
    zone = _classify_position_zone(angle_deg, prev_zone)

    if not hasattr(track, "seen_frames"):
        return _position_label(zone)

    if prev_zone is None or track.seen_frames <= 2:
        current_zone = zone
        pending_zone = zone
        pending_frames = 0
    else:
        current_zone = prev_zone
        pending_zone = getattr(track, "_position_pending_zone", current_zone)
        pending_frames = getattr(track, "_position_pending_frames", 0)
        if zone == current_zone:
            pending_zone = zone
            pending_frames = 0
        else:
            if zone == pending_zone:
                pending_frames += 1
            else:
                pending_zone = zone
                pending_frames = 1
            if (pending_frames >= POSITION_SWITCH_FRAMES
                    or abs(angle_deg) >= POSITION_FORCE_SWITCH_DEG):
                current_zone = zone
                pending_zone = zone
                pending_frames = 0

    track._position_zone = current_zone
    track._position_pending_zone = pending_zone
    track._position_pending_frames = pending_frames
    track._position_angle_deg = angle_deg
    return _position_label(current_zone)


def _position_angle_deg(box, frame_width):
    if box is None or frame_width <= 1:
        return 0.0
    x1, _, x2, _ = box
    cx = (x1 + x2) / 2.0
    norm = ((2.0 * cx) / frame_width) - 1.0
    norm = max(-1.0, min(1.0, norm))
    half_fov_rad = math.radians(POSITION_CAMERA_HFOV_DEG / 2.0)
    return math.degrees(math.atan(norm * math.tan(half_fov_rad)))


def _classify_position_zone(angle_deg, prev_zone=None):
    if prev_zone == "left" and angle_deg <= -POSITION_SIDE_EXIT_DEG:
        return "left"
    if prev_zone == "right" and angle_deg >= POSITION_SIDE_EXIT_DEG:
        return "right"
    if angle_deg < -POSITION_SIDE_ENTER_DEG:
        return "left"
    if angle_deg > POSITION_SIDE_ENTER_DEG:
        return "right"
    return "center"


def _position_label(zone):
    if zone == "left":
        return "on your left"
    if zone == "right":
        return "on your right"
    return "ahead"


def evaluate_track_motion(track, user_moving=False, ego_reliable=True):
    raw_velocity = track.velocity if getattr(track, "velocity_valid", False) else 0.0
    if track.distance is None:
        return {
            "raw_velocity": raw_velocity,
            "effective_velocity": 0.0,
            "velocity_reliable": False,
            "noise_floor": 0.0,
            "ttc": 999.0,
            "ttc_allowed": False,
            "approaching": False,
            "fast_approach": False,
        }

    velocity_reliable = track.velocity_valid and (ego_reliable or not user_moving)
    effective_velocity = raw_velocity if velocity_reliable else 0.0
    noise_floor = 3.0 + track.distance * 0.02
    ttc = 999.0

    if abs(effective_velocity) <= noise_floor:
        effective_velocity = 0.0
    elif effective_velocity < -1.0:
        ttc = track.distance / abs(effective_velocity)
        if ttc > 12.0:
            effective_velocity = 0.0
            ttc = 999.0

    ttc_allowed = effective_velocity < -2.0 and ttc < 100.0
    if not ttc_allowed:
        ttc = 999.0

    return {
        "raw_velocity": raw_velocity,
        "effective_velocity": effective_velocity,
        "velocity_reliable": velocity_reliable,
        "noise_floor": noise_floor,
        "ttc": ttc,
        "ttc_allowed": ttc_allowed,
        "approaching": ttc_allowed,
        "fast_approach": effective_velocity < -50.0,
    }


def letterbox_resize(img, target_size, buf=None):
    h, w = img.shape[:2]
    scale = min(target_size/h, target_size/w)
    nh, nw = int(h*scale), int(w*scale)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    if buf is None: buf = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
    else: buf.fill(114)
    top  = (target_size - nh) // 2
    left = (target_size - nw) // 2
    buf[top:top+nh, left:left+nw] = resized
    return buf, scale, (left, top)


def yolo_postprocess(output, orig_w, orig_h, scale, pad_left, pad_top):
    if output is None: return []
    preds = output[0] if output.ndim == 3 else output
    detections = []
    for row in preds:
        conf = float(row[4])
        if conf < CONF_THRESH: continue
        cls_id = int(row[5])
        if cls_id >= len(CLASS_NAMES): continue
        name = CLASS_NAMES[cls_id]
        if name in IGNORED_CLASSES: continue
        x1 = int(max(0, min((float(row[0])-pad_left)/scale, orig_w-1)))
        y1 = int(max(0, min((float(row[1])-pad_top) /scale, orig_h-1)))
        x2 = int(max(0, min((float(row[2])-pad_left)/scale, orig_w-1)))
        y2 = int(max(0, min((float(row[3])-pad_top) /scale, orig_h-1)))
        if x2<=x1 or y2<=y1 or (x2-x1)<10 or (y2-y1)<10: continue
        detections.append({"class_id":cls_id,"class_name":name,"score":conf,"box":[x1,y1,x2,y2]})
    return detections


def get_smart_distance(depth_data, bbox):
    x1, y1, x2, y2 = bbox
    bw, bh = x2-x1, y2-y1
    if bw < 8 or bh < 8: return None, 0
    cw = int(bw * DEPTH_CENTER_CROP_RATIO)
    ch = int(bh * DEPTH_CENTER_CROP_RATIO)
    cx, cy = x1+bw//2, y1+bh//2
    sx1 = max(x1, cx-cw//2); sy1 = max(y1, cy-ch//2)
    sx2 = min(x2, cx+cw//2); sy2 = min(y2, cy+ch//2)
    region_area = max(1, (sy2-sy1)*(sx2-sx1))
    adaptive_stride = max(DEPTH_STRIDE, int(math.sqrt(region_area/MAX_DEPTH_SAMPLE_PIXELS)))
    region = depth_data[sy1:sy2:adaptive_stride, sx1:sx2:adaptive_stride]
    valid  = region[(region > 0) & (region < 8000)]
    min_samples = max(30, min(100, len(valid)//3))
    if len(valid) < min_samples: return None, 0
    p10, p90 = np.percentile(valid, 10), np.percentile(valid, 90)
    trimmed = valid[(valid >= p10) & (valid <= p90)]
    if len(trimmed) < 20: trimmed = valid
    median_cm = int(np.median(trimmed) / 10)
    iqr = np.percentile(trimmed, 75) - np.percentile(trimmed, 25)
    if iqr < 450: return median_cm, 1
    try:
        hist, edges = np.histogram(trimmed, bins=range(0,8000,300))
        clusters = []
        for i in range(1, len(hist)-1):
            if hist[i]>hist[i-1] and hist[i]>hist[i+1] and hist[i]>=len(trimmed)*0.18:
                clusters.append((int((edges[i]+edges[i+1])/20), hist[i]))
        if len(clusters) > 1:
            return max(clusters, key=lambda c: c[1])[0], len(clusters)
        return median_cm, len(clusters)
    except Exception:
        return median_cm, 0


# ============= EGO-MOTION COMPENSATOR (v3.25) =============
class EgoMotionCompensator:
    """
    FIX 2: Clamped ego-Z + confidence gating.

    Raw background-depth delta can spike to +/-300–400 cm/s from a single
    bad frame (IR saturation, depth dropout, abrupt pan). Before v3.25 these
    values went directly into the rolling history and produced absurd smoothed
    estimates that corrupted every track velocity that frame.

    Changes:
      _MAX_Z_CM_S = 160.0 cm/s (fast jogging) — hard ceiling on raw estimate
      ego_confident: True only when rolling-window std < 40 cm/s
      camera_z_velocity -> 0 when not confident (no compensation worse than
      wrong compensation)
    """
    _LK_PARAMS = dict(winSize=(15,15), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    _FEATURE_PARAMS = dict(maxCorners=80, qualityLevel=0.3, minDistance=10, blockSize=7)
    _BG_MIN_DEPTH_MM = 1000
    _BG_MAX_DEPTH_MM = 8000
    _MIN_CAMERA_Z_CM_S = 2.0
    _SMOOTH_WINDOW = 5
    _MAX_Z_CM_S = 160.0          # FIX 2: hard clamp on raw estimate
    _EGO_STD_THRESHOLD = 40.0   # FIX 2: max acceptable window std

    def __init__(self, use_optical_flow=True):
        self.use_optical_flow  = use_optical_flow
        self.camera_z_velocity = 0.0
        self.camera_lateral_px = (0.0, 0.0)
        self.ego_confident     = False   # FIX 2: confidence flag for callers
        self._prev_bg_depth_mm = None
        self._z_vel_history    = []
        self._prev_gray        = None
        self._prev_bg_pts      = None
        self._last_time        = 0.0
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

    def update(self, color_image, depth_data, tracks, current_time):
        dt = current_time - self._last_time if self._last_time > 0 else 0.033
        self._last_time = current_time
        bg_mask = self._build_background_mask(depth_data.shape, tracks)
        self._update_depth_z(depth_data, bg_mask, dt)
        if self.use_optical_flow:
            self._update_lk_lateral(color_image, bg_mask)

    def _build_background_mask(self, shape, tracks):
        h, w = shape
        mask = np.ones((h,w), dtype=bool)
        for track in tracks:
            if track.box is None: continue
            x1,y1,x2,y2 = map(int, track.box)
            mask[max(0,y1-10):min(h,y2+10), max(0,x1-10):min(w,x2+10)] = False
        return mask

    def _update_depth_z(self, depth_data, bg_mask, dt):
        bg_depths = depth_data[bg_mask]
        valid = bg_depths[(bg_depths > self._BG_MIN_DEPTH_MM) &
                          (bg_depths < self._BG_MAX_DEPTH_MM)]
        if len(valid) < 150:
            self.ego_confident = False
            return
        current_median_mm = float(np.median(valid))
        if self._prev_bg_depth_mm is not None and dt > 0:
            delta_cm = (current_median_mm - self._prev_bg_depth_mm) / 10.0
            raw_z_vel = delta_cm / dt
            # FIX 2: clamp to physical maximum before adding to history
            raw_z_vel = max(-self._MAX_Z_CM_S, min(self._MAX_Z_CM_S, raw_z_vel))
            self._z_vel_history.append(raw_z_vel)
            if len(self._z_vel_history) > self._SMOOTH_WINDOW:
                self._z_vel_history.pop(0)
            smoothed = float(np.median(self._z_vel_history))
            # FIX 2: confidence = window std is low (stable estimate)
            if len(self._z_vel_history) >= 3:
                window_std = float(np.std(self._z_vel_history))
                self.ego_confident = window_std < self._EGO_STD_THRESHOLD
            else:
                self.ego_confident = False
            # FIX 2: zero out when not confident — wrong compensation is
            # worse than no compensation
            self.camera_z_velocity = smoothed if self.ego_confident else 0.0
        self._prev_bg_depth_mm = current_median_mm

    def _update_lk_lateral(self, color_image, bg_mask):
        gray = self._clahe.apply(cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY))
        bg_mask_u8 = bg_mask.astype(np.uint8) * 255
        if self._prev_gray is None or self._prev_bg_pts is None or len(self._prev_bg_pts) < 5:
            self._refresh_features(gray, bg_mask_u8); return
        new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_bg_pts, None, **self._LK_PARAMS)
        if new_pts is None or status is None:
            self._refresh_features(gray, bg_mask_u8); return
        good = status.flatten() == 1
        if good.sum() < 5:
            self._refresh_features(gray, bg_mask_u8); return
        flow = (new_pts[good] - self._prev_bg_pts[good]).reshape(-1, 2)
        self.camera_lateral_px = (float(np.median(flow[:,0])), float(np.median(flow[:,1])))
        self._prev_gray = gray
        self._refresh_features(gray, bg_mask_u8)

    def _refresh_features(self, gray, bg_mask_u8):
        self._prev_bg_pts = cv2.goodFeaturesToTrack(gray, mask=bg_mask_u8, **self._FEATURE_PARAMS)
        self._prev_gray   = gray


# ============= MAIN =============
def main():
    global VOICE_ENABLED

    print("=" * 70)
    print("INTELLIGENT NAVIGATION ASSISTANT v3.27 HEADLESS")
    print("RealSense D435 + YOLO26n + IMU + Piper TTS + Claude Vision")
    print("FIXES: piper alerts, cached clips, ego guard, filtered motion, wide-angle zones, clean shutdown")
    print("=" * 70)

    csv_file = open(CSV_FILE, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp", "fps", "tracked_objects", "top_threat", "threat_score",
        "distance_cm", "velocity_cm_s", "velocity_valid", "threat_level",
        "clusters", "user_moving", "position", "bbox_cx_norm",
        "ego_z_cm_s", "ego_confident", "all_tracks",
    ])
    print(f"[LOG] {CSV_FILE}")

    EVENT_LOG = CSV_FILE.replace("log_", "events_").replace(".csv", ".log")
    event_file = open(EVENT_LOG, "w", encoding="utf-8")
    def log_event(msg):
        ts = datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3]
        event_file.write(f"[{ts}] {msg}\n")
        event_file.flush()
    print(f"[LOG] Events: {EVENT_LOG}")

    motion = MotionDetector()
    voice  = VoiceAssistant(event_logger=log_event)
    scene  = SceneDescriber(voice)
    transition_tracker = ThreatTransitionTracker()
    ego    = EgoMotionCompensator(use_optical_flow=True)

    print(f"\n[YOLO] Loading {MODEL_PATH}...")
    try:
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # FIX 1: 3 threads (was 4) + SEQUENTIAL — frees 1 core for Piper synthesis
        opts.intra_op_num_threads = 3
        opts.inter_op_num_threads = 1
        opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        sess = ort.InferenceSession(MODEL_PATH, opts, providers=["CPUExecutionProvider"])
        inp_name = sess.get_inputs()[0].name
        print(f"[OK] YOLO loaded ({IMG_SIZE}x{IMG_SIZE}, 3 threads)")
    except Exception as e:
        print(f"[ERROR] YOLO: {e}"); csv_file.close(); return

    print("\n[REALSENSE] Initializing D435...")
    pipeline = rs.pipeline()
    cfg = rs.config()
    if PLAYBACK_FILE:
        if not os.path.exists(PLAYBACK_FILE):
            print(f"[ERROR] Bag file not found: {PLAYBACK_FILE}"); csv_file.close(); return
        cfg.enable_device_from_file(PLAYBACK_FILE, repeat_playback=PLAYBACK_REPEAT)
    else:
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if RECORD_TO_FILE: cfg.enable_record_to_file(RECORD_TO_FILE)
    try:
        pipeline.start(cfg)
        print("[OK] RealSense started")
    except Exception as e:
        print(f"[ERROR] RealSense: {e}"); csv_file.close(); return

    align = rs.align(rs.stream.color)
    print("[INFO] Warming up (30 frames)...")
    for _ in range(30): pipeline.wait_for_frames()
    print("[OK] Warmup done")

    tracker  = ObjectTracker()
    rgb_buf  = np.empty((480, 640, 3), dtype=np.uint8)
    lb_buf   = np.full((IMG_SIZE, IMG_SIZE, 3), 114, dtype=np.uint8)
    float_buf= np.empty((IMG_SIZE, IMG_SIZE, 3), dtype=np.float32)
    trans_buf= np.empty((1, 3, IMG_SIZE, IMG_SIZE), dtype=np.float32)

    _frame_q  = queue.Queue(maxsize=2)
    _cap_stop = threading.Event()

    def _capture_worker():
        while not _cap_stop.is_set():
            try:
                frames  = pipeline.wait_for_frames(timeout_ms=1000)
                aligned = align.process(frames)
                df = aligned.get_depth_frame()
                cf = aligned.get_color_frame()
                if not df or not cf: continue
                try:
                    _frame_q.put_nowait((np.asanyarray(cf.get_data()),
                                        np.asanyarray(df.get_data())))
                except queue.Full:
                    pass
            except Exception as exc:
                if not _cap_stop.is_set(): print(f"[CAPTURE] {exc}")

    capture_thread = threading.Thread(target=_capture_worker, daemon=True, name="capture")
    capture_thread.start()
    print("[INFO] Capture pipeline started (async align)")
    print("\n[READY] Navigation active")
    print(f"[MODE]  {'PLAYBACK' if PLAYBACK_FILE else 'LIVE'}")
    print(f"[KEYS]  d = describe scene | Ctrl+C = quit")
    print(f"[SCENE] Model: {SCENE_MODEL} | Cooldown: {SCENE_COOLDOWN}s")
    print(f"[IMU]   Threshold: {IMU_MOTION_THRESHOLD}g")
    print(
        f"[VOICE] 3-slot priority queue | no aplay SIGTERM | "
        f"voice={os.path.splitext(os.path.basename(PIPER_MODEL))[0]} | "
        f"alert_tts={ALERT_TTS_MODE} | "
        f"cache={'on' if ALERT_TTS_MODE == 'piper' else 'off'} | "
        f"silence={FAST_ALERT_SILENCE_MS if ALERT_TTS_MODE == 'espeak' else PIPER_SILENCE_MS}ms\n"
    )

    frame_count       = 0
    last_threat_print = time.time()
    last_paused_print = time.time()
    last_wall_alert   = 0.0
    last_busy_alert   = 0.0
    detect_ms, depth_ms = [], []
    fps_start  = time.time()
    fps_frames = 0
    avg_fps    = 0.0
    last_fps   = 0.0

    try:
        while True:
            try:
                color_image, depth_data = _frame_q.get(timeout=2.0)
            except queue.Empty:
                print("[WARN] No frame for 2s"); continue

            h, w = color_image.shape[:2]
            detections = []
            if frame_count % DETECTION_INTERVAL == 0:
                t0 = time.time()
                try:
                    cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB, dst=rgb_buf)
                    img_lb, scale, (pl, pt) = letterbox_resize(rgb_buf, IMG_SIZE, lb_buf)
                    np.divide(img_lb, 255.0, out=float_buf)
                    trans_buf[0] = np.transpose(float_buf, (2,0,1))
                    outputs = sess.run(None, {inp_name: trans_buf})
                    detections = yolo_postprocess(outputs[0], w, h, scale, pl, pt)
                except Exception as e:
                    print(f"[ERROR] Detection: {e}")
                detect_ms.append((time.time()-t0)*1000)
                if len(detect_ms) > 60: detect_ms.pop(0)

            now    = time.time()
            tracks = tracker.update(detections, now)

            t0 = time.time()
            for track in tracks:
                dist, nc = get_smart_distance(depth_data, track.box)
                track.num_clusters = nc
                track.update_distance(dist, now, ego.camera_z_velocity)
            depth_ms.append((time.time()-t0)*1000)
            if len(depth_ms) > 60: depth_ms.pop(0)

            if frame_count % DETECTION_INTERVAL == 0:
                ego.update(color_image, depth_data, tracks, now)

            # FIX 2: use ego confidence flag for wording decisions
            ego_reliable = ego.ego_confident

            motion.update()
            user_moving = motion.is_moving()
            if motion.state_changed():
                print("[IMU] Moving — all threats active" if user_moving
                      else "[IMU] Stationary — static objects suppressed")

            scene.capture_and_describe(color_image, tracks)
            threats = ThreatAssessment.prioritize_threats(
                tracks, user_moving=user_moving, ego_reliable=ego_reliable)
            transition_tracker.update(threats, voice)

            # Busy area
            confirmed_count = sum(1 for t in tracks if t.seen_frames >= 3)
            if (confirmed_count >= BUSY_TRACK_THRESHOLD and user_moving
                    and now - last_busy_alert > BUSY_COOLDOWN_S):
                voice.speak_awareness(f"Busy area, {confirmed_count} objects",
                                      key="busy_area")
                last_busy_alert = now
                print(f"[BUSY] {confirmed_count} confirmed tracks")

            # Wall fallback — URGENT (was WARNING in v3.24)
            if user_moving and now - last_wall_alert > WALL_COOLDOWN_S:
                h_d, w_d = depth_data.shape
                cy0 = int(h_d*(0.5-WALL_CENTER_CROP/2)); cy1 = int(h_d*(0.5+WALL_CENTER_CROP/2))
                cx0 = int(w_d*(0.5-WALL_CENTER_CROP/2)); cx1 = int(w_d*(0.5+WALL_CENTER_CROP/2))
                center_crop = depth_data[cy0:cy1, cx0:cx1]
                valid_px = center_crop[(center_crop > 100) & (center_crop < 8000)]
                if len(valid_px) >= WALL_MIN_VALID_PX:
                    wall_dist_cm = int(np.median(valid_px) / 10)
                    yolo_covers = any(t.distance is not None
                                      and t.distance < (WALL_DISTANCE_CM+20)
                                      and t.seen_frames >= 3 for t in tracks)
                    if wall_dist_cm < WALL_DISTANCE_CM and not yolo_covers:
                        voice.speak_urgent(
                            f"Obstacle, {wall_dist_cm/100:.1f} meters",
                            key="wall_fallback")
                        last_wall_alert = now
                        print(f"[WALL] Depth fallback: {wall_dist_cm}cm")

            top = threats[0] if threats else None

            if top:
                score, track = top
                dist_cm = track.distance if track.distance else 999
                dist_m  = dist_cm / 100.0
                obj     = track.class_name
                pos     = get_position(track, w)
                motion_eval = evaluate_track_motion(
                    track, user_moving=user_moving, ego_reliable=ego_reliable)
                vel     = motion_eval["effective_velocity"]

                if track.seen_frames >= 3:
                    ttc_allowed = motion_eval["ttc_allowed"]
                    ttc = motion_eval["ttc"]
                    approaching   = motion_eval["approaching"]
                    very_close    = dist_cm < 40
                    close         = dist_cm < 70
                    fast_approach = motion_eval["fast_approach"]

                    if not user_moving and not very_close and not close and not approaching:
                        if now - last_paused_print > 3.0:
                            print(f"[PAUSED] Static {obj} {dist_m:.1f}m — suppressed")
                            last_paused_print = now
                    else:
                        dbucket = int(dist_cm // 30)
                        tier, msg = _select_voice_message(
                            obj, pos, dist_m, dist_cm, vel,
                            user_moving, ego_reliable,
                            approaching, very_close, close, fast_approach, ttc)

                        if tier == "urgent":
                            voice.speak_urgent(
                                msg, key=_voice_key(pos, obj,
                                    "dist_urg" if (very_close or close) else "ttc_urg",
                                    dbucket))
                        elif tier == "warning":
                            voice.speak_warning(
                                msg, key=_voice_key(pos, obj,
                                    "dist_wrn" if close else "ttc_wrn",
                                    dbucket))
                        elif tier == "awareness":
                            voice.speak_awareness(
                                msg, key=_voice_key(pos, obj, "aware", 0))

                    if ttc_allowed and ttc < 100:
                        tier = ("URGENT" if ttc<2 else "WARN" if ttc<4
                                else "AWARE" if ttc<8 else "NONE")
                        line = (f"[TTC]  {obj}#{track.id}(f{track.seen_frames}) {pos}: "
                                f"dist={dist_cm}cm vel={vel:.1f}cm/s "
                                f"TTC={ttc:.1f}s tier={tier} "
                                f"ego={'OK' if ego_reliable else 'BAD'}")
                        print(line)
                        log_event(line)

            # Cleanup stale zone keys
            if frame_count % 60 == 0:
                active_keys = set()
                for t in tracks:
                    p = get_position(t, w)
                    o = t.class_name
                    for b in range(11):
                        active_keys.update([
                            _voice_key(p, o, "dist_urg", b),
                            _voice_key(p, o, "dist_wrn", b),
                            _voice_key(p, o, "ttc_urg",  b),
                            _voice_key(p, o, "ttc_wrn",  b),
                        ])
                    active_keys.add(_voice_key(p, o, "aware", 0))
                    active_keys.add(f"cleared_{t.id}")
                active_keys.update(["path_clear", "wall_fallback", "busy_area"])
                voice.cleanup_old_keys(active_keys)

            # Console threats
            if threats and now - last_threat_print > 2.0:
                imu_str = (f"{'MOVING' if user_moving else 'STILL'} ({motion.std_dev:.3f}g)"
                           if motion.available else "N/A")
                print(f"\n[THREATS] {len(threats)} | IMU={imu_str} "
                      f"| ego={'OK' if ego_reliable else 'BAD'} "
                      f"({ego.camera_z_velocity:.0f}cm/s)")
                for s, t in threats[:3]:
                    lv = ThreatAssessment.get_threat_level(s)
                    motion_eval = evaluate_track_motion(
                        t, user_moving=user_moving, ego_reliable=ego_reliable)
                    display_v = motion_eval["effective_velocity"]
                    vs = ("INVALID!" if not t.velocity_valid else
                          "<<FAST" if display_v<-50 else "<-APP" if display_v<-5 else
                          "FAST>>" if display_v>50 else "AWAY->" if display_v>5 else "STATIC")
                    cc = f", C={t.num_clusters}" if t.num_clusters>1 else ""
                    p  = get_position(t, w)
                    print(f"  [{lv}] {t.class_name}#{t.id} {p}: "
                          f"{t.distance if t.distance else '???'}cm "
                          f"{vs} {abs(display_v):.1f}cm/s (score:{s:.1f}){cc}")
                last_threat_print = now

            # FPS
            fps_frames += 1
            elapsed = time.time() - fps_start
            if elapsed >= 1.0:
                last_fps = fps_frames / elapsed
                avg_fps  = last_fps
                fps_start  = time.time()
                fps_frames = 0
            else:
                avg_fps = fps_frames / elapsed if elapsed > 0 else last_fps

            if frame_count % 60 == 0 and frame_count > 0 and detect_ms:
                dx, dy = ego.camera_lateral_px
                conf_str = "confident" if ego_reliable else "unreliable"
                print(f"[PERF] yolo={np.mean(detect_ms):.1f}ms "
                      f"depth={np.mean(depth_ms):.1f}ms "
                      f"queue={_frame_q.qsize()} -> {avg_fps:.1f} FPS | "
                      f"ego: Z={ego.camera_z_velocity:.1f}cm/s ({conf_str}) "
                      f"lat=({dx:.1f},{dy:.1f})px")

            # CSV
            if frame_count % 30 == 0 and frame_count >= 30:
                if top:
                    t1 = top[1]
                    top_motion = evaluate_track_motion(
                        t1, user_moving=user_moving, ego_reliable=ego_reliable)
                    pos_csv = get_position(t1, w)
                    x1c, _, x2c, _ = t1.box if t1.box else (0,0,0,0)
                    cx_norm = round((x1c+x2c)/(2.0*w), 3) if t1.box else -1
                else:
                    pos_csv = ""; cx_norm = -1
                all_tracks_str = "|".join(
                    f"{t.class_name}:{t.distance}cm:{get_position(t,w)}"
                    for t in tracks if t.distance is not None and t.seen_frames >= 3
                ) or ""
                csv_writer.writerow([
                    datetime.now(timezone.utc).isoformat(), f"{avg_fps:.1f}", len(tracks),
                    top[1].class_name if top else "",
                    f"{top[0]:.1f}" if top else "0",
                    top[1].distance if (top and top[1].distance) else -1,
                    f"{top_motion['effective_velocity']:.1f}" if top else "0",
                    top[1].velocity_valid if top else True,
                    ThreatAssessment.get_threat_level(top[0]) if top else "NONE",
                    top[1].num_clusters if top else 0,
                    user_moving, pos_csv, cx_norm,
                    round(ego.camera_z_velocity, 1),
                    ego_reliable,
                    all_tracks_str,
                ])
                csv_file.flush()

            frame_count += 1

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
    finally:
        print("\n[CLEANUP] Shutting down...")
        _cap_stop.set()
        scene.shutdown()
        try:
            pipeline.stop()
        except Exception as exc:
            print(f"[CLEANUP] pipeline.stop(): {exc}")
        capture_thread.join(timeout=2.0)
        voice.shutdown(timeout=6.0)
        csv_file.close()
        event_file.close()
        print(f"[LOG] Saved: {CSV_FILE}")
        print(f"[LOG] Events: {EVENT_LOG}")
        print("[OK] Done")
        print("=" * 70)

if __name__ == "__main__":
    main()

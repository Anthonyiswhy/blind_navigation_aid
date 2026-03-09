#!/usr/bin/env python3
"""
Intelligent Navigation Assistant v3.20 HEADLESS
Builds on v3.19 with:
  - Removed legacy YOLO11 NMS fallback from yolo_postprocess().
    YOLO26n always outputs (1,300,6) -- the fallback was dead code.
    Cleaner function, no runtime impact (it never executed anyway).
  - Confirmed: two-heads claim is accurate per Ultralytics docs.
    One-to-one head (default export) = NMS-free (300,6).
    One-to-many head (end2end=False) = requires NMS (nc+4,8400).
    We export with defaults so always get the NMS-free head.

NEW IN v3.19:
  FEATURE — Bag file recording and playback (RealSense SDK native)
    Lets you record real sensor sessions to .bag files and replay them
    as many times as needed without the live camera. This is the practical
    equivalent of a test suite for hardware-dependent code.

    RECORDING: Set RECORD_TO_FILE to a .bag path before running.
    The system runs normally but also writes every depth+color frame
    to the file. Stop with Ctrl+C as usual.

    PLAYBACK: Set PLAYBACK_FILE to a recorded .bag path. The system
    runs exactly as if the live camera were connected — all detection,
    ego-motion, voice, and logging work identically. repeat_playback=True
    loops the recording so you can watch behavior across multiple passes.

    RECOMMENDED SCENARIOS TO RECORD:
    - Person walking toward camera at normal pace (tests TTC logic)
    - Walking toward static chair (tests ego-motion compensation)
    - Person standing still at 0.4m (tests close-range FPS fix)
    - Plain white wall corridor (tests LK fallback)
    - Person approaching then turning away (tests transition tracker)

INHERITED FROM v3.17:
  FEATURE — Optical Flow Ego-Motion Compensation (EgoMotionCompensator)
    ROOT CAUSE OF FALSE VELOCITY: When the user walks forward at 80 cm/s,
    every stationary object appears to be approaching at 80 cm/s. The
    depth-based velocity calculation has no way to distinguish "object
    approaching me" from "I am approaching a static object."

    TWO COMPLEMENTARY METHODS:
    1. Background depth tracking: measures camera's Z (forward/backward)
       motion by sampling how median background pixel depths change frame
       to frame. Fast, stable, directly corrects the dominant false-positive
       cause (walking forward past stationary objects).

    2. Lucas-Kanade sparse optical flow: detects camera lateral sway and
       rotational shake by tracking ~80 feature points in background regions
       (outside all bounding boxes). Median pixel displacement = camera
       motion that is NOT forward motion. Uses cv2.calcOpticalFlowPyrLK
       with 3-level pyramids — robust to large inter-frame motion.
       ~3-4ms on Raspberry Pi 4 (sparse, not dense).

    COMPENSATION: corrected_velocity = raw_velocity - camera_z_velocity
    If camera moves forward at 80 cm/s and object is static:
      raw_velocity  = -80 cm/s (appears approaching)
      camera_z      = -80 cm/s (background depth decreasing at 80 cm/s)
      corrected     =  -80 - (-80) = 0 cm/s (correctly identified as static)

    FAILURE MODES AND MITIGATIONS (documented for field testing):
    - Texture-poor environments (plain white wall): few LK feature points
      found. Falls back gracefully — depth tracking still works.
    - Motion blur from fast head turns: LK brightness-constancy assumption
      breaks down. CLAHE preprocessing added to improve contrast before
      feature detection.
    - All background occluded (standing in doorway): background mask may
      return no valid pixels. Compensation set to 0, system reverts to
      uncorrected velocity.
    - Very slow motion (< 2 cm/s camera movement): ignored as noise.

NEW IN v3.16:
  FIX 1 — Voice spam / missed announcements at close range
    ROOT CAUSE: speak_urgent() was calling _interrupt_current() which killed
    aplay mid-stream. On Bluetooth A2DP, killing and restarting aplay
    forces a codec re-negotiation (1-2s delay) every time. So the log showed
    [VOICE] URGENT firing every 1.5s but the user only heard it once because
    each restart triggered another BT delay.
    SOLUTION: Pending queue — urgent messages wait for current speech to finish
    instead of interrupting. No interrupt = BT stream stays warm = 0ms latency
    on subsequent phrases. Also: [VOICE] now only prints when espeak/aplay
    actually starts, not when the condition is met.

  FIX 2 — "Moved away" feedback for blind user confidence
    NEW CLASS: ThreatTransitionTracker watches for threat state changes:
      - Was CRITICAL/WARNING, now SAFE with positive velocity
        → "person moved away, path is clear"
      - Was CRITICAL/WARNING, now SAFE and stopped
        → "person stopped, X meters away"
      - Track disappeared from frame while still threatening
        → "path clear"
    Requires MIN_THREAT_FRAMES = 5 before announcing cleared (avoids false
    clears from momentary detection glitches).

  FIX 3 — FPS drop at close range (3.6 FPS → target 12+ FPS)
    ROOT CAUSE: At 0.4m the bounding box fills most of the frame. The depth
    center-crop was ~200k pixels, making np.percentile/median very slow.
    SOLUTION: Adaptive stride in get_smart_distance() — stride scales with
    box size to cap samples at MAX_DEPTH_SAMPLE_PIXELS (4000). Same accuracy,
    no FPS penalty for large boxes.

INHERITED FROM v3.15:
  - Piper TTS (en_US-amy-medium, silence prefix, no first-word cutoff)
  - TTC-based alert tiers (URGENT/WARNING/AWARENESS)
  - IMU gate: stationary + static = suppress; approaching = always warn
  - Claude scene description on demand ('d' key)
  - Ghost filter (seen_frames < 3 = ignore)
  - IGNORED_CLASSES for outdoor false positives
  - [PAUSED] throttled to every 3s
"""

import os
import sys
import io
import time
import csv
import wave
import threading
import math
import subprocess
import tempfile
import base64
from datetime import datetime

import queue
import numpy as np
import cv2
import pyrealsense2 as rs
import onnxruntime as ort

# ============= CONFIGURATION =============
MODEL_PATH = os.path.expanduser("~/yolo_models/yolo26n.onnx")  # YOLO26n: no-NMS head
IMG_SIZE = 224
DETECTION_INTERVAL = 2
CONF_THRESH = 0.38
# NMS_THRESH removed -- YOLO26n has built-in one-to-one head

# THREAT THRESHOLDS (cm)
CRITICAL_DISTANCE = 50
WARNING_DISTANCE = 150
SAFE_DISTANCE = 300

# VELOCITY LIMITS (cm/s)
MAX_HUMAN_VELOCITY = 800
MAX_OBJECT_VELOCITY = 2000

# DISTANCE FILTERING
MAX_DISTANCE_JUMP = 150
TRACK_IOU_THRESH = 0.30
DEPTH_CENTER_CROP_RATIO = 0.6
DEPTH_STRIDE = 2
MAX_DEPTH_SAMPLE_PIXELS = 4000  # FIX 3: cap samples to prevent FPS drop on large boxes

# Classes to ignore — outdoor-only objects causing indoor false positives
IGNORED_CLASSES = {
    "kite", "airplane", "boat", "train", "surfboard", "skis",
    "snowboard", "frisbee", "sports ball", "baseball bat",
    "baseball glove", "skateboard", "bird",
}

# VOICE SETTINGS
VOICE_ENABLED = True
VOICE_COOLDOWN = 5.0

# PIPER TTS SETTINGS
PIPER_MODEL = os.path.expanduser("~/piper_voices/en_US-amy-medium.onnx")
PIPER_CONFIG = os.path.expanduser("~/piper_voices/en_US-amy-medium.onnx.json")
PIPER_SPEED = 0.85
PIPER_SILENCE_MS = 300  # Silence prefix — fixes ALSA first-word cutoff

# IMU SETTINGS
IMU_I2C_ADDR = 0x68
IMU_MOTION_THRESHOLD = 0.05
IMU_WINDOW_SIZE = 20
IMU_READ_INTERVAL = 0.05

# SCENE DESCRIPTION SETTINGS
SCENE_MODEL = "claude-haiku-4-5-20251001"
SCENE_MAX_TOKENS = 150
SCENE_JPEG_QUALITY = 60
SCENE_COOLDOWN = 8.0
SCENE_MAX_OBJECTS = 5

# LOGGING
SNAPSHOT_FOLDER = os.path.expanduser("~/blindnav_snapshots")
os.makedirs(SNAPSHOT_FOLDER, exist_ok=True)
# BAG FILE RECORDING / PLAYBACK
# Set RECORD_TO_FILE to a path to record a session, e.g.:
#   RECORD_TO_FILE = os.path.expanduser("~/bags/test_walking.bag")
# Set PLAYBACK_FILE to replay a recording instead of using live camera, e.g.:
#   PLAYBACK_FILE = os.path.expanduser("~/bags/test_walking.bag")
# Leave both as "" for normal live operation.
RECORD_TO_FILE = ""
PLAYBACK_FILE  = ""
PLAYBACK_REPEAT = True   # Loop the recording when it ends

BAG_FOLDER = os.path.expanduser("~/bags")
CSV_FOLDER = os.path.expanduser("~/blindnav_logs")
os.makedirs(BAG_FOLDER, exist_ok=True)
os.makedirs(CSV_FOLDER, exist_ok=True)
CSV_FILE = os.path.join(CSV_FOLDER, datetime.utcnow().strftime("log_%Y%m%d_%H%M%S.csv"))

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


# ============= PIPER VOICE CLASS =============
class PiperVoice:
    """
    Neural TTS using Piper (en_US-amy-medium).
    Silence-prefix fix prevents ALSA first-word cutoff.
    Falls back to espeak if Piper unavailable.
    """

    def __init__(self):
        self.available = False
        self._voice = None
        self._syn_config = None
        self._sample_rate = 22050
        self._n_channels = 1
        self._sampwidth = 2
        self._use_piper = False

        try:
            from piper.voice import PiperVoice as _PiperVoice
            from piper.config import SynthesisConfig

            if not os.path.exists(PIPER_MODEL):
                print(f"[VOICE] WARN: Piper model not found: {PIPER_MODEL}")
                print("[VOICE] Falling back to espeak")
                self._init_espeak()
                return

            self._voice = _PiperVoice.load(PIPER_MODEL, config_path=PIPER_CONFIG)
            self._syn_config = SynthesisConfig(length_scale=PIPER_SPEED)

            print("[VOICE] Warming up Piper TTS...")
            buf = io.BytesIO()
            with wave.open(buf, "wb") as wav:
                self._voice.synthesize_wav("ready", wav, syn_config=self._syn_config)
            buf.seek(0)
            with wave.open(buf, "rb") as wr:
                self._sample_rate = wr.getframerate()
                self._n_channels = wr.getnchannels()
                self._sampwidth = wr.getsampwidth()

            self.available = True
            self._use_piper = True
            print(f"[OK] Piper TTS ready ({self._sample_rate}Hz, speed={PIPER_SPEED})")

        except ImportError:
            print("[VOICE] piper-tts not installed — falling back to espeak")
            self._init_espeak()
        except Exception as e:
            print(f"[VOICE] Piper init failed: {e} — falling back to espeak")
            self._init_espeak()

    def _init_espeak(self):
        self._use_piper = False
        try:
            result = subprocess.run(
                ["espeak", "--version"], capture_output=True, timeout=2, check=False
            )
            self.available = result.returncode == 0
            if self.available:
                print("[OK] Voice fallback: espeak")
        except Exception:
            self.available = False
            print("[WARN] No TTS available")

    def synthesize_to_file(self, text):
        """Synthesize text → temp WAV with silence prefix. Returns path or None."""
        try:
            speech_buf = io.BytesIO()
            with wave.open(speech_buf, "wb") as wav:
                self._voice.synthesize_wav(text, wav, syn_config=self._syn_config)

            speech_buf.seek(0)
            with wave.open(speech_buf, "rb") as wr:
                params = wr.getparams()
                speech_frames = wr.readframes(wr.getnframes())

            silence_samples = int(
                params.framerate * params.nchannels * PIPER_SILENCE_MS / 1000
            )
            silence_frames = b"\x00" * (silence_samples * params.sampwidth)

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                tmpfile = f.name

            with wave.open(tmpfile, "wb") as out:
                out.setparams(params)
                out.writeframes(silence_frames + speech_frames)

            return tmpfile

        except Exception as e:
            print(f"[VOICE] Synthesis error: {e}")
            return None


# ============= VOICE ASSISTANT CLASS (v3.16 REWRITE) =============
class VoiceAssistant:
    """
    FIX v3.16: Pending-queue architecture replaces interrupt-based approach.

    WHY THE OLD INTERRUPT APPROACH FAILED:
    v3.15 called _interrupt_current() which sent SIGTERM to aplay.
    On Bluetooth A2DP, killing aplay drops the audio stream. The next
    aplay invocation must re-negotiate the codec (1-2s delay). So even
    though [VOICE] URGENT appeared in the log every 1.5s, audio output
    only happened once because every restart triggered BT re-negotiation.

    HOW PENDING QUEUE WORKS:
    - speak_urgent(): if currently speaking, saves as pending (replaces old pending)
    - speak_warning(): if currently speaking, drops silently
    - When _speak_thread() finishes, it immediately checks pending and fires it
    - No interrupts = BT stream stays warm = instant next phrase
    - [VOICE] only prints when aplay/espeak actually starts (not when condition met)

    COOLDOWNS:
      URGENT:    1.5s  (per track per tier key)
      WARNING:   3.0s
      AWARENESS: 8.0s

    CLEARED messages use COOLDOWN_CLEARED = 4.0s (separate key space)
    """

    COOLDOWN_URGENT    = 1.5
    COOLDOWN_WARNING   = 3.0
    COOLDOWN_AWARENESS = 8.0
    COOLDOWN_CLEARED   = 4.0

    def __init__(self):
        self.is_speaking = False
        self._lock = threading.Lock()
        self._pending = None          # (text, key, cooldown, label)
        self._current_proc = None
        self._last_announcement = {}
        self._tts = PiperVoice()
        self.available = self._tts.available

    # ---- PUBLIC API ----

    def speak_urgent(self, text, key=None):
        """Urgent tier: queued as pending if speaking, 1.5s cooldown."""
        self._enqueue(text, key, self.COOLDOWN_URGENT, "URGENT", allow_pending=True)

    def speak_warning(self, text, key=None):
        """Warning tier: dropped if speaking, 3s cooldown."""
        self._enqueue(text, key, self.COOLDOWN_WARNING, "WARNING", allow_pending=False)

    def speak_awareness(self, text, key=None):
        """Awareness tier: dropped if speaking, 8s cooldown."""
        self._enqueue(text, key, self.COOLDOWN_AWARENESS, "AWARE", allow_pending=False)

    def speak_cleared(self, text, key=None):
        """Cleared/path-clear tier: queued as pending, 4s cooldown."""
        self._enqueue(text, key, self.COOLDOWN_CLEARED, "CLEAR", allow_pending=True)

    def speak_info(self, text):
        """System messages — no cooldown, queued if speaking."""
        self._enqueue(text, None, 0, "INFO", allow_pending=True)

    def cleanup_old_keys(self, active_ids):
        with self._lock:
            stale = [k for k in self._last_announcement if k not in active_ids]
            for k in stale:
                del self._last_announcement[k]

    # ---- INTERNAL ----

    def _enqueue(self, text, key, cooldown, label, allow_pending):
        if not VOICE_ENABLED or not self.available:
            return

        with self._lock:
            # Cooldown check
            if key and cooldown > 0:
                last = self._last_announcement.get(key, 0)
                if (time.time() - last) < cooldown:
                    return  # Still in cooldown — drop silently

            if self.is_speaking:
                if allow_pending:
                    # Save as pending — fires the moment current speech ends
                    self._pending = (text, key, cooldown, label)
                return  # Drop non-pending tiers if already speaking

            # Not speaking — start immediately
            self._start_locked(text, key, cooldown, label)

    def _start_locked(self, text, key, cooldown, label):
        """Start speaking. Must be called with self._lock held."""
        if key and cooldown > 0:
            self._last_announcement[key] = time.time()
        self.is_speaking = True
        print(f"[VOICE] {label}: \"{text}\"")
        t = threading.Thread(
            target=self._speak_thread, args=(text,), daemon=True
        )
        t.start()

    def _speak_thread(self, text):
        try:
            if self._tts._use_piper:
                tmpfile = self._tts.synthesize_to_file(text)
                if tmpfile:
                    proc = subprocess.Popen(
                        ["aplay", tmpfile],
                        stderr=subprocess.DEVNULL,
                    )
                    with self._lock:
                        self._current_proc = proc
                    proc.wait()
                    with self._lock:
                        self._current_proc = None
                    try:
                        os.unlink(tmpfile)
                    except Exception:
                        pass
            else:
                proc = subprocess.Popen(
                    ["espeak", "-s", "150", "-p", "40", "-g", "5", "-a", "200", text],
                    stderr=subprocess.DEVNULL,
                )
                with self._lock:
                    self._current_proc = proc
                proc.wait()
                with self._lock:
                    self._current_proc = None

        except Exception as e:
            print(f"[VOICE] Thread error: {e}")
        finally:
            with self._lock:
                self.is_speaking = False
                # Fire pending immediately — no gap = BT stream stays warm
                if self._pending is not None:
                    ptext, pkey, pcooldown, plabel = self._pending
                    self._pending = None
                    # Re-check cooldown in case time passed during synthesis
                    if pkey and pcooldown > 0:
                        last = self._last_announcement.get(pkey, 0)
                        if (time.time() - last) < pcooldown:
                            return  # Pending expired in cooldown — drop
                    self._start_locked(ptext, pkey, pcooldown, plabel)


# ============= THREAT TRANSITION TRACKER (NEW v3.16) =============
class ThreatTransitionTracker:
    """
    Watches for threat state changes and announces "cleared" events.

    This is critical for blind users — silence after a threat is ambiguous.
    Did the person move away? Did the system crash? This class removes that
    uncertainty by explicitly announcing when a threat is resolved.

    SCENARIOS:
      A) Was CRITICAL/WARNING → now SAFE with velocity > 10 cm/s (walking away)
         → "person moved away, path is clear"

      B) Was CRITICAL/WARNING → now SAFE with velocity > 0 but slow (stepped aside)
         → "person moved aside"

      C) Was CRITICAL/WARNING → now SAFE and static (stopped in place)
         → "person stopped, X point Y meters away"

      D) Track disappeared from camera while still threatening (walked past/off frame)
         → "path clear"

    GUARD RAILS:
      - MIN_THREAT_FRAMES = 5: object must be tracked for 5+ frames as threatening
        before we announce cleared. Prevents false clears from momentary detections.
      - Once cleared is announced for a track, flag is set until it becomes
        threatening again (prevents repeated "path clear" for same static object).
    """

    MIN_THREAT_FRAMES = 5

    def __init__(self):
        # track_id → {"level": str, "threat_frames": int, "cleared_announced": bool, "last_dist": float}
        self._state = {}

    def update(self, prioritized_threats, voice: VoiceAssistant):
        """Call once per frame after ThreatAssessment.prioritize_threats()."""
        current_ids = set()

        for score, track in prioritized_threats:
            tid = track.id
            current_ids.add(tid)
            level = ThreatAssessment.get_threat_level(score)
            prev = self._state.get(tid, {
                "level": "SAFE", "threat_frames": 0,
                "cleared_announced": False, "last_dist": 999
            })

            if level in ("CRITICAL", "WARNING"):
                # Accumulate threat frames, reset cleared flag
                self._state[tid] = {
                    "level": level,
                    "threat_frames": prev["threat_frames"] + 1,
                    "cleared_announced": False,
                    "last_dist": track.distance or prev["last_dist"],
                }
            else:
                # Transitioned to safe — check if we need cleared announcement
                was_threatening   = prev["level"] in ("CRITICAL", "WARNING")
                enough_frames     = prev["threat_frames"] >= self.MIN_THREAT_FRAMES
                already_announced = prev["cleared_announced"]

                if was_threatening and enough_frames and not already_announced:
                    self._announce_cleared(track, prev, voice)
                    self._state[tid] = {
                        "level": level,
                        "threat_frames": 0,
                        "cleared_announced": True,
                        "last_dist": track.distance or prev["last_dist"],
                    }
                else:
                    self._state[tid] = {
                        "level": level,
                        "threat_frames": prev["threat_frames"],
                        "cleared_announced": prev["cleared_announced"],
                        "last_dist": track.distance or prev["last_dist"],
                    }

        # Handle tracks that disappeared from frame
        disappeared = set(self._state.keys()) - current_ids
        for tid in disappeared:
            prev = self._state.pop(tid)
            was_threatening   = prev["level"] in ("CRITICAL", "WARNING")
            enough_frames     = prev["threat_frames"] >= self.MIN_THREAT_FRAMES
            already_announced = prev["cleared_announced"]

            if was_threatening and enough_frames and not already_announced:
                voice.speak_cleared("path clear", key="path_clear")

    def _announce_cleared(self, track, prev_state, voice: VoiceAssistant):
        """Pick the right message based on velocity and distance."""
        name = track.class_name
        vel  = track.velocity if hasattr(track, "velocity") else 0
        dist_m = (track.distance / 100.0) if track.distance else 0

        if vel > 30:
            # Walking away briskly
            msg = f"{name} moved away, path is clear"
        elif vel > 10:
            # Moving away slowly
            msg = f"{name} moving away"
        elif vel > 3:
            # Drifting away / stepping aside
            msg = f"{name} moved aside"
        else:
            # Stopped in place — tell user where they are
            msg = f"{name} stopped, {dist_m:.1f} meters away"

        voice.speak_cleared(msg, key=f"cleared_{track.id}")


# ============= SCENE DESCRIBER CLASS =============
class SceneDescriber:
    """
    On-demand scene description using Claude Vision.
    Press 'd' → frame + depth context → Claude → Piper TTS.
    """

    SYSTEM_PROMPT = (
        "You are a navigation assistant for a blind user wearing a camera. "
        "Describe what you see in 2 to 3 short sentences optimized for spoken audio. "
        "Be spatial and direct: use left, right, ahead, with distances. "
        "IMPORTANT: Always use meters for distances, never feet or inches. "
        "Focus only on what matters for safe navigation — obstacles, people, open paths. "
        "Never say 'I see' or 'the image shows'. Speak directly to the user."
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
            self._start_keyboard_thread()
            return

        try:
            import anthropic
            self._client = anthropic.Anthropic(api_key=api_key)
            self.available = True
            print(f"[OK] Scene Describer ready ({SCENE_MODEL})")
            print("[SCENE] Press 'd' anytime for scene description")
        except ImportError:
            print("[SCENE] WARN: pip install anthropic")
        except Exception as e:
            print(f"[SCENE] WARN: {e}")

        self._start_keyboard_thread()

    def _start_keyboard_thread(self):
        t = threading.Thread(target=self._keyboard_worker, daemon=True)
        t.start()

    def _keyboard_worker(self):
        try:
            import tty
            import termios
            import select as sel
        except ImportError:
            return
        fd = sys.stdin.fileno()
        try:
            old = termios.tcgetattr(fd)
        except termios.error:
            return
        try:
            tty.setcbreak(fd)
            while not self._stop:
                ready, _, _ = sel.select([sys.stdin], [], [], 0.1)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch.lower() == "d":
                        self._trigger()
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    def _trigger(self):
        if not self.available:
            print("[SCENE] Not available — check API key")
            return
        if self._processing:
            print("[SCENE] Still processing — please wait")
            return
        now = time.time()
        elapsed = now - self._last_request_time
        if elapsed < SCENE_COOLDOWN:
            print(f"[SCENE] Cooldown — wait {SCENE_COOLDOWN - elapsed:.1f}s")
            return
        print("[SCENE] Request received — capturing frame...")
        self._pending.set()

    def capture_and_describe(self, color_frame, tracks):
        if not self._pending.is_set():
            return
        self._pending.clear()
        self._last_request_time = time.time()

        frame_copy = color_frame.copy()
        tracks_snapshot = [
            {
                "class_name": t.class_name,
                "distance": t.distance,
                "velocity": t.velocity,
                "box": list(t.box),
            }
            for t in tracks if t.distance is not None
        ]

        t = threading.Thread(
            target=self._describe_thread,
            args=(frame_copy, tracks_snapshot),
            daemon=True,
        )
        t.start()

    def _describe_thread(self, frame, tracks_snapshot):
        self._processing = True
        try:
            self.voice.speak_info("Analyzing scene")

            depth_context = self._build_depth_context(tracks_snapshot, frame.shape[1])

            encode_params = [cv2.IMWRITE_JPEG_QUALITY, SCENE_JPEG_QUALITY]
            success, jpeg_buf = cv2.imencode(".jpg", frame, encode_params)
            if not success:
                raise RuntimeError("JPEG encoding failed")
            image_b64 = base64.standard_b64encode(jpeg_buf.tobytes()).decode("utf-8")

            if depth_context:
                user_text = (
                    f"Depth sensor measurements (use these exact values in meters): "
                    f"{depth_context}.\n\n"
                    "Describe this scene for a blind user navigating on foot. "
                    "Use ONLY meters for distances. 2 to 3 sentences maximum."
                )
            else:
                user_text = (
                    "No objects currently tracked by depth sensor.\n\n"
                    "Describe what you see for a blind user. "
                    "Use ONLY meters for distances. 2 to 3 sentences maximum."
                )

            response = self._client.messages.create(
                model=SCENE_MODEL,
                max_tokens=SCENE_MAX_TOKENS,
                system=self.SYSTEM_PROMPT,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": "image/jpeg",
                                    "data": image_b64,
                                },
                            },
                            {"type": "text", "text": user_text},
                        ],
                    }
                ],
            )

            description = response.content[0].text.strip()
            print(f"\n[SCENE DESCRIPTION]\n  {description}\n")
            self.voice.speak_info(description)

        except Exception as e:
            err = str(e)
            print(f"[SCENE] Error: {err}")
            if "authentication" in err.lower():
                print("[SCENE] Check ANTHROPIC_API_KEY")
            elif "credit" in err.lower():
                print("[SCENE] Check Anthropic account credits")
            self.voice.speak_info("Scene description failed")
        finally:
            self._processing = False

    def _build_depth_context(self, tracks_snapshot, frame_width):
        valid = [t for t in tracks_snapshot if t["distance"] is not None]
        if not valid:
            return ""

        sorted_tracks = sorted(valid, key=lambda t: t["distance"])[:SCENE_MAX_OBJECTS]
        third = frame_width / 3.0
        parts = []

        for t in sorted_tracks:
            x1, _, x2, _ = t["box"]
            cx = (x1 + x2) / 2.0
            position = "left" if cx < third else "right" if cx > 2 * third else "ahead"
            dist_m = t["distance"] / 100.0
            v = t["velocity"]
            motion = (
                "approaching fast" if v < -15 else
                "approaching"      if v < -5  else
                "moving away fast" if v > 15  else
                "moving away"      if v > 5   else
                "stationary"
            )
            parts.append(f"{t['class_name']} {position} at {dist_m:.1f}m ({motion})")

        return ", ".join(parts)

    def shutdown(self):
        self._stop = True


# ============= IMU MOTION DETECTOR =============
class MotionDetector:
    def __init__(self):
        try:
            from icm20948 import ICM20948
            self.imu = ICM20948(i2c_addr=IMU_I2C_ADDR)
            self.available = True
            print(f"[OK] IMU initialized (ICM-20948 at 0x{IMU_I2C_ADDR:02x})")
        except ImportError:
            print("[WARN] icm20948 not installed")
            self.available = False
            self.imu = None
        except Exception as e:
            print(f"[WARN] IMU unavailable: {e}")
            self.available = False
            self.imu = None

        self.accel_history = []
        self.last_read_time = 0
        self._last_moving_state = True
        self._was_moving = True

    def update(self):
        if not self.available:
            return
        now = time.time()
        if now - self.last_read_time < IMU_READ_INTERVAL:
            return
        self.last_read_time = now
        try:
            ax, ay, az, _, _, _ = self.imu.read_accelerometer_gyro_data()
            self.accel_history.append(math.sqrt(ax**2 + ay**2 + az**2))
            if len(self.accel_history) > IMU_WINDOW_SIZE:
                self.accel_history.pop(0)
        except Exception:
            pass

    def is_moving(self):
        if not self.available or len(self.accel_history) < 10:
            return True
        mean = sum(self.accel_history) / len(self.accel_history)
        variance = sum((x - mean) ** 2 for x in self.accel_history) / len(self.accel_history)
        moving = math.sqrt(variance) > IMU_MOTION_THRESHOLD
        self._last_moving_state = moving
        return moving

    def state_changed(self):
        current = self._last_moving_state
        changed = current != self._was_moving
        self._was_moving = current
        return changed

    @property
    def std_dev(self):
        if len(self.accel_history) < 2:
            return 0.0
        mean = sum(self.accel_history) / len(self.accel_history)
        variance = sum((x - mean) ** 2 for x in self.accel_history) / len(self.accel_history)
        return math.sqrt(variance)


# ============= OBJECT TRACKER =============
class ObjectTracker:
    def __init__(self, max_age=1.0):
        self.tracks = {}
        self.next_id = 0
        self.max_age = max_age

    def update(self, detections, current_time):
        expired = [
            tid for tid, t in self.tracks.items()
            if current_time - t.last_seen > self.max_age
        ]
        for tid in expired:
            del self.tracks[tid]

        if not detections:
            return list(self.tracks.values())

        detections_sorted = sorted(detections, key=lambda d: d["score"], reverse=True)
        matched_ids = set()

        for det in detections_sorted:
            best_match = None
            best_iou = TRACK_IOU_THRESH
            for tid, track in self.tracks.items():
                if tid in matched_ids or track.class_name != det["class_name"]:
                    continue
                iou = self._iou(track.box, det["box"])
                if iou > best_iou:
                    best_iou = iou
                    best_match = tid
            if best_match is not None:
                self.tracks[best_match].update(det, current_time)
                matched_ids.add(best_match)
            else:
                self.tracks[self.next_id] = Track(self.next_id, det, current_time)
                self.next_id += 1

        return list(self.tracks.values())

    def _iou(self, b1, b2):
        x1, y1 = max(b1[0], b2[0]), max(b1[1], b2[1])
        x2, y2 = min(b1[2], b2[2]), min(b1[3], b2[3])
        if x2 <= x1 or y2 <= y1:
            return 0.0
        inter = (x2 - x1) * (y2 - y1)
        a1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
        a2 = (b2[2] - b2[0]) * (b2[3] - b2[1])
        union = a1 + a2 - inter
        return inter / union if union > 0 else 0.0


class Track:
    def __init__(self, track_id, detection, timestamp):
        self.id = track_id
        self.class_name = detection["class_name"]
        self.class_id = detection["class_id"]
        self.box = detection["box"]
        self.score = detection["score"]
        self.last_seen = timestamp
        self.distance = None
        self.distance_history = []
        self.velocity = 0.0
        self.velocity_valid = True
        self.num_clusters = 0
        self._prev_velocity_valid = True
        self.seen_frames = 1

    def update(self, detection, timestamp):
        # EMA smoothing on bounding box (NEW v3.19)
        # Damps YOLO's 5-15px frame-to-frame jitter so depth sampling hits
        # a more consistent region of the object each frame.
        # alpha=1.0 on first update so the initial position is exact.
        alpha = 0.6 if self.seen_frames > 1 else 1.0
        nb = detection["box"]
        self.box = [int(alpha*nb[i] + (1-alpha)*self.box[i]) for i in range(4)]
        self.score = detection["score"]
        self.last_seen = timestamp
        self.seen_frames += 1

    def update_distance(self, new_distance, timestamp, camera_z_velocity=0.0):
        """
        Update distance and recalculate velocity with ego-motion compensation.

        camera_z_velocity (cm/s): estimated camera forward velocity from
        EgoMotionCompensator. Subtracted from raw depth-derived velocity so
        that walking toward a static obstacle doesn't register as the obstacle
        approaching. Sign convention: negative = camera moving forward.
        """
        if new_distance is None:
            return
        if self.distance is not None:
            if abs(new_distance - self.distance) > MAX_DISTANCE_JUMP:
                self.velocity_valid = False
                self.velocity = 0.0
                self.distance = new_distance
                self.distance_history = [(timestamp, new_distance)]
                return
        self.distance = new_distance
        self.distance_history.append((timestamp, new_distance))
        self.distance_history = self.distance_history[-5:]
        if len(self.distance_history) >= 3:
            old_time, old_dist = self.distance_history[0]
            dt = timestamp - old_time
            if dt > 0.15:
                raw_v = (new_distance - old_dist) / dt
                max_v = MAX_HUMAN_VELOCITY if self.class_name == "person" else MAX_OBJECT_VELOCITY
                if abs(raw_v) > max_v:
                    self.velocity_valid = False
                    self.velocity = 0.0
                    self._prev_velocity_valid = False
                    return
                if abs(raw_v) < 3:
                    raw_v = 0.0
                # NEW v3.17: subtract camera's own Z motion
                # Without this, walking forward makes static objects appear
                # to approach at walking speed → false URGENT alerts
                compensated_v = raw_v - camera_z_velocity
                if abs(compensated_v) < 3:
                    compensated_v = 0.0
                alpha = 1.0 if not self._prev_velocity_valid else 0.5
                self.velocity = alpha * compensated_v + (1 - alpha) * self.velocity
                self.velocity_valid = True
                self._prev_velocity_valid = True


# ============= THREAT ASSESSMENT =============
class ThreatAssessment:
    @staticmethod
    def calculate_threat_score(track):
        if track.distance is None:
            return 0.0
        if not track.velocity_valid:
            if track.distance < CRITICAL_DISTANCE:
                return 50.0
            if track.distance < WARNING_DISTANCE:
                return 20.0
            return 0.0

        score = 0.0
        d, v = track.distance, track.velocity

        if d < CRITICAL_DISTANCE:
            score += 50 + 50 * (CRITICAL_DISTANCE - d) / CRITICAL_DISTANCE
        elif d < WARNING_DISTANCE:
            score += 50 * (WARNING_DISTANCE - d) / WARNING_DISTANCE
        elif d < SAFE_DISTANCE:
            score += 10 * (SAFE_DISTANCE - d) / SAFE_DISTANCE

        if v < -5:
            score += min(400, abs(v) * 10)
            if v < -50:
                score += min(200, abs(v) * 2)
        elif v > 5:
            score *= 0.2

        score *= OBJECT_THREAT_WEIGHTS.get(track.class_name, 1.0)
        if track.score > 0.7:
            score *= 1.1
        elif track.score < 0.3:
            score *= 0.9

        return min(600, score)

    @staticmethod
    def get_threat_level(score):
        if score > 80:
            return "CRITICAL"
        if score > 40:
            return "WARNING"
        if score > 10:
            return "CAUTION"
        return "SAFE"

    @staticmethod
    def prioritize_threats(tracks):
        scored = [(ThreatAssessment.calculate_threat_score(t), t) for t in tracks]
        scored.sort(reverse=True, key=lambda x: x[0])
        return scored


# ============= HELPER FUNCTIONS =============

def letterbox_resize(img, target_size, buf=None):
    h, w = img.shape[:2]
    scale = min(target_size / h, target_size / w)
    nh, nw = int(h * scale), int(w * scale)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    if buf is None:
        buf = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
    else:
        buf.fill(114)
    top = (target_size - nh) // 2
    left = (target_size - nw) // 2
    buf[top:top + nh, left:left + nw] = resized
    return buf, scale, (left, top)


def yolo_postprocess(output, orig_w, orig_h, scale, pad_left, pad_top):
    """
    YOLO26n postprocess -- NMS-free.

    YOLO26n's one-to-one head (default export) outputs (1, 300, 6).
    Each row: [x1, y1, x2, y2, confidence, class_id] in letterbox space.
    The model has already suppressed duplicates internally -- no NMS needed.
    Just threshold by confidence, unpad, and rescale.
    """
    if output is None:
        return []
    preds = output[0] if output.ndim == 3 else output  # unwrap batch dim

    detections = []
    for row in preds:
        conf = float(row[4])
        if conf < CONF_THRESH:
            continue
        cls_id = int(row[5])
        if cls_id >= len(CLASS_NAMES):
            continue
        name = CLASS_NAMES[cls_id]
        if name in IGNORED_CLASSES:
            continue
        # unpad + rescale from letterbox space to original image coords
        x1 = int(max(0, min((float(row[0]) - pad_left) / scale, orig_w - 1)))
        y1 = int(max(0, min((float(row[1]) - pad_top)  / scale, orig_h - 1)))
        x2 = int(max(0, min((float(row[2]) - pad_left) / scale, orig_w - 1)))
        y2 = int(max(0, min((float(row[3]) - pad_top)  / scale, orig_h - 1)))
        if x2 <= x1 or y2 <= y1 or (x2 - x1) < 10 or (y2 - y1) < 10:
            continue
        detections.append({"class_id": cls_id, "class_name": name,
                            "score": conf, "box": [x1, y1, x2, y2]})
    return detections


def get_smart_distance(depth_data, bbox):
    """
    FIX v3.16: Adaptive stride prevents FPS drop for large (close) bounding boxes.

    At 0.4m the box fills most of the frame, giving ~200k pixels in the crop.
    np.percentile and np.median on 200k values is slow. Adaptive stride scales
    with box size to cap samples at MAX_DEPTH_SAMPLE_PIXELS (4000 px).
    Same depth accuracy, no FPS impact.
    """
    x1, y1, x2, y2 = bbox
    bw, bh = x2 - x1, y2 - y1
    if bw < 8 or bh < 8:
        return None, 0

    cw = int(bw * DEPTH_CENTER_CROP_RATIO)
    ch = int(bh * DEPTH_CENTER_CROP_RATIO)
    cx, cy = x1 + bw // 2, y1 + bh // 2
    sx1 = max(x1, cx - cw // 2)
    sy1 = max(y1, cy - ch // 2)
    sx2 = min(x2, cx + cw // 2)
    sy2 = min(y2, cy + ch // 2)

    # FIX v3.16: Adaptive stride — scale up for large boxes to keep samples bounded
    region_area = max(1, (sy2 - sy1) * (sx2 - sx1))
    adaptive_stride = max(DEPTH_STRIDE, int(math.sqrt(region_area / MAX_DEPTH_SAMPLE_PIXELS)))

    region = depth_data[sy1:sy2:adaptive_stride, sx1:sx2:adaptive_stride]
    valid = region[(region > 0) & (region < 8000)]

    # Lower minimum threshold since adaptive stride reduces sample count
    min_samples = max(30, min(100, len(valid) // 3))
    if len(valid) < min_samples:
        return None, 0

    p10, p90 = np.percentile(valid, 10), np.percentile(valid, 90)
    trimmed = valid[(valid >= p10) & (valid <= p90)]
    if len(trimmed) < 20:
        trimmed = valid

    median_cm = int(np.median(trimmed) / 10)
    iqr = np.percentile(trimmed, 75) - np.percentile(trimmed, 25)

    if iqr < 450:
        return median_cm, 1

    try:
        hist, edges = np.histogram(trimmed, bins=range(0, 8000, 300))
        clusters = []
        for i in range(1, len(hist) - 1):
            if hist[i] > hist[i - 1] and hist[i] > hist[i + 1]:
                if hist[i] >= len(trimmed) * 0.18:
                    clusters.append((int((edges[i] + edges[i + 1]) / 20), hist[i]))
        if len(clusters) > 1:
            return max(clusters, key=lambda c: c[1])[0], len(clusters)
        return median_cm, len(clusters)
    except Exception:
        return median_cm, 0


# ============= EGO-MOTION COMPENSATOR (NEW v3.17) =============

class EgoMotionCompensator:
    """
    Removes camera's own motion from object velocity readings.

    The depth-based velocity formula (new_dist - old_dist) / dt cannot
    distinguish "object moved toward camera" from "camera moved toward
    object." When the user walks at 80 cm/s, every static obstacle looks
    like it's approaching at 80 cm/s — triggering false URGENT alerts.

    TWO METHODS, used together:

    METHOD 1 — Background depth tracking (handles Z / forward motion):
      Sample depth values from pixels that are NOT inside any bounding box
      AND have depth > 1m (so they're likely background, not floor).
      The median of these depths is the "background plane" distance.
      If it decreases, the camera moved forward. The rate of change is
      the camera's Z velocity, which we subtract from each track's raw
      velocity.

    METHOD 2 — Lucas-Kanade sparse optical flow (handles shake and sway):
      Track ~80 Shi-Tomasi feature points from background regions between
      consecutive grayscale frames. The median displacement vector of
      successfully tracked background points is the camera's lateral/
      rotational motion in pixels per frame.
      Currently used for diagnostics/logging. Future: convert to cm/s
      using depth and camera intrinsics for full 3D compensation.

    SIGN CONVENTION (important):
      velocity in the system = cm/s, negative = approaching camera.
      If camera moves FORWARD: background depth DECREASES.
        → background_depth_change = negative
        → camera_z_velocity = negative
      Object raw_velocity when camera moves forward = negative (approaching).
        corrected = raw - camera_z = neg - neg → near zero. ✓

    FAILURE MODES:
      - Texture-poor scene: LK finds few background features → falls back
        to depth-only (camera_lateral reported as 0,0)
      - All background occluded: no valid bg depths → compensation = 0
      - Fast head rotation → blur → LK fails → feature refresh triggered
      - Very slow camera motion (< 2 cm/s) → treated as 0 (noise floor)
    """

    # Lucas-Kanade parameters (sparse optical flow)
    _LK_PARAMS = dict(
        winSize=(15, 15),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
    )

    # Shi-Tomasi feature detection parameters
    _FEATURE_PARAMS = dict(
        maxCorners=80,
        qualityLevel=0.3,
        minDistance=10,
        blockSize=7,
    )

    # Minimum background depth to count as "background" (mm)
    _BG_MIN_DEPTH_MM = 1000   # 1 metre
    _BG_MAX_DEPTH_MM = 8000   # 8 metres

    # Noise floor — ignore camera velocities smaller than this
    _MIN_CAMERA_Z_CM_S = 2.0

    # Rolling window length for smoothing camera Z velocity
    _SMOOTH_WINDOW = 5

    def __init__(self, use_optical_flow=True):
        self.use_optical_flow = use_optical_flow
        self.camera_z_velocity  = 0.0   # cm/s, negative = camera moving forward
        self.camera_lateral_px  = (0.0, 0.0)  # (dx, dy) pixels/frame

        self._prev_bg_depth_mm  = None
        self._z_vel_history     = []
        self._prev_gray         = None
        self._prev_bg_pts       = None
        self._last_time         = 0.0

        # CLAHE for improving contrast before feature detection
        # Helps in dim corridors and under-exposed frames
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    # ---- PUBLIC API ----

    def update(self, color_image, depth_data, tracks, current_time):
        """
        Called every DETECTION_INTERVAL frame with fresh aligned data.
        Updates self.camera_z_velocity and self.camera_lateral_px.
        """
        dt = current_time - self._last_time if self._last_time > 0 else 0.033
        self._last_time = current_time

        bg_mask = self._build_background_mask(depth_data.shape, tracks)

        self._update_depth_z(depth_data, bg_mask, dt)

        if self.use_optical_flow:
            self._update_lk_lateral(color_image, bg_mask)

    def compensate(self, raw_velocity_cm_s):
        """
        Subtract camera Z motion from a raw depth-derived velocity.
        Call this instead of using raw velocity directly.
        """
        if abs(self.camera_z_velocity) < self._MIN_CAMERA_Z_CM_S:
            return raw_velocity_cm_s
        return raw_velocity_cm_s - self.camera_z_velocity

    # ---- INTERNAL: Background depth Z estimation ----

    def _build_background_mask(self, shape, tracks):
        """
        Boolean mask: True = background pixel, False = object bounding box region.
        depth_data is (H, W), so shape is (H, W).
        """
        h, w = shape
        mask = np.ones((h, w), dtype=bool)
        for track in tracks:
            if track.box is None:
                continue
            x1, y1, x2, y2 = map(int, track.box)
            # Pad bbox by 10px to avoid edge depth bleed
            x1 = max(0, x1 - 10)
            y1 = max(0, y1 - 10)
            x2 = min(w, x2 + 10)
            y2 = min(h, y2 + 10)
            mask[y1:y2, x1:x2] = False
        return mask

    def _update_depth_z(self, depth_data, bg_mask, dt):
        """
        Estimate camera Z velocity from change in background depth median.
        """
        bg_depths = depth_data[bg_mask]
        valid = bg_depths[(bg_depths > self._BG_MIN_DEPTH_MM) &
                          (bg_depths < self._BG_MAX_DEPTH_MM)]

        if len(valid) < 150:
            # Not enough background pixels — can't estimate, leave as-is
            return

        current_median_mm = float(np.median(valid))

        if self._prev_bg_depth_mm is not None and dt > 0:
            # Depth change: negative = camera moved forward (background got closer)
            delta_cm = (current_median_mm - self._prev_bg_depth_mm) / 10.0
            raw_z_vel = delta_cm / dt  # cm/s

            # Rolling median smoothing — robust against sudden scene changes
            self._z_vel_history.append(raw_z_vel)
            if len(self._z_vel_history) > self._SMOOTH_WINDOW:
                self._z_vel_history.pop(0)
            self.camera_z_velocity = float(np.median(self._z_vel_history))

        self._prev_bg_depth_mm = current_median_mm

    # ---- INTERNAL: Lucas-Kanade lateral/rotational shake ----

    def _update_lk_lateral(self, color_image, bg_mask):
        """
        Track Shi-Tomasi background feature points with Lucas-Kanade.
        Median flow vector = camera lateral/rotational motion.
        """
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # CLAHE improves LK accuracy in low-contrast / dim environments
        gray = self._clahe.apply(gray)

        bg_mask_u8 = bg_mask.astype(np.uint8) * 255

        # Need at least a previous frame and some prior background points
        if self._prev_gray is None or self._prev_bg_pts is None \
                or len(self._prev_bg_pts) < 5:
            self._refresh_features(gray, bg_mask_u8)
            return

        # Track previous background points into current frame
        new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray,
            self._prev_bg_pts, None,
            **self._LK_PARAMS,
        )

        if new_pts is None or status is None:
            self._refresh_features(gray, bg_mask_u8)
            return

        good = status.flatten() == 1
        if good.sum() < 5:
            # Tracking failed (blur, scene change) — refresh points
            self._refresh_features(gray, bg_mask_u8)
            return

        flow = (new_pts[good] - self._prev_bg_pts[good]).reshape(-1, 2)

        # Median is robust against a few points that drift onto foreground objects
        dx = float(np.median(flow[:, 0]))
        dy = float(np.median(flow[:, 1]))
        self.camera_lateral_px = (dx, dy)

        # Refresh background feature points every frame to avoid drift
        self._prev_gray = gray
        self._refresh_features(gray, bg_mask_u8)

    def _refresh_features(self, gray, bg_mask_u8):
        """Detect new Shi-Tomasi corners in background regions."""
        pts = cv2.goodFeaturesToTrack(gray, mask=bg_mask_u8, **self._FEATURE_PARAMS)
        self._prev_bg_pts = pts
        self._prev_gray   = gray


# ============= MAIN =============

def main():
    global VOICE_ENABLED

    print("=" * 70)
    print("INTELLIGENT NAVIGATION ASSISTANT v3.20 HEADLESS")
    print("RealSense D435 + YOLOv11n + IMU + Piper TTS + Claude Vision")
    print("FIXES: voice queue (no BT drops), moved-away alerts, FPS @ close range")
    print("=" * 70)

    csv_file = open(CSV_FILE, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp", "fps", "tracked_objects", "top_threat", "threat_score",
        "distance_cm", "velocity_cm_s", "velocity_valid", "threat_level",
        "clusters", "user_moving",
    ])
    print(f"[LOG] {CSV_FILE}")

    motion = MotionDetector()
    voice = VoiceAssistant()
    scene = SceneDescriber(voice)
    transition_tracker = ThreatTransitionTracker()  # NEW v3.16
    ego = EgoMotionCompensator(use_optical_flow=True)  # NEW v3.17

    print(f"\n[YOLO] Loading {MODEL_PATH}...")
    try:
        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        opts.intra_op_num_threads = 4
        opts.inter_op_num_threads = 1
        opts.execution_mode = ort.ExecutionMode.ORT_PARALLEL
        sess = ort.InferenceSession(MODEL_PATH, opts, providers=["CPUExecutionProvider"])
        inp_name = sess.get_inputs()[0].name
        print(f"[OK] YOLO loaded ({IMG_SIZE}x{IMG_SIZE}, 4 threads)")
    except Exception as e:
        print(f"[ERROR] YOLO: {e}")
        csv_file.close()
        return

    print("\n[REALSENSE] Initializing D435...")
    pipeline = rs.pipeline()
    cfg = rs.config()

    if PLAYBACK_FILE:
        # ---- PLAYBACK MODE ----
        if not os.path.exists(PLAYBACK_FILE):
            print(f"[ERROR] Bag file not found: {PLAYBACK_FILE}")
            csv_file.close()
            return
        cfg.enable_device_from_file(PLAYBACK_FILE, repeat_playback=PLAYBACK_REPEAT)
        print(f"[OK] Playback mode: {PLAYBACK_FILE}")
        if PLAYBACK_REPEAT:
            print("[INFO] Looping recording — press Ctrl+C to stop")
    else:
        # ---- LIVE MODE (with optional recording) ----
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if RECORD_TO_FILE:
            cfg.enable_record_to_file(RECORD_TO_FILE)
            print(f"[OK] Recording to: {RECORD_TO_FILE}")

    try:
        pipeline.start(cfg)
        if PLAYBACK_FILE:
            print("[OK] Playback started")
        else:
            print("[OK] RealSense started")
    except Exception as e:
        print(f"[ERROR] RealSense: {e}")
        csv_file.close()
        return

    align = rs.align(rs.stream.color)
    print("[INFO] Warming up (30 frames)...")
    for _ in range(30):
        pipeline.wait_for_frames()
    print("[OK] Warmup done")

    tracker = ObjectTracker()
    rgb_buf   = np.empty((480, 640, 3), dtype=np.uint8)
    lb_buf    = np.full((IMG_SIZE, IMG_SIZE, 3), 114, dtype=np.uint8)
    float_buf = np.empty((IMG_SIZE, IMG_SIZE, 3), dtype=np.float32)
    trans_buf = np.empty((1, 3, IMG_SIZE, IMG_SIZE), dtype=np.float32)

    # NEW v3.19: producer-consumer capture pipeline.
    # Capture+align runs in its own thread so it overlaps with YOLO inference
    # instead of running sequentially. Effective throughput:
    #   old: capture(33ms) + align(22ms) + YOLO(69ms) = 124ms
    #   new: max(capture+align=55ms, YOLO=69ms) = 69ms  -- ~30% FPS gain
    # maxsize=2 + put_nowait(): drops frames if main loop lags (stays real-time).
    _frame_q  = queue.Queue(maxsize=2)
    _cap_stop = threading.Event()

    def _capture_worker():
        while not _cap_stop.is_set():
            try:
                frames  = pipeline.wait_for_frames(timeout_ms=1000)
                aligned = align.process(frames)
                df = aligned.get_depth_frame()
                cf = aligned.get_color_frame()
                if not df or not cf:
                    continue
                color_img  = np.asanyarray(cf.get_data())
                depth_data = np.asanyarray(df.get_data())
                try:
                    _frame_q.put_nowait((color_img, depth_data))
                except queue.Full:
                    pass  # main loop behind -- discard stale frame
            except Exception as exc:
                if not _cap_stop.is_set():
                    print(f"[CAPTURE] {exc}")

    threading.Thread(target=_capture_worker, daemon=True, name="capture").start()
    print("[INFO] Capture pipeline started (async align)")

    print("\n[READY] Navigation active")
    if PLAYBACK_FILE:
        print(f"[MODE]  PLAYBACK — {os.path.basename(PLAYBACK_FILE)}")
    elif RECORD_TO_FILE:
        print(f"[MODE]  LIVE + RECORDING — {os.path.basename(RECORD_TO_FILE)}")
    else:
        print("[MODE]  LIVE")
    print("[KEYS]  d = describe scene | Ctrl+C = quit")
    print(f"[SCENE] Model: {SCENE_MODEL} | Cooldown: {SCENE_COOLDOWN}s")
    print(f"[IMU]   Threshold: {IMU_MOTION_THRESHOLD}g\n")

    frame_count       = 0
    last_threat_print = time.time()
    last_paused_print = time.time()
    detect_ms, depth_ms = [], []
    fps_start  = time.time()
    fps_frames = 0
    avg_fps    = 0.0
    last_fps   = 0.0

    try:
        while True:
            # Pull a pre-captured, pre-aligned frame from the capture thread.
            # Blocks up to 2s; warns if camera stalls.
            try:
                color_image, depth_data = _frame_q.get(timeout=2.0)
            except queue.Empty:
                print("[WARN] No frame for 2s -- check camera connection")
                continue

            h, w = color_image.shape[:2]

            # YOLO (still gated to every DETECTION_INTERVAL frames)
            detections = []
            if frame_count % DETECTION_INTERVAL == 0:
                t0 = time.time()
                try:
                    cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB, dst=rgb_buf)
                    img_lb, scale, (pl, pt) = letterbox_resize(rgb_buf, IMG_SIZE, lb_buf)
                    np.divide(img_lb, 255.0, out=float_buf)
                    trans_buf[0] = np.transpose(float_buf, (2, 0, 1))
                    outputs = sess.run(None, {inp_name: trans_buf})
                    detections = yolo_postprocess(outputs[0], w, h, scale, pl, pt)
                except Exception as e:
                    print(f"[ERROR] Detection: {e}")
                detect_ms.append((time.time() - t0) * 1000)
                if len(detect_ms) > 60:
                    detect_ms.pop(0)

            now = time.time()
            tracks = tracker.update(detections, now)

            t0 = time.time()
            for track in tracks:
                dist, nc = get_smart_distance(depth_data, track.box)
                track.num_clusters = nc
                track.update_distance(dist, now, ego.camera_z_velocity)
            depth_ms.append((time.time() - t0) * 1000)
            if len(depth_ms) > 60:
                depth_ms.pop(0)

            # NEW v3.17: Update ego-motion estimate every DETECTION_INTERVAL frame
            # (same cadence as YOLO — already have fresh depth and color)
            if frame_count % DETECTION_INTERVAL == 0:
                ego.update(color_image, depth_data, tracks, now)

            motion.update()
            user_moving = motion.is_moving()
            if motion.state_changed():
                print("[IMU] Moving — all threats active" if user_moving
                      else "[IMU] Stationary — static objects suppressed, approaching still warned")

            scene.capture_and_describe(color_image, tracks)

            threats = ThreatAssessment.prioritize_threats(tracks)

            # NEW v3.16: Check for threat transitions (moved away, stopped, disappeared)
            transition_tracker.update(threats, voice)

            top = threats[0] if threats else None

            # ---- VOICE ANNOUNCEMENT LOGIC ----
            #
            # TWO-LAYER system (same as v3.15):
            #   Layer 1 — Distance override (handles slow lean-in that TTC misses):
            #     dist < 40cm → URGENT always
            #     dist < 70cm → WARNING always
            #   Layer 2 — TTC (time-to-collision = dist / approach_speed):
            #     TTC < 2s  → URGENT
            #     TTC 2-4s  → WARNING
            #     TTC 4-8s  → AWARENESS
            #     TTC > 8s  → silent
            #
            # Ghost filter: ignore tracks seen < 3 frames
            # IMU gate: stationary + not close + not approaching = suppress

            if top:
                score, track = top
                dist_cm = track.distance if track.distance else 999
                dist_m  = dist_cm / 100.0
                obj     = track.class_name
                vel     = track.velocity

                if track.seen_frames >= 3:

                    # TTC calculation
                    if track.velocity_valid and vel < -2:
                        ttc = dist_cm / abs(vel)
                    else:
                        ttc = 999.0

                    approaching   = ttc < 100
                    very_close    = dist_cm < 40
                    close         = dist_cm < 70
                    fast_approach = vel < -50 if track.velocity_valid else False

                    # IMU gate
                    if not user_moving and not very_close and not close and not approaching:
                        if now - last_paused_print > 3.0:
                            print(f"[PAUSED] Static {obj} {dist_m:.1f}m TTC=∞ — suppressed")
                            last_paused_print = now

                    else:
                        # --- LAYER 1: Distance override ---
                        if very_close:
                            if user_moving and not approaching:
                                msg = f"Stop! {obj} right in front of you, {dist_m:.1f} meters"
                            elif approaching and not user_moving:
                                msg = f"Someone approaching you, {dist_m:.1f} meters"
                            elif approaching and user_moving:
                                msg = f"Stop! {obj} approaching fast, {dist_m:.1f} meters"
                            else:
                                msg = f"{obj} right in front of you, {dist_m:.1f} meters"
                            voice.speak_urgent(msg, key=f"{track.id}_dist_urgent")

                        elif close:
                            if user_moving and not approaching:
                                msg = f"Slow down, {obj} ahead, {dist_m:.1f} meters"
                            elif approaching and not user_moving:
                                msg = f"{obj} getting closer to you, {dist_m:.1f} meters"
                            elif approaching and user_moving:
                                msg = f"Watch out, {obj} getting very close"
                            else:
                                msg = f"{obj} nearby, {dist_m:.1f} meters"
                            voice.speak_warning(msg, key=f"{track.id}_dist_warn")

                        # --- LAYER 2: TTC-based ---
                        elif ttc < 2:
                            if fast_approach and not user_moving:
                                msg = f"{obj} approaching you fast, {dist_m:.1f} meters"
                            elif fast_approach:
                                msg = f"Stop! {obj} approaching fast, {dist_m:.1f} meters"
                            elif not user_moving:
                                msg = f"{obj} moving toward you, {dist_m:.1f} meters"
                            else:
                                msg = f"Stop! {obj} {dist_m:.1f} meters, move away"
                            voice.speak_urgent(msg, key=f"{track.id}_ttc_urgent")

                        elif ttc < 4:
                            if not user_moving:
                                msg = f"{obj} approaching you, {dist_m:.1f} meters"
                            else:
                                msg = f"Watch out, {obj} getting closer, {dist_m:.1f} meters"
                            voice.speak_warning(msg, key=f"{track.id}_ttc_warn")

                        elif ttc < 8:
                            msg = f"Heads up, {obj} ahead, {dist_m:.1f} meters"
                            voice.speak_awareness(msg, key=f"{track.id}_ttc_aware")

                    # Log TTC when object is approaching
                    if ttc < 100:
                        tier = ("URGENT" if ttc < 2 else "WARN" if ttc < 4
                                else "AWARE" if ttc < 8 else "NONE")
                        print(
                            f"[TTC]  {obj}#{track.id}(f{track.seen_frames}): "
                            f"dist={dist_cm}cm vel={vel:.1f}cm/s "
                            f"TTC={ttc:.1f}s tier={tier}"
                        )

            # Cleanup stale voice keys every 60 frames
            if frame_count % 60 == 0:
                active_keys = set()
                for t in tracks:
                    active_keys.update([
                        f"{t.id}_dist_urgent", f"{t.id}_dist_warn",
                        f"{t.id}_ttc_urgent", f"{t.id}_ttc_warn", f"{t.id}_ttc_aware",
                        f"cleared_{t.id}",
                    ])
                active_keys.add("path_clear")
                voice.cleanup_old_keys(active_keys)

            # Console threats — throttled to every 2s
            if threats and now - last_threat_print > 2.0:
                imu_str = (
                    f"{'MOVING' if user_moving else 'STILL'} ({motion.std_dev:.3f}g)"
                    if motion.available else "N/A"
                )
                scene_str = "PROCESSING" if scene._processing else "ready (press d)"
                print(f"\n[THREATS] {len(threats)} | IMU={imu_str} | Scene={scene_str}")
                for s, t in threats[:3]:
                    lv = ThreatAssessment.get_threat_level(s)
                    vel_str = (
                        "INVALID!" if not t.velocity_valid
                        else "<<FAST" if t.velocity < -50
                        else "<-APP"  if t.velocity < -5
                        else "FAST>>" if t.velocity > 50
                        else "AWAY->" if t.velocity > 5
                        else "STATIC"
                    )
                    cc = f", C={t.num_clusters}" if t.num_clusters > 1 else ""
                    print(
                        f"  [{lv}] {t.class_name}#{t.id}: "
                        f"{t.distance if t.distance else '???'}cm "
                        f"{vel_str} {abs(t.velocity):.1f}cm/s "
                        f"(score:{s:.1f}){cc}"
                    )
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
                print(
                    f"[PERF] yolo={np.mean(detect_ms):.1f}ms "
                    f"depth={np.mean(depth_ms):.1f}ms "
                    f"queue={_frame_q.qsize()} "
                    f"-> {avg_fps:.1f} FPS | "
                    f"ego: Z={ego.camera_z_velocity:.1f}cm/s "
                    f"lat=({dx:.1f},{dy:.1f})px"
                )

            # CSV
            if frame_count % 30 == 0 and frame_count >= 30:
                csv_writer.writerow([
                    datetime.utcnow().isoformat(), f"{avg_fps:.1f}", len(tracks),
                    top[1].class_name if top else "",
                    f"{top[0]:.1f}" if top else "0",
                    top[1].distance if (top and top[1].distance) else -1,
                    f"{top[1].velocity:.1f}" if top else "0",
                    top[1].velocity_valid if top else True,
                    ThreatAssessment.get_threat_level(top[0]) if top else "NONE",
                    top[1].num_clusters if top else 0,
                    user_moving,
                ])
                csv_file.flush()

            frame_count += 1

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
    finally:
        print("\n[CLEANUP] Shutting down...")
        _cap_stop.set()      # stop capture thread before pipeline.stop()
        scene.shutdown()
        pipeline.stop()
        csv_file.close()
        print(f"[LOG] Saved: {CSV_FILE}")
        print("[OK] Done")
        print("=" * 70)


if __name__ == "__main__":
    main()

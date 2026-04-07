#!/usr/bin/env python3
"""
Intelligent Navigation Assistant v3.23 HEADLESS
Builds on v3.22 with:
  FIX 1 — Velocity noise hallucination (root cause of false CRITICAL at 3m+).
    Dynamic noise floor = 3 + dist*0.02 cm/s (scales with distance).
    TTC gate: if time-to-collision > 12s, velocity zeroed before scoring.
    Person 3m away, apparent vel=-17cm/s: TTC=17s → treated as static. ✓
    Person 2m away, vel=-60cm/s: TTC=3.3s → CRITICAL. ✓

  FIX 2 — Left / right / ahead position in every voice message.
    Bounding box centre X vs frame width:
      left third  (<33%) → "on your left"
      centre third (33-67%) → "ahead"
      right third (>67%) → "on your right"

  FIX 3 — Reverted to float32 yolo26n.onnx.
    INT8 quantization was 252ms vs 130ms float32 on Pi 4 ARM.
    ONNX Runtime dynamic quantization does not speed up this op set on ARM.

  FIX 4 — Richer CSV + separate events log.
    CSV adds: position, bbox_cx_norm, ego_z_cm_s, all_tracks columns.
    events.log records every [VOICE] and [TTC] line — terminal truncation
    can no longer lose important debug data.

INHERITED FROM v3.22:
  - Distance bucket cooldown keys (re-announces every 30cm closer)
  - ThreatTransitionTracker: MIN_THREAT_FRAMES=12, velocity guard
  - IMU hysteresis (8 frames before MOVING/STILL switch)
INHERITED FROM v3.21:
  - Wall/obstacle depth fallback
  - Busy area detection (>= 4 confirmed tracks)
INHERITED FROM v3.17-v3.20:
  - Ego-motion compensation (LK + background depth)
  - YOLO26n NMS-free postprocess
  - Pipelined capture (_capture_worker thread)
  - Piper TTS with pending-queue (no BT drops)
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
from datetime import datetime, timezone

import queue
import numpy as np
import cv2
import pyrealsense2 as rs
import onnxruntime as ort

# ============= CONFIGURATION =============
MODEL_PATH = os.path.expanduser("~/yolo_models/yolo26n.onnx")  # float32 — int8 is SLOWER on Pi ARM
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

PIPER_MODEL  = os.path.expanduser("~/piper_voices/en_US-amy-medium.onnx")
PIPER_CONFIG = os.path.expanduser("~/piper_voices/en_US-amy-medium.onnx.json")
PIPER_SPEED  = 0.85
PIPER_SILENCE_MS = 300

IMU_I2C_ADDR        = 0x68
IMU_MOTION_THRESHOLD = 0.05
IMU_WINDOW_SIZE     = 20
IMU_READ_INTERVAL   = 0.05
IMU_HYSTERESIS_FRAMES = 8

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
            result = subprocess.run(["espeak", "--version"], capture_output=True, timeout=2, check=False)
            self.available = result.returncode == 0
            if self.available: print("[OK] Voice fallback: espeak")
        except Exception:
            self.available = False
            print("[WARN] No TTS available")

    def synthesize_to_file(self, text):
        try:
            speech_buf = io.BytesIO()
            with wave.open(speech_buf, "wb") as wav:
                self._voice.synthesize_wav(text, wav, syn_config=self._syn_config)
            speech_buf.seek(0)
            with wave.open(speech_buf, "rb") as wr:
                params = wr.getparams()
                speech_frames = wr.readframes(wr.getnframes())
            silence_samples = int(params.framerate * params.nchannels * PIPER_SILENCE_MS / 1000)
            silence_frames  = b"\x00" * (silence_samples * params.sampwidth)
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                tmpfile = f.name
            with wave.open(tmpfile, "wb") as out:
                out.setparams(params)
                out.writeframes(silence_frames + speech_frames)
            return tmpfile
        except Exception as e:
            print(f"[VOICE] Synthesis error: {e}")
            return None

# ============= VOICE ASSISTANT =============
class VoiceAssistant:
    COOLDOWN_URGENT    = 1.5
    COOLDOWN_WARNING   = 3.0
    COOLDOWN_AWARENESS = 8.0
    COOLDOWN_CLEARED   = 4.0

    def __init__(self):
        self.is_speaking = False
        self._lock = threading.Lock()
        self._pending = None
        self._current_proc = None
        self._last_announcement = {}
        self._tts = PiperVoice()
        self.available = self._tts.available

    def speak_urgent(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_URGENT, "URGENT", allow_pending=True)
    def speak_warning(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_WARNING, "WARNING", allow_pending=False)
    def speak_awareness(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_AWARENESS, "AWARE", allow_pending=False)
    def speak_cleared(self, text, key=None):
        self._enqueue(text, key, self.COOLDOWN_CLEARED, "CLEAR", allow_pending=True)
    def speak_info(self, text):
        self._enqueue(text, None, 0, "INFO", allow_pending=True)

    def cleanup_old_keys(self, active_ids):
        with self._lock:
            stale = [k for k in self._last_announcement if k not in active_ids]
            for k in stale:
                del self._last_announcement[k]

    def _enqueue(self, text, key, cooldown, label, allow_pending):
        if not VOICE_ENABLED or not self.available:
            return
        with self._lock:
            if key and cooldown > 0:
                last = self._last_announcement.get(key, 0)
                if (time.time() - last) < cooldown:
                    return
            if self.is_speaking:
                if allow_pending:
                    self._pending = (text, key, cooldown, label)
                return
            self._start_locked(text, key, cooldown, label)

    def _start_locked(self, text, key, cooldown, label):
        if key and cooldown > 0:
            self._last_announcement[key] = time.time()
        self.is_speaking = True
        print(f"[VOICE] {label}: \"{text}\"")
        threading.Thread(target=self._speak_thread, args=(text,), daemon=True).start()

    def _speak_thread(self, text):
        try:
            if self._tts._use_piper:
                tmpfile = self._tts.synthesize_to_file(text)
                if tmpfile:
                    proc = subprocess.Popen(["aplay", tmpfile], stderr=subprocess.DEVNULL)
                    with self._lock: self._current_proc = proc
                    proc.wait()
                    with self._lock: self._current_proc = None
                    try: os.unlink(tmpfile)
                    except Exception: pass
            else:
                proc = subprocess.Popen(
                    ["espeak", "-s", "150", "-p", "40", "-g", "5", "-a", "200", text],
                    stderr=subprocess.DEVNULL)
                with self._lock: self._current_proc = proc
                proc.wait()
                with self._lock: self._current_proc = None
        except Exception as e:
            print(f"[VOICE] Thread error: {e}")
        finally:
            with self._lock:
                self.is_speaking = False
                if self._pending is not None:
                    ptext, pkey, pcooldown, plabel = self._pending
                    self._pending = None
                    if pkey and pcooldown > 0:
                        last = self._last_announcement.get(pkey, 0)
                        if (time.time() - last) < pcooldown:
                            return
                    self._start_locked(ptext, pkey, pcooldown, plabel)

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
            prev = self._state.get(tid, {"level":"SAFE","threat_frames":0,"cleared_announced":False,"last_dist":999})
            if level in ("CRITICAL", "WARNING"):
                self._state[tid] = {"level":level, "threat_frames":prev["threat_frames"]+1,
                                    "cleared_announced":False, "last_dist":track.distance or prev["last_dist"]}
            else:
                was_threatening = prev["level"] in ("CRITICAL","WARNING")
                enough_frames   = prev["threat_frames"] >= self.MIN_THREAT_FRAMES
                already         = prev["cleared_announced"]
                if was_threatening and enough_frames and not already:
                    self._announce_cleared(track, prev, voice)
                    self._state[tid] = {"level":level,"threat_frames":0,"cleared_announced":True,
                                        "last_dist":track.distance or prev["last_dist"]}
                else:
                    self._state[tid] = {"level":level,"threat_frames":prev["threat_frames"],
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
        if vel > 30:   msg = f"{track.class_name} moved away, path is clear"
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
        snap = [{"class_name":t.class_name,"distance":t.distance,"velocity":t.velocity,"box":list(t.box)}
                for t in tracks if t.distance is not None]
        threading.Thread(target=self._describe_thread, args=(frame_copy, snap), daemon=True).start()

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
        third = frame_width / 3.0
        parts = []
        for t in valid:
            x1, _, x2, _ = t["box"]
            cx = (x1+x2)/2.0
            pos = "left" if cx < third else "right" if cx > 2*third else "ahead"
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
        try:
            from icm20948 import ICM20948
            self.imu = ICM20948(i2c_addr=IMU_I2C_ADDR)
            self.available = True
            print(f"[OK] IMU initialized (ICM-20948 at 0x{IMU_I2C_ADDR:02x})")
        except Exception as e:
            print(f"[WARN] IMU unavailable: {e}")
            self.available = False
            self.imu = None
        self.accel_history = []
        self.last_read_time = 0
        self._last_moving_state = True
        self._was_moving = True
        self._candidate_state = True
        self._candidate_count = 0

    def update(self):
        if not self.available: return
        now = time.time()
        if now - self.last_read_time < IMU_READ_INTERVAL: return
        self.last_read_time = now
        try:
            ax, ay, az, _, _, _ = self.imu.read_accelerometer_gyro_data()
            self.accel_history.append(math.sqrt(ax**2 + ay**2 + az**2))
            if len(self.accel_history) > IMU_WINDOW_SIZE:
                self.accel_history.pop(0)
        except Exception: pass

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
        expired = [tid for tid, t in self.tracks.items() if current_time - t.last_seen > self.max_age]
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
    def calculate_threat_score(track):
        if track.distance is None: return 0.0
        if not track.velocity_valid:
            if track.distance < CRITICAL_DISTANCE: return 50.0
            if track.distance < WARNING_DISTANCE:  return 20.0
            return 0.0

        d = track.distance
        v = track.velocity

        # FIX v3.23 — dynamic noise floor + TTC gate.
        # RealSense depth noise at distance produces apparent velocity even on
        # a stationary object. Flat 3cm/s noise floor was too low at >2m.
        # (a) Zero velocity below the dynamic noise floor (scales with distance).
        # (b) If TTC > 12s the object is not a meaningful threat regardless
        #     of velocity magnitude — zero its velocity contribution to score.
        noise_floor = 3.0 + d * 0.02          # e.g. 9cm/s at 3m
        if abs(v) <= noise_floor:
            v = 0.0
        elif v < -1:
            ttc_check = d / abs(v)
            if ttc_check > 12.0:               # > 12s = treat as stationary
                v = 0.0

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
    def prioritize_threats(tracks):
        scored = [(ThreatAssessment.calculate_threat_score(t), t) for t in tracks]
        scored.sort(reverse=True, key=lambda x: x[0])
        return scored

# ============= HELPER FUNCTIONS =============

def get_position(track, frame_width=640):
    """
    FIX v3.23: Returns spatial position of object relative to camera.
    Left third (<33%) → 'on your left'
    Centre third (33-67%) → 'ahead'
    Right third (>67%) → 'on your right'
    """
    if track.box is None: return "ahead"
    x1, _, x2, _ = track.box
    ratio = (x1 + x2) / 2.0 / frame_width
    if ratio < 0.33:   return "on your left"
    elif ratio > 0.67: return "on your right"
    return "ahead"


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

# ============= EGO-MOTION COMPENSATOR =============
class EgoMotionCompensator:
    _LK_PARAMS = dict(winSize=(15,15), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    _FEATURE_PARAMS = dict(maxCorners=80, qualityLevel=0.3, minDistance=10, blockSize=7)
    _BG_MIN_DEPTH_MM = 1000
    _BG_MAX_DEPTH_MM = 8000
    _MIN_CAMERA_Z_CM_S = 2.0
    _SMOOTH_WINDOW = 5

    def __init__(self, use_optical_flow=True):
        self.use_optical_flow  = use_optical_flow
        self.camera_z_velocity = 0.0
        self.camera_lateral_px = (0.0, 0.0)
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
        valid = bg_depths[(bg_depths > self._BG_MIN_DEPTH_MM) & (bg_depths < self._BG_MAX_DEPTH_MM)]
        if len(valid) < 150: return
        current_median_mm = float(np.median(valid))
        if self._prev_bg_depth_mm is not None and dt > 0:
            delta_cm = (current_median_mm - self._prev_bg_depth_mm) / 10.0
            raw_z_vel = delta_cm / dt
            self._z_vel_history.append(raw_z_vel)
            if len(self._z_vel_history) > self._SMOOTH_WINDOW:
                self._z_vel_history.pop(0)
            self.camera_z_velocity = float(np.median(self._z_vel_history))
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
    print("INTELLIGENT NAVIGATION ASSISTANT v3.23 HEADLESS")
    print("RealSense D435 + YOLO26n + IMU + Piper TTS + Claude Vision")
    print("FIXES: noise-floor hallucination, left/right/ahead, richer logs")
    print("=" * 70)

    # CSV — richer columns
    csv_file = open(CSV_FILE, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp", "fps", "tracked_objects", "top_threat", "threat_score",
        "distance_cm", "velocity_cm_s", "velocity_valid", "threat_level",
        "clusters", "user_moving", "position", "bbox_cx_norm", "ego_z_cm_s", "all_tracks",
    ])
    print(f"[LOG] {CSV_FILE}")

    # Separate event log — never truncated by terminal scroll
    EVENT_LOG = CSV_FILE.replace("log_", "events_").replace(".csv", ".log")
    event_file = open(EVENT_LOG, "w", encoding="utf-8")
    def log_event(msg):
        ts = datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3]
        event_file.write(f"[{ts}] {msg}\n")
        event_file.flush()
    print(f"[LOG] Events: {EVENT_LOG}")

    # Patch VoiceAssistant._start_locked to also write to event log
    _orig_start = VoiceAssistant._start_locked
    def _patched_start(self, text, key, cooldown, label):
        log_event(f"[VOICE] {label}: \"{text}\"")
        _orig_start(self, text, key, cooldown, label)
    VoiceAssistant._start_locked = _patched_start

    motion = MotionDetector()
    voice  = VoiceAssistant()
    scene  = SceneDescriber(voice)
    transition_tracker = ThreatTransitionTracker()
    ego    = EgoMotionCompensator(use_optical_flow=True)

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
                    _frame_q.put_nowait((np.asanyarray(cf.get_data()), np.asanyarray(df.get_data())))
                except queue.Full:
                    pass
            except Exception as exc:
                if not _cap_stop.is_set(): print(f"[CAPTURE] {exc}")

    threading.Thread(target=_capture_worker, daemon=True, name="capture").start()
    print("[INFO] Capture pipeline started (async align)")
    print("\n[READY] Navigation active")
    print(f"[MODE]  {'PLAYBACK' if PLAYBACK_FILE else 'LIVE'}")
    print(f"[KEYS]  d = describe scene | Ctrl+C = quit")
    print(f"[SCENE] Model: {SCENE_MODEL} | Cooldown: {SCENE_COOLDOWN}s")
    print(f"[IMU]   Threshold: {IMU_MOTION_THRESHOLD}g\n")

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

            motion.update()
            user_moving = motion.is_moving()
            if motion.state_changed():
                print("[IMU] Moving — all threats active" if user_moving
                      else "[IMU] Stationary — static objects suppressed")

            scene.capture_and_describe(color_image, tracks)
            threats = ThreatAssessment.prioritize_threats(tracks)
            transition_tracker.update(threats, voice)

            # Busy area
            confirmed_count = sum(1 for t in tracks if t.seen_frames >= 3)
            if (confirmed_count >= BUSY_TRACK_THRESHOLD and user_moving
                    and now - last_busy_alert > BUSY_COOLDOWN_S):
                voice.speak_awareness(f"Busy area, {confirmed_count} objects around you, slow down", key="busy_area")
                last_busy_alert = now
                print(f"[BUSY] {confirmed_count} confirmed tracks")

            # Wall fallback
            if user_moving and now - last_wall_alert > WALL_COOLDOWN_S:
                h_d, w_d = depth_data.shape
                cy0 = int(h_d*(0.5-WALL_CENTER_CROP/2)); cy1 = int(h_d*(0.5+WALL_CENTER_CROP/2))
                cx0 = int(w_d*(0.5-WALL_CENTER_CROP/2)); cx1 = int(w_d*(0.5+WALL_CENTER_CROP/2))
                center_crop = depth_data[cy0:cy1, cx0:cx1]
                valid_px = center_crop[(center_crop > 100) & (center_crop < 8000)]
                if len(valid_px) >= WALL_MIN_VALID_PX:
                    wall_dist_cm = int(np.median(valid_px) / 10)
                    yolo_covers = any(t.distance is not None and t.distance < (WALL_DISTANCE_CM+20)
                                      and t.seen_frames >= 3 for t in tracks)
                    if wall_dist_cm < WALL_DISTANCE_CM and not yolo_covers:
                        voice.speak_warning(f"Obstacle ahead, {wall_dist_cm/100:.1f} meters", key="wall_fallback")
                        last_wall_alert = now
                        print(f"[WALL] Depth fallback: {wall_dist_cm}cm")

            top = threats[0] if threats else None

            if top:
                score, track = top
                dist_cm = track.distance if track.distance else 999
                dist_m  = dist_cm / 100.0
                obj     = track.class_name
                vel     = track.velocity
                pos     = get_position(track, w)   # FIX v3.23: left/right/ahead

                if track.seen_frames >= 3:
                    if track.velocity_valid and vel < -2:
                        ttc = dist_cm / abs(vel)
                    else:
                        ttc = 999.0

                    approaching   = ttc < 100
                    very_close    = dist_cm < 40
                    close         = dist_cm < 70
                    fast_approach = vel < -50 if track.velocity_valid else False

                    if not user_moving and not very_close and not close and not approaching:
                        if now - last_paused_print > 3.0:
                            print(f"[PAUSED] Static {obj} {dist_m:.1f}m — suppressed")
                            last_paused_print = now
                    else:
                        dbucket = int(dist_cm // 30)

                        if very_close:
                            if user_moving and not approaching:
                                msg = f"Stop! {obj} right in front of you, {dist_m:.1f} meters"
                            elif approaching and not user_moving:
                                msg = f"Someone approaching you {pos}, {dist_m:.1f} meters"
                            elif approaching and user_moving:
                                msg = f"Stop! {obj} {pos} approaching fast, {dist_m:.1f} meters"
                            else:
                                msg = f"{obj} right in front of you, {dist_m:.1f} meters"
                            voice.speak_urgent(msg, key=f"{track.id}_dist_urgent_{dbucket}")

                        elif close:
                            if user_moving and not approaching:
                                msg = f"Slow down, {obj} {pos}, {dist_m:.1f} meters"
                            elif approaching and not user_moving:
                                msg = f"{obj} {pos} getting closer, {dist_m:.1f} meters"
                            elif approaching and user_moving:
                                msg = f"Watch out, {obj} {pos} getting very close"
                            else:
                                msg = f"{obj} {pos}, {dist_m:.1f} meters"
                            voice.speak_warning(msg, key=f"{track.id}_dist_warn_{dbucket}")

                        elif ttc < 2:
                            if fast_approach and not user_moving:
                                msg = f"{obj} {pos} approaching you fast, {dist_m:.1f} meters"
                            elif fast_approach:
                                msg = f"Stop! {obj} {pos} approaching fast, {dist_m:.1f} meters"
                            elif not user_moving:
                                msg = f"{obj} {pos} moving toward you, {dist_m:.1f} meters"
                            else:
                                msg = f"Stop! {obj} {pos} {dist_m:.1f} meters, move away"
                            voice.speak_urgent(msg, key=f"{track.id}_ttc_urgent_{dbucket}")

                        elif ttc < 4:
                            if not user_moving:
                                msg = f"{obj} {pos} approaching you, {dist_m:.1f} meters"
                            else:
                                msg = f"Watch out, {obj} {pos} getting closer, {dist_m:.1f} meters"
                            voice.speak_warning(msg, key=f"{track.id}_ttc_warn_{dbucket}")

                        elif ttc < 8:
                            msg = f"Heads up, {obj} {pos}, {dist_m:.1f} meters"
                            voice.speak_awareness(msg, key=f"{track.id}_ttc_aware")

                    if ttc < 100:
                        tier = ("URGENT" if ttc<2 else "WARN" if ttc<4 else "AWARE" if ttc<8 else "NONE")
                        line = (f"[TTC]  {obj}#{track.id}(f{track.seen_frames}) {pos}: "
                                f"dist={dist_cm}cm vel={vel:.1f}cm/s TTC={ttc:.1f}s tier={tier}")
                        print(line)
                        log_event(line)

            # Cleanup stale voice keys
            if frame_count % 60 == 0:
                active_keys = set()
                for t in tracks:
                    for b in range(11):
                        active_keys.update([f"{t.id}_dist_urgent_{b}", f"{t.id}_dist_warn_{b}",
                                            f"{t.id}_ttc_urgent_{b}", f"{t.id}_ttc_warn_{b}"])
                    active_keys.add(f"{t.id}_ttc_aware")
                    active_keys.add(f"cleared_{t.id}")
                active_keys.add("path_clear")
                voice.cleanup_old_keys(active_keys)

            # Console threats
            if threats and now - last_threat_print > 2.0:
                imu_str = (f"{'MOVING' if user_moving else 'STILL'} ({motion.std_dev:.3f}g)"
                           if motion.available else "N/A")
                print(f"\n[THREATS] {len(threats)} | IMU={imu_str}")
                for s, t in threats[:3]:
                    lv = ThreatAssessment.get_threat_level(s)
                    vs = ("INVALID!" if not t.velocity_valid else
                          "<<FAST" if t.velocity<-50 else "<-APP" if t.velocity<-5 else
                          "FAST>>" if t.velocity>50 else "AWAY->" if t.velocity>5 else "STATIC")
                    cc = f", C={t.num_clusters}" if t.num_clusters>1 else ""
                    p  = get_position(t, w)
                    print(f"  [{lv}] {t.class_name}#{t.id} {p}: "
                          f"{t.distance if t.distance else '???'}cm "
                          f"{vs} {abs(t.velocity):.1f}cm/s (score:{s:.1f}){cc}")
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
                print(f"[PERF] yolo={np.mean(detect_ms):.1f}ms depth={np.mean(depth_ms):.1f}ms "
                      f"queue={_frame_q.qsize()} -> {avg_fps:.1f} FPS | "
                      f"ego: Z={ego.camera_z_velocity:.1f}cm/s lat=({dx:.1f},{dy:.1f})px")

            # CSV — richer
            if frame_count % 30 == 0 and frame_count >= 30:
                if top:
                    t1  = top[1]
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
                    f"{top[1].velocity:.1f}" if top else "0",
                    top[1].velocity_valid if top else True,
                    ThreatAssessment.get_threat_level(top[0]) if top else "NONE",
                    top[1].num_clusters if top else 0,
                    user_moving,
                    pos_csv, cx_norm,
                    round(ego.camera_z_velocity, 1),
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
        pipeline.stop()
        csv_file.close()
        event_file.close()
        print(f"[LOG] Saved: {CSV_FILE}")
        print(f"[LOG] Events: {EVENT_LOG}")
        print("[OK] Done")
        print("=" * 70)

if __name__ == "__main__":
    main()

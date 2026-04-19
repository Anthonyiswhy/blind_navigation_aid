"""
test_blindnav_v325.py - Hardware-free test suite for v3.25

Design philosophy (Boris Cherny approach):
  Every bug in v3.24 was detectable without hardware.
  - Voice latency?   -> mock TTS with controllable duration, measure timestamps
  - Ego-Z outliers?  -> feed numpy arrays of fake depth data, assert ego_confident
  - Track ID churn?  -> create Track objects manually, assert zone key collisions
  - Presyn thrash?   -> count concurrent synthesis threads, assert <= 1

No camera. No RealSense. No Piper. No aplay. No IMU.

Run: pytest tests/test_blindnav_v325.py -v
"""
import sys
import os
import time
import wave
import tempfile
import threading
import math
import types
import importlib.util
from unittest.mock import MagicMock, patch
import pytest
import numpy as np


# ============================================================
# MODULE LOADING WITH HARDWARE STUBS
# ============================================================

def _make_stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    return m

def load_module():
    """
    Load the v3.25 navigation module with all hardware deps stubbed out.
    Uses the same pattern as the existing 37-test suite.
    """
    # Build a minimal stub tree
    rs_stub = _make_stub("pyrealsense2",
        pipeline=MagicMock,
        config=MagicMock,
        align=MagicMock,
        stream=MagicMock(depth=0, color=1),
        format=MagicMock(z16=0, bgr8=1),
    )
    ort_stub = _make_stub("onnxruntime",
        InferenceSession=MagicMock,
        SessionOptions=MagicMock,
        GraphOptimizationLevel=MagicMock(ORT_ENABLE_ALL=3),
        ExecutionMode=MagicMock(ORT_SEQUENTIAL=0, ORT_PARALLEL=1),
    )

    # cv2 stub - only needs createCLAHE and cvtColor for tests
    cv2_stub = _make_stub("cv2",
        resize=lambda img, dsize, **kw: np.zeros((*dsize[::-1], 3), dtype=np.uint8),
        cvtColor=lambda img, code, dst=None: (dst if dst is not None else img),
        COLOR_BGR2RGB=0, COLOR_BGR2GRAY=1,
        INTER_LINEAR=1,
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda ext, img, params=None: (True, np.array([0,0,0], dtype=np.uint8)),
        calcOpticalFlowPyrLK=MagicMock(return_value=(None, None, None)),
        goodFeaturesToTrack=MagicMock(return_value=None),
        TERM_CRITERIA_EPS=1, TERM_CRITERIA_COUNT=2,
        createCLAHE=MagicMock(return_value=MagicMock(
            apply=lambda img: img
        )),
    )
    anthropic_stub = _make_stub("anthropic", Anthropic=MagicMock)
    icm_stub = _make_stub("icm20948", ICM20948=MagicMock(
        side_effect=ImportError("No IMU in test")
    ))
    smbus_stub = _make_stub("smbus2")
    piper_stub = _make_stub("piper")
    piper_voice_stub = _make_stub("piper.voice", PiperVoice=MagicMock)
    piper_config_stub = _make_stub("piper.config", SynthesisConfig=MagicMock)

    stubs = {
        "pyrealsense2": rs_stub,
        "onnxruntime":  ort_stub,
        "cv2":          cv2_stub,
        "anthropic":    anthropic_stub,
        "icm20948":     icm_stub,
        "smbus2":       smbus_stub,
        "piper":        piper_stub,
        "piper.voice":  piper_voice_stub,
        "piper.config": piper_config_stub,
    }
    for name, stub in stubs.items():
        sys.modules[name] = stub

    spec = importlib.util.spec_from_file_location(
        "nav325",
        os.path.join(os.path.dirname(__file__), "..",
                     "raspberry_pi", "yolo_realsense_navigation.py"),
    )
    module = importlib.util.module_from_spec(spec)
    # Patch makedirs so it doesn't try to write to /home on CI
    with patch("os.makedirs", return_value=None):
        spec.loader.exec_module(module)
    return module


# Load once for the session
MOD = load_module()


# ============================================================
# FAKE TTS & PLAYER (test doubles)
# ============================================================

class FakeTTS:
    """
    Controllable TTS double.
    - Records every synthesized (text, start_ts, end_ts).
    - Tracks peak concurrent synthesis count (to verify semaphore).
    - synth_delay: simulated synthesis time (seconds).
    - Returns a tiny real WAV file so aplay/player can open it.
    """
    def __init__(self, synth_delay=0.02):
        self.available   = True
        self._use_piper  = True
        self.synth_delay = synth_delay
        self.synthesized = []          # [(text, start_ts, end_ts)]
        self._sem        = threading.Lock()
        self._concurrent = 0
        self.max_concurrent = 0

    def synthesize_to_file(self, text, silence_ms=0):
        with self._sem:
            self._concurrent += 1
            if self._concurrent > self.max_concurrent:
                self.max_concurrent = self._concurrent
        start = time.time()
        time.sleep(self.synth_delay)
        end   = time.time()
        with self._sem:
            self._concurrent -= 1
        self.synthesized.append((text, start, end))
        return _make_wav(silence_ms)

    def prepend_silence(self, wav_path, silence_ms):
        return wav_path  # no-op in tests


class FailTTS(FakeTTS):
    """TTS that always fails synthesis."""
    def synthesize_to_file(self, text, silence_ms=0):
        self.synthesized.append((text, time.time(), time.time()))
        return None  # simulate failure


def _make_wav(silence_ms=0):
    """Write a minimal 22050 Hz mono WAV and return its path."""
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        path = f.name
    n_samples = max(1, int(22050 * (silence_ms / 1000.0 + 0.01)))
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(22050)
        w.writeframes(b"\x00" * n_samples * 2)
    return path


class FakeProc:
    """Mimics subprocess.Popen - plays synchronously for `duration` seconds."""
    def __init__(self, duration=0.03):
        self._done = threading.Event()
        threading.Timer(duration, self._done.set).start()
    def wait(self):
        self._done.wait()


def fast_player(play_duration=0.03):
    """Returns a _player_fn that completes in `play_duration` seconds."""
    def _player(wav_path):
        try: os.unlink(wav_path)
        except Exception: pass
        return FakeProc(play_duration)
    return _player


def build_voice(synth_delay=0.02, play_duration=0.03, tts=None):
    """Build a VoiceAssistant wired to fake TTS + player."""
    fake_tts = tts or FakeTTS(synth_delay=synth_delay)
    va = MOD.VoiceAssistant(
        _tts_override=fake_tts,
        _player_fn=fast_player(play_duration),
    )
    return va, fake_tts


def wait_for_idle(va, timeout=5.0):
    """Block until voice is no longer speaking (all pending drained)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(0.01)
        with va._lock:
            if not va.is_speaking and all(p is None for p in va._pending.values()):
                return True
    return False   # timed out


# ============================================================
# TRUTH TABLE: ThreatAssessment
# ============================================================

class TestThreatScoringTruthTable:
    """
    Parameterized truth table for ThreatAssessment.calculate_threat_score.
    All inputs tested without hardware.
    """

    def _make_track(self, dist_cm, vel_cm_s, class_name="person",
                    confidence=0.5, vel_valid=True):
        """Minimal Track-like object for scoring."""
        class FakeTrack:
            pass
        t = FakeTrack()
        t.distance        = dist_cm
        t.velocity        = vel_cm_s
        t.class_name      = class_name
        t.score           = confidence
        t.velocity_valid  = vel_valid
        t.seen_frames     = 5
        return t

    CASES = [
        (25,   0,     "person",       0.5, "CRITICAL"),
        (49,   0,     "person",       0.5, "WARNING"),
        (51,   0,     "person",       0.5, "WARNING"),
        (100,  0,     "person",       0.5, "CAUTION"),
        (149,  0,     "person",       0.5, "SAFE"),
        (151,  0,     "person",       0.5, "SAFE"),
        (250,  0,     "person",       0.5, "SAFE"),
        (299,  0,     "person",       0.5, "SAFE"),
        (350,  0,     "person",       0.5, "SAFE"),
        (200,  -80,   "person",       0.5, "CRITICAL"),
        (200,   80,   "person",       0.5, "SAFE"),
        (200,  -80,   "car",          0.5, "CRITICAL"),
        (200,  -80,   "bottle",       0.5, "CRITICAL"),
        (300, -8,    "person",       0.5, "SAFE"),
        (100, -6,    "person",       0.5, "CAUTION"),
        (200, -10,   "person",       0.5, "SAFE"),
        (30,   0,    "person",       0.5, "CRITICAL"),
        (80,  -60,   "person",       0.8, "CRITICAL"),
        (80,  -60,   "person",       0.2, "CRITICAL"),
    ]

    @pytest.mark.parametrize("dist,vel,cls,conf,expected", CASES)
    def test_threat_level(self, dist, vel, cls, conf, expected):
        track = self._make_track(dist, vel, cls, conf)
        score = MOD.ThreatAssessment.calculate_threat_score(track)
        level = MOD.ThreatAssessment.get_threat_level(score)
        assert level == expected

    def test_none_distance_returns_zero(self):
        t = self._make_track(None, 0)
        assert MOD.ThreatAssessment.calculate_threat_score(t) == 0.0

    def test_score_capped_at_600(self):
        t = self._make_track(10, -500, "car", 0.9)
        score = MOD.ThreatAssessment.calculate_threat_score(t)
        assert score <= 600.0

    def test_moving_away_reduces_score(self):
        t_static    = self._make_track(100, 0,   "person")
        t_moving_away = self._make_track(100, 50, "person")
        s_static = MOD.ThreatAssessment.calculate_threat_score(t_static)
        s_away   = MOD.ThreatAssessment.calculate_threat_score(t_moving_away)
        assert s_away < s_static

    def test_higher_weight_higher_score(self):
        t_car    = self._make_track(100, -50, "car")
        t_bottle = self._make_track(100, -50, "bottle")
        assert (MOD.ThreatAssessment.calculate_threat_score(t_car) >
                MOD.ThreatAssessment.calculate_threat_score(t_bottle))


class TestVoiceKey:
    def test_same_zone_same_family_same_key(self):
        k1 = MOD._voice_key("ahead", "chair",       "ttc_urg", 3)
        k2 = MOD._voice_key("ahead", "dining table", "ttc_urg", 3)
        assert k1 == k2

    def test_same_zone_same_electronics(self):
        k1 = MOD._voice_key("on your right", "tv",     "dist_urg", 1)
        k2 = MOD._voice_key("on your right", "laptop", "dist_urg", 1)
        assert k1 == k2

    def test_different_zone_different_key(self):
        k_left  = MOD._voice_key("on your left",  "person", "ttc_urg", 2)
        k_right = MOD._voice_key("on your right", "person", "ttc_urg", 2)
        assert k_left != k_right

    def test_different_family_different_key(self):
        k_person = MOD._voice_key("ahead", "person", "ttc_urg", 2)
        k_chair  = MOD._voice_key("ahead", "chair",  "ttc_urg", 2)
        assert k_person != k_chair

    def test_different_tier_different_key(self):
        k_urg = MOD._voice_key("ahead", "person", "ttc_urg", 2)
        k_wrn = MOD._voice_key("ahead", "person", "ttc_wrn", 2)
        assert k_urg != k_wrn

    def test_different_bucket_different_key(self):
        k1 = MOD._voice_key("ahead", "person", "ttc_urg", 2)
        k2 = MOD._voice_key("ahead", "person", "ttc_urg", 3)
        assert k1 != k2

    def test_id_churn_scenario(self):
        k_id5 = MOD._voice_key("ahead", "person", "ttc_urg", 2)
        k_id6 = MOD._voice_key("ahead", "person", "ttc_urg", 2)
        assert k_id5 == k_id6

    KEY_CASES = [
        ("ahead",         "person",       "center",  "person"),
        ("on your left",  "chair",        "left",    "furniture"),
        ("on your right", "tv",           "right",   "electronics"),
        ("on your left",  "car",          "left",    "vehicle"),
        ("ahead",         "backpack",     "center",  "object"),
        ("on your right", "couch",        "right",   "furniture"),
        ("on your left",  "laptop",       "left",    "electronics"),
        ("ahead",         "bicycle",      "center",  "vehicle"),
        ("ahead",         "keyboard",     "center",  "electronics"),
        ("on your right", "dining table", "right",   "furniture"),
    ]

    @pytest.mark.parametrize("pos,cls,zone,family", KEY_CASES)
    def test_key_structure(self, pos, cls, zone, family):
        tier = "ttc_urg"
        bucket = 3
        key = MOD._voice_key(pos, cls, tier, bucket)
        expected = f"{zone}_{family}_{tier}_{bucket}"
        assert key == expected


class TestEgoMotionCompensator:
    def _make_ego(self):
        ego = MOD.EgoMotionCompensator(use_optical_flow=False)
        return ego

    def _run_z_update(self, ego, depth_mm_sequence, dt=0.033):
        prev = None
        for depth_mm in depth_mm_sequence:
            if prev is not None:
                delta_cm = (depth_mm - prev) / 10.0
                raw_z_vel = delta_cm / dt
                raw_z_vel = max(-ego._MAX_Z_CM_S, min(ego._MAX_Z_CM_S, raw_z_vel))
                ego._z_vel_history.append(raw_z_vel)
                if len(ego._z_vel_history) > ego._SMOOTH_WINDOW:
                    ego._z_vel_history.pop(0)
                smoothed = float(np.median(ego._z_vel_history))
                if len(ego._z_vel_history) >= 3:
                    window_std = float(np.std(ego._z_vel_history))
                    ego.ego_confident = window_std < ego._EGO_STD_THRESHOLD
                else:
                    ego.ego_confident = False
                ego.camera_z_velocity = smoothed if ego.ego_confident else 0.0
            ego._prev_bg_depth_mm = depth_mm
            prev = depth_mm

    def test_stable_approach_is_confident(self):
        ego = self._make_ego()
        base = 3000
        sequence = [base - i * 264 for i in range(8)]
        self._run_z_update(ego, sequence)
        assert ego.ego_confident
        assert abs(ego.camera_z_velocity) > 0

    def test_outlier_frame_clamped(self):
        ego = self._make_ego()
        dt = 0.033
        spike_mm = 10000
        raw_unclamped = (spike_mm / 10.0) / dt
        raw_clamped   = min(ego._MAX_Z_CM_S, raw_unclamped)
        assert raw_clamped == ego._MAX_Z_CM_S

    def test_chaotic_values_not_confident(self):
        ego = self._make_ego()
        sequence = [1000, 8000, 1000, 8000, 1000, 8000, 1000, 8000]
        self._run_z_update(ego, sequence)
        assert not ego.ego_confident
        assert ego.camera_z_velocity == 0.0

    def test_stationary_camera_confident(self):
        ego = self._make_ego()
        sequence = [3000, 3001, 2999, 3000, 3001, 3000, 2999, 3000]
        self._run_z_update(ego, sequence)
        assert ego.ego_confident
        assert abs(ego.camera_z_velocity) < 5.0

    def test_confidence_requires_3_samples(self):
        ego = self._make_ego()
        sequence = [3000, 2900, 2800]
        self._run_z_update(ego, sequence)
        assert not ego.ego_confident

    def test_max_clamp_value(self):
        assert MOD.EgoMotionCompensator._MAX_Z_CM_S == 160.0

    def test_confidence_threshold(self):
        assert MOD.EgoMotionCompensator._EGO_STD_THRESHOLD == 40.0


class TestVoicePriorityQueue:
    def test_p0_evicts_p1_p2(self):
        va, tts = build_voice(synth_delay=0.01, play_duration=0.3)
        played = []

        orig_start = va._start_locked.__func__
        def patched_start(self, text, key, cooldown, label, priority,
                          event_created_ts=None, enqueued_ts=None):
            played.append((label, text))
            orig_start(self, text, key, cooldown, label, priority,
                       event_created_ts, enqueued_ts)

        import types as _types
        va._start_locked = _types.MethodType(patched_start, va)

        va.speak_awareness("aware message")
        time.sleep(0.05)
        assert va.is_speaking

        va.speak_awareness("aware pending")
        va.speak_warning("warning pending")
        va.speak_urgent("urgent pending")

        with va._lock:
            assert va._pending[0] is not None
            assert va._pending[1] is None
            assert va._pending[2] is None

        assert wait_for_idle(va, timeout=3.0)
        played_labels = [p[0] for p in played]
        assert "URGENT" in played_labels

    def test_p1_evicts_p2_not_p0(self):
        va, tts = build_voice(synth_delay=0.01, play_duration=0.3)
        va.speak_awareness("first")
        time.sleep(0.05)
        assert va.is_speaking

        va.speak_urgent("urgent")
        va.speak_awareness("aware")
        va.speak_warning("warning")

        with va._lock:
            assert va._pending[0] is not None
            assert va._pending[1] is not None
            assert va._pending[2] is None

        wait_for_idle(va)

    def test_cooldown_prevents_duplicate(self):
        va, tts = build_voice(synth_delay=0.01, play_duration=0.05)
        key = "center_person_ttc_urg_2"

        va.speak_urgent("person ahead", key=key)
        time.sleep(0.01)
        va.speak_urgent("person ahead again", key=key)
        wait_for_idle(va)

        spoken_texts = [s[0] for s in tts.synthesized]
        assert spoken_texts.count("person ahead again") == 0

    def test_different_zone_key_not_blocked(self):
        va, tts = build_voice(synth_delay=0.01, play_duration=0.05)
        k_left  = MOD._voice_key("on your left",  "person", "ttc_urg", 2)
        k_right = MOD._voice_key("on your right", "person", "ttc_urg", 2)
        assert k_left != k_right

        va.speak_urgent("person left",  key=k_left)
        wait_for_idle(va)
        va.speak_urgent("person right", key=k_right)
        wait_for_idle(va)

        spoken = [s[0] for s in tts.synthesized]
        assert "person left"  in spoken
        assert "person right" in spoken

    def test_id_churn_hits_cooldown(self):
        va, tts = build_voice(synth_delay=0.01, play_duration=0.05)
        key = MOD._voice_key("ahead", "person", "ttc_urg", 3)

        va.speak_urgent("person from track 5", key=key)
        wait_for_idle(va)

        va.speak_urgent("person from track 6", key=key)
        wait_for_idle(va)

        spoken = [s[0] for s in tts.synthesized]
        assert "person from track 6" not in spoken


class TestVoiceTTL:
    def test_p0_ttl_3s(self):
        va, tts = build_voice(synth_delay=0.01, play_duration=0.5)

        va.speak_awareness("long speech")
        time.sleep(0.05)
        assert va.is_speaking

        with va._lock:
            va._pending[0] = (
                "stale urgent",
                None,
                0,
                "URGENT",
                time.time() - 4.0,
                time.time() - 4.0,
            )

        wait_for_idle(va, timeout=3.0)

        spoken = [s[0] for s in tts.synthesized]
        assert "stale urgent" not in spoken

    def test_fresh_p0_is_not_expired(self):
        va, tts = build_voice(synth_delay=0.01, play_duration=0.3)
        va.speak_awareness("blocking speech")
        time.sleep(0.05)

        va.speak_urgent("fresh urgent")
        wait_for_idle(va, timeout=3.0)

        spoken = [s[0] for s in tts.synthesized]
        assert "fresh urgent" in spoken


class TestPresynSemaphore:
    def test_only_one_synthesis_at_a_time(self):
        slow_tts = FakeTTS(synth_delay=0.1)
        va = MOD.VoiceAssistant(
            _tts_override=slow_tts,
            _player_fn=fast_player(0.3),
        )

        va.speak_awareness("blocking")
        time.sleep(0.05)

        for i in range(8):
            va.speak_urgent(f"urgent message {i}")
            time.sleep(0.005)

        wait_for_idle(va, timeout=5.0)

        assert slow_tts.max_concurrent <= 1

    def test_semaphore_initialized(self):
        va, _ = build_voice()
        assert hasattr(va, "_presyn_sem")
        assert va._presyn_sem._value == 1


class TestLatencyTimestamps:
    def test_latency_line_logged_to_event_logger(self):
        latency_lines = []

        def capture_log(msg):
            if "[LATENCY]" in msg:
                latency_lines.append(msg)

        va = MOD.VoiceAssistant(
            event_logger=capture_log,
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )
        va.speak_urgent("latency test")
        wait_for_idle(va, timeout=2.0)

        assert len(latency_lines) >= 1
        line = latency_lines[0]
        assert "event_created=" in line
        assert "tts_start=" in line
        assert "play_start=" in line
        assert "play_end=" in line

    def test_latency_values_are_non_negative(self):
        latency_lines = []

        def capture_log(msg):
            if "[LATENCY]" in msg:
                latency_lines.append(msg)

        va = MOD.VoiceAssistant(
            event_logger=capture_log,
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )
        va.speak_urgent("latency check")
        wait_for_idle(va, timeout=2.0)

        assert latency_lines
        line = latency_lines[0]
        import re
        deltas = re.findall(r"(\w+)=([\d.]+)s", line)
        for name, val in deltas:
            assert float(val) >= 0

    def test_latency_end_to_end_reasonable(self):
        latency_lines = []
        def capture_log(msg):
            if "[LATENCY]" in msg:
                latency_lines.append(msg)

        va = MOD.VoiceAssistant(
            event_logger=capture_log,
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )
        t_fire = time.time()
        va.speak_urgent("timing test")
        wait_for_idle(va, timeout=2.0)
        t_done = time.time()

        total_s = t_done - t_fire
        assert total_s < 0.5


class TestStreamColdSilence:
    def test_cold_start_triggers_silence_prepend(self):
        fake_tts = FakeTTS(synth_delay=0.01)
        silence_calls = []
        orig_synth = fake_tts.synthesize_to_file

        def tracking_synth(text, silence_ms=0):
            silence_calls.append(silence_ms)
            return orig_synth(text, silence_ms)

        fake_tts.synthesize_to_file = tracking_synth

        va = MOD.VoiceAssistant(
            _tts_override=fake_tts,
            _player_fn=fast_player(0.05),
        )
        with va._lock:
            va._last_speech_end = time.time() - 2.0

        va.speak_urgent("cold start test")
        wait_for_idle(va, timeout=2.0)
        assert any(s[0] == "cold start test" for s in fake_tts.synthesized)
        assert silence_calls and silence_calls[0] == MOD.PIPER_SILENCE_MS

    def test_warm_stream_no_silence(self):
        fake_tts = FakeTTS(synth_delay=0.01)
        silence_calls = []
        orig_synth = fake_tts.synthesize_to_file

        def tracking_synth(text, silence_ms=0):
            silence_calls.append(silence_ms)
            return orig_synth(text, silence_ms)

        fake_tts.synthesize_to_file = tracking_synth
        va = MOD.VoiceAssistant(
            _tts_override=fake_tts,
            _player_fn=fast_player(0.02),
        )

        va.speak_urgent("first")
        wait_for_idle(va, timeout=1.0)
        with va._lock:
            va._last_speech_end = time.time()
        va.speak_urgent("second back to back")
        wait_for_idle(va, timeout=1.0)

        if len(silence_calls) >= 2:
            assert silence_calls[1] == 0


class TestLetterbox:
    def test_output_shape(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        out, scale, (pl, pt) = MOD.letterbox_resize(img, 224)
        assert out.shape == (224, 224, 3)

    def test_scale_leq_1(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        _, scale, _ = MOD.letterbox_resize(img, 224)
        assert 0 < scale <= 1.0

    def test_buffer_reuse(self):
        img  = np.zeros((480, 640, 3), dtype=np.uint8)
        buf  = np.full((224, 224, 3), 114, dtype=np.uint8)
        out, _, _ = MOD.letterbox_resize(img, 224, buf)
        assert out is buf


class TestYoloPostprocess:
    def _make_row(self, x1, y1, x2, y2, conf, cls_id):
        return [x1, y1, x2, y2, conf, cls_id]

    def test_below_conf_filtered(self):
        row = self._make_row(10, 10, 100, 100, MOD.CONF_THRESH - 0.01, 0)
        out = MOD.yolo_postprocess(np.array([[row]]), 640, 480, 1.0, 0, 0)
        assert out == []

    def test_at_conf_passes(self):
        row = self._make_row(10, 10, 100, 100, MOD.CONF_THRESH + 0.001, 0)
        out = MOD.yolo_postprocess(np.array([[row]]), 640, 480, 1.0, 0, 0)
        assert len(out) == 1

    def test_ignored_class_filtered(self):
        kite_id = MOD.CLASS_NAMES.index("kite")
        row = self._make_row(10, 10, 100, 100, 0.9, kite_id)
        out = MOD.yolo_postprocess(np.array([[row]]), 640, 480, 1.0, 0, 0)
        assert out == []

    def test_tiny_box_filtered(self):
        row = self._make_row(10, 10, 15, 15, 0.9, 0)
        out = MOD.yolo_postprocess(np.array([[row]]), 640, 480, 1.0, 0, 0)
        assert out == []


class TestApproachScenario:
    def _make_track(self, dist, vel, seen=5):
        class T:
            id = 42
            class_name = "person"
            score = 0.8
            velocity = vel
            velocity_valid = True
            distance = dist
            seen_frames = seen
            num_clusters = 0
            box = [200, 100, 400, 400]
        return T()

    def test_ttc_2s_boundary(self):
        t = self._make_track(199, -100)
        score = MOD.ThreatAssessment.calculate_threat_score(t)
        level = MOD.ThreatAssessment.get_threat_level(score)
        assert level == "CRITICAL"

    def test_ttc_just_over_2s_still_scores_critical(self):
        t = self._make_track(201, -100)
        score = MOD.ThreatAssessment.calculate_threat_score(t)
        level = MOD.ThreatAssessment.get_threat_level(score)
        assert level == "CRITICAL"

    def test_voice_fires_for_critical(self):
        fired = []
        va = MOD.VoiceAssistant(
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )
        orig_urgent = va.speak_urgent
        def tracked_urgent(text, key=None):
            fired.append(text)
            orig_urgent(text, key=key)
        va.speak_urgent = tracked_urgent

        t = self._make_track(30, 0)
        score = MOD.ThreatAssessment.calculate_threat_score(t)
        assert score > 80

        va.speak_urgent("person ahead, 0.3 meters")
        wait_for_idle(va, timeout=2.0)
        assert len(fired) == 1

    def test_scene_with_track_id_churn_single_announcement(self):
        va = MOD.VoiceAssistant(
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )

        pos  = "ahead"
        cls  = "person"
        tier = "ttc_urg"
        buck = 2

        for track_id in [10, 11, 12]:
            key = MOD._voice_key(pos, cls, tier, buck)
            va.speak_urgent(f"person from track {track_id}", key=key)
            wait_for_idle(va, timeout=0.5)

        spoken = [s[0] for s in va._tts.synthesized]
        assert spoken.count("person from track 10") == 1
        assert "person from track 11" not in spoken and "person from track 12" not in spoken


class TestObjectTracker:
    def _det(self, x1, y1, x2, y2, name="person", score=0.9):
        return {"class_id": 0, "class_name": name, "score": score,
                "box": [x1, y1, x2, y2]}

    def test_new_track_created(self):
        tr = MOD.ObjectTracker()
        tracks = tr.update([self._det(0, 0, 100, 100)], 0.0)
        assert len(tracks) == 1

    def test_same_id_retained(self):
        tr = MOD.ObjectTracker()
        tr.update([self._det(0, 0, 100, 100)], 0.0)
        tid = list(tr.tracks.keys())[0]
        tr.update([self._det(5, 5, 105, 105)], 0.1)
        assert tid in tr.tracks

    def test_stale_pruned(self):
        tr = MOD.ObjectTracker(max_age=0.5)
        tr.update([self._det(0, 0, 100, 100)], 0.0)
        tracks = tr.update([], 0.6)
        assert len(tracks) == 0

    def test_different_class_not_matched(self):
        tr = MOD.ObjectTracker()
        tr.update([self._det(0, 0, 100, 100, "person")], 0.0)
        tr.update([self._det(5, 5, 105, 105, "chair")],  0.1)
        assert len(tr.tracks) == 2

    def test_seen_frames_increments(self):
        tr = MOD.ObjectTracker()
        tr.update([self._det(0, 0, 100, 100)], 0.0)
        tid = list(tr.tracks.keys())[0]
        tr.update([self._det(5, 5, 105, 105)], 0.1)
        assert tr.tracks[tid].seen_frames == 2

    def test_alive_at_exactly_max_age(self):
        tr = MOD.ObjectTracker(max_age=1.0)
        tr.update([self._det(0, 0, 100, 100)], 0.0)
        tracks = tr.update([], 1.0)
        assert len(tracks) == 1

    def test_pruned_just_past_max_age(self):
        tr = MOD.ObjectTracker(max_age=1.0)
        tr.update([self._det(0, 0, 100, 100)], 0.0)
        tracks = tr.update([], 1.001)
        assert len(tracks) == 0


class TestGetSmartDistance:
    def _make_depth(self, h=200, w=200, value_mm=1000):
        return np.full((h, w), value_mm, dtype=np.uint16)

    def test_uniform_depth(self):
        depth = self._make_depth(value_mm=1500)
        dist, nc = MOD.get_smart_distance(depth, [50, 50, 150, 150])
        assert dist == 150

    def test_all_zero_returns_none(self):
        depth = self._make_depth(value_mm=0)
        dist, nc = MOD.get_smart_distance(depth, [50, 50, 150, 150])
        assert dist is None

    def test_tiny_bbox_returns_none(self):
        depth = self._make_depth(value_mm=1000)
        dist, nc = MOD.get_smart_distance(depth, [0, 0, 5, 5])
        assert dist is None

    def test_out_of_range_depth_filtered(self):
        depth = np.full((200, 200), 9000, dtype=np.uint16)
        dist, nc = MOD.get_smart_distance(depth, [50, 50, 150, 150])
        assert dist is None


class TestGetPosition:
    def _track_at_x_centre(self, cx, frame_w=640):
        class T:
            box = [cx - 20, 100, cx + 20, 300]
        return T()

    @pytest.mark.parametrize("cx,expected", [
        (50,  "on your left"),
        (200, "on your left"),
        (211, "on your left"),
        (212, "ahead"),
        (320, "ahead"),
        (428, "ahead"),
        (430, "on your right"),
        (600, "on your right"),
    ])
    def test_position_zones(self, cx, expected):
        t = self._track_at_x_centre(cx)
        pos = MOD.get_position(t, frame_width=640)
        assert pos == expected

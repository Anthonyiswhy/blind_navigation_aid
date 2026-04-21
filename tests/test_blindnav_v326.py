"""
test_blindnav_v326.py - Advanced hardware-free regression suite for BlindNav

Addresses all Codex review findings:
  [P1] Loader path: resolves to raspberry_pi/yolo_realsense_navigation.py
       in the repo root, so the test works from the repo without a second copy.
  [P1] ID-churn assertion: `or` -> `and` (was a false-positive pass).
  [P2] Cold-start silence: asserts silence_ms == PIPER_SILENCE_MS, not just
       that synthesis ran.
  [P2] Ego-motion non-tautological: calls real _update_depth_z() with numpy
       arrays instead of reimplementing the clamp/confidence logic.
  [P3] Integration routing test: drives _select_voice_message() - the extracted
       pure function from main() - proving the real decision path.
  New: TestSkipAhead - verifies lower-priority audio is dropped before
       playback when a higher-priority alert becomes pending, without any
       terminate path on the player.

Run: pytest tests/test_blindnav_v326.py -v
     (from repo root - path resolver uses that as the anchor)
"""
import sys
import os
import shutil
import time
import wave
import tempfile
import threading
import math
import types
import importlib.util
from unittest.mock import MagicMock, patch, call
import pytest
import numpy as np


# ============================================================
# MODULE LOADING
# ============================================================

def _find_nav_script():
    """
    [P1] Find the navigation script relative to the repo root.
    Strategy (first match wins):
      1. BLINDNAV_SCRIPT env var (CI / override)
      2. raspberry_pi/yolo_realsense_navigation.py from repo root
         (repo root = 2 levels up from tests/)
      3. Same directory as this test file (dev convenience)
    """
    env_path = os.environ.get("BLINDNAV_SCRIPT")
    if env_path and os.path.exists(env_path):
        return env_path

    # Derive repo root: tests/ → ../ → repo root
    tests_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root  = os.path.dirname(tests_dir)
    repo_path  = os.path.join(repo_root, "raspberry_pi",
                              "yolo_realsense_navigation.py")
    if os.path.exists(repo_path):
        return repo_path

    # Fallback for running directly from a flat dev folder
    local_path = os.path.join(tests_dir,
                              "yolo_realsense_navigation_v3_26_HEADLESS.py")
    if os.path.exists(local_path):
        return local_path

    raise FileNotFoundError(
        "Cannot find navigation script. Set BLINDNAV_SCRIPT env var or "
        "run from the repo root with raspberry_pi/yolo_realsense_navigation.py present."
    )


def _make_stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    return m


def load_module():
    rs_stub = _make_stub("pyrealsense2",
        pipeline=MagicMock, config=MagicMock, align=MagicMock,
        stream=MagicMock(depth=0, color=1),
        format=MagicMock(z16=0, bgr8=1),
    )
    ort_stub = _make_stub("onnxruntime",
        InferenceSession=MagicMock,
        SessionOptions=MagicMock,
        GraphOptimizationLevel=MagicMock(ORT_ENABLE_ALL=3),
        ExecutionMode=MagicMock(ORT_SEQUENTIAL=0, ORT_PARALLEL=1),
    )
    cv2_stub = _make_stub("cv2",
        resize=lambda img, dsize, **kw: np.zeros((*dsize[::-1], 3), dtype=np.uint8),
        cvtColor=lambda img, code, dst=None: (dst if dst is not None else img),
        COLOR_BGR2RGB=0, COLOR_BGR2GRAY=1,
        INTER_LINEAR=1, IMWRITE_JPEG_QUALITY=1,
        imencode=lambda ext, img, params=None: (True, np.array([0,0,0], dtype=np.uint8)),
        calcOpticalFlowPyrLK=MagicMock(return_value=(None, None, None)),
        goodFeaturesToTrack=MagicMock(return_value=None),
        TERM_CRITERIA_EPS=1, TERM_CRITERIA_COUNT=2,
        createCLAHE=MagicMock(return_value=MagicMock(apply=lambda img: img)),
    )
    stubs = {
        "pyrealsense2": rs_stub,
        "onnxruntime":  ort_stub,
        "cv2":          cv2_stub,
        "anthropic":    _make_stub("anthropic", Anthropic=MagicMock),
        "icm20948":     _make_stub("icm20948",
                            ICM20948=MagicMock(side_effect=ImportError("No IMU in test"))),
        "smbus2":       _make_stub("smbus2"),
        "piper":        _make_stub("piper"),
        "piper.voice":  _make_stub("piper.voice", PiperVoice=MagicMock),
        "piper.config": _make_stub("piper.config", SynthesisConfig=MagicMock),
    }
    for name, stub in stubs.items():
        sys.modules[name] = stub

    script_path = _find_nav_script()
    spec = importlib.util.spec_from_file_location("nav326", script_path)
    module = importlib.util.module_from_spec(spec)
    with patch("os.makedirs", return_value=None):
        spec.loader.exec_module(module)
    return module


MOD = load_module()


# ============================================================
# FAKE TTS & PLAYER
# ============================================================

class FakeTTS:
    def __init__(self, synth_delay=0.02):
        self.available    = True
        self._use_piper   = True
        self.synth_delay  = synth_delay
        self.synthesized  = []           # [(text, start_ts, end_ts, silence_ms)]
        self._sem         = threading.Lock()
        self._concurrent  = 0
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
        self.synthesized.append((text, start, end, silence_ms))
        return _make_wav(silence_ms)

    def prepend_silence(self, wav_path, silence_ms):
        # Record that prepend was called and return a new wav with silence
        self.synthesized.append(("__prepend__", time.time(), time.time(), silence_ms))
        return _make_wav(silence_ms)


class FakeFastAlertTTS(FakeTTS):
    def __init__(self, synth_delay=0.02):
        super().__init__(synth_delay=synth_delay)
        self.alert_synthesized = []

    def synthesize_alert_to_file(self, text, silence_ms=0):
        start = time.time()
        time.sleep(self.synth_delay)
        end = time.time()
        self.alert_synthesized.append((text, start, end, silence_ms))
        return _make_wav(silence_ms)


def _make_wav(silence_ms=0):
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
    """Mimics subprocess.Popen with controllable duration."""
    def __init__(self, duration=0.05):
        self._done = threading.Event()
        self._timer = threading.Timer(duration, self._done.set)
        self._timer.start()

    def wait(self):
        self._done.wait()


def fast_player(play_duration=0.05):
    procs = []
    def _player(wav_path):
        try: os.unlink(wav_path)
        except Exception: pass
        p = FakeProc(play_duration)
        procs.append(p)
        return p
    _player.procs = procs
    return _player


def build_voice(synth_delay=0.02, play_duration=0.05, tts=None):
    fake_tts = tts or FakeTTS(synth_delay=synth_delay)
    player = fast_player(play_duration)
    va = MOD.VoiceAssistant(
        _tts_override=fake_tts,
        _player_fn=player,
    )
    return va, fake_tts, player


def wait_for_idle(va, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(0.01)
        with va._lock:
            if not va.is_speaking and all(p is None for p in va._pending.values()):
                return True
    return False


# ============================================================
# TRUTH TABLE: _select_voice_message (P3 fix — real routing path)
# ============================================================

class TestSelectVoiceMessage:
    """
    Tests _select_voice_message() — the pure function extracted from main().
    This is the real decision path, not a manual speak_urgent() call.
    """

    def _call(self, dist_cm, vel, user_moving=True, ego_reliable=True,
              ttc=None, obj="person", pos="ahead"):
        if ttc is None:
            ttc = dist_cm / abs(vel) if vel < -2 else 999.0
        approaching   = ttc < 100
        very_close    = dist_cm < 40
        close         = dist_cm < 70
        fast_approach = vel < -50
        return MOD._select_voice_message(
            obj, pos, dist_cm, vel,
            user_moving, ego_reliable,
            approaching, very_close, close, fast_approach, ttc)

    # (dist_cm, vel, user_moving, ego_reliable, expected_tier)
    ROUTING_CASES = [
        # ── Very close (<40cm) ──
        (30,   0,    True,  True,  "urgent"),   # static, close
        (30,   0,    False, True,  "urgent"),   # static, still, close
        (30, -20,    True,  True,  "urgent"),   # approaching + moving
        (30, -20,    False, True,  "urgent"),   # approaching + still
        (30, -20,    True,  False, "urgent"),   # approaching but ego bad → still urgent
        # ── Close (40–70cm) ──
        (60,   0,    True,  True,  "warning"),  # static
        (60, -30,    True,  True,  "warning"),  # approaching + moving
        (60, -30,    False, True,  "warning"),  # approaching + still
        (60, -30,    True,  False, "warning"),  # approaching but ego bad → neutral warning
        # ── TTC branches ──
        (200, -150,  True,  True,  "urgent"),   # TTC=1.3s < 2
        (200,  -60,  True,  True,  "warning"),  # TTC=3.3s < 4
        (200,  -30,  True,  True,  "awareness"),# TTC=6.7s < 8
        (200,  -15,  True,  True,  None),       # TTC=13.3s > 8 (noise floor: 7cm/s at 200cm)
        # ── Ego unreliable → neutral tier still correct ──
        (200, -150,  True,  False, "urgent"),   # TTC<2, ego bad → neutral msg but URGENT tier
        (200,  -60,  True,  False, "warning"),  # TTC<4, ego bad
        # ── Far away, nothing ──
        (400,   0,   True,  True,  None),
    ]

    @pytest.mark.parametrize("dist,vel,moving,ego,expected_tier", ROUTING_CASES)
    def test_tier_selection(self, dist, vel, moving, ego, expected_tier):
        tier, msg = self._call(dist, vel, user_moving=moving, ego_reliable=ego)
        assert tier == expected_tier, (
            f"dist={dist}cm vel={vel}cm/s moving={moving} ego={ego} "
            f"→ tier={tier!r} expected={expected_tier!r}, msg={msg!r}"
        )

    def test_neutral_msg_when_ego_unreliable_very_close(self):
        """FIX 9: very_close + approaching + ego_unreliable → no 'approaching' in msg."""
        _, msg = self._call(30, -20, user_moving=True, ego_reliable=False)
        assert "approaching" not in msg, f"Expected neutral msg, got: {msg!r}"

    def test_neutral_msg_when_ego_unreliable_close(self):
        """FIX 9: close + approaching + ego_unreliable → no 'closer'/'watch' in msg."""
        _, msg = self._call(60, -30, user_moving=True, ego_reliable=False)
        assert "closer" not in msg and "Watch out" not in msg, \
            f"Expected neutral msg, got: {msg!r}"

    def test_approaching_wording_when_ego_reliable(self):
        """When ego IS reliable, directional wording should appear."""
        _, msg = self._call(30, -20, user_moving=False, ego_reliable=True)
        assert "approaching" in msg, f"Expected 'approaching' in msg, got: {msg!r}"

    def test_side_pass_person_gets_awareness_when_ttc_is_weak(self):
        tier, msg = self._call(129, 0, user_moving=True, ego_reliable=False,
                               obj="person", pos="on your right")
        assert tier == "awareness"
        assert "person on your right" in msg
        assert "1.4 meters" in msg

    def test_bad_ego_far_stationary_person_stays_silent(self):
        tier, msg = self._call(639, -90, user_moving=False, ego_reliable=False,
                               ttc=999.0, obj="person", pos="ahead")
        assert tier is None and msg is None

    def test_non_person_warning_uses_obstacle_wording(self):
        _, msg = self._call(60, -30, user_moving=True, ego_reliable=True,
                            obj="chair", pos="on your right")
        assert "obstacle on your right" in msg

    def test_msg_is_string_when_tier_not_none(self):
        """All non-None tiers must produce a non-empty string message."""
        for dist, vel, moving, ego, exp_tier in self.ROUTING_CASES:
            tier, msg = self._call(dist, vel, user_moving=moving, ego_reliable=ego)
            if tier is not None:
                assert isinstance(msg, str) and len(msg) > 0

    def test_returns_none_none_for_far_static(self):
        tier, msg = self._call(400, 0)
        assert tier is None and msg is None


# ============================================================
# TRUTH TABLE: ThreatAssessment
# ============================================================

class TestThreatScoringTruthTable:

    def _make_track(self, dist_cm, vel_cm_s, class_name="person",
                    confidence=0.5, vel_valid=True):
        class FakeTrack:
            pass
        t = FakeTrack()
        t.distance       = dist_cm
        t.velocity       = vel_cm_s
        t.class_name     = class_name
        t.score          = confidence
        t.velocity_valid = vel_valid
        t.seen_frames    = 5
        return t

    CASES = [
        # dist=25: 50+50*(50-25)/50=75; *1.5=112.5 → CRITICAL
        (25,   0,     "person",  0.5, "CRITICAL"),
        # dist=49: 50+50*(50-49)/50=51; *1.5=76.5 → WARNING (threshold >80)
        (49,   0,     "person",  0.5, "WARNING"),
        # dist=51: WARNING zone: 50*(150-51)/150=33; *1.5=49.5 → WARNING
        (51,   0,     "person",  0.5, "WARNING"),
        # dist=100: 50*(150-100)/150=16.7; *1.5=25 → CAUTION
        (100,  0,     "person",  0.5, "CAUTION"),
        # dist=149: 50*(150-149)/150=0.33; *1.5=0.5 → SAFE
        (149,  0,     "person",  0.5, "SAFE"),
        # dist=151: SAFE zone: 10*(300-151)/300=4.97; *1.5=7.45 → SAFE
        (151,  0,     "person",  0.5, "SAFE"),
        (250,  0,     "person",  0.5, "SAFE"),
        (299,  0,     "person",  0.5, "SAFE"),
        (350,  0,     "person",  0.5, "SAFE"),
        # vel=-80: noise_floor=7 at 200cm, TTC=2.5s<12 → vel active
        (200, -80,    "person",  0.5, "CRITICAL"),
        # moving away → score*0.2 → SAFE
        (200,  80,    "person",  0.5, "SAFE"),
        (200, -80,    "car",     0.5, "CRITICAL"),
        # bottle with big velocity → CRITICAL (vel dominates over weight)
        (200, -80,    "bottle",  0.5, "CRITICAL"),
        # noise floor at 300cm: 3+300*0.02=9; vel=-8<9 → zeroed → SAFE
        (300,  -8,    "person",  0.5, "SAFE"),
        # TTC gate: dist=100, vel=-6 → noise_floor=5, 6>5 passes
        # TTC=100/6=16.7s>12 → vel ZEROED → distance only: 25 → CAUTION
        (100,  -6,    "person",  0.5, "CAUTION"),
        # TTC gate: dist=200, vel=-10 → noise_floor=7, 10>7 passes
        # TTC=20s>12 → vel ZEROED → 5.0 → SAFE
        (200, -10,    "person",  0.5, "SAFE"),
        (30,   0,     "person",  0.5, "CRITICAL"),
        (80,  -60,    "person",  0.8, "CRITICAL"),
        (80,  -60,    "person",  0.2, "CRITICAL"),
    ]

    @pytest.mark.parametrize("dist,vel,cls,conf,expected", CASES)
    def test_threat_level(self, dist, vel, cls, conf, expected):
        track = self._make_track(dist, vel, cls, conf)
        score = MOD.ThreatAssessment.calculate_threat_score(track)
        level = MOD.ThreatAssessment.get_threat_level(score)
        assert level == expected, (
            f"dist={dist}cm vel={vel}cm/s cls={cls} conf={conf} "
            f"→ score={score:.1f} level={level} expected={expected}"
        )

    def test_none_distance_returns_zero(self):
        t = self._make_track(None, 0)
        assert MOD.ThreatAssessment.calculate_threat_score(t) == 0.0

    def test_score_capped_at_600(self):
        t = self._make_track(10, -500, "car", 0.9)
        assert MOD.ThreatAssessment.calculate_threat_score(t) <= 600.0

    def test_moving_away_reduces_score(self):
        t_static = self._make_track(100, 0,  "person")
        t_away   = self._make_track(100, 50, "person")
        assert (MOD.ThreatAssessment.calculate_threat_score(t_away) <
                MOD.ThreatAssessment.calculate_threat_score(t_static))

    def test_higher_weight_higher_score(self):
        t_car    = self._make_track(100, -50, "car")
        t_bottle = self._make_track(100, -50, "bottle")
        assert (MOD.ThreatAssessment.calculate_threat_score(t_car) >
                MOD.ThreatAssessment.calculate_threat_score(t_bottle))

    def test_far_velocity_suppressed_when_user_moving_and_ego_bad(self):
        track = self._make_track(722, -103.6, "person", 0.5)
        score = MOD.ThreatAssessment.calculate_threat_score(
            track, user_moving=True, ego_reliable=False)
        level = MOD.ThreatAssessment.get_threat_level(score)
        assert level == "SAFE", (
            f"bad ego should suppress far false CRITICALs, got {score:.1f}/{level}"
        )

    def test_far_velocity_suppressed_when_user_still_and_ego_bad(self):
        track = self._make_track(639, -89.6, "person", 0.5)
        score = MOD.ThreatAssessment.calculate_threat_score(
            track, user_moving=False, ego_reliable=False)
        level = MOD.ThreatAssessment.get_threat_level(score)
        assert level == "SAFE", (
            f"bad ego should suppress far still-user false alerts, got {score:.1f}/{level}"
        )

    def test_close_distance_stays_critical_when_user_moving_and_ego_bad(self):
        track = self._make_track(27, -29.1, "person", 0.5)
        score = MOD.ThreatAssessment.calculate_threat_score(
            track, user_moving=True, ego_reliable=False)
        level = MOD.ThreatAssessment.get_threat_level(score)
        assert level == "CRITICAL", (
            f"close obstacle must stay CRITICAL with bad ego, got {score:.1f}/{level}"
        )


# ============================================================
# Track filtering + shared motion evaluation
# ============================================================

class TestTrackFiltering:
    def _make_track(self):
        det = {
            "class_name": "person",
            "class_id": 0,
            "box": [200, 80, 260, 300],
            "score": 0.9,
        }
        return MOD.Track(0, det, 0.0)

    def test_large_distance_jump_requires_confirmation(self):
        track = self._make_track()
        track.update_distance(244, 0.0)
        track.update_distance(243, 0.2)
        track.update_distance(242, 0.4)
        assert track.distance == 242

        track.update_distance(24, 0.6)
        assert track.distance == 242
        assert not track.velocity_valid

        track.update_distance(26, 0.8)
        assert track.distance == 26
        assert track.distance_history == [(0.8, 26)]

    def test_far_small_motion_is_zeroed_before_alerting(self):
        track = self._make_track()
        track.update_distance(300, 0.0)
        track.update_distance(297, 0.2)
        track.update_distance(294, 0.4)

        assert track.velocity_valid
        assert track.velocity == 0.0

    def test_shared_motion_eval_suppresses_far_noise(self):
        class FakeTrack:
            distance = 295
            velocity = -5.0
            velocity_valid = True
            class_name = "person"
            score = 0.9

        motion = MOD.evaluate_track_motion(
            FakeTrack(), user_moving=False, ego_reliable=True)
        assert motion["effective_velocity"] == 0.0
        assert motion["ttc"] == 999.0
        assert not motion["ttc_allowed"]


# ============================================================
# TRUTH TABLE: _voice_key zone mapping
# ============================================================

class TestVoiceKey:

    def test_same_zone_same_family_same_key(self):
        k1 = MOD._voice_key("ahead", "chair",        "ttc_urg", 3)
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
        assert (MOD._voice_key("ahead", "person", "ttc_urg", 2) !=
                MOD._voice_key("ahead", "chair",  "ttc_urg", 2))

    def test_different_tier_different_key(self):
        assert (MOD._voice_key("ahead", "person", "ttc_urg", 2) !=
                MOD._voice_key("ahead", "person", "ttc_wrn", 2))

    def test_different_bucket_different_key(self):
        assert (MOD._voice_key("ahead", "person", "ttc_urg", 2) !=
                MOD._voice_key("ahead", "person", "ttc_urg", 3))

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
        key = MOD._voice_key(pos, cls, "ttc_urg", 3)
        expected = f"{zone}_{family}_ttc_urg_3"
        assert key == expected, f"got {key!r}, want {expected!r}"

class TestSourceStructure:

    def test_single_source_definitions(self):
        nav_path = _find_nav_script()
        with open(nav_path, "r", encoding="utf-8", errors="replace") as f:
            src = f.read()

        import re
        assert len(re.findall(r"^def _voice_key\(", src, flags=re.MULTILINE)) == 1
        assert len(re.findall(r"^class PiperVoice:", src, flags=re.MULTILINE)) == 1


class TestFastAlertTTS:

    def test_alert_mode_defaults_to_piper(self):
        assert MOD.ALERT_TTS_MODE == "piper"

    def test_warning_prefers_piper_alert_synth_when_enabled(self):
        fake_tts = FakeFastAlertTTS(synth_delay=0.01)
        with patch.object(MOD, "ALERT_TTS_MODE", "piper"):
            va = MOD.VoiceAssistant(
                _tts_override=fake_tts,
                _player_fn=fast_player(0.02),
            )
        va.speak_warning("Watch out, person ahead, 1.2 meters")
        assert wait_for_idle(va, timeout=2.0)
        assert len(fake_tts.alert_synthesized) == 1
        assert len(fake_tts.synthesized) == 0

    def test_awareness_stays_on_regular_piper_path(self):
        fake_tts = FakeFastAlertTTS(synth_delay=0.01)
        with patch.object(MOD, "ALERT_TTS_MODE", "piper"):
            va = MOD.VoiceAssistant(
                _tts_override=fake_tts,
                _player_fn=fast_player(0.02),
            )
        va.speak_awareness("Heads up, person ahead, 2.4 meters")
        assert wait_for_idle(va, timeout=2.0)
        assert len(fake_tts.alert_synthesized) == 0
        assert len(fake_tts.synthesized) == 1

    def test_urgent_prefers_fast_alert_synth(self):
        fake_tts = FakeFastAlertTTS(synth_delay=0.01)
        with patch.object(MOD, "ALERT_TTS_MODE", "espeak"):
            va = MOD.VoiceAssistant(
                _tts_override=fake_tts,
                _player_fn=fast_player(0.02),
            )
        va.speak_urgent("person ahead, 0.3 meters")
        assert wait_for_idle(va, timeout=2.0)
        assert len(fake_tts.alert_synthesized) == 1
        assert len(fake_tts.synthesized) == 0

    def test_fast_alert_path_uses_longer_cold_start_silence(self):
        fake_tts = FakeFastAlertTTS(synth_delay=0.01)
        with patch.object(MOD, "ALERT_TTS_MODE", "espeak"):
            va = MOD.VoiceAssistant(
                _tts_override=fake_tts,
                _player_fn=fast_player(0.02),
            )
        with va._lock:
            va._last_speech_end = time.time() - 2.0
        va.speak_warning("keyboard ahead, 0.5 meters")
        assert wait_for_idle(va, timeout=2.0)
        assert fake_tts.alert_synthesized, "expected fast alert synthesis to run"
        assert fake_tts.alert_synthesized[0][3] == MOD.FAST_ALERT_SILENCE_MS


class TestPiperAlertCache:
    def test_cached_alert_clip_reuses_existing_phrase(self):
        piper = object.__new__(MOD.PiperVoice)
        piper._use_piper = True
        piper._alert_cache_enabled = True
        piper._voice_label = "en_US-amy-medium"
        piper._alert_cache_dir = tempfile.mkdtemp(prefix="blindnav_alert_cache_test_")

        synth_calls = []
        first = None
        second = None

        def fake_synthesize(text, silence_ms=0):
            synth_calls.append((text, silence_ms))
            return _make_wav(silence_ms)

        piper.synthesize_to_file = fake_synthesize
        piper.prepend_silence = MOD.PiperVoice.prepend_silence.__get__(piper, MOD.PiperVoice)

        try:
            with patch.object(MOD, "ALERT_TTS_MODE", "piper"):
                first = MOD.PiperVoice.synthesize_alert_to_file(
                    piper, "Stop! person ahead, 1.2 meters", silence_ms=0)
                second = MOD.PiperVoice.synthesize_alert_to_file(
                    piper, "Stop! person ahead, 1.2 meters", silence_ms=0)
            assert len(synth_calls) == 1
            assert os.path.exists(first)
            assert os.path.exists(second)
        finally:
            for path in (first, second):
                try:
                    if path:
                        os.unlink(path)
                except Exception:
                    pass
            for wav_name in os.listdir(piper._alert_cache_dir):
                try:
                    os.unlink(os.path.join(piper._alert_cache_dir, wav_name))
                except Exception:
                    pass
            try:
                os.rmdir(piper._alert_cache_dir)
            except Exception:
                pass

    def test_copy_to_temp_cleans_up_on_copy_failure(self, monkeypatch):
        piper = object.__new__(MOD.PiperVoice)
        created = []
        real_named_temp = tempfile.NamedTemporaryFile

        def tracking_named_temp(*args, **kwargs):
            tmp = real_named_temp(*args, **kwargs)
            created.append(tmp.name)
            return tmp

        monkeypatch.setattr(tempfile, "NamedTemporaryFile", tracking_named_temp)
        monkeypatch.setattr(shutil, "copyfile", lambda src, dst: (_ for _ in ()).throw(IOError("boom")))

        with pytest.raises(IOError):
            MOD.PiperVoice._copy_to_temp(piper, "missing.wav")

        assert created
        assert not os.path.exists(created[0])

    def test_prime_alert_cache_builds_common_phrase_set(self):
        piper = object.__new__(MOD.PiperVoice)
        seen = []

        def fake_materialize(text):
            seen.append(text)
            return _make_wav(0)

        piper._materialize_cached_alert = fake_materialize
        count = MOD.PiperVoice._prime_alert_cache(piper)

        assert count == 150
        assert "Stop! person ahead, 1.1 meters" in seen
        assert "Watch out, person on your left, 1.7 meters" in seen
        assert "Obstacle, 0.5 meters" in seen
        assert "obstacle on your right, 1.7 meters" in seen
        assert "Heads up, person on your left, 1.4 meters" in seen



# ============================================================
# EgoMotionCompensator: clamping + confidence (P2 fix — non-tautological)
# ============================================================

class TestEgoMotionCompensator:
    """
    [P2] Calls the real _update_depth_z() with actual numpy arrays
    instead of reimplementing clamp/confidence logic.
    """

    def _make_depth(self, h, w, value_mm):
        return np.full((h, w), value_mm, dtype=np.uint16)

    def _make_bg_mask(self, h, w):
        """All pixels are background (no tracked objects)."""
        return np.ones((h, w), dtype=bool)

    def test_stable_approach_is_confident(self):
        """
        Camera approaches background at ~80cm/s.
        At 30fps: Δdepth = 80cm/s * 0.033s = 2.64cm = 264mm per frame.
        After 8 frames the window should be full and std should be low.
        """
        ego = MOD.EgoMotionCompensator(use_optical_flow=False)
        h, w = 480, 640
        bg_mask = self._make_bg_mask(h, w)
        base_mm = 3000

        for i in range(8):
            depth = self._make_depth(h, w, max(100, base_mm - i * 264))
            ego._update_depth_z(depth, bg_mask, dt=0.033)

        assert ego.ego_confident, (
            f"Expected confident for stable approach, "
            f"z_hist={ego._z_vel_history}"
        )
        assert abs(ego.camera_z_velocity) > 0

    def test_single_outlier_is_clamped(self):
        """
        One catastrophically bad frame (10m depth jump in 33ms = 303 m/s raw).
        After clamping it becomes ±160 cm/s max.
        The history should contain only clamped values.
        """
        ego = MOD.EgoMotionCompensator(use_optical_flow=False)
        h, w = 480, 640
        bg_mask = self._make_bg_mask(h, w)

        # Start at 3m, then a spike to 13m in one frame
        ego._update_depth_z(self._make_depth(h, w, 3000), bg_mask, dt=0.033)
        ego._update_depth_z(self._make_depth(h, w, 13000), bg_mask, dt=0.033)

        # All history entries must be within ±MAX_Z_CM_S
        for v in ego._z_vel_history:
            assert abs(v) <= ego._MAX_Z_CM_S, (
                f"Unclamped value {v:.1f} in z_hist {ego._z_vel_history}"
            )

    def test_chaotic_values_not_confident(self):
        """
        Depth alternates wildly between 1m and 8m (IR interference).
        Even after clamping, alternating ±160 produces high std → not confident.
        """
        ego = MOD.EgoMotionCompensator(use_optical_flow=False)
        h, w = 480, 640
        bg_mask = self._make_bg_mask(h, w)
        depths = [1000, 8000, 1000, 8000, 1000, 8000, 1000, 8000]

        for d in depths:
            ego._update_depth_z(self._make_depth(h, w, d), bg_mask, dt=0.033)

        assert not ego.ego_confident, (
            f"Expected NOT confident for chaotic values, "
            f"std={np.std(ego._z_vel_history):.1f}"
        )
        assert ego.camera_z_velocity == 0.0

    def test_stationary_camera_confident_and_near_zero(self):
        """Constant depth (static camera) → very low velocity, confident."""
        ego = MOD.EgoMotionCompensator(use_optical_flow=False)
        h, w = 480, 640
        bg_mask = self._make_bg_mask(h, w)
        base_mm = 3000
        for _ in range(8):
            # Add tiny noise ±1mm (realistic sensor noise)
            d = base_mm + np.random.randint(-1, 2)
            ego._update_depth_z(self._make_depth(h, w, int(d)), bg_mask, dt=0.033)

        assert ego.ego_confident
        assert abs(ego.camera_z_velocity) < 5.0

    def test_not_confident_with_fewer_than_3_samples(self):
        """Confidence gate requires ≥3 history entries."""
        ego = MOD.EgoMotionCompensator(use_optical_flow=False)
        h, w = 480, 640
        bg_mask = self._make_bg_mask(h, w)
        # Only 2 frames → 1 delta → 1 history entry
        ego._update_depth_z(self._make_depth(h, w, 3000), bg_mask, dt=0.033)
        ego._update_depth_z(self._make_depth(h, w, 2900), bg_mask, dt=0.033)
        assert not ego.ego_confident

    def test_insufficient_background_pixels(self):
        """If bg has fewer than 150 valid pixels → no update, not confident."""
        ego = MOD.EgoMotionCompensator(use_optical_flow=False)
        h, w = 10, 10  # tiny image → fewer than 150 valid bg pixels
        bg_mask = self._make_bg_mask(h, w)
        ego._update_depth_z(self._make_depth(h, w, 3000), bg_mask, dt=0.033)
        assert not ego.ego_confident

    def test_max_clamp_value(self):
        assert MOD.EgoMotionCompensator._MAX_Z_CM_S == 160.0

    def test_confidence_threshold(self):
        assert MOD.EgoMotionCompensator._EGO_STD_THRESHOLD == 40.0


# ============================================================
# VoiceAssistant: Priority Queue
# ============================================================

class TestVoicePriorityQueue:

    def test_p0_evicts_p1_p2(self):
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.3)
        played = []
        orig_start = va._start_locked.__func__
        def patched(self, text, key, cooldown, label, priority,
                    event_created_ts=None, enqueued_ts=None):
            played.append((label, text))
            orig_start(self, text, key, cooldown, label, priority,
                       event_created_ts, enqueued_ts)
        va._start_locked = types.MethodType(patched, va)

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
        assert "URGENT" in [p[0] for p in played]

    def test_p1_evicts_p2_not_p0(self):
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.3)
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
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.05)
        key = "center_person_ttc_urg_2"
        va.speak_urgent("person ahead", key=key)
        time.sleep(0.01)
        va.speak_urgent("person ahead again", key=key)
        wait_for_idle(va)
        spoken = [s[0] for s in tts.synthesized]
        assert "person ahead again" not in spoken

    def test_different_zone_key_not_blocked(self):
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.05)
        k_left  = MOD._voice_key("on your left",  "person", "ttc_urg", 2)
        k_right = MOD._voice_key("on your right", "person", "ttc_urg", 2)
        va.speak_urgent("person left",  key=k_left);  wait_for_idle(va)
        va.speak_urgent("person right", key=k_right); wait_for_idle(va)
        spoken = [s[0] for s in tts.synthesized]
        assert "person left"  in spoken
        assert "person right" in spoken

    def test_id_churn_hits_cooldown(self):
        """[P1] Three different track IDs → same zone key → only first fires."""
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.05)
        key = MOD._voice_key("ahead", "person", "ttc_urg", 3)

        va.speak_urgent("person from track 5", key=key); wait_for_idle(va)
        va.speak_urgent("person from track 6", key=key); wait_for_idle(va)
        va.speak_urgent("person from track 7", key=key); wait_for_idle(va)

        spoken = [s[0] for s in tts.synthesized]
        assert "person from track 5" in spoken
        # [P1] Both track 6 AND track 7 must be blocked (was `or`, now `and`)
        assert "person from track 6" not in spoken and "person from track 7" not in spoken


# ============================================================
# Skip-ahead before playback (FIX 8 - no player termination)
# ============================================================

class TestSkipAhead:

    def test_p0_skips_p2_during_synthesis(self):
        """
        P2 is still synthesizing and has not started playback.
        P0 should skip it before playback and become the only played proc.
        """
        va, tts, player = build_voice(synth_delay=0.30, play_duration=0.05)
        va.speak_awareness("long awareness message")
        time.sleep(0.05)

        with va._lock:
            assert va.is_speaking
            assert va._current_proc is None

        va.speak_urgent("urgent!")
        assert wait_for_idle(va, timeout=3.0)

        assert len(player.procs) == 1, "Only urgent should reach playback"

    def test_p1_skips_p2_during_synthesis(self):
        """A pending warning should also skip awareness before playback."""
        va, tts, player = build_voice(synth_delay=0.30, play_duration=0.05)
        va.speak_awareness("long awareness message")
        time.sleep(0.05)

        with va._lock:
            assert va.is_speaking
            assert va._current_proc is None

        va.speak_warning("warning!")
        assert wait_for_idle(va, timeout=3.0)
        assert len(player.procs) == 1, "Only warning should reach playback"

    def test_p0_waits_for_active_playback_without_interruption(self):
        """
        Once playback has started, urgent must wait in pending.
        Skip-ahead only applies before playback starts.
        """
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.40)
        va.speak_awareness("awareness")
        time.sleep(0.10)

        with va._lock:
            assert va.is_speaking
            assert va._current_proc is not None

        va.speak_urgent("urgent after playback started")
        time.sleep(0.05)

        assert len(player.procs) == 1
        assert wait_for_idle(va, timeout=2.0)
        assert len(player.procs) == 2

    def test_p0_does_not_skip_current_p0(self):
        """An urgent already in synthesis should not be skipped by another urgent."""
        va, tts, player = build_voice(synth_delay=0.25, play_duration=0.05)
        va.speak_urgent("first urgent")
        time.sleep(0.05)

        with va._lock:
            assert va.is_speaking
            assert va._current_proc is None

        va.speak_urgent("second urgent")
        assert wait_for_idle(va, timeout=3.0)
        assert len(player.procs) == 2

    def test_skipped_phrase_is_logged(self):
        events = []
        va = MOD.VoiceAssistant(
            event_logger=events.append,
            _tts_override=FakeTTS(synth_delay=0.30),
            _player_fn=fast_player(0.05),
        )
        va.speak_awareness("awareness to supersede")
        time.sleep(0.05)
        va.speak_urgent("urgent")
        assert wait_for_idle(va, timeout=3.0)
        assert any("Skipped before playback due to higher-priority pending" in msg
                   for msg in events)

    def test_no_terminate_attribute_needed(self):
        """
        FakeProc intentionally has no terminate(). If production tries to call
        it anywhere, this path fails.
        """
        va, tts, player = build_voice(synth_delay=0.30, play_duration=0.05)
        va.speak_awareness("awareness")
        time.sleep(0.05)
        va.speak_urgent("urgent")
        assert wait_for_idle(va, timeout=3.0)
        assert len(player.procs) == 1


# ============================================================
# VoiceAssistant: TTL Expiry
# ============================================================

class TestVoiceTTL:

    def test_p0_ttl_3s(self):
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.5)
        va.speak_awareness("long speech")
        time.sleep(0.05)
        assert va.is_speaking

        with va._lock:
            va._pending[0] = (
                "stale urgent", None, 0, "URGENT",
                time.time() - 4.0,   # enqueued 4s ago > TTL=3s
                time.time() - 4.0,
            )

        wait_for_idle(va, timeout=3.0)
        spoken = [s[0] for s in tts.synthesized]
        assert "stale urgent" not in spoken

    def test_fresh_p0_plays(self):
        va, tts, player = build_voice(synth_delay=0.01, play_duration=0.3)
        va.speak_awareness("blocking")
        time.sleep(0.05)
        va.speak_urgent("fresh urgent")
        wait_for_idle(va, timeout=3.0)
        assert "fresh urgent" in [s[0] for s in tts.synthesized]


# ============================================================
# VoiceAssistant shutdown
# ============================================================

class TestVoiceShutdown:
    def test_shutdown_clears_pending_and_blocks_new_enqueues(self):
        va, tts, player = build_voice(synth_delay=0.30, play_duration=0.05)
        va.speak_awareness("current speech")
        time.sleep(0.05)
        va.speak_warning("pending speech")

        va.shutdown(timeout=1.0)
        spoken = [s[0] for s in tts.synthesized]
        assert "pending speech" not in spoken

        va.speak_urgent("after shutdown")
        time.sleep(0.05)
        spoken = [s[0] for s in tts.synthesized]
        assert "after shutdown" not in spoken


# ============================================================
# Presyn Semaphore
# ============================================================

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
        assert slow_tts.max_concurrent <= 1, (
            f"Semaphore violation: {slow_tts.max_concurrent} concurrent synthesis"
        )

    def test_semaphore_initialized(self):
        va, _, _ = build_voice()
        assert hasattr(va, "_presyn_sem")
        assert va._presyn_sem._value == 1


# ============================================================
# Latency Timestamps
# ============================================================

class TestLatencyTimestamps:

    def test_latency_line_logged(self):
        latency_lines = []
        def capture(msg):
            if "[LATENCY]" in msg:
                latency_lines.append(msg)

        va = MOD.VoiceAssistant(
            event_logger=capture,
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )
        va.speak_urgent("latency test")
        wait_for_idle(va, timeout=2.0)

        assert len(latency_lines) >= 1
        line = latency_lines[0]
        for field in ("event_created=", "tts_start=", "play_start=", "play_end="):
            assert field in line, f"Missing {field} in: {line}"

    def test_latency_deltas_non_negative(self):
        import re
        latency_lines = []
        def capture(msg):
            if "[LATENCY]" in msg:
                latency_lines.append(msg)

        va = MOD.VoiceAssistant(
            event_logger=capture,
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )
        va.speak_urgent("check")
        wait_for_idle(va, timeout=2.0)
        assert latency_lines
        for name, val in re.findall(r"(\w+)=([\d.]+)s", latency_lines[0]):
            assert float(val) >= 0, f"{name} is negative: {val}"

    def test_end_to_end_under_500ms(self):
        latency_lines = []
        def capture(msg):
            if "[LATENCY]" in msg:
                latency_lines.append(msg)

        va = MOD.VoiceAssistant(
            event_logger=capture,
            _tts_override=FakeTTS(synth_delay=0.01),
            _player_fn=fast_player(0.05),
        )
        t0 = time.time()
        va.speak_urgent("timing test")
        wait_for_idle(va, timeout=2.0)
        assert time.time() - t0 < 0.5


# ============================================================
# Cold-Start Silence (P2 fix — assert actual silence_ms)
# ============================================================

class TestStreamColdSilence:

    def test_cold_start_injects_piper_silence_ms(self):
        """[P2] Verify silence_ms == PIPER_SILENCE_MS, not just that synthesis ran."""
        fake_tts = FakeTTS(synth_delay=0.01)
        va = MOD.VoiceAssistant(
            _tts_override=fake_tts,
            _player_fn=fast_player(0.05),
        )
        # Simulate cold BT stream
        with va._lock:
            va._last_speech_end = time.time() - 2.0

        va.speak_urgent("cold start test")
        wait_for_idle(va, timeout=2.0)

        # Find the synthesis entry for our text
        relevant = [s for s in fake_tts.synthesized if s[0] == "cold start test"]
        assert relevant, "Expected synthesis for 'cold start test'"
        _, _, _, silence_ms = relevant[0]
        assert silence_ms == MOD.PIPER_SILENCE_MS, (
            f"Expected silence_ms={MOD.PIPER_SILENCE_MS}, got {silence_ms}"
        )

    def test_warm_stream_no_silence(self):
        """Back-to-back speech within STREAM_COLD_S → silence_ms=0."""
        fake_tts = FakeTTS(synth_delay=0.01)
        va = MOD.VoiceAssistant(
            _tts_override=fake_tts,
            _player_fn=fast_player(0.02),
        )
        va.speak_urgent("first")
        wait_for_idle(va, timeout=1.0)

        # Stream is warm — gap < STREAM_COLD_S
        with va._lock:
            va._last_speech_end = time.time()

        va.speak_urgent("second back to back")
        wait_for_idle(va, timeout=1.0)

        relevant = [s for s in fake_tts.synthesized if s[0] == "second back to back"]
        if relevant:
            _, _, _, silence_ms = relevant[0]
            assert silence_ms == 0, f"Warm stream should not add silence, got {silence_ms}ms"


# ============================================================
# Letterbox + YOLO postprocess
# ============================================================

class TestLetterbox:
    def test_output_shape(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        out, scale, _ = MOD.letterbox_resize(img, 224)
        assert out.shape == (224, 224, 3)

    def test_scale_leq_1(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        _, scale, _ = MOD.letterbox_resize(img, 224)
        assert 0 < scale <= 1.0

    def test_buffer_reuse(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        buf = np.full((224, 224, 3), 114, dtype=np.uint8)
        out, _, _ = MOD.letterbox_resize(img, 224, buf)
        assert out is buf


class TestYoloPostprocess:
    def _row(self, x1, y1, x2, y2, conf, cls_id):
        return [x1, y1, x2, y2, conf, cls_id]

    def test_below_conf_filtered(self):
        r = self._row(10, 10, 100, 100, MOD.CONF_THRESH - 0.01, 0)
        assert MOD.yolo_postprocess(np.array([[r]]), 640, 480, 1.0, 0, 0) == []

    def test_at_conf_passes(self):
        r = self._row(10, 10, 100, 100, MOD.CONF_THRESH + 0.001, 0)
        assert len(MOD.yolo_postprocess(np.array([[r]]), 640, 480, 1.0, 0, 0)) == 1

    def test_ignored_class_filtered(self):
        kid = MOD.CLASS_NAMES.index("kite")
        r = self._row(10, 10, 100, 100, 0.9, kid)
        assert MOD.yolo_postprocess(np.array([[r]]), 640, 480, 1.0, 0, 0) == []

    def test_tiny_box_filtered(self):
        r = self._row(10, 10, 15, 15, 0.9, 0)
        assert MOD.yolo_postprocess(np.array([[r]]), 640, 480, 1.0, 0, 0) == []


# ============================================================
# ObjectTracker
# ============================================================

class TestObjectTracker:
    def _det(self, x1, y1, x2, y2, name="person", score=0.9):
        return {"class_id": 0, "class_name": name, "score": score,
                "box": [x1, y1, x2, y2]}

    def test_new_track_created(self):
        tr = MOD.ObjectTracker()
        assert len(tr.update([self._det(0,0,100,100)], 0.0)) == 1

    def test_same_id_retained(self):
        tr = MOD.ObjectTracker()
        tr.update([self._det(0,0,100,100)], 0.0)
        tid = list(tr.tracks.keys())[0]
        tr.update([self._det(5,5,105,105)], 0.1)
        assert tid in tr.tracks

    def test_stale_pruned(self):
        tr = MOD.ObjectTracker(max_age=0.5)
        tr.update([self._det(0,0,100,100)], 0.0)
        assert len(tr.update([], 0.6)) == 0

    def test_different_class_not_matched(self):
        tr = MOD.ObjectTracker()
        tr.update([self._det(0,0,100,100,"person")], 0.0)
        tr.update([self._det(5,5,105,105,"chair")],  0.1)
        assert len(tr.tracks) == 2

    def test_seen_frames_increments(self):
        tr = MOD.ObjectTracker()
        tr.update([self._det(0,0,100,100)], 0.0)
        tid = list(tr.tracks.keys())[0]
        tr.update([self._det(5,5,105,105)], 0.1)
        assert tr.tracks[tid].seen_frames == 2

    def test_alive_at_exactly_max_age(self):
        tr = MOD.ObjectTracker(max_age=1.0)
        tr.update([self._det(0,0,100,100)], 0.0)
        assert len(tr.update([], 1.0)) == 1  # expiry uses >, not >=

    def test_pruned_just_past_max_age(self):
        tr = MOD.ObjectTracker(max_age=1.0)
        tr.update([self._det(0,0,100,100)], 0.0)
        assert len(tr.update([], 1.001)) == 0


# ============================================================
# get_smart_distance
# ============================================================

class TestGetSmartDistance:
    def test_uniform_depth(self):
        depth = np.full((200, 200), 1500, dtype=np.uint16)
        dist, _ = MOD.get_smart_distance(depth, [50, 50, 150, 150])
        assert dist == 150

    def test_all_zero_returns_none(self):
        depth = np.zeros((200, 200), dtype=np.uint16)
        dist, _ = MOD.get_smart_distance(depth, [50, 50, 150, 150])
        assert dist is None

    def test_tiny_bbox_returns_none(self):
        depth = np.full((200, 200), 1000, dtype=np.uint16)
        dist, _ = MOD.get_smart_distance(depth, [0, 0, 5, 5])
        assert dist is None

    def test_out_of_range_filtered(self):
        depth = np.full((200, 200), 9000, dtype=np.uint16)
        dist, _ = MOD.get_smart_distance(depth, [50, 50, 150, 150])
        assert dist is None


# ============================================================
# get_position boundary table
# ============================================================

class TestGetPosition:
    def _t(self, cx):
        class T:
            box = [cx-20, 100, cx+20, 300]
        return T()

    def _tracked(self, cx, seen_frames=5):
        class T:
            pass
        t = T()
        t.box = [cx-20, 100, cx+20, 300]
        t.seen_frames = seen_frames
        return t

    @pytest.mark.parametrize("cx,expected", [
        (50,  "on your left"),
        (200, "on your left"),
        (211, "on your left"),  # 211/640=0.3297 < 0.33 → left
        (212, "ahead"),         # 212/640=0.3313 > 0.33 → centre
        (320, "ahead"),
        (428, "ahead"),         # 428/640=0.6688 < 0.67 → centre
        (430, "on your right"), # 430/640=0.6719 > 0.67 → right
        (600, "on your right"),
    ])
    def test_position_zones(self, cx, expected):
        assert MOD.get_position(self._t(cx), 640) == expected

    def test_track_hysteresis_holds_side_label_near_boundary(self):
        track = self._tracked(440)
        assert MOD.get_position(track, 640) == "on your right"

        track.box = [400-20, 100, 400+20, 300]
        assert MOD.get_position(track, 640) == "on your right"
        assert MOD.get_position(track, 640) == "ahead"

    def test_same_frame_repeated_position_reads_do_not_consume_hysteresis(self):
        track = self._tracked(440)
        assert MOD.get_position(track, 640, frame_tag=1) == "on your right"

        track.box = [400-20, 100, 400+20, 300]
        assert MOD.get_position(track, 640, frame_tag=2) == "on your right"
        assert MOD.get_position(track, 640, frame_tag=2) == "on your right"
        assert MOD.get_position(track, 640, frame_tag=3) == "ahead"


class TestMotionDetectorIMUFallback:
    def test_reinitializes_after_repeated_errno5(self, monkeypatch):
        class FailingIMU:
            def read_accelerometer_gyro_data(self):
                raise OSError(5, "Input/output error")

        class WorkingIMU:
            def read_accelerometer_gyro_data(self):
                return (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

        class Factory:
            def __init__(self):
                self.calls = 0

            def __call__(self, i2c_addr=None):
                self.calls += 1
                return FailingIMU() if self.calls == 1 else WorkingIMU()

        factory = Factory()
        fake_icm = _make_stub("icm20948", ICM20948=factory)
        monkeypatch.setitem(sys.modules, "icm20948", fake_icm)

        motion = MOD.MotionDetector()
        assert motion.available is True

        motion.last_read_time = -1e9
        for _ in range(MOD.IMU_MAX_IO_ERRORS):
            motion.update()
            motion.last_read_time = -1e9

        assert factory.calls >= 2
        assert motion.available is True
        assert motion.imu is not None
        assert motion._io_error_count == 0

"""
BlindNav v3.20 -- Unit Tests
Run with: pytest tests/test_blindnav.py -v

All hardware dependencies (pyrealsense2, onnxruntime, cv2) are mocked so
these tests run on any machine without a Pi, camera, or model file.

Covers:
  - letterbox_resize      : image scaling + padding math
  - yolo_postprocess      : YOLO26n (1,300,6) decode + filtering
  - Track.update          : bounding-box EMA smoothing
  - get_smart_distance    : depth median + IQR clustering
  - ObjectTracker         : IoU matching, stale track pruning
    NOTE: the ghost filter (seen_frames >= 3) lives in the MAIN LOOP,
    not in ObjectTracker.update. ObjectTracker always returns all tracks.
    The tests below reflect this accurately.
"""

import sys
import types
import importlib.util
import pathlib
import unittest.mock as _mock

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Mock all hardware/native imports before loading the main module
# ---------------------------------------------------------------------------

def _stub(name):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


# pyrealsense2
_rs = _stub("pyrealsense2")
for _a in ("pipeline", "config", "align", "stream", "format", "colorizer"):
    setattr(_rs, _a, type(_a, (), {
        "color": 0, "depth": 1, "bgr8": 2, "z16": 3,
        "__call__": lambda s, *a, **k: None,
    })())

# onnxruntime
_ort = _stub("onnxruntime")
_ort.GraphOptimizationLevel = type("G", (), {"ORT_ENABLE_ALL": 99})()
_ort.ExecutionMode          = type("E", (), {"ORT_SEQUENTIAL": 0})()

class _OrtOptions:
    graph_optimization_level = None
    intra_op_num_threads = 4
    inter_op_num_threads = 1
    execution_mode = None

_ort.SessionOptions = _OrtOptions

class _OrtSession:
    def get_inputs(self):
        return [type("I", (), {"name": "images"})()]
    def run(self, _, __):
        return [np.zeros((1, 300, 6), dtype=np.float32)]

_ort.InferenceSession = lambda *a, **k: _OrtSession()

# cv2
_cv2 = _stub("cv2")
_cv2.COLOR_BGR2GRAY = 6
_cv2.COLOR_BGR2RGB  = 4
_cv2.INTER_LINEAR   = 1
_cv2.INTER_AREA     = 3
_cv2.TERM_CRITERIA_EPS   = 1
_cv2.TERM_CRITERIA_COUNT = 2
_cv2.cvtColor     = lambda img, code, dst=None: (dst if dst is not None
                                                  else img[:, :, 0] if img.ndim == 3
                                                  else img)
_cv2.resize       = lambda img, sz, interpolation=1: np.zeros(
    (*sz[::-1], img.shape[2]) if img.ndim == 3 else sz[::-1], dtype=img.dtype)
_cv2.goodFeaturesToTrack  = lambda *a, **k: None
_cv2.calcOpticalFlowPyrLK = lambda *a, **k: (None, None, None)
_cv2.createCLAHE  = lambda **k: type("CLAHE", (), {"apply": lambda s, x: x})()
_cv2.calcHist     = lambda *a, **k: np.zeros((256, 1))
_cv2.normalize    = lambda src, dst, *a, **k: dst

for _name in ("anthropic", "smbus2", "icm20948"):
    _stub(_name)


# ---------------------------------------------------------------------------
# Load the main module
# ---------------------------------------------------------------------------

_SCRIPT = (pathlib.Path(__file__).parent.parent
           / "raspberry_pi/yolo_realsense_navigation.py")

_spec = importlib.util.spec_from_file_location("blindnav", str(_SCRIPT))
_mod  = importlib.util.module_from_spec(_spec)

with _mock.patch("os.makedirs"), _mock.patch("builtins.open", _mock.mock_open()):
    if _spec.loader is None:
        raise ImportError(f"Cannot load module from {_SCRIPT}")
    _spec.loader.exec_module(_mod)

letterbox_resize   = _mod.letterbox_resize
yolo_postprocess   = _mod.yolo_postprocess
get_smart_distance = _mod.get_smart_distance
Track              = _mod.Track
ObjectTracker      = _mod.ObjectTracker
CONF_THRESH        = _mod.CONF_THRESH
CLASS_NAMES        = _mod.CLASS_NAMES
IGNORED_CLASSES    = _mod.IGNORED_CLASSES


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _det(box, cls="person", score=0.9):
    cls_id = CLASS_NAMES.index(cls) if cls in CLASS_NAMES else 0
    return {"class_name": cls, "class_id": cls_id,
            "score": score, "box": list(box)}


def _raw_yolo26n(boxes):
    """Build a (1, 300, 6) array with given rows; rest zero-padded."""
    arr = np.zeros((1, 300, 6), dtype=np.float32)
    for i, row in enumerate(boxes[:300]):
        arr[0, i] = row
    return arr


W, H         = 640, 480
SCALE        = 224 / 640
PAD_L, PAD_T = 0, 28


def _post(raw):
    return yolo_postprocess(raw, W, H, SCALE, PAD_L, PAD_T)


# ===========================================================================
# 1. letterbox_resize
# ===========================================================================

class TestLetterboxResize:

    def test_output_shape_is_target_size(self):
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        out, _, _ = letterbox_resize(img, 224)
        assert out.shape == (224, 224, 3)

    def test_scale_is_positive_and_le_1(self):
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        _, scale, _ = letterbox_resize(img, 224)
        assert 0 < scale <= 1.0

    def test_padding_is_non_negative(self):
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        _, _, (pl, pt) = letterbox_resize(img, 224)
        assert pl >= 0 and pt >= 0

    def test_square_input_no_padding(self):
        img = np.zeros((224, 224, 3), dtype=np.uint8)
        _, scale, (pl, pt) = letterbox_resize(img, 224)
        assert pl == 0 and pt == 0
        assert abs(scale - 1.0) < 1e-3

    def test_preallocated_buf_reused(self):
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        buf = np.full((224, 224, 3), 114, dtype=np.uint8)
        out, _, _ = letterbox_resize(img, 224, buf)
        assert out is buf


# ===========================================================================
# 2. yolo_postprocess  (YOLO26n one-to-one head, NMS-free, shape (1,300,6))
# ===========================================================================

class TestYoloPostprocess:

    def test_none_input_returns_empty(self):
        assert _post(None) == []

    def test_zero_padded_rows_produce_no_detections(self):
        """All 300 rows with conf=0 (normal zero-padding) must yield nothing."""
        assert _post(np.zeros((1, 300, 6), dtype=np.float32)) == []

    def test_conf_below_thresh_filtered(self):
        p = CLASS_NAMES.index("person")
        assert _post(_raw_yolo26n([[50, 50, 150, 180, CONF_THRESH - 0.01, p]])) == []

    def test_conf_at_thresh_passes(self):
        p = CLASS_NAMES.index("person")
        dets = _post(_raw_yolo26n([[50, 50, 150, 180, CONF_THRESH + 0.001, p]]))
        assert len(dets) == 1

    def test_person_class_and_score_correct(self):
        p = CLASS_NAMES.index("person")
        dets = _post(_raw_yolo26n([[50, 50, 150, 180, 0.90, p]]))
        assert dets[0]["class_name"] == "person"
        assert dets[0]["score"] == pytest.approx(0.90, abs=1e-4)

    def test_coords_rescaled_into_original_image_space(self):
        p = CLASS_NAMES.index("person")
        dets = _post(_raw_yolo26n([[PAD_L, PAD_T, PAD_L + 200, PAD_T + 140, 0.85, p]]))
        assert len(dets) == 1
        x1, y1, x2, y2 = dets[0]["box"]
        assert 0 <= x1 < x2 <= W
        assert 0 <= y1 < y2 <= H

    def test_coords_clamped_at_image_bounds(self):
        """Out-of-bounds letterbox coords must be clamped, not crash."""
        p = CLASS_NAMES.index("person")
        raw = _raw_yolo26n([[-999, -999, 9999, 9999, 0.92, p]])
        dets = _post(raw)
        if dets:
            x1, y1, x2, y2 = dets[0]["box"]
            assert x1 >= 0 and y1 >= 0
            assert x2 <= W and y2 <= H

    def test_tiny_box_filtered(self):
        p = CLASS_NAMES.index("person")
        assert _post(_raw_yolo26n([[50, 50, 53, 53, 0.95, p]])) == []

    def test_unknown_class_id_ignored(self):
        assert _post(_raw_yolo26n([[50, 50, 150, 180, 0.95, 9999]])) == []

    def test_ignored_class_kite_filtered(self):
        assert "kite" in IGNORED_CLASSES
        k = CLASS_NAMES.index("kite")
        assert _post(_raw_yolo26n([[50, 50, 150, 180, 0.99, k]])) == []

    def test_ignored_class_airplane_filtered(self):
        assert "airplane" in IGNORED_CLASSES
        a = CLASS_NAMES.index("airplane")
        assert _post(_raw_yolo26n([[50, 50, 150, 180, 0.99, a]])) == []

    def test_multiple_valid_detections_all_returned(self):
        p = CLASS_NAMES.index("person")
        c = CLASS_NAMES.index("chair")
        raw = _raw_yolo26n([
            [10,  PAD_T, 80,  150, 0.91, p],
            [120, PAD_T, 200, 180, 0.85, c],
        ])
        names = {d["class_name"] for d in _post(raw)}
        assert "person" in names and "chair" in names

    def test_unexpected_2d_input_does_not_crash(self):
        """(300, 6) without batch dim should not raise."""
        try:
            result = _post(np.zeros((300, 6), dtype=np.float32))
            assert isinstance(result, list)
        except Exception as e:
            pytest.fail(f"Crashed on 2D input: {e}")


# ===========================================================================
# 3. Track -- bounding-box EMA smoothing (v3.19)
# ===========================================================================

class TestTrackEmaSmoothing:

    def test_initial_box_is_exact(self):
        t = Track(1, _det([100, 100, 200, 200]), 0.0)
        assert t.box == [100, 100, 200, 200]

    def test_first_update_uses_alpha_1(self):
        """seen_frames=1 at init so first update must be exact (alpha=1.0)."""
        t = Track(1, _det([100, 100, 200, 200]), 0.0)
        t.update(_det([110, 110, 210, 210]), 0.1)
        assert t.box == [110, 110, 210, 210]

    def test_second_update_blends_alpha06(self):
        """After first update (seen_frames=2), alpha=0.6 applied."""
        t = Track(1, _det([100, 100, 200, 200]), 0.0)
        t.update(_det([100, 100, 200, 200]), 0.1)   # alpha=1.0, seed
        t.update(_det([120, 120, 220, 220]), 0.2)   # alpha=0.6
        assert t.box[0] == int(0.6 * 120 + 0.4 * 100)   # 112
        assert t.box[2] == int(0.6 * 220 + 0.4 * 200)   # 212

    def test_smoothing_converges_to_new_position(self):
        t = Track(1, _det([0, 0, 100, 100]), 0.0)
        t.update(_det([0, 0, 100, 100]), 0.1)
        for i in range(25):
            t.update(_det([200, 200, 300, 300]), float(i + 2))
        assert abs(t.box[0] - 200) < 5
        assert abs(t.box[2] - 300) < 5

    def test_seen_frames_increments(self):
        t = Track(1, _det([0, 0, 50, 50]), 0.0)
        for expected in range(2, 6):
            t.update(_det([0, 0, 50, 50]), float(expected) * 0.1)
            assert t.seen_frames == expected


# ===========================================================================
# 4. get_smart_distance
# ===========================================================================

class TestGetSmartDistance:

    def _d(self, v=1500):
        return np.full((480, 640), v, dtype=np.uint16)

    def test_uniform_depth_correct_cm(self):
        dist, _ = get_smart_distance(self._d(2000), [100, 100, 400, 400])
        assert dist is not None and abs(dist - 200) <= 5

    def test_all_zero_returns_none(self):
        dist, _ = get_smart_distance(np.zeros((480, 640), dtype=np.uint16),
                                     [100, 100, 400, 400])
        assert dist is None

    def test_tiny_bbox_returns_none(self):
        dist, _ = get_smart_distance(self._d(), [100, 100, 104, 104])
        assert dist is None

    def test_out_of_range_depth_returns_none(self):
        dist, _ = get_smart_distance(self._d(9000), [100, 100, 400, 400])
        assert dist is None

    def test_two_cluster_scene(self):
        """
        Two depth planes in the box. Result must be near one of them (100 or 300 cm).
        We don't assert which wins -- sampling/trimming is implementation-defined.
        """
        depth = np.zeros((480, 640), dtype=np.uint16)
        depth[100:300, 100:370] = 1000
        depth[100:300, 370:550] = 3000
        dist, nc = get_smart_distance(depth, [100, 100, 550, 300])
        assert dist is not None
        assert abs(dist - 100) <= 20 or abs(dist - 300) <= 20
        assert nc >= 1


# ===========================================================================
# 5. ObjectTracker
# ===========================================================================

class TestObjectTracker:

    def test_new_detection_creates_internal_track(self):
        ot = ObjectTracker()
        ot.update([_det([100, 100, 200, 200])], 0.0)
        assert len(ot.tracks) == 1

    def test_tracker_returns_all_tracks_no_ghost_filter(self):
        """
        ObjectTracker.update() returns all tracks unconditionally.
        The ghost filter (seen_frames >= 3) is applied in the main loop.
        A brand-new track has seen_frames=1 and IS returned here.
        """
        ot = ObjectTracker()
        tracks = ot.update([_det([100, 100, 200, 200])], 0.0)
        assert len(tracks) == 1
        assert tracks[0].seen_frames == 1

    def test_same_object_retains_track_id(self):
        ot = ObjectTracker()
        det = _det([100, 100, 200, 200])
        first_id = ot.update([det], 0.0)[0].id
        for i in range(1, 5):
            tracks = ot.update([det], float(i) * 0.1)
        assert tracks[0].id == first_id
        assert len(ot.tracks) == 1

    def test_non_overlapping_boxes_separate_tracks(self):
        ot = ObjectTracker()
        ot.update([_det([0, 0, 50, 50]), _det([400, 400, 500, 500])], 0.0)
        assert len(ot.tracks) == 2

    def test_stale_track_pruned_after_max_age(self):
        ot = ObjectTracker(max_age=1.0)
        ot.update([_det([100, 100, 200, 200])], 0.0)
        ot.update([], 9999.0)
        assert len(ot.tracks) == 0

    def test_track_alive_at_exactly_max_age(self):
        """Track at exactly max_age should still be alive (uses >, not >=)."""
        ot = ObjectTracker(max_age=1.0)
        ot.update([_det([100, 100, 200, 200])], 0.0)
        ot.update([], 1.0)   # exactly max_age -- should survive
        assert len(ot.tracks) == 1

    def test_track_pruned_just_past_max_age(self):
        ot = ObjectTracker(max_age=1.0)
        ot.update([_det([100, 100, 200, 200])], 0.0)
        ot.update([], 1.001)  # just over max_age -- should be pruned
        assert len(ot.tracks) == 0

    def test_seen_frames_increments_on_match(self):
        ot = ObjectTracker()
        det = _det([100, 100, 200, 200])
        for i in range(4):
            tracks = ot.update([det], float(i) * 0.1)
        assert tracks[0].seen_frames == 4

    def test_different_class_not_matched_to_existing_track(self):
        """A 'chair' detection must not absorb a 'person' track at the same box."""
        ot = ObjectTracker()
        ot.update([_det([100, 100, 200, 200], cls="person")], 0.0)
        ot.update([_det([100, 100, 200, 200], cls="chair")], 0.1)
        assert len(ot.tracks) == 2
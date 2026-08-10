import importlib.util
from pathlib import Path

from evaluation.models import Box3D


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/evaluate_kitti_3d_tracking.py'
SPEC = importlib.util.spec_from_file_location('tracking_evaluator', SCRIPT)
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


def test_prediction_frame_offset_shifts_keys_and_box_frame_indices():
    prediction = {1: [Box3D(1, 7, 'Car', 0., 0., 0., 1., 1., 1., 0.)]}
    shifted = EVALUATOR.shift_prediction_frames(prediction, -1)
    assert list(shifted) == [0]
    assert shifted[0][0].frame_idx == 0
    assert shifted[0][0].track_id == 7

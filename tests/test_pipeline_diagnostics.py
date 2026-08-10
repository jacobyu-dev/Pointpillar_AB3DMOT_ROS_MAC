import importlib.util
from pathlib import Path

from evaluation.geometry.iou3d import compute_bev_iou_matrix
from evaluation.models import Box3D


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/diagnostics/analyze_pointpillars_pipeline.py'
SPEC = importlib.util.spec_from_file_location('pipeline_diagnostics', SCRIPT)
PIPELINE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PIPELINE)


def box(frame, object_id, score=None):
    return Box3D(frame, object_id, 'Car', 10., 0., 0., 1.6, 1.8, 4., 0., score)


def test_offset_sweep_uses_prediction_frame_equal_to_gt_frame_plus_offset():
    gt = {0: [box(0, 1)]}
    predictions = {1: [box(1, 2, .9)]}
    aligned = PIPELINE._evaluate(gt, predictions, [0], compute_bev_iou_matrix, offset=1)
    unaligned = PIPELINE._evaluate(gt, predictions, [0], compute_bev_iou_matrix, offset=0)
    assert aligned['tp'] == 1
    assert unaligned['tp'] == 0

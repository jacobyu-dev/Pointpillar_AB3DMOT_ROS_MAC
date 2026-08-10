from evaluation.detection_metrics import evaluate_detections
from evaluation.geometry.iou3d import oriented_iou3d
from evaluation.models import Box3D


def box(frame, object_id, x=0.0, score=None):
    return Box3D(frame, object_id, 'Car', x, 0., 0., 2., 2., 4., 0., score)


def test_perfect_raw_detection_has_perfect_ap():
    gt = {0: [box(0, 1)], 1: [box(1, 1)]}
    predictions = {0: [box(0, 10, score=.9)], 1: [box(1, 11, score=.8)]}
    metrics, rows = evaluate_detections(gt, predictions, .5, oriented_iou3d)
    assert metrics['average_precision'] == 1.0
    assert metrics['true_positives'] == 2
    assert [row['tp'] for row in rows] == [1, 1]


def test_duplicate_detection_is_false_positive_but_does_not_change_ap():
    gt = {0: [box(0, 1)]}
    predictions = {0: [box(0, 10, score=.9), box(0, 11, score=.8)]}
    metrics, rows = evaluate_detections(gt, predictions, .5)
    assert metrics['average_precision'] == 1.0
    assert metrics['false_positives'] == 1
    assert [row['tp'] for row in rows] == [1, 0]


def test_high_score_false_positive_reduces_ap():
    gt = {0: [box(0, 1)]}
    predictions = {0: [box(0, 10, x=20., score=.9), box(0, 11, score=.8)]}
    metrics, rows = evaluate_detections(gt, predictions, .5)
    assert metrics['average_precision'] == .5
    assert rows[0]['fp'] == 1

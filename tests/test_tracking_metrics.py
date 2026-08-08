from evaluation.models import Box3D
from evaluation.trackeval_adapter import build_sequence_data, evaluate_trackeval


TRACK_EVAL = 'third_party/TrackEval'


def box(frame, track_id, x=0.0):
    return Box3D(frame, track_id, 'Car', x, 0., 0., 2., 2., 4., 0.)


def test_perfect_tracking_metrics_are_maximal():
    gt = {0: [box(0, 1)], 1: [box(1, 1)]}
    prediction = {0: [box(0, 10)], 1: [box(1, 10)]}
    result = evaluate_trackeval(build_sequence_data(gt, prediction), TRACK_EVAL, 0.5)
    assert result['FP'] == result['FN'] == result['IDSW'] == 0
    assert result['HOTA_3D'] == result['DetA_3D'] == result['AssA_3D'] == result['IDF1'] == 1.0


def test_missing_detection_and_id_switch_are_detected():
    gt = {0: [box(0, 1)], 1: [box(1, 1)]}
    missing = {0: [box(0, 10)]}
    switched = {0: [box(0, 10)], 1: [box(1, 11)]}
    missing_result = evaluate_trackeval(build_sequence_data(gt, missing), TRACK_EVAL, 0.5)
    switched_result = evaluate_trackeval(build_sequence_data(gt, switched), TRACK_EVAL, 0.5)
    assert missing_result['FN'] == 1
    assert switched_result['IDSW'] == 1
    assert switched_result['AssA_3D'] < 1.0


def test_false_positive_and_localization_error_reduce_metrics():
    gt = {0: [box(0, 1)]}
    false_positive = {0: [box(0, 10), box(0, 11, x=10.0)]}
    shifted = {0: [box(0, 10, x=1.0)]}
    false_positive_result = evaluate_trackeval(build_sequence_data(gt, false_positive), TRACK_EVAL, 0.5)
    shifted_result = evaluate_trackeval(build_sequence_data(gt, shifted), TRACK_EVAL, 0.1)
    assert false_positive_result['FP'] == 1
    assert shifted_result['LocA_3D'] < 1.0

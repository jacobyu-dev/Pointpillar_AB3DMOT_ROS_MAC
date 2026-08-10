"""Confidence-ranked 3D object-detection metrics, independent of tracking."""

from collections.abc import Callable

from evaluation.geometry.iou3d import oriented_iou3d
from evaluation.models import Box3D


SimilarityFn = Callable[[Box3D, Box3D], float]


def evaluate_detections(
    gt_by_frame: dict[int, list[Box3D]],
    predictions_by_frame: dict[int, list[Box3D]],
    iou_threshold: float,
    similarity_fn: SimilarityFn = oriented_iou3d,
    frame_indices=None,
) -> tuple[dict[str, float | int], list[dict[str, float | int | str]]]:
    """Calculate one-class AP with per-frame, confidence-ordered matching.

    A prediction can match at most one GT from its own frame.  AP uses the
    continuous precision envelope, which avoids a dependency on TrackEval and
    intentionally has no tracking/identity component.
    """
    if not 0.0 <= iou_threshold <= 1.0:
        raise ValueError('iou_threshold must be within [0, 1]')
    frames = sorted(frame_indices) if frame_indices is not None else sorted(
        set(gt_by_frame) | set(predictions_by_frame))
    allowed_frames = set(frames)
    gt = {frame: gt_by_frame.get(frame, []) for frame in frames}
    candidates = []
    for frame in frames:
        for detection_index, box in enumerate(predictions_by_frame.get(frame, [])):
            if box.score is None:
                raise ValueError('every raw detection must have a confidence score')
            candidates.append((float(box.score), frame, detection_index, box))
    # Stable secondary keys make equal-score reports reproducible.
    candidates.sort(key=lambda item: (-item[0], item[1], item[2]))

    matched_gt: dict[int, set[int]] = {frame: set() for frame in frames}
    total_gt = sum(len(boxes) for boxes in gt.values())
    tp = fp = 0
    rows: list[dict[str, float | int | str]] = []
    precisions, recalls = [], []
    for rank, (score, frame, detection_index, prediction) in enumerate(candidates, start=1):
        best_index, best_iou = None, 0.0
        for gt_index, gt_box in enumerate(gt[frame]):
            if gt_index in matched_gt[frame]:
                continue
            iou = similarity_fn(gt_box, prediction)
            if iou > best_iou:
                best_index, best_iou = gt_index, iou
        is_tp = best_index is not None and best_iou >= iou_threshold
        if is_tp:
            matched_gt[frame].add(best_index)
            tp += 1
        else:
            fp += 1
        precision = tp / (tp + fp)
        recall = tp / total_gt if total_gt else 0.0
        precisions.append(precision)
        recalls.append(recall)
        rows.append({
            'rank': rank, 'frame_idx': frame, 'detection_index': detection_index,
            'score': score, 'tp': int(is_tp), 'fp': int(not is_tp),
            'cumulative_tp': tp, 'cumulative_fp': fp, 'precision': precision,
            'recall': recall, 'matched_gt_id': gt[frame][best_index].track_id if is_tp else '',
            'matched_iou': best_iou,
        })

    # Integral under the monotonically non-increasing precision envelope.
    average_precision = 0.0
    previous_recall = 0.0
    for index in range(len(precisions) - 1, -1, -1):
        if index + 1 < len(precisions):
            precisions[index] = max(precisions[index], precisions[index + 1])
    for precision, recall in zip(precisions, recalls):
        if recall > previous_recall:
            average_precision += (recall - previous_recall) * precision
            previous_recall = recall
    return ({
        'gt_detections': total_gt,
        'prediction_detections': len(candidates),
        'true_positives': tp,
        'false_positives': fp,
        'false_negatives': total_gt - tp,
        'precision': tp / (tp + fp) if tp + fp else 0.0,
        'recall': tp / total_gt if total_gt else 0.0,
        'average_precision': average_precision,
    }, rows)

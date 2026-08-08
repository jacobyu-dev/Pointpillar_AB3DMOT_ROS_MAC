from collections import defaultdict
import csv
from pathlib import Path

from evaluation.geometry.iou3d import compute_iou_matrix
from evaluation.matching import match_iou_matrix
from evaluation.models import Box3D


def build_diagnostics(gt_by_frame: dict[int, list[Box3D]], prediction_by_frame: dict[int, list[Box3D]], threshold: float,
                      similarity_fn=compute_iou_matrix):
    frame_rows, switch_rows = [], []
    track_rows = defaultdict(lambda: {'num_gt_frames': 0, 'num_matched_frames': 0, 'num_missed_frames': 0,
                                      'num_id_switches': 0, 'ious': [], 'tracker_ids': set(),
                                      'class_name': '', 'first_frame': None, 'last_frame': None})
    previous_tracker_for_gt: dict[int, tuple[int, float]] = {}
    for frame in sorted(set(gt_by_frame) | set(prediction_by_frame)):
        gt_boxes, prediction_boxes = gt_by_frame.get(frame, []), prediction_by_frame.get(frame, [])
        matrix = similarity_fn(gt_boxes, prediction_boxes)
        matches, unmatched_gt, unmatched_predictions = match_iou_matrix(matrix, threshold)
        matched_ious = [match.iou for match in matches]
        for gt_box in gt_boxes:
            row = track_rows[gt_box.track_id]
            row['num_gt_frames'] += 1; row['class_name'] = gt_box.class_name
            row['first_frame'] = frame if row['first_frame'] is None else min(row['first_frame'], frame)
            row['last_frame'] = frame if row['last_frame'] is None else max(row['last_frame'], frame)
        for index in unmatched_gt:
            track_rows[gt_boxes[index].track_id]['num_missed_frames'] += 1
        for match in matches:
            gt_box, prediction_box = gt_boxes[match.gt_index], prediction_boxes[match.prediction_index]
            row = track_rows[gt_box.track_id]
            row['num_matched_frames'] += 1; row['ious'].append(match.iou); row['tracker_ids'].add(prediction_box.track_id)
            previous = previous_tracker_for_gt.get(gt_box.track_id)
            if previous is not None and previous[0] != prediction_box.track_id:
                row['num_id_switches'] += 1
                switch_rows.append({'frame_idx': frame, 'gt_id': gt_box.track_id,
                                    'previous_tracker_id': previous[0], 'current_tracker_id': prediction_box.track_id,
                                    'previous_iou': previous[1], 'current_iou': match.iou})
            previous_tracker_for_gt[gt_box.track_id] = (prediction_box.track_id, match.iou)
        frame_rows.append({'frame_idx': frame, 'num_gt': len(gt_boxes), 'num_predictions': len(prediction_boxes),
                           'tp': len(matches), 'fp': len(unmatched_predictions), 'fn': len(unmatched_gt),
                           'id_switches': sum(row['frame_idx'] == frame for row in switch_rows),
                           'mean_matched_iou': sum(matched_ious) / len(matched_ious) if matched_ious else 0.0,
                           'min_matched_iou': min(matched_ious) if matched_ious else 0.0,
                           'max_matched_iou': max(matched_ious) if matched_ious else 0.0})
    tracks = []
    for track_id, row in sorted(track_rows.items()):
        ious = row.pop('ious'); tracker_ids = row.pop('tracker_ids')
        tracks.append({'gt_id': track_id, **row, 'mean_iou': sum(ious) / len(ious) if ious else 0.0,
                       'min_iou': min(ious) if ious else 0.0, 'matched_tracker_ids': ';'.join(map(str, sorted(tracker_ids)))})
    return frame_rows, switch_rows, tracks


def write_csv(path: str | Path, rows: list[dict], fields: list[str]):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)

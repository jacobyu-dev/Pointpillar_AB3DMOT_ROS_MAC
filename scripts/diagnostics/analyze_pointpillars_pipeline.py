#!/usr/bin/env python3
"""Audit raw PointPillars detections before attributing tracking failure.

This script is intentionally detector/tracker agnostic after CSV export.  It
never changes an experiment result: all generated files go to a new analysis
directory and source outputs are read only.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from evaluation.diagnostics import write_csv
from evaluation.geometry.iou3d import compute_bev_iou_matrix, compute_iou_matrix
from evaluation.io.detection_csv_loader import load_detection_csv
from evaluation.io.kitti_tracklet_loader import load_tracklets_pointpillars
from evaluation.io.tracker_csv_loader import load_tracker_csv
from evaluation.matching import match_iou_matrix


ROI = {'x_min': 0.0, 'x_max': 69.12, 'y_min': -39.68, 'y_max': 39.68,
       'z_min': -3.0, 'z_max': 1.0, 'pillar_x': 0.16, 'pillar_y': 0.16,
       'grid_x': 432, 'grid_y': 496}
THRESHOLDS = (.05, .10, .15, .20, .30, .50, .70)


def _frames(gt):
    return range(min(gt), max(gt) + 1)


def _filter(predictions, threshold):
    return {frame: [box for box in boxes if box.score is not None and box.score >= threshold]
            for frame, boxes in predictions.items()}


def _evaluate(gt, predictions, frames, similarity, threshold=.5, offset=0):
    """Evaluate with prediction_frame = gt_frame + offset."""
    tp = fp = fn = 0
    ious, best_ious = [], []
    for frame in frames:
        gt_boxes = gt.get(frame, [])
        prediction_boxes = predictions.get(frame + offset, [])
        matrix = similarity(gt_boxes, prediction_boxes)
        matches, unmatched_gt, unmatched_predictions = match_iou_matrix(matrix, threshold)
        tp += len(matches); fp += len(unmatched_predictions); fn += len(unmatched_gt)
        ious.extend(match.iou for match in matches)
        best_ious.extend(matrix[row].max() if len(prediction_boxes) else 0.0
                         for row in range(len(gt_boxes)))
    return {
        'tp': tp, 'fp': fp, 'fn': fn,
        'precision': tp / (tp + fp) if tp + fp else 0.0,
        'recall': tp / (tp + fn) if tp + fn else 0.0,
        'mean_matched_iou': sum(ious) / len(ious) if ious else 0.0,
        'mean_best_iou': sum(best_ious) / len(best_ious) if best_ious else 0.0,
    }


def _yaw_error(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def _best_iou_rows(gt, predictions, frames):
    rows, distribution = [], Counter()
    bev_only_3d_fail = 0
    for frame in frames:
        gt_boxes, prediction_boxes = gt.get(frame, []), predictions.get(frame, [])
        bev = compute_bev_iou_matrix(gt_boxes, prediction_boxes)
        iou3d = compute_iou_matrix(gt_boxes, prediction_boxes)
        for index, gt_box in enumerate(gt_boxes):
            if not prediction_boxes:
                best_index = None; best_bev = best_3d = best_score = 0.0
                category = 'no_prediction'
                distance = yaw_error = height_error = ''
            else:
                best_index = int(bev[index].argmax())
                best_prediction = prediction_boxes[best_index]
                best_bev, best_3d = float(bev[index, best_index]), float(iou3d[index, best_index])
                best_score = float(best_prediction.score)
                distance = math.hypot(gt_box.x - best_prediction.x, gt_box.y - best_prediction.y)
                yaw_error = _yaw_error(gt_box.yaw, best_prediction.yaw)
                height_error = abs(gt_box.h - best_prediction.h)
                if best_bev >= .5: category = 'bev_ge_0.5'
                elif best_bev >= .3: category = 'bev_0.3_to_0.5'
                elif best_bev >= .1: category = 'bev_0.1_to_0.3'
                else: category = 'bev_lt_0.1'
                bev_only_3d_fail += int(best_bev >= .5 and best_3d < .5)
            distribution[category] += 1
            rows.append({
                'frame_idx': frame, 'gt_id': gt_box.track_id, 'gt_x': gt_box.x, 'gt_y': gt_box.y,
                'gt_z': gt_box.z, 'best_bev_iou': best_bev, 'best_3d_iou': best_3d,
                'best_score': best_score, 'center_distance': distance, 'yaw_error_rad': yaw_error,
                'height_error': height_error, 'category': category,
                'matched_bev_0.5': int(best_bev >= .5), 'matched_3d_0.5': int(best_3d >= .5),
            })
    return rows, distribution, bev_only_3d_fail


def _inside_roi(box):
    """Use the GT bottom centre, matching the detector's z convention."""
    bottom_z = box.z - box.h / 2.0
    return (ROI['x_min'] <= box.x < ROI['x_max'] and ROI['y_min'] <= box.y < ROI['y_max']
            and ROI['z_min'] <= bottom_z < ROI['z_max'])


def _roi_report(gt, frames):
    inside = outside = 0
    outside_by_axis = Counter()
    for frame in frames:
        for box in gt.get(frame, []):
            # Tracklets are converted to centre-z; PointPillars uses bottom-z
            # for its pre-decode anchors, so classify by bottom centre here.
            axes = {
                'x': ROI['x_min'] <= box.x < ROI['x_max'],
                'y': ROI['y_min'] <= box.y < ROI['y_max'],
                'bottom_z': ROI['z_min'] <= box.z - box.h / 2.0 < ROI['z_max'],
            }
            if all(axes.values()): inside += 1
            else:
                outside += 1
                outside_by_axis.update(axis for axis, valid in axes.items() if not valid)
    return {
        'roi': ROI,
        'roi_rule': 'GT bottom centre (x, y, z - h/2) is within half-open PointPillars preprocessing bounds.',
        'gt_total': inside + outside, 'gt_inside_roi': inside, 'gt_outside_roi': outside,
        'outside_by_axis': dict(outside_by_axis),
    }


def _gt_inside_roi(gt):
    return {frame: [box for box in boxes if _inside_roi(box)] for frame, boxes in gt.items()}


def _read_manifest(path):
    if path is None:
        return {}
    with Path(path).open(newline='') as handle:
        return {int(row['frame_idx']): row for row in csv.DictReader(handle)}


def _write_pipeline_frames(output, gt, predictions, tracks, detector_manifest, tracker_manifest, frames):
    fields = ['frame_idx', 'gt_count', 'raw_csv_detection_count', 'tracker_csv_count',
              'detector_callback_observed', 'detector_status', 'detector_num_points',
              'detector_inference_ran', 'detector_num_score_candidates',
              'detector_num_after_nms', 'detector_num_published', 'tracker_callback_observed',
              'tracker_detection_source', 'tracker_num_input_detections', 'tracker_num_output_tracks']
    rows = []
    for frame in frames:
        detector = detector_manifest.get(frame, {})
        tracker = tracker_manifest.get(frame, {})
        rows.append({
            'frame_idx': frame, 'gt_count': len(gt.get(frame, [])),
            'raw_csv_detection_count': len(predictions.get(frame, [])),
            'tracker_csv_count': len(tracks.get(frame, [])),
            'detector_callback_observed': int(bool(detector)), 'detector_status': detector.get('status', ''),
            'detector_num_points': detector.get('num_points', ''),
            'detector_inference_ran': detector.get('inference_ran', ''),
            'detector_num_score_candidates': detector.get('num_score_candidates', ''),
            'detector_num_after_nms': detector.get('num_after_nms', ''),
            'detector_num_published': detector.get('num_published', ''),
            'tracker_callback_observed': int(bool(tracker)),
            'tracker_detection_source': tracker.get('detection_source', ''),
            'tracker_num_input_detections': tracker.get('num_input_detections', ''),
            'tracker_num_output_tracks': tracker.get('num_output_tracks', ''),
        })
    write_csv(output, rows, fields)


def _write_conversion_samples(output, predictions, frames, threshold):
    rows = []
    for frame in frames:
        for detection_index, box in enumerate(predictions.get(frame, [])):
            if box.score < threshold:
                continue
            rows.append({
                'frame_idx': frame, 'detection_index': detection_index, 'score': box.score,
                'published_x': box.x, 'published_y': box.y, 'published_center_z': box.z,
                'published_h': box.h, 'published_w': box.w, 'published_l': box.l,
                'published_yaw': box.yaw,
                'ab3dmot_input_h': box.h, 'ab3dmot_input_w': box.w,
                'ab3dmot_input_l': box.l, 'ab3dmot_input_x': box.x,
                'ab3dmot_input_y': box.y, 'ab3dmot_input_center_z': box.z,
                'ab3dmot_input_yaw': box.yaw,
            })
            if len(rows) == 20:
                break
        if len(rows) == 20:
            break
    fields = list(rows[0]) if rows else ['frame_idx', 'detection_index', 'score']
    write_csv(output, rows, fields)


def _audit_markdown(args, raw_count, raw_frames, gt_count, roi, threshold_metrics, offset_rows, manifest_available):
    best_offset = max(offset_rows, key=lambda row: row['bev_recall'])
    return f"""# 0032 PointPillars + AB3DMOT Failure Analysis

## 1. Current baseline

This analysis consumes the supplied raw-detection CSV and does not modify any
tracking evaluation output. Raw CSV rows: {raw_count}; frames containing rows:
{raw_frames}; GT Car boxes: {gt_count}.

## 2. Pipeline audit

`_score_threshold` is applied in `PointPillarsONNX.infer` before box decode and
before NMS: score sigmoid and anchor-mask candidates must pass it. `_aabb_nms`
then runs on those decoded candidates. `0032_raw.csv` is therefore **post-NMS,
pre-`_publish_score_threshold`**, not every network anchor.

`_publish_score_threshold` is applied in `callback` after CSV writing. Only
those boxes are sent in `/detection/lidar_detector/boxes`; AB3DMOT receives
only that ROS `BoundingBoxArray`. The adapter maps JSK dimensions `(l,w,h)` to
AB3DMOT `(h,w,l,x,y,centre_z,yaw)` with no PointPillars-path x/y/z or yaw
offset. Sample mappings are in `adapter_conversion_samples.csv`.

Detector and tracker callback evidence: {'available' if manifest_available else 'not available for this run'}. A missing raw CSV row alone is never treated as a missing PointCloud callback.

## 3. Frame synchronization

Offset convention: prediction frame = GT frame + offset. At analysis score
threshold {args.analysis_score_threshold:.2f}, the best BEV-recall offset is
{best_offset['offset']} (recall {best_offset['bev_recall']:.4f}). See
`frame_offset_sweep.csv`; this is only conclusive when detector manifests cover
the complete sequence.

## 4. Raw detection quality

At threshold {args.analysis_score_threshold:.2f}: BEV precision/recall are
{threshold_metrics['bev_precision']:.4f}/{threshold_metrics['bev_recall']:.4f};
3D precision/recall are {threshold_metrics['3d_precision']:.4f}/{threshold_metrics['3d_recall']:.4f}.
Per-GT best overlaps are in `gt_best_iou.csv`.

## 5. Detector ROI

The code-defined ROI is x=[0,69.12), y=[-39.68,39.68), z=[-3,1). GT inside:
{roi['gt_inside_roi']}; outside: {roi['gt_outside_roi']}. See
`detector_roi_report.json` for the exact bottom-centre rule.
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracklets', required=True)
    parser.add_argument('--detections', required=True)
    parser.add_argument('--tracks', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--class-name', default='Car')
    parser.add_argument('--analysis-score-threshold', type=float, default=.15)
    parser.add_argument('--detector-manifest')
    parser.add_argument('--tracker-manifest')
    args = parser.parse_args()
    if not 0 <= args.analysis_score_threshold <= 1:
        parser.error('--analysis-score-threshold must be in [0, 1]')
    output = Path(args.output_dir)
    if output.exists() and any(output.iterdir()):
        parser.error(f'output directory is not empty (refusing to overwrite): {output}')
    output.mkdir(parents=True, exist_ok=False)

    gt = load_tracklets_pointpillars(args.tracklets, args.class_name)
    predictions = load_detection_csv(args.detections, args.class_name)
    tracks = load_tracker_csv(args.tracks, args.class_name)
    if not gt:
        parser.error('no matching GT boxes')
    frames = _frames(gt)
    detector_manifest, tracker_manifest = _read_manifest(args.detector_manifest), _read_manifest(args.tracker_manifest)
    _write_pipeline_frames(output / 'frame_pipeline.csv', gt, predictions, tracks,
                           detector_manifest, tracker_manifest, frames)

    filtered = _filter(predictions, args.analysis_score_threshold)
    offset_rows = []
    for offset in range(-5, 6):
        bev = _evaluate(gt, filtered, frames, compute_bev_iou_matrix, offset=offset)
        iou3d = _evaluate(gt, filtered, frames, compute_iou_matrix, offset=offset)
        offset_rows.append({'offset': offset, 'bev_tp': bev['tp'], 'bev_precision': bev['precision'],
                            'bev_recall': bev['recall'], 'mean_best_bev_iou': bev['mean_best_iou'],
                            '3d_tp': iou3d['tp'], '3d_precision': iou3d['precision'],
                            '3d_recall': iou3d['recall'], 'mean_best_3d_iou': iou3d['mean_best_iou']})
    write_csv(output / 'frame_offset_sweep.csv', offset_rows, list(offset_rows[0]))

    best_rows, distribution, bev_only_3d_fail = _best_iou_rows(gt, filtered, frames)
    write_csv(output / 'gt_best_iou.csv', best_rows, list(best_rows[0]))
    roi = _roi_report(gt, frames)
    roi_gt = _gt_inside_roi(gt)
    roi_bev = _evaluate(roi_gt, filtered, frames, compute_bev_iou_matrix)
    roi_3d = _evaluate(roi_gt, filtered, frames, compute_iou_matrix)
    roi['detection_metrics_at_analysis_threshold'] = {
        'score_threshold': args.analysis_score_threshold,
        'all_gt_bev_recall': _evaluate(gt, filtered, frames, compute_bev_iou_matrix)['recall'],
        'inside_roi_bev_recall': roi_bev['recall'],
        'all_gt_3d_recall': _evaluate(gt, filtered, frames, compute_iou_matrix)['recall'],
        'inside_roi_3d_recall': roi_3d['recall'],
    }
    (output / 'detector_roi_report.json').write_text(json.dumps(roi, indent=2) + '\n')

    sweep = []
    for threshold in THRESHOLDS:
        selected = _filter(predictions, threshold)
        bev, iou3d = (_evaluate(gt, selected, frames, fn) for fn in (compute_bev_iou_matrix, compute_iou_matrix))
        sweep.append({'score_threshold': threshold, 'prediction_count': sum(map(len, selected.values())),
                      'bev_tp': bev['tp'], 'bev_fp': bev['fp'], 'bev_fn': bev['fn'],
                      'bev_precision': bev['precision'], 'bev_recall': bev['recall'],
                      'bev_mean_matched_iou': bev['mean_matched_iou'],
                      '3d_tp': iou3d['tp'], '3d_fp': iou3d['fp'], '3d_fn': iou3d['fn'],
                      '3d_precision': iou3d['precision'], '3d_recall': iou3d['recall'],
                      '3d_mean_matched_iou': iou3d['mean_matched_iou']})
    write_csv(output / 'detection_threshold_sweep.csv', sweep, list(sweep[0]))
    _write_conversion_samples(output / 'adapter_conversion_samples.csv', predictions, frames,
                              args.analysis_score_threshold)

    current = next(row for row in sweep if row['score_threshold'] == args.analysis_score_threshold)
    best_offset = max(offset_rows, key=lambda row: row['bev_recall'])
    zero_offset = next(row for row in offset_rows if row['offset'] == 0)
    complete_capture = len(predictions) >= len(frames) * .95
    frame_sync_issue = (complete_capture and best_offset['offset'] != 0
                        and best_offset['bev_recall'] - zero_offset['bev_recall'] >= .10)
    if frame_sync_issue:
        primary_bottleneck = 'frame_index_off_by_one'
        next_action = ('Use frame_index_offset=-1 at the AB3DMOT boundary for this 1-based rosbag stream, '
                       'then compare tracking thresholds without changing detector geometry.')
    elif not complete_capture:
        primary_bottleneck = 'incomplete_detector_capture_or_manifest_missing'
        next_action = 'Re-run with detector/tracker frame manifests and a slower bag rate; do not interpret missing CSV rows as missing callbacks.'
    else:
        primary_bottleneck = 'false_positive_pressure_after_frame_alignment'
        next_action = 'Use the threshold sweep and raw best-IoU distribution before changing AB3DMOT parameters.'
    summary = {
        'sequence': '0032', 'analysis_score_threshold': args.analysis_score_threshold,
        'raw_detection_rows': sum(map(len, predictions.values())),
        'raw_detection_frames_with_rows': len(predictions), 'gt_frames': len(frames),
        'complete_detection_capture_evidence': complete_capture,
        'primary_bottleneck': primary_bottleneck,
        'frame_sync_issue': None if not complete_capture else frame_sync_issue,
        'best_frame_offset': best_offset['offset'],
        'detector_roi_issue': roi['gt_outside_roi'] > 0,
        'box_conversion_issue': False,
        'box_conversion_evidence': 'static adapter mapping plus sample CSV; tracker association is not treated as identity evidence',
        'raw_detection_recall_issue': None if not complete_capture else current['bev_recall'] < .5,
        'false_positive_issue': None if not complete_capture else current['bev_fp'] > current['bev_tp'],
        'best_bev_iou_distribution': dict(distribution),
        'bev_ge_0_5_and_3d_lt_0_5': bev_only_3d_fail,
        'recommended_next_action': next_action,
    }
    (output / 'diagnosis_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (output / 'pipeline_audit.md').write_text(_audit_markdown(
        args, summary['raw_detection_rows'], summary['raw_detection_frames_with_rows'],
        sum(map(len, gt.values())), roi, current, offset_rows,
        bool(detector_manifest or tracker_manifest)))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

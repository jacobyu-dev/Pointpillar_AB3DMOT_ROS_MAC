#!/usr/bin/env python3
"""Evaluate raw PointPillars detections against KITTI Raw tracklets."""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.detection_metrics import evaluate_detections
from evaluation.diagnostics import write_csv
from evaluation.geometry.iou3d import oriented_iou3d, oriented_iou_bev
from evaluation.io.detection_csv_loader import load_detection_csv
from evaluation.io.kitti_tracklet_loader import load_tracklets_pointpillars


PR_FIELDS = [
    'rank', 'frame_idx', 'detection_index', 'score', 'tp', 'fp',
    'cumulative_tp', 'cumulative_fp', 'precision', 'recall', 'matched_gt_id',
    'matched_iou',
]


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Evaluate confidence-scored, untracked PointPillars detections.')
    parser.add_argument('--tracklets', required=True)
    parser.add_argument('--predictions', required=True,
                        help='CSV from lidar_point_pillars_onnx_node.py _detections_csv')
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--experiment', default='pointpillars_raw')
    parser.add_argument('--class-name', default='Car')
    parser.add_argument('--iou-threshold', type=float, default=0.5)
    parser.add_argument('--metric', choices=('3d', 'bev', 'both'), default='both')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    if not 0.0 <= args.iou_threshold <= 1.0:
        parser.error('--iou-threshold must be within [0, 1]')

    gt = load_tracklets_pointpillars(args.tracklets, args.class_name)
    predictions = load_detection_csv(args.predictions, args.class_name)
    if not gt:
        parser.error('no ground-truth boxes found for the selected class')
    frame_indices = range(min(gt), max(gt) + 1)
    allowed_frames = set(frame_indices)
    excluded_frames = sorted(set(predictions) - allowed_frames)
    excluded_detections = sum(len(predictions[frame]) for frame in excluded_frames)

    summary = {
        'experiment': args.experiment, 'sequence': args.sequence,
        'class_name': args.class_name, 'iou_threshold': args.iou_threshold,
        'coordinate_convention': 'pointpillars_ros_velodyne',
        'frame_start': min(frame_indices), 'frame_end': max(frame_indices),
        'num_frames': len(frame_indices), 'metric_mode': args.metric,
        'excluded_prediction_frames': excluded_frames,
        'excluded_prediction_detections': excluded_detections,
    }
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    modes = []
    if args.metric in ('3d', 'both'):
        modes.append(('3D', oriented_iou3d, '3d_precision_recall.csv'))
    if args.metric in ('bev', 'both'):
        modes.append(('BEV', oriented_iou_bev, 'bev_precision_recall.csv'))
    for suffix, similarity, filename in modes:
        metrics, rows = evaluate_detections(
            gt, predictions, args.iou_threshold, similarity, frame_indices=frame_indices)
        summary.update({f'{key}_{suffix}': value for key, value in metrics.items()})
        write_csv(output_dir / filename, rows, PR_FIELDS)
    (output_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (output_dir / 'summary.txt').write_text(
        '\n'.join(f'{key}: {value}' for key, value in summary.items()) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

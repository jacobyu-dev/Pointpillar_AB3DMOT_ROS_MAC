#!/usr/bin/env python3
"""Evaluate AB3DMOT CSV output against KITTI Raw tracklets using 3D IoU."""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.diagnostics import build_diagnostics, write_csv
from evaluation.geometry.iou3d import compute_bev_iou_matrix
from evaluation.io.kitti_tracklet_loader import load_tracklets
from evaluation.io.tracker_csv_loader import load_tracker_csv
from evaluation.trackeval_adapter import build_sequence_data, evaluate_trackeval


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tracklets', required=True)
    parser.add_argument('--predictions', required=True)
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--experiment', default='baseline')
    parser.add_argument('--class-name', default='Car')
    parser.add_argument('--iou-threshold', type=float, default=0.5)
    parser.add_argument('--metric', choices=('3d', 'bev', 'both'), default='both',
                        help='Similarity used for tracking metrics. "both" writes 3D and BEV results.')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--trackeval-root', default='third_party/TrackEval')
    args = parser.parse_args()
    if not 0.0 <= args.iou_threshold <= 1.0:
        parser.error('--iou-threshold must be within [0, 1]')

    gt = load_tracklets(args.tracklets, args.class_name)
    predictions = load_tracker_csv(args.predictions, args.class_name)
    # 3D remains the legacy default report; BEV is added for the perception
    # comparison where height localisation should not affect association.
    data = build_sequence_data(gt, predictions)
    metrics = {}
    frame_rows = switch_rows = track_rows = []
    if args.metric in ('3d', 'both'):
        metrics.update(evaluate_trackeval(data, args.trackeval_root, args.iou_threshold, '3D'))
        frame_rows, switch_rows, track_rows = build_diagnostics(gt, predictions, args.iou_threshold)
        matched_iou_sum = sum(row['mean_matched_iou'] * row['tp'] for row in frame_rows)
        matched_count = sum(row['tp'] for row in frame_rows)
        metrics['mean_matched_3d_iou'] = matched_iou_sum / matched_count if matched_count else 0.0
    if args.metric in ('bev', 'both'):
        bev_data = build_sequence_data(gt, predictions, compute_bev_iou_matrix)
        metrics.update(evaluate_trackeval(bev_data, args.trackeval_root, args.iou_threshold, 'BEV'))
        bev_frames, bev_switches, bev_tracks = build_diagnostics(
            gt, predictions, args.iou_threshold, compute_bev_iou_matrix)
        bev_iou_sum = sum(row['mean_matched_iou'] * row['tp'] for row in bev_frames)
        bev_match_count = sum(row['tp'] for row in bev_frames)
        metrics['mean_matched_bev_iou'] = bev_iou_sum / bev_match_count if bev_match_count else 0.0
    summary = {
        'experiment': args.experiment, 'sequence': args.sequence, 'class_name': args.class_name,
        'iou_threshold': args.iou_threshold, 'coordinate_convention': 'legacy_tracker_rviz_velodyne',
        'num_frames': data['num_timesteps'], 'gt_detections': data['num_gt_dets'],
        'prediction_detections': data['num_tracker_dets'], 'gt_tracks': data['num_gt_ids'],
        'prediction_tracks': data['num_tracker_ids'], 'metric_mode': args.metric,
        **metrics,
    }
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    lines = [f'{key}: {value}' for key, value in summary.items()]
    (output_dir / 'summary.txt').write_text('\n'.join(lines) + '\n')
    fields = ['frame_idx', 'num_gt', 'num_predictions', 'tp', 'fp', 'fn', 'id_switches',
              'mean_matched_iou', 'min_matched_iou', 'max_matched_iou']
    switch_fields = ['frame_idx', 'gt_id', 'previous_tracker_id', 'current_tracker_id', 'previous_iou', 'current_iou']
    track_fields = ['gt_id', 'class_name', 'first_frame', 'last_frame', 'num_gt_frames', 'num_matched_frames',
                    'num_missed_frames', 'num_id_switches', 'mean_iou', 'min_iou', 'matched_tracker_ids']
    if args.metric in ('3d', 'both'):
        write_csv(output_dir / 'frame_metrics.csv', frame_rows, fields)
        write_csv(output_dir / 'id_switches.csv', switch_rows, switch_fields)
        write_csv(output_dir / 'track_metrics.csv', track_rows, track_fields)
    if args.metric in ('bev', 'both'):
        write_csv(output_dir / 'bev_frame_metrics.csv', bev_frames, fields)
        write_csv(output_dir / 'bev_id_switches.csv', bev_switches, switch_fields)
        write_csv(output_dir / 'bev_track_metrics.csv', bev_tracks, track_fields)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

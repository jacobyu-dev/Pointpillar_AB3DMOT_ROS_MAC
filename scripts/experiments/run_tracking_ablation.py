#!/usr/bin/env python3
"""Run one reproducible PointPillars + AB3DMOT offline ablation.

The detector CSV is deliberately post-NMS and pre-publish-threshold.  A score
filter here therefore reproduces ``~publish_score_threshold`` without rerunning
PointPillars.  NMS alternatives still require their own detector CSV.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
TRACKER_SRC = ROOT / 'mot_kf_tracking' / 'src'
sys.path.insert(0, str(TRACKER_SRC))

from evaluation.detection_metrics import evaluate_detections
from evaluation.geometry.iou3d import oriented_iou3d
from evaluation.geometry.iou3d import oriented_iou_bev
from evaluation.io.detection_csv_loader import load_detection_csv
from evaluation.io.kitti_tracklet_loader import load_tracklets_pointpillars
from kalman_filter import KalmanBoxTracker
from model import AB3DMOT


TRACK_FIELDS = ('frame_idx', 'track_id', 'class_name', 'x', 'y', 'z', 'h', 'w', 'l', 'yaw')


def _filter_by_score(detections, threshold: float):
    return {
        frame: [box for box in boxes if float(box.score) >= threshold]
        for frame, boxes in detections.items()
    }


def _empty_dets():
    return {
        'dets': np.empty((0, 7), dtype=np.float64),
        'info': np.empty((0, 2), dtype=object),
    }


def track_raw_detections(detections, frame_indices, *, min_hits: int,
                         max_age: int, association_iou_threshold: float):
    """Use the same PointPillars-to-AB3DMOT box mapping as the ROS node."""
    KalmanBoxTracker.count = 0
    tracker = AB3DMOT(
        max_age=max_age,
        min_hits=min_hits,
        association_iou_threshold=association_iou_threshold,
    )
    rows = []
    for frame in frame_indices:
        boxes = detections.get(frame, [])
        if boxes:
            dets_all = {
                'dets': np.asarray([
                    [box.h, box.w, box.l, box.x, box.y, box.z, box.yaw]
                    for box in boxes
                ], dtype=np.float64),
                'info': np.asarray([[str(frame), box.class_name] for box in boxes], dtype=object),
            }
        else:
            dets_all = _empty_dets()
        active = tracker.update(dets_all)
        for track in active:
            # AB3DMOT returns h,w,l,x,y,z,yaw,id,frame,class[, score].
            rows.append({
                'frame_idx': frame,
                'track_id': int(track[7]),
                'class_name': str(track[9]),
                'x': float(track[3]), 'y': float(track[4]), 'z': float(track[5]),
                'h': float(track[0]), 'w': float(track[1]), 'l': float(track[2]),
                'yaw': float(track[6]),
            })
    return rows


def _write_csv(path: Path, rows, fields):
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--detections-csv', required=True)
    parser.add_argument('--tracklets', required=True)
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--experiment-name', required=True)
    parser.add_argument('--output-dir', required=True,
                        help='New directory for this one experiment; must not exist.')
    parser.add_argument('--class-name', default='Car')
    parser.add_argument('--score-threshold', type=float, required=True,
                        help='Offline equivalent of PointPillars publish_score_threshold.')
    parser.add_argument('--nms-threshold', type=float,
                        help='Metadata only: NMS has already been applied to this CSV.')
    parser.add_argument('--min-hits', type=int, default=2)
    parser.add_argument('--max-age', type=int, default=3)
    parser.add_argument('--association-threshold', type=float, default=0.01)
    parser.add_argument('--iou-threshold', type=float, default=0.5)
    args = parser.parse_args()

    if not 0.0 <= args.score_threshold <= 1.0:
        parser.error('--score-threshold must be within [0, 1]')
    if args.nms_threshold is not None and not 0.0 <= args.nms_threshold <= 1.0:
        parser.error('--nms-threshold must be within [0, 1]')
    if args.min_hits < 1 or args.max_age < 1:
        parser.error('--min-hits and --max-age must be positive')
    if not 0.0 <= args.association_threshold <= 1.0:
        parser.error('--association-threshold must be within [0, 1]')

    output_dir = Path(args.output_dir)
    if output_dir.exists():
        parser.error(f'output directory already exists; refusing to overwrite: {output_dir}')
    output_dir.mkdir(parents=True)

    gt = load_tracklets_pointpillars(args.tracklets, args.class_name)
    if not gt:
        parser.error('no selected-class GT boxes found')
    frames = range(min(gt), max(gt) + 1)
    raw = load_detection_csv(args.detections_csv, args.class_name)
    detections = _filter_by_score(raw, args.score_threshold)

    config = {
        'experiment': args.experiment_name,
        'sequence': args.sequence,
        'class_name': args.class_name,
        'source_detections_csv': str(Path(args.detections_csv).resolve()),
        'score_threshold_semantics': 'offline equivalent of publish_score_threshold',
        'score_threshold': args.score_threshold,
        'nms_threshold': args.nms_threshold,
        'min_hits': args.min_hits,
        'max_age': args.max_age,
        'association_iou_threshold': args.association_threshold,
        'association_iou_space': 'oriented_3d',
        'evaluation_gt_convention': 'pointpillars',
        'prediction_frame_offset': 0,
        'frame_start': min(frames), 'frame_end': max(frames),
    }
    (output_dir / 'config.json').write_text(json.dumps(config, indent=2) + '\n')

    detection_summary = {}
    for suffix, similarity in (('3D', oriented_iou3d), ('BEV', oriented_iou_bev)):
        metrics, _ = evaluate_detections(gt, detections, args.iou_threshold, similarity, frames)
        detection_summary.update({f'detection_{key}_{suffix}': value for key, value in metrics.items()})
    (output_dir / 'detection_summary.json').write_text(json.dumps(detection_summary, indent=2) + '\n')

    rows = track_raw_detections(
        detections, frames,
        min_hits=args.min_hits,
        max_age=args.max_age,
        association_iou_threshold=args.association_threshold,
    )
    tracks_csv = output_dir / 'tracks.csv'
    _write_csv(tracks_csv, rows, TRACK_FIELDS)

    evaluation_dir = output_dir / 'evaluation'
    command = [
        sys.executable, str(ROOT / 'scripts' / 'evaluate_kitti_3d_tracking.py'),
        '--tracklets', args.tracklets,
        '--predictions', str(tracks_csv),
        '--sequence', args.sequence,
        '--experiment', args.experiment_name,
        '--class-name', args.class_name,
        '--gt-convention', 'pointpillars',
        '--iou-threshold', str(args.iou_threshold),
        '--metric', 'both',
        '--output-dir', str(evaluation_dir),
    ]
    subprocess.run(command, check=True)
    tracking_summary = json.loads((evaluation_dir / 'summary.json').read_text())
    summary = {**config, **detection_summary, **tracking_summary}
    (output_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    _write_csv(output_dir / 'tracking_metrics.csv', [summary], tuple(summary))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

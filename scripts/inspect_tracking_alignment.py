#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.geometry.iou3d import compute_iou_matrix
from evaluation.io.kitti_tracklet_loader import load_tracklets
from evaluation.io.tracker_csv_loader import load_tracker_csv
from evaluation.matching import match_iou_matrix


def show(name, boxes):
    print(name)
    for box in boxes:
        print(f'  ID {box.track_id} {box.class_name}: center=({box.x:.3f}, {box.y:.3f}, {box.z:.3f}) '
              f'hwl=({box.h:.3f}, {box.w:.3f}, {box.l:.3f}) yaw={box.yaw:.3f}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tracklets', required=True); parser.add_argument('--predictions', required=True)
    parser.add_argument('--frame', type=int, required=True); parser.add_argument('--class-name', default='Car')
    parser.add_argument('--iou-threshold', type=float, default=0.5)
    args = parser.parse_args()
    gt = load_tracklets(args.tracklets, args.class_name).get(args.frame, [])
    predictions = load_tracker_csv(args.predictions, args.class_name).get(args.frame, [])
    matrix = compute_iou_matrix(gt, predictions)
    matches, unmatched_gt, unmatched_predictions = match_iou_matrix(matrix, args.iou_threshold)
    print(f'Frame {args.frame}\n'); show('GT', gt); show('\nPredictions', predictions)
    print('\n3D IoU matrix\n', matrix)
    print('\nMatches')
    for match in matches:
        print(f'  GT {gt[match.gt_index].track_id} -> Tracker {predictions[match.prediction_index].track_id}: {match.iou:.4f}')
    print('Unmatched GT:', [gt[index].track_id for index in unmatched_gt])
    print('Unmatched predictions:', [predictions[index].track_id for index in unmatched_predictions])


if __name__ == '__main__':
    main()

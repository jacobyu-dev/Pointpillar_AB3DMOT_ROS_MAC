#!/usr/bin/env python3
"""Deterministically reproduce the node's AB3DMOT CSV without ROS playback.

This is useful for repeatable baseline evaluation. It calls the same
``readXML`` transformation and the same ``AB3DMOT.update`` implementation as
the node, once for every integer KITTI frame. Runtime ROS exports can instead
be created with the node's ``~tracks_csv`` parameter.
"""

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TRACKER_SRC = ROOT / 'mot_kf_tracking' / 'src'
sys.path.insert(0, str(TRACKER_SRC))
from kalman_filter import KalmanBoxTracker
from model import AB3DMOT
from mot_ab3dmot_track_node import readXML


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tracklets', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    detections = readXML(args.tracklets)
    max_frame = max(map(int, detections[:, 0]))
    tracker = AB3DMOT()
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=(
            'frame_idx', 'track_id', 'class_name', 'x', 'y', 'z', 'h', 'w', 'l', 'yaw'))
        writer.writeheader()
        for frame in range(max_frame + 1):
            if frame == 0:
                tracker.__init__(); KalmanBoxTracker.count = 0
            frame_rows = detections[detections[:, 0] == str(frame)]
            bboxinfo = frame_rows[:, 2:9].astype(np.float64)
            info = frame_rows[:, 0:2]
            # Exact node boundary: [tx,ty,tz,h,w,l,ry] -> [h,w,l,x,y,z,theta].
            result = tracker.update({'dets': bboxinfo[:, [3, 4, 5, 0, 1, 2, 6]], 'info': info})
            for row in result:
                h, w, l, camera_x, camera_y, camera_z, camera_yaw, track_id, _, class_name = row[:10]
                # Exact node AB3DMOT-output -> RViz pose conversion.
                x, y, z = float(camera_z) + 1.3, -float(camera_x), -float(camera_y)
                yaw = -float(camera_yaw) + np.pi / 2.0
                writer.writerow({'frame_idx': frame, 'track_id': int(track_id), 'class_name': class_name,
                                 'x': x, 'y': y, 'z': z / 2.0, 'h': h, 'w': w, 'l': l, 'yaw': yaw})
    print(f'Wrote {output}')


if __name__ == '__main__':
    main()

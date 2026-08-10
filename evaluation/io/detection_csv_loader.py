"""Load confidence-scored, untracked 3D detector outputs."""

import csv
from collections import defaultdict
from pathlib import Path

from evaluation.models import Box3D


REQUIRED_COLUMNS = {
    'frame_idx', 'class_name', 'x', 'y', 'z', 'h', 'w', 'l', 'yaw', 'score',
}


def load_detection_csv(path: str | Path, class_name: str | None = None) -> dict[int, list[Box3D]]:
    """Load PointPillars post-NMS detections written by its ROS node.

    ``detection_id`` is optional because it is only useful for tracing a row;
    raw detections deliberately have no temporal track identity.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(
            f'Detection CSV not found: {path}\n'
            'Run lidar_point_pillars_onnx_node.py with _detections_csv:=<path> '
            'while replaying the bag.'
        )
    frames: dict[int, list[Box3D]] = defaultdict(list)
    with path.open(newline='') as handle:
        reader = csv.DictReader(handle)
        missing = REQUIRED_COLUMNS - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f'Detection CSV missing columns: {sorted(missing)}')
        for row_index, row in enumerate(reader):
            if class_name is not None and row['class_name'] != class_name:
                continue
            frame_idx = int(row['frame_idx'])
            # The ID only makes every row distinct inside Box3D.  It has no
            # temporal meaning and is never used by detection metrics.
            detection_id = int(row.get('detection_id') or row_index)
            frames[frame_idx].append(Box3D(
                frame_idx=frame_idx, track_id=detection_id,
                class_name=row['class_name'], x=float(row['x']), y=float(row['y']),
                z=float(row['z']), h=float(row['h']), w=float(row['w']),
                l=float(row['l']), yaw=float(row['yaw']), score=float(row['score']),
            ))
    return dict(frames)

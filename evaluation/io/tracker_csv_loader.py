import csv
from collections import defaultdict
from pathlib import Path

from evaluation.models import Box3D


REQUIRED_COLUMNS = {'frame_idx', 'track_id', 'class_name', 'x', 'y', 'z', 'h', 'w', 'l', 'yaw'}


def load_tracker_csv(path: str | Path, class_name: str | None = None) -> dict[int, list[Box3D]]:
    """Load active-track RViz-pose CSV rows produced by the tracking node."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(
            f'Prediction CSV not found: {path}\n'
            'Create it first with scripts/export_baseline_tracks_offline.py, or run '
            'mot_ab3dmot_track_node.py with _tracks_csv:=<path> during bag playback.'
        )
    frames: dict[int, list[Box3D]] = defaultdict(list)
    with path.open(newline='') as handle:
        reader = csv.DictReader(handle)
        missing = REQUIRED_COLUMNS - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f'Prediction CSV missing columns: {sorted(missing)}')
        for row in reader:
            if class_name is not None and row['class_name'] != class_name:
                continue
            frames[int(row['frame_idx'])].append(Box3D(
                frame_idx=int(row['frame_idx']), track_id=int(row['track_id']),
                class_name=row['class_name'], x=float(row['x']), y=float(row['y']),
                z=float(row['z']), h=float(row['h']), w=float(row['w']),
                l=float(row['l']), yaw=float(row['yaw']),
                score=float(row['score']) if row.get('score') else None,
            ))
    return dict(frames)

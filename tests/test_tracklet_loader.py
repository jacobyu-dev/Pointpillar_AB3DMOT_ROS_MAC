from pathlib import Path

from evaluation.io.kitti_tracklet_loader import load_tracklets


def test_0032_loader_preserves_stable_tracklet_ids():
    root = Path(__file__).resolve().parents[1]
    tracklets = root / 'data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml'
    frames = load_tracklets(tracklets, 'Car')
    assert min(frames) == 0
    assert len({box.track_id for boxes in frames.values() for box in boxes}) == 21
    repeated_ids = [box.track_id for box in frames[0] if any(
        box.track_id == later.track_id for later in frames.get(1, []))]
    assert repeated_ids

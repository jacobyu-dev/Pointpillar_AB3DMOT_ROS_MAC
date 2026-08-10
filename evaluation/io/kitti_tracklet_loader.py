"""Load KITTI Raw tracklets using the project's established RViz convention."""

from collections import defaultdict
from pathlib import Path
import math
import xml.etree.ElementTree as ET

from evaluation.models import Box3D


def raw_pose_to_legacy_rviz_box(
    *, frame_idx: int, track_id: int, class_name: str, tx: float, ty: float,
    tz: float, h: float, w: float, l: float, rz: float,
) -> Box3D:
    """Reproduce the existing node's GT-to-RViz boundary conversion.

    The node first converts raw tracklet poses to its camera-like AB3DMOT
    representation (``[-ty, -tz, tx - 0.27, ..., -rz + pi/2]``), then maps
    tracker output back before publishing.  Its RViz marker consequently uses
    ``x=tx+1.03``, ``y=ty``, centre ``z=tz/2``, and yaw ``rz``.  This helper
    deliberately preserves that verified legacy behaviour rather than
    introducing a new calibration convention for evaluation.
    """
    return Box3D(
        frame_idx=frame_idx,
        track_id=track_id,
        class_name=class_name,
        x=tx + 1.03,
        y=ty,
        z=tz / 2.0,
        h=h,
        w=w,
        l=l,
        yaw=rz,
    )


def raw_pose_to_pointpillars_box(
    *, frame_idx: int, track_id: int, class_name: str, tx: float, ty: float,
    tz: float, h: float, w: float, l: float, rz: float,
) -> Box3D:
    """Convert a KITTI Raw tracklet to the native PointPillars LiDAR frame.

    KITTI Raw stores a tracklet pose at the bottom centre of its 3D box.  The
    PointPillars ROS message instead uses its geometric centre.  Unlike
    :func:`raw_pose_to_legacy_rviz_box`, this path intentionally applies no
    tracker/RViz compatibility offset.
    """
    return Box3D(
        frame_idx=frame_idx, track_id=track_id, class_name=class_name,
        x=tx, y=ty, z=tz + h / 2.0, h=h, w=w, l=l, yaw=rz,
    )


def load_tracklets(path: str | Path, class_name: str | None = None) -> dict[int, list[Box3D]]:
    """Return frame-indexed GT boxes with stable XML-item IDs (starting at 1)."""
    root = ET.parse(Path(path)).getroot()
    frames: dict[int, list[Box3D]] = defaultdict(list)
    for tracklet_idx, item in enumerate(root.findall('./tracklets/item'), start=1):
        object_class = item.findtext('objectType')
        if class_name is not None and object_class != class_name:
            continue
        h, w, l = (float(item.findtext(field)) for field in ('h', 'w', 'l'))
        first_frame = int(item.findtext('first_frame'))
        for offset, pose in enumerate(item.findall('./poses/item')):
            frames[first_frame + offset].append(raw_pose_to_legacy_rviz_box(
                frame_idx=first_frame + offset,
                track_id=tracklet_idx,
                class_name=object_class,
                tx=float(pose.findtext('tx')),
                ty=float(pose.findtext('ty')),
                tz=float(pose.findtext('tz')),
                h=h, w=w, l=l, rz=float(pose.findtext('rz')),
            ))
    return dict(frames)


def load_tracklets_pointpillars(path: str | Path, class_name: str | None = None) -> dict[int, list[Box3D]]:
    """Return GT in the raw detector's native centre-Z Velodyne convention."""
    root = ET.parse(Path(path)).getroot()
    frames: dict[int, list[Box3D]] = defaultdict(list)
    for tracklet_idx, item in enumerate(root.findall('./tracklets/item'), start=1):
        object_class = item.findtext('objectType')
        if class_name is not None and object_class != class_name:
            continue
        h, w, l = (float(item.findtext(field)) for field in ('h', 'w', 'l'))
        first_frame = int(item.findtext('first_frame'))
        for offset, pose in enumerate(item.findall('./poses/item')):
            frames[first_frame + offset].append(raw_pose_to_pointpillars_box(
                frame_idx=first_frame + offset, track_id=tracklet_idx,
                class_name=object_class, tx=float(pose.findtext('tx')),
                ty=float(pose.findtext('ty')), tz=float(pose.findtext('tz')),
                h=h, w=w, l=l, rz=float(pose.findtext('rz')),
            ))
    return dict(frames)

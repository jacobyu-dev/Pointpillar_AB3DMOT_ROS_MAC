from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Box3D:
    """A yawed 3D box in the legacy tracker RViz/Velodyne frame.

    ``x, y, z`` is the box centre used by RViz markers.  ``yaw`` is a
    right-handed rotation around the Velodyne +Z axis.
    """

    frame_idx: int
    track_id: int
    class_name: str
    x: float
    y: float
    z: float
    h: float
    w: float
    l: float
    yaw: float
    score: Optional[float] = None

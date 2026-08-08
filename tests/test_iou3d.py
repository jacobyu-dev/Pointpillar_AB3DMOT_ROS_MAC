from evaluation.geometry.iou3d import oriented_iou3d
from evaluation.models import Box3D


def box(**overrides):
    values = dict(frame_idx=0, track_id=1, class_name='Car', x=0., y=0., z=0., h=2., w=2., l=4., yaw=0.)
    values.update(overrides)
    return Box3D(**values)


def test_identical_boxes_have_unit_iou():
    assert oriented_iou3d(box(), box(track_id=2)) == 1.0


def test_non_overlapping_boxes_have_zero_iou():
    assert oriented_iou3d(box(), box(track_id=2, x=10.0)) == 0.0


def test_yaw_changes_iou_for_non_square_box():
    value = oriented_iou3d(box(), box(track_id=2, yaw=1.5707963267948966))
    assert 0.0 < value < 1.0

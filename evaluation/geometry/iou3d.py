"""Oriented 3D IoU for boxes whose vertical axis is +Z."""

import math
import numpy as np

from evaluation.models import Box3D


def _rectangle(box: Box3D) -> list[tuple[float, float]]:
    half_l, half_w = box.l / 2.0, box.w / 2.0
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    # Counter-clockwise local corners are required by Sutherland-Hodgman.
    local = ((half_l, half_w), (-half_l, half_w), (-half_l, -half_w), (half_l, -half_w))
    return [(box.x + c * x - s * y, box.y + s * x + c * y) for x, y in local]


def _inside(point, edge_start, edge_end) -> bool:
    return ((edge_end[0] - edge_start[0]) * (point[1] - edge_start[1])
            - (edge_end[1] - edge_start[1]) * (point[0] - edge_start[0])) >= -1e-12


def _intersection(start, end, clip_start, clip_end):
    dx1, dy1 = end[0] - start[0], end[1] - start[1]
    dx2, dy2 = clip_end[0] - clip_start[0], clip_end[1] - clip_start[1]
    denominator = dx1 * dy2 - dy1 * dx2
    if abs(denominator) < 1e-12:
        return end
    t = ((clip_start[0] - start[0]) * dy2 - (clip_start[1] - start[1]) * dx2) / denominator
    return start[0] + t * dx1, start[1] + t * dy1


def _clip(subject, clipper):
    output = list(subject)
    for index, edge_start in enumerate(clipper):
        edge_end = clipper[(index + 1) % len(clipper)]
        input_points, output = output, []
        if not input_points:
            break
        start = input_points[-1]
        for end in input_points:
            end_inside = _inside(end, edge_start, edge_end)
            start_inside = _inside(start, edge_start, edge_end)
            if end_inside:
                if not start_inside:
                    output.append(_intersection(start, end, edge_start, edge_end))
                output.append(end)
            elif start_inside:
                output.append(_intersection(start, end, edge_start, edge_end))
            start = end
    return output


def _area(points) -> float:
    if len(points) < 3:
        return 0.0
    arr = np.asarray(points, dtype=float)
    return abs(float(np.dot(arr[:, 0], np.roll(arr[:, 1], -1)) - np.dot(arr[:, 1], np.roll(arr[:, 0], -1)))) / 2.0


def oriented_iou3d(first: Box3D, second: Box3D) -> float:
    """Return yaw-aware 3D IoU in [0, 1]."""
    intersection_area = _area(_clip(_rectangle(first), _rectangle(second)))
    first_bottom, first_top = first.z - first.h / 2.0, first.z + first.h / 2.0
    second_bottom, second_top = second.z - second.h / 2.0, second.z + second.h / 2.0
    intersection_height = max(0.0, min(first_top, second_top) - max(first_bottom, second_bottom))
    intersection = intersection_area * intersection_height
    union = first.h * first.w * first.l + second.h * second.w * second.l - intersection
    return 0.0 if union <= 0.0 else max(0.0, min(1.0, intersection / union))


def oriented_iou_bev(first: Box3D, second: Box3D) -> float:
    """Return yaw-aware bird's-eye-view IoU, ignoring vertical overlap."""
    intersection = _area(_clip(_rectangle(first), _rectangle(second)))
    union = first.w * first.l + second.w * second.l - intersection
    return 0.0 if union <= 0.0 else max(0.0, min(1.0, intersection / union))


def compute_iou_matrix(gt_boxes: list[Box3D], prediction_boxes: list[Box3D]) -> np.ndarray:
    matrix = np.zeros((len(gt_boxes), len(prediction_boxes)), dtype=float)
    for gt_idx, gt_box in enumerate(gt_boxes):
        for prediction_idx, prediction_box in enumerate(prediction_boxes):
            matrix[gt_idx, prediction_idx] = oriented_iou3d(gt_box, prediction_box)
    return matrix


def compute_bev_iou_matrix(gt_boxes: list[Box3D], prediction_boxes: list[Box3D]) -> np.ndarray:
    """Return an oriented BEV-IoU similarity matrix for TrackEval."""
    matrix = np.zeros((len(gt_boxes), len(prediction_boxes)), dtype=float)
    for gt_idx, gt_box in enumerate(gt_boxes):
        for prediction_idx, prediction_box in enumerate(prediction_boxes):
            matrix[gt_idx, prediction_idx] = oriented_iou_bev(gt_box, prediction_box)
    return matrix

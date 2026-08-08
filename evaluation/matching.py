from dataclasses import dataclass
import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass(frozen=True)
class Match:
    gt_index: int
    prediction_index: int
    iou: float


def match_iou_matrix(iou_matrix: np.ndarray, threshold: float) -> tuple[list[Match], list[int], list[int]]:
    """Hungarian matching with an explicit minimum oriented-3D-IoU gate."""
    gt_count, prediction_count = iou_matrix.shape
    if gt_count == 0 or prediction_count == 0:
        return [], list(range(gt_count)), list(range(prediction_count))
    rows, columns = linear_sum_assignment(-iou_matrix)
    matches = [Match(int(row), int(column), float(iou_matrix[row, column]))
               for row, column in zip(rows, columns) if iou_matrix[row, column] >= threshold]
    matched_gt = {match.gt_index for match in matches}
    matched_predictions = {match.prediction_index for match in matches}
    return (matches, [idx for idx in range(gt_count) if idx not in matched_gt],
            [idx for idx in range(prediction_count) if idx not in matched_predictions])

"""Adapter that supplies oriented-3D similarities to unmodified TrackEval metrics."""

from pathlib import Path
import sys
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import numpy as np

from evaluation.geometry.iou3d import compute_iou_matrix
from evaluation.models import Box3D


def _import_trackeval(trackeval_root: str | Path):
    root = str(Path(trackeval_root).resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    # Current TrackEval still refers to aliases removed in NumPy 2.  This is a
    # process-local compatibility shim; third_party metric sources stay intact.
    if not hasattr(np, 'float'):
        np.float = float
    if not hasattr(np, 'int'):
        np.int = int
    # TrackEval imports optional dataset adapters at package import time. Some
    # optional datasets warn about unavailable packages (e.g. pycocotools),
    # although HOTA/CLEAR/Identity themselves are available.
    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
        import trackeval
    return trackeval


def build_sequence_data(gt_by_frame: dict[int, list[Box3D]], prediction_by_frame: dict[int, list[Box3D]],
                        similarity_fn=compute_iou_matrix, frame_indices=None) -> dict:
    """Build the TrackEval contract over an explicit, fixed frame set when given."""
    frames = sorted(frame_indices) if frame_indices is not None else sorted(set(gt_by_frame) | set(prediction_by_frame))
    gt_raw_ids = sorted({box.track_id for frame in frames for box in gt_by_frame.get(frame, [])})
    tracker_raw_ids = sorted({box.track_id for frame in frames for box in prediction_by_frame.get(frame, [])})
    gt_id_map = {track_id: index for index, track_id in enumerate(gt_raw_ids)}
    tracker_id_map = {track_id: index for index, track_id in enumerate(tracker_raw_ids)}
    gt_ids, tracker_ids, similarities = [], [], []
    for frame in frames:
        gt_boxes, prediction_boxes = gt_by_frame.get(frame, []), prediction_by_frame.get(frame, [])
        gt_ids.append(np.asarray([gt_id_map[box.track_id] for box in gt_boxes], dtype=int))
        tracker_ids.append(np.asarray([tracker_id_map[box.track_id] for box in prediction_boxes], dtype=int))
        similarities.append(similarity_fn(gt_boxes, prediction_boxes))
    result = {
        'frames': frames,
        'gt_ids': gt_ids,
        'tracker_ids': tracker_ids,
        'similarity_scores': similarities,
        'num_gt_ids': len(gt_raw_ids),
        'num_tracker_ids': len(tracker_raw_ids),
        'num_gt_dets': sum(len(ids) for ids in gt_ids),
        'num_tracker_dets': sum(len(ids) for ids in tracker_ids),
        'num_timesteps': len(frames),
    }
    return result


def evaluate_trackeval(data: dict, trackeval_root: str | Path, iou_threshold: float,
                       metric_suffix: str = '3D') -> dict:
    """Evaluate unmodified TrackEval HOTA, CLEAR and Identity implementations."""
    trackeval = _import_trackeval(trackeval_root)
    hota = trackeval.metrics.HOTA().eval_sequence(data)
    clear = trackeval.metrics.CLEAR({'THRESHOLD': iou_threshold, 'PRINT_CONFIG': False}).eval_sequence(data)
    identity = trackeval.metrics.Identity({'THRESHOLD': iou_threshold, 'PRINT_CONFIG': False}).eval_sequence(data)
    # HOTA values are averaged across TrackEval's alpha grid (0.05..0.95).
    result = {
        f'HOTA_{metric_suffix}': float(np.mean(hota['HOTA'])),
        f'DetA_{metric_suffix}': float(np.mean(hota['DetA'])),
        f'AssA_{metric_suffix}': float(np.mean(hota['AssA'])),
        f'LocA_{metric_suffix}': float(np.mean(hota['LocA'])),
        f'DetRe_{metric_suffix}': float(np.mean(hota['DetRe'])),
        f'DetPr_{metric_suffix}': float(np.mean(hota['DetPr'])),
        f'AssRe_{metric_suffix}': float(np.mean(hota['AssRe'])),
        f'AssPr_{metric_suffix}': float(np.mean(hota['AssPr'])),
        f'MOTA_{metric_suffix}': float(clear['MOTA']), f'MOTP_{metric_suffix}': float(clear['MOTP']),
        f'FP_{metric_suffix}': int(clear['CLR_FP']), f'FN_{metric_suffix}': int(clear['CLR_FN']),
        f'IDSW_{metric_suffix}': int(clear['IDSW']), f'Frag_{metric_suffix}': int(clear['Frag']),
        f'IDF1_{metric_suffix}': float(identity['IDF1']), f'IDP_{metric_suffix}': float(identity['IDP']),
        f'IDR_{metric_suffix}': float(identity['IDR']),
    }
    # Preserve the original public result keys for existing baseline scripts.
    if metric_suffix == '3D':
        result.update({
            'MOTA': result['MOTA_3D'], 'MOTP': result['MOTP_3D'],
            'FP': result['FP_3D'], 'FN': result['FN_3D'],
            'IDSW': result['IDSW_3D'], 'Frag': result['Frag_3D'],
            'IDF1': result['IDF1_3D'], 'IDP': result['IDP_3D'], 'IDR': result['IDR_3D'],
        })
    return result

#!/usr/bin/env python3
"""Collect one-row experiment summaries without overwriting source results."""

from __future__ import annotations

import argparse
import csv
import glob
import json
from pathlib import Path


DEFAULT_FIELDS = (
    'score_threshold', 'nms_threshold', 'min_hits', 'max_age',
    'association_iou_threshold', 'detection_prediction_detections_BEV',
    'detection_precision_BEV', 'detection_recall_BEV', 'prediction_detections',
    'prediction_tracks', 'HOTA_BEV', 'DetA_BEV', 'AssA_BEV', 'DetRe_BEV',
    'DetPr_BEV', 'IDF1_BEV', 'MOTA_BEV', 'FP_BEV', 'FN_BEV', 'IDSW_BEV', 'Frag_BEV',
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-glob', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    paths = sorted(Path(item) for item in glob.glob(args.input_glob))
    if not paths:
        parser.error(f'no summary files matched: {args.input_glob}')
    output = Path(args.output)
    if output.exists():
        parser.error(f'refusing to overwrite: {output}')
    rows = []
    for path in paths:
        summary = json.loads(path.read_text())
        rows.append({'experiment': summary.get('experiment', path.parent.name),
                     **{field: summary.get(field, '') for field in DEFAULT_FIELDS}})
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=('experiment', *DEFAULT_FIELDS))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == '__main__':
    main()

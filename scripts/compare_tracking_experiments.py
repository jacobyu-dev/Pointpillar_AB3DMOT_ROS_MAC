#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


FIELDS = (
    'HOTA_3D', 'AssA_3D', 'DetA_3D', 'IDF1', 'MOTA', 'IDSW', 'FP', 'FN', 'mean_matched_3d_iou',
    'HOTA_BEV', 'AssA_BEV', 'DetA_BEV', 'IDF1_BEV', 'MOTA_BEV', 'IDSW_BEV',
    'FP_BEV', 'FN_BEV', 'mean_matched_bev_iou',
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('baseline'); parser.add_argument('experiment'); parser.add_argument('--output-csv')
    args = parser.parse_args()
    baseline = json.loads(Path(args.baseline).read_text()); experiment = json.loads(Path(args.experiment).read_text())
    missing = [field for field in FIELDS if field not in baseline or field not in experiment]
    if missing:
        parser.error('Both summaries must be generated with --metric both; missing: ' + ', '.join(missing))
    rows = []
    print(f"{'Metric':28} {'Baseline':>12} {'Experiment':>12} {'Delta':>12}")
    for field in FIELDS:
        before, after = baseline[field], experiment[field]
        row = {'metric': field, 'baseline': before, 'experiment': after, 'delta': after - before}; rows.append(row)
        print(f'{field:28} {before:12.4f} {after:12.4f} {after - before:+12.4f}')
    if args.output_csv:
        with Path(args.output_csv).open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


if __name__ == '__main__':
    main()

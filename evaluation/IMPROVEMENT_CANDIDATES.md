# Evidence-based future experiments

The completed 0009/0023/0032 ablation selected the PointPillars profile
`publish_score_threshold=0.30`, NMS `0.20`, `min_hits=3`, `max_age=2`, and
the existing oriented-3D-IoU association gate `0.01`. Do not retune those
parameters without a new multi-sequence experiment.

The remaining evidence-based bottleneck is detector recall: the final profile
substantially reduces false positives but increases false negatives, especially
on 0023. Future work should validate the PointPillars checkpoint or
training/configuration rather than continue tracker parameter sweeps.
- The legacy tracklet/RViz conversion includes a `-0.27` then `+1.3` x
  translation. It is preserved for baseline comparability. Alignment reports
  should be reviewed before replacing it with a calibration-derived transform.
- XML object order is now retained as GT identity only by the evaluator. The
  live tracker intentionally creates its own IDs, enabling ID-switch analysis.

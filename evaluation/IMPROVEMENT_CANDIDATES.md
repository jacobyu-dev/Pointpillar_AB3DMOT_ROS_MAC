# Evidence-based future experiments

- `AB3DMOT` currently uses `max_age=3` and `min_hits=2`; export/evaluate
  these choices before changing lifecycle behaviour.
- Association is Hungarian matching of oriented 3D IoU with a fixed `0.01`
  gate. The frame diagnostics can identify whether changing this gate is
  justified.
- The legacy tracklet/RViz conversion includes a `-0.27` then `+1.3` x
  translation. It is preserved for baseline comparability. Alignment reports
  should be reviewed before replacing it with a calibration-derived transform.
- XML object order is now retained as GT identity only by the evaluator. The
  live tracker intentionally creates its own IDs, enabling ID-switch analysis.

# Raw PointPillars 3D detection evaluation

`scripts/evaluate_kitti_3d_detection.py` evaluates the detector output before
AB3DMOT.  It does not consume a tracker CSV and does not report identity or
tracking metrics.

The PointPillars node writes its post-NMS detections when `~detections_csv` is
configured. For a meaningful AP curve, keep candidates down to a low score
threshold during playback. The command also uses the validated tracking NMS
and publish profile; `detections_csv` remains post-NMS and pre-publish filtering.

```zsh
python {project}/pointpillar_object_detection/lidar_point_pillars_onnx_node.py \
  _input_topic:=/kitti/velo/pointcloud \
  _output_topic:=/detection/lidar_detector/boxes \
  _score_threshold:=0.01 \
  _publish_score_threshold:=0.30 \
  _nms_overlap_threshold:=0.20 \
  _detections_csv:={project}/outputs/detections/pointpillars/0032_raw.csv
```

The tracker does not need to be running.  After replaying the full bag, run:

```zsh
python {project}/scripts/evaluate_kitti_3d_detection.py \
  --tracklets {project}/data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --predictions {project}/outputs/detections/pointpillars/0032_raw.csv \
  --sequence 0032 --experiment pointpillars_raw --class-name Car \
  --iou-threshold 0.5 --metric both \
  --output-dir {project}/outputs/evaluation/pointpillars_raw/0032
```

The result contains `summary.json`, plus score-ranked 3D and BEV
precision/recall CSVs.  `average_precision_3D` uses yaw-aware 3D IoU and
`average_precision_BEV` ignores height overlap.  The evaluator fixes the time
range to the GT sequence and lists late/early predictions separately.

Raw PointPillars boxes and its GT loader use native LiDAR `x/y`, centre `z`,
`h/w/l`, and yaw.  This intentionally differs from the tracking evaluator's
legacy RViz conversion, so the two evaluators must not share prediction CSVs.

# PointPillars pipeline diagnostics

Use a new output directory per replay. The detector manifest records every
PointCloud callback, including frames that produce zero detections; the tracker
manifest records every synchronized AB3DMOT update.

```zsh
# Detector
python pointpillar_object_detection/lidar_point_pillars_onnx_node.py \
  _input_topic:=/kitti/velo/pointcloud \
  _output_topic:=/detection/lidar_detector/boxes \
  _score_threshold:=0.01 \
  _publish_score_threshold:=0.30 \
  _nms_overlap_threshold:=0.20 \
  _detections_csv:={project}/outputs/detections/pointpillars/0032_run_001_raw.csv \
  _frame_pipeline_csv:={project}/outputs/analysis/0032/run_001/detector_frame_pipeline.csv

# Tracker: PointPillars must be the source; do not pass _tracklet_path.
python mot_kf_tracking/src/mot_ab3dmot_track_node.py \
  _detection_source:=pointpillars \
  _pointpillars_topic:=/detection/lidar_detector/boxes \
  _min_hits:=3 \
  _max_age:=2 \
  _association_iou_threshold:=0.01 \
  _tracks_csv:={project}/outputs/tracks/pointpillars/0032_run_001_tracks.csv \
  _frame_pipeline_csv:={project}/outputs/analysis/0032/run_001/tracker_frame_pipeline.csv
```

After both nodes print their ready messages, replay at a rate slow enough for
the detector to process all frames. Then run the offline analysis:

```zsh
python scripts/diagnostics/analyze_pointpillars_pipeline.py \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --detections outputs/detections/pointpillars/0032_run_001_raw.csv \
  --tracks outputs/tracks/pointpillars/0032_run_001_tracks.csv \
  --detector-manifest outputs/analysis/0032/run_001/detector_frame_pipeline.csv \
  --tracker-manifest outputs/analysis/0032/run_001/tracker_frame_pipeline.csv \
  --analysis-score-threshold 0.30 \
  --output-dir outputs/analysis/0032/run_001/report
```

The analyzer refuses to overwrite a nonempty output directory.

# KITTI Raw 3D tracking evaluation

아래 명령의 `{프로젝트경로}`는 이 저장소를 clone한 루트 디렉터리로 바꿉니다.

This evaluator measures this repository's AB3DMOT experiments with oriented
**3D IoU** and yaw-aware **BEV IoU**. It is an experiment tool, not a KITTI
Tracking leaderboard submission.

## Coordinate convention

`Box3D` is in the same legacy Velodyne/RViz convention emitted by
`mot_ab3dmot_track_node.py`: centre `(x, y, z)`, dimensions `(h, w, l)`, and
yaw around +Z. The GT loader deliberately reproduces the existing
`readXML -> AB3DMOT -> RViz` conversion, including its legacy x offset, so GT
and exported predictions are compared in precisely the already visualised
frame. It does not invent a new calibration transform.

## What is measured

```text
tracklet_labels.xml -> GT Box3D (stable XML-item IDs)
tracker CSV         -> predicted Box3D (AB3DMOT IDs)
                         | oriented 3D IoU
                         v
                 unmodified TrackEval metrics
```

`*_3D` metrics use oriented 3D IoU; `*_BEV` metrics use yaw-aware footprint
IoU, so a vertical-box error does not change association. `HOTA`, `DetA`,
`AssA`, `MOTA`, `MOTP`, `IDF1`, FP/FN, and ID switches are reported for both.
They are not KITTI official leaderboard metrics: the official KITTI Tracking
protocol evaluates projected 2D camera boxes.

The evaluated timeline is always fixed to the minimum-through-maximum Tracklet
GT frame range. Predictions outside that range are excluded from metrics and
listed in `summary.json`, ensuring Tracklet and PointPillars experiments use
the same number of frames.

## Baseline export

Keep the existing ROS playback and RViz flow. Start the node with a private
ROS parameter so active tracks are exported at the exact point they are sent to
RViz:

```bash
cd {프로젝트경로}/mot_kf_tracking/src
python mot_ab3dmot_track_node.py \
  _tracks_csv:={프로젝트경로}/outputs/tracks/baseline/0032_tracks.csv
```

Replay the complete sequence once with the Python bag player. Do not stop the
node before playback ends; the CSV is flushed per tracked box.

For deterministic evaluator development or a ROS-free baseline rerun, the
following script calls the same parser, frame ordering, AB3DMOT update, and
RViz-output conversion as the node. It does not replace the live exporter.

```bash
python scripts/export_baseline_tracks_offline.py \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --output outputs/tracks/baseline/0032_tracks.csv
```

## Evaluation

```bash
cd {프로젝트경로}
python scripts/evaluate_kitti_3d_tracking.py \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --predictions outputs/tracks/baseline/0032_tracks.csv \
  --sequence 0032 --experiment baseline --class-name Car --iou-threshold 0.5 \
  --output-dir outputs/evaluation/baseline/0032 --metric both
```

Outputs include `summary.json`, `frame_metrics.csv`, `id_switches.csv`, and
`track_metrics.csv`. Inspect a suspicious frame with:

```bash
python scripts/inspect_tracking_alignment.py --frame 120 \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --predictions outputs/tracks/baseline/0032_tracks.csv
```

Compare experiments with `scripts/compare_tracking_experiments.py`.

## PointPillars + AB3DMOT

Run the macOS ONNX/CoreML PointPillars detector as a separate node, then
connect its `jsk_recognition_msgs/BoundingBoxArray` output to the alternate
tracking source. The tracker synchronizes the detection array with the KITTI
ego-velocity topic and writes the identical tracker CSV schema, so the same
evaluator compares it against `tracklet_labels.xml`.

```bash
python pointpillar_object_detection/lidar_point_pillars_onnx_node.py \
  _input_topic:=/kitti/velo/pointcloud \
  _output_topic:=/detection/lidar_detector/boxes \
  _score_threshold:=0.01 \
  _publish_score_threshold:=0.30 \
  _nms_overlap_threshold:=0.20 \
  _detections_csv:={프로젝트경로}/outputs/detections/pointpillars/0032_raw.csv \
  _frame_pipeline_csv:={프로젝트경로}/outputs/detections/pointpillars/0032_detector_frames.csv

python mot_kf_tracking/src/mot_ab3dmot_track_node.py \
  _detection_source:=pointpillars \
  _pointpillars_topic:=/detection/lidar_detector/boxes \
  _min_hits:=3 \
  _max_age:=2 \
  _association_iou_threshold:=0.01 \
  _tracks_csv:={프로젝트경로}/outputs/tracks/pointpillars/0032_tracks.csv \
  _frame_pipeline_csv:={프로젝트경로}/outputs/tracks/pointpillars/0032_tracker_frames.csv

python scripts/evaluate_kitti_3d_tracking.py \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --predictions outputs/tracks/pointpillars/0032_tracks.csv \
  --sequence 0032 --experiment pointpillars --class-name Car --iou-threshold 0.5 \
  --gt-convention pointpillars \
  --output-dir outputs/evaluation/pointpillars/0032 --metric both
```

These commands use the validated PointPillars profile: publish score `0.30`,
NMS `0.20`, `min_hits=3`, `max_age=2`, and association 3D-IoU gate `0.01`.
Use `--rate 0.2` with `scripts/play_bag_python.py` for complete Mac mini
capture.

The PointPillars path deliberately does not load `tracklet.xml` as detection
input. Before treating a score as valid, verify in RViz that PointPillars and
the tracker boxes share the expected LiDAR frame. The CSV's `frame_idx` is
`BoundingBoxArray.header.seq`, which must match the KITTI playback frame
sequence for quantitative comparison. The complete runtime procedure is in
the repository [README](../README.md).

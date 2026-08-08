# KITTI PointPillars + AB3DMOT Tracking

macOS (Apple Silicon/RoboStack)에서 KITTI Raw bag을 재생해 PointPillars
3D 검출, AB3DMOT 추적, RViz 시각화, 3D/BEV 정량평가를 수행하는 프로젝트입니다.

아래 명령의 `{프로젝트경로}`는 이 저장소를 clone한 루트 디렉터리로 바꿉니다.

두 실험 경로를 제공합니다.

```text
Tracklet baseline:  rosbag + tracklet.xml -> AB3DMOT -> RViz / CSV
PointPillars path:  rosbag -> PointPillars -> AB3DMOT -> RViz / CSV
```

PointPillars 결과는 `/detection/lidar_detector/boxes`로, 추적 결과는
`/kitti_box_track`으로 발행됩니다. 두 경로 모두 동일한 CSV 형식으로 저장되어
동일한 평가기로 비교할 수 있습니다.

## Requirements

- macOS + Apple Silicon 권장
- RoboStack Conda 환경: `ros_env`
- ROS1 Python 패키지: `rospy`, `sensor_msgs`, `geometry_msgs`,
  `jsk_recognition_msgs`, `tf`
- Python: `numpy`, `scipy`, `onnxruntime`
- KITTI Raw sequence 0032 bag 및 `tracklet_labels.xml`

`onnxruntime`가 아직 없다면 `ros_env`에서 설치합니다.

```zsh
conda activate ros_env
python -m pip install onnxruntime
```

RoboStack 환경에서는 `source devel/setup.bash`가 필요하지 않습니다. 각
터미널에서 `conda activate ros_env`만 실행하면 됩니다.

## Repository layout

```text
data/                         KITTI bag, point clouds, tracklet.xml
pointpillar_object_detection/
  lidar_point_pillars_onnx_node.py   macOS ONNX/CoreML detector
  models/{pfe,rpn}.onnx              PointPillars models
mot_kf_tracking/             AB3DMOT ROS node and RViz configuration
scripts/                     Python bag player and evaluation helpers
evaluation/                  3D/BEV evaluation adapter
third_party/TrackEval/       HOTA/MOTA/IDF1 metric implementation
outputs/                     Generated tracks and evaluation results
```

`data/` and `outputs/` are intentionally excluded from Git. Before running,
place the KITTI bag at `data/kitti_2011_09_26_drive_0032_synced.bag` and the
KITTI tracklet XML at the path used in the commands below.

## Quick start: PointPillars + AB3DMOT

Open five terminals. Activate `ros_env` in every terminal.

### 1. Start ROS master

```zsh
conda activate ros_env
roscore
```

### 2. Start the PointPillars detector

```zsh
conda activate ros_env
python {프로젝트경로}/pointpillar_object_detection/lidar_point_pillars_onnx_node.py \
  _input_topic:=/kitti/velo/pointcloud \
  _output_topic:=/detection/lidar_detector/boxes \
  _flip_x:=false
```

The detector uses CoreML when available and falls back to ONNX Runtime CPU.
The default models are `pointpillar_object_detection/models/pfe.onnx` and
`pointpillar_object_detection/models/rpn.onnx`.

### 3. Start AB3DMOT tracking

```zsh
conda activate ros_env
python {프로젝트경로}/mot_kf_tracking/src/mot_ab3dmot_track_node.py \
  _detection_source:=pointpillars \
  _pointpillars_topic:=/detection/lidar_detector/boxes \
  _tracks_csv:={프로젝트경로}/outputs/tracks/pointpillars/0032_tracks.csv
```

### 4. Start RViz

```zsh
conda activate ros_env
rviz -d {프로젝트경로}/mot_kf_tracking/config/tracklet_tracking.rviz
```

### 5. Replay the bag

```zsh
conda activate ros_env
cd {프로젝트경로}
python scripts/play_bag_python.py data/kitti_2011_09_26_drive_0032_synced.bag \
  --rate 0.4 --no-wait
```

Use `--rate 0.4` on the Mac mini to avoid detector frame drops. Restart both
the detector and tracker before replaying a sequence again, so old track state
is not carried over.

## Tracklet.xml baseline

For the baseline, do not start the PointPillars detector. Start `roscore`,
RViz, and the bag player as above; run the tracker with the XML source instead.

```zsh
conda activate ros_env
python {프로젝트경로}/mot_kf_tracking/src/mot_ab3dmot_track_node.py \
  _detection_source:=tracklet \
  _tracklet_path:={프로젝트경로}/data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  _tracks_csv:={프로젝트경로}/outputs/tracks/tracklet/0032_tracks.csv
```

## Runtime checks

Run these while the bag is playing.

```zsh
rostopic echo /detection/lidar_detector/boxes
rostopic echo /kitti_box_track
```

- `boxes: []` means that particular frame has no detection; it does not by
  itself indicate a broken topic connection.
- `/kitti_box_track` is only meaningful while `/clock` is being published by
  the bag player.
- The track CSV is flushed as tracks are published to RViz.

## 3D and BEV quantitative evaluation

The evaluator reports TrackEval metrics such as HOTA, DetA, AssA, MOTA,
MOTP, IDF1, FP, FN, and ID switches for both oriented 3D IoU and yaw-aware
BEV IoU. These are experiment metrics, not official KITTI Tracking leaderboard
metrics.

Evaluate PointPillars tracks:

```zsh
cd {프로젝트경로}
python scripts/evaluate_kitti_3d_tracking.py \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --predictions outputs/tracks/pointpillars/0032_tracks.csv \
  --sequence 0032 \
  --experiment pointpillars \
  --metric both \
  --output-dir outputs/evaluation/pointpillars
```

Evaluate the Tracklet baseline by changing `--predictions`, `--experiment`,
and `--output-dir` to the Tracklet output locations. See
[evaluation/README.md](evaluation/README.md) for metric details.

## Notes

- The macOS detector currently emits the `Car` class only.
- KITTI point-cloud reflectance is read from either `i` or `intensity`; values
  already in `[0, 1]` are not normalized again.
- The detector converts the model yaw to the ROS convention used by the
  tracker, so detection boxes, direction arrows, and trajectory markers share
  the same LiDAR frame.
- Historical implementation details are recorded in [CHANGELOG.md](CHANGELOG.md).

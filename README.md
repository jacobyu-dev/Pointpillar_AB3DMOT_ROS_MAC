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

## KITTI Dataset 다운로드 및 ROS bag 설정

이 프로젝트는 Velodyne Point Cloud와 ego-velocity를 같은 프레임 기준으로
동기화해야 하므로, **synced+rectified** KITTI Raw 녹화본
`2011_09_26_drive_0032_sync`을 사용합니다. KITTI Raw 데이터는
[공식 KITTI Raw Data 페이지](https://www.cvlibs.net/datasets/kitti/raw_data.php)에서
계정을 만든 뒤 다운로드할 수 있습니다.

### 1. KITTI Dataset 다운로드

로그인 후 날짜 `2011_09_26`, 주행 시퀀스 `0032`에 대해 다음 세 파일을
다운로드합니다.

- `2011_09_26_drive_0032_sync.zip` — 동기화된 Velodyne Point Cloud, OXTS, timestamp
- `2011_09_26_drive_0032_tracklets.zip` — GT인 `tracklet_labels.xml`
- `2011_09_26_calib.zip` — ROS bag 변환에 필요한 calibration 파일

다운로드한 압축 파일을 `data/downloads/`에 저장한 후, 저장소 루트에서
압축을 풉니다.

```zsh
mkdir -p data/downloads
# Copy the three downloaded .zip files into data/downloads/ before continuing.
unzip data/downloads/2011_09_26_drive_0032_sync.zip -d data
unzip data/downloads/2011_09_26_drive_0032_tracklets.zip -d data
unzip data/downloads/2011_09_26_calib.zip -d data
```

평가와 변환 전 아래 구조가 만들어졌는지 확인합니다.

```text
data/2011_09_26/
  2011_09_26_drive_0032_sync/
    velodyne_points/data/
    oxts/data/
    tracklet_labels.xml
  calib_cam_to_cam.txt
  calib_imu_to_velo.txt
  calib_velo_to_cam.txt
```

### 2. `kitti2bag`으로 ROS1 bag 생성

이 저장소에는 KITTI 데이터와 생성된 bag을 포함하지 않습니다. 공식 KITTI
페이지에서 안내하는 [kitti2bag](https://github.com/tomas789/kitti2bag)를 이용해
다운로드한 Raw 데이터를 ROS1 bag으로 변환합니다. ROS1 메시지 타입을 포함한
bag을 생성하도록 ROS 환경 안에서 실행합니다.

```zsh
conda activate ros_env
python -m pip install kitti2bag
cd {프로젝트경로}/data
kitti2bag -t 2011_09_26 -r 0032 raw_synced .
```

정상적으로 완료되면 아래 bag 파일이 생성됩니다.

```text
data/kitti_2011_09_26_drive_0032_synced.bag
```

`kitti2bag` 실행 파일이 `PATH`에 없다면 다음처럼 실행합니다.

```zsh
python -m kitti2bag -t 2011_09_26 -r 0032 raw_synced .
```

### 3. 생성된 bag의 필수 topic 확인

Pipeline을 실행하기 전, bag에 아래 두 입력 stream이 포함됐는지 확인합니다.
Detector는 Velodyne Point Cloud를 받고, Tracker는 검출 결과와 ego-velocity를
timestamp 기준으로 동기화합니다.

```zsh
cd {프로젝트경로}
rosbag info data/kitti_2011_09_26_drive_0032_synced.bag
```

필수 topic은 다음과 같습니다.

| Topic | ROS type | Used by |
| --- | --- | --- |
| `/kitti/velo/pointcloud` | `sensor_msgs/PointCloud2` | PointPillars detector |
| `/kitti/oxts/gps/vel` | `geometry_msgs/TwistStamped` | AB3DMOT detection/velocity synchronization |

### 4. bag 변환 후 처리하는 호환성 이슈

`kitti2bag`으로 생성한 PointCloud2 메시지에는 `tracklet_labels.xml`의 GT와
직접 연결할 별도 `frame_idx` 필드가 없습니다. 이 상태로 Detection/Tracking
결과를 평가하면 프레임 오프셋이 생겨 ID switch, FP/FN 등의 지표를 잘못
계산할 수 있습니다.

이를 해결하기 위해 PointPillars 입력 경계에서 `Header.seq`를 기준으로
0-base 평가용 `frame_idx`를 직접 생성하고, 검출·추적·평가 CSV까지 전파합니다.

```text
PointCloud2.header.seq (source frame)
  - first input Header.seq
  = frame_idx (0-based evaluation frame)
  -> BoundingBoxArray.header.seq
  -> detection CSV / track CSV / TrackEval input
```

| 이슈 | 처리 방식 | 확인 방법 |
| --- | --- | --- |
| `frame_idx` 부재 | 첫 `PointCloud2.header.seq`를 기준으로 `frame_idx = source Header.seq - first Header.seq`를 계산 | Detector/Tracker 로그에서 첫 입력이 `evaluation frame 0`으로 출력되는지 확인 |
| 1-base 또는 임의의 source sequence | 생성한 `frame_idx`를 `BoundingBoxArray.header.seq`에 명시적으로 설정해 모든 평가 결과를 0-base로 정규화 | Detection CSV와 Tracking CSV의 `frame_idx`가 GT frame 범위와 일치하는지 확인 |
| Detector 추론 지연으로 velocity 메시지가 먼저 소멸 | `ApproximateTimeSynchronizer`의 velocity 이력 queue를 `400`, timestamp 허용 범위를 `0.1s`로 설정 | Tracker frame manifest에서 detector callback과 tracker callback이 같은 frame에 기록되는지 확인 |

Detector는 원본 timestamp를 유지한 채 계산된 index를
`BoundingBoxArray.header.seq`로 발행합니다. Tracker는 이 값을 `frame_idx`로
기록하며, detector/tracker frame manifest에는 `source_frame_idx`와
`frame_idx`를 모두 저장합니다. 따라서 bag 재생부터 GT 평가까지 프레임 정합
상태를 추적할 수 있습니다.

정량 캡처 시에는 Quick start의 `detections_csv`, `tracks_csv`,
`frame_pipeline_csv` 경로를 모두 지정합니다. 재생 이후 아래 진단 도구로
frame drop과 offset 여부를 확인할 수 있습니다.

```zsh
cd {프로젝트경로}
python scripts/diagnostics/analyze_pointpillars_pipeline.py \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --detections outputs/detections/pointpillars/0032_raw.csv \
  --tracks outputs/tracks/pointpillars/0032_tracks.csv \
  --detector-manifest outputs/detections/pointpillars/0032_detector_frames.csv \
  --tracker-manifest outputs/tracks/pointpillars/0032_tracker_frames.csv \
  --analysis-score-threshold 0.30 \
  --output-dir outputs/analysis/0032/frame_diagnostics
```

## Results and implementation contributions

| Outcome | My implementation | Evidence / result |
| --- | --- | --- |
| Lightweight 3D perception pipeline | Integrated PointPillars and Kalman Filter + Hungarian Algorithm-based AB3DMOT through ROS topics, without an additional tracking neural network. | Runs detector, tracker, RViz, and bag playback as separate inspectable nodes. |
| Detector–tracker alignment | Implemented a box adapter that reconciles 3D box parameter order, axes, signs, offsets, and yaw conventions. | Detection boxes, tracking boxes, orientation arrows, and trajectories share the LiDAR/RViz frame. |
| Frame-level evaluation alignment | Created and propagated a 0-based `frame_idx` from `PointCloud2.header.seq`, then preserved source/derived frame indices in pipeline manifests. | Detector CSV, tracker CSV, and `tracklet_labels.xml` can be joined frame by frame without an implicit offset. |
| Track ID continuity improvement | Analyzed ID-loss frames, then ablated score threshold, NMS, `min_hits`, `max_age`, and 3D-IoU association settings across KITTI 0009/0023/0032. | Selected `score=0.30`, `NMS=0.20`, `min_hits=3`, `max_age=2`, `IoU=0.01`; mean BEV HOTA **0.4190 → 0.5114 (+22.1%)**, IDF1 **0.5033 → 0.6439 (+27.9%)**. |
| Dynamic-object information | Managed active/terminated tracks and visual lifetimes; extended per-frame detections with object ID, velocity, direction, trajectory, and risk-distance information in the map frame. | Eliminates stale visualization state and supports object-centric dynamic-map updates. |
| Reproducible evaluation | Built CSV exports, frame manifests, and TrackEval-based 3D/BEV evaluation with a fixed GT timeline. | Reports HOTA, MOTA, IDF1, FP/FN, and ID switches under identical frame ranges. |

## Quick start: PointPillars + AB3DMOT

Open five terminals. Activate `ros_env` in every terminal. The commands below
use the validated PointPillars tracking profile: score `0.30`, AABB BEV NMS
`0.20`, `min_hits=3`, `max_age=2`, and association 3D-IoU gate `0.01`.
Across 0009/0023/0032 this profile improved mean BEV HOTA from `0.4190` to
`0.5114` and mean IDF1 from `0.5033` to `0.6439`.

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
  _flip_x:=false \
  _score_threshold:=0.01 \
  _publish_score_threshold:=0.30 \
  _nms_overlap_threshold:=0.20 \
  _detections_csv:={프로젝트경로}/outputs/detections/pointpillars/0032_raw.csv \
  _frame_pipeline_csv:={프로젝트경로}/outputs/detections/pointpillars/0032_detector_frames.csv
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
  _min_hits:=3 \
  _max_age:=2 \
  _association_iou_threshold:=0.01 \
  _tracks_csv:={프로젝트경로}/outputs/tracks/pointpillars/0032_tracks.csv \
  _frame_pipeline_csv:={프로젝트경로}/outputs/tracks/pointpillars/0032_tracker_frames.csv
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
  --rate 0.2 --no-wait
```

Use `--rate 0.2` for quantitative capture on the Mac mini. The validated
0009/0023/0032 runs completed without detector callback loss at this rate.
Restart both the detector and tracker before replaying a sequence again, so old
track state is not carried over. `detections_csv` is post-NMS/pre-publish data
for raw detection analysis; AB3DMOT receives only boxes at score `>= 0.30`.

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
metrics. The evaluator fixes the sequence timeline to the Tracklet GT frame
range, so delayed predictions outside that range are excluded and reported in
`summary.json`.

Evaluate PointPillars tracks:

```zsh
cd {프로젝트경로}
python scripts/evaluate_kitti_3d_tracking.py \
  --tracklets data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml \
  --predictions outputs/tracks/pointpillars/0032_tracks.csv \
  --sequence 0032 \
  --experiment pointpillars \
  --gt-convention pointpillars \
  --iou-threshold 0.5 \
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
- PointPillars defaults are the validated tracking profile above. Override
  parameters only for a newly named experiment directory; keep the Tracklet
  baseline on its independent legacy defaults.
- Historical implementation details are recorded in [CHANGELOG.md](CHANGELOG.md).

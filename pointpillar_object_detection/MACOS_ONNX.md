# macOS ONNX/CoreML PointPillars

아래 명령의 `{프로젝트경로}`는 이 저장소를 clone한 루트 디렉터리로 바꿉니다.

`lidar_point_pillars_onnx_node.py` replaces the CUDA/TensorRT detector with a
NumPy preprocessing pipeline and ONNX Runtime inference.  On Apple Silicon it
uses `CoreMLExecutionProvider` first and falls back to CPU when CoreML is not
available.

The default PFE/RPN models are `models/pfe.onnx` and `models/rpn.onnx`.
The node subscribes to
`/kitti/velo/pointcloud` and publishes `jsk_recognition_msgs/BoundingBoxArray`
on `/detection/lidar_detector/boxes`.  Its boxes use centre-Z, length/width/
height dimensions, and yaw around +Z, matching the PointPillars tracker path.
It also publishes standard RViz `visualization_msgs/MarkerArray` boxes on
`/detection/lidar_detector/markers`.  This second topic is intended for
RoboStack RViz installations without the optional JSK BoundingBox display.

Run this detector as its own node after activating `ros_env` and starting your
usual `roscore`:

```zsh
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

These are the validated PointPillars tracking parameters. They keep a low
candidate floor before NMS and publish only score `>= 0.30` boxes to RViz and
AB3DMOT.

If boxes appear behind the ego vehicle in RViz, retry with `_flip_x:=true`.

## View raw detections in RViz

Add a **MarkerArray** display, then select
`/detection/lidar_detector/markers`.  Do not select the grey `/boxes` item
from RViz's *By topic* list unless the JSK RViz plugin is installed: it is a
`jsk_recognition_msgs/BoundingBoxArray`, which stock RViz cannot render.

# Changelog

All notable changes to this project are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and uses the categories **Added**, **Changed**, **Fixed**, and **Removed**.

## [Unreleased]

### Added

- macOS-compatible PointPillars ROS node based on NumPy preprocessing and
  ONNX Runtime with CoreML provider preference.
- `pointpillars` detection source for `mot_ab3dmot_track_node.py`, consuming
  `jsk_recognition_msgs/BoundingBoxArray` from
  `/detection/lidar_detector/boxes`.
- CSV export of active AB3DMOT tracks for both Tracklet and PointPillars
  experiments.
- Oriented 3D-IoU and yaw-aware BEV-IoU evaluation paths using TrackEval.
- Python ROS bag player with simulated `/clock`, playback-rate control, and
  non-interactive `--no-wait` mode.
- Root README describing the supported macOS/RoboStack workflow.

### Changed

- PointPillars ONNX models now live in
  `pointpillar_object_detection/models/{pfe,rpn}.onnx`, independent of the
  removed CUDA/TensorRT package tree.
- PointPillars, tracker, RViz, and bag playback are run as separate nodes for
  easier debugging and inspection.
- Tracker detection/ego-velocity synchronization queue was increased to retain
  the velocity message while CoreML inference completes.

### Fixed

- Read KITTI PointCloud2 reflectance from field `i` as well as `intensity`.
- Avoid re-normalizing intensity values that are already in the `[0, 1]`
  KITTI range.
- Match the legacy PointPillars ROS yaw conversion, aligning detection boxes,
  orientation arrows, and trajectory markers in RViz.
- Fix evaluation to the Tracklet GT frame range and exclude delayed
  out-of-sequence predictions, so compared experiments use identical frame
  counts.
- Removed the unavailable `autoware_msgs` dependency from the macOS tracking
  path in favor of JSK bounding-box messages.

### Removed

- Legacy CUDA/TensorRT PointPillars packages and front-camera variants.
- Legacy detected-object visualizers; standard RViz `MarkerArray` output from
  the tracker is used instead.
- Historical catkin workspace, backup packages, unused mapping/calibration/
  lane/YOLO experiments, and generated cache files.

### Compatibility notes

- The supported runtime is the RoboStack Conda environment (`ros_env`); do not
  rely on `source devel/setup.bash`.
- The current detector is intended for the supplied KITTI sequence and emits
  `Car` detections.
- Evaluation results are intended for internal experiment comparison. They are
  not KITTI leaderboard submissions.

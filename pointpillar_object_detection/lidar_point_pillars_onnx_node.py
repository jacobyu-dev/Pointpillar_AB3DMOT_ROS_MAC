#!/usr/bin/env python3
"""CPU/CoreML PointPillars ROS node for macOS.

This is a NumPy + ONNX Runtime implementation of the original TensorRT node's
two-model pipeline.  It publishes JSK BoundingBoxArray so it can feed the
repository's AB3DMOT node without Autoware messages.
"""

from __future__ import annotations

import csv
import os
from pathlib import Path

import numpy as np
import onnxruntime as ort
import rospy
import tf.transformations
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


MAX_PILLARS = 12000
MAX_POINTS = 100
GRID_X, GRID_Y = 432, 496
PILLAR_X, PILLAR_Y = 0.16, 0.16
MIN_X, MIN_Y, MIN_Z = 0.0, -39.68, -3.0
MAX_X, MAX_Y, MAX_Z = 69.12, 39.68, 1.0
ANCHOR_X, ANCHOR_Y = 216, 248


class PointPillarsONNX:
    def __init__(self) -> None:
        script_dir = Path(__file__).resolve().parent
        default_model_dir = script_dir / 'models'
        self.pfe_path = Path(rospy.get_param('~pfe_onnx_file', str(default_model_dir / 'pfe.onnx')))
        self.rpn_path = Path(rospy.get_param('~rpn_onnx_file', str(default_model_dir / 'rpn.onnx')))
        self.input_topic = rospy.get_param('~input_topic', '/kitti/velo/pointcloud')
        self.output_topic = rospy.get_param('~output_topic', '/detection/lidar_detector/boxes')
        self.marker_topic = rospy.get_param('~marker_topic', '/detection/lidar_detector/markers')
        self._input_frame_base = None
        # Defaults are the validated PointPillars + AB3DMOT tracking profile.
        # Keep a low candidate floor before NMS, then use the separate publish
        # threshold to keep RViz and the tracker free from false detections.
        self.score_threshold = float(rospy.get_param('~score_threshold', 0.01))
        self.publish_score_threshold = float(rospy.get_param(
            '~publish_score_threshold', 0.30))
        self.nms_threshold = float(rospy.get_param('~nms_overlap_threshold', 0.20))
        self.detection_csv_file = None
        self.detection_csv_writer = None
        detections_csv_path = rospy.get_param('~detections_csv', '')
        if detections_csv_path:
            detections_csv_path = os.path.abspath(os.path.expanduser(detections_csv_path))
            directory = os.path.dirname(detections_csv_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            self.detection_csv_file = open(detections_csv_path, 'w', newline='')
            self.detection_csv_writer = csv.DictWriter(self.detection_csv_file, fieldnames=(
                'frame_idx', 'detection_id', 'class_name', 'x', 'y', 'z',
                'h', 'w', 'l', 'yaw', 'score'))
            self.detection_csv_writer.writeheader()
            rospy.on_shutdown(self.detection_csv_file.close)
        self.pipeline_csv_file = None
        self.pipeline_csv_writer = None
        pipeline_csv_path = rospy.get_param('~frame_pipeline_csv', '')
        if pipeline_csv_path:
            pipeline_csv_path = os.path.abspath(os.path.expanduser(pipeline_csv_path))
            directory = os.path.dirname(pipeline_csv_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            self.pipeline_csv_file = open(pipeline_csv_path, 'w', newline='')
            self.pipeline_csv_writer = csv.DictWriter(self.pipeline_csv_file, fieldnames=(
                'source_frame_idx', 'frame_idx', 'source_frame_base', 'frame_index_mode', 'timestamp', 'status',
                'num_points', 'inference_ran',
                'num_score_candidates', 'num_after_nms', 'num_published'))
            self.pipeline_csv_writer.writeheader()
            rospy.on_shutdown(self.pipeline_csv_file.close)
        # The legacy normal node negated input x while the "front" variant did
        # not.  KITTI Raw PointCloud2 normally uses forward-positive x, so the
        # native macOS node defaults to no flip and leaves it configurable.
        self.flip_x = bool(rospy.get_param('~flip_x', False))

        for model_path in (self.pfe_path, self.rpn_path):
            if not model_path.is_file():
                raise FileNotFoundError('ONNX model not found: {}'.format(model_path))

        available = ort.get_available_providers()
        providers = ['CoreMLExecutionProvider', 'CPUExecutionProvider'] if 'CoreMLExecutionProvider' in available else ['CPUExecutionProvider']
        options = ort.SessionOptions()
        options.log_severity_level = 3
        self.pfe = ort.InferenceSession(str(self.pfe_path), sess_options=options, providers=providers)
        self.rpn = ort.InferenceSession(str(self.rpn_path), sess_options=options, providers=providers)
        self._validate_model_contract()
        self.anchors = self._make_anchors()

        self.publisher = rospy.Publisher(self.output_topic, BoundingBoxArray, queue_size=1)
        # BoundingBoxArray needs the optional JSK RViz plugin.  Publish native
        # MarkerArray as well so a stock RoboStack RViz can show raw detector
        # boxes without routing them through the tracker.
        self.marker_publisher = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.subscriber = rospy.Subscriber(self.input_topic, PointCloud2, self.callback, queue_size=1)
        rospy.loginfo('PointPillars ONNX ready: %s -> %s (%s)', self.input_topic, self.output_topic,
                      ', '.join(self.pfe.get_providers()))
        rospy.loginfo('Raw PointPillars RViz markers: %s', self.marker_topic)
        rospy.loginfo('PointPillars candidate/publish score thresholds: %.3f / %.3f',
                      self.score_threshold, self.publish_score_threshold)
        rospy.loginfo('PointPillars CSV frame indices are normalized from the first input Header.seq')
        if self.detection_csv_writer is not None:
            rospy.loginfo('Writing raw post-NMS PointPillars detections to %s', detections_csv_path)
        if self.pipeline_csv_writer is not None:
            rospy.loginfo('Writing PointPillars frame pipeline manifest to %s', pipeline_csv_path)

    def _validate_model_contract(self) -> None:
        pfe_shapes = [tuple(item.shape) for item in self.pfe.get_inputs()]
        rpn_shapes = [tuple(item.shape) for item in self.rpn.get_inputs()]
        if len(pfe_shapes) != 8 or pfe_shapes[:4] != [(1, 1, MAX_PILLARS, MAX_POINTS)] * 4:
            raise RuntimeError('Unsupported PFE ONNX input contract: {}'.format(pfe_shapes))
        if rpn_shapes != [(1, 64, GRID_Y, GRID_X)]:
            raise RuntimeError('Unsupported RPN ONNX input contract: {}'.format(rpn_shapes))

    @staticmethod
    def _make_anchors() -> np.ndarray:
        """Return anchors ordered exactly as [y, x, rotation] in the C++ node."""
        ys = np.arange(ANCHOR_Y, dtype=np.float32) * (PILLAR_Y * 2.0) + (MIN_Y + PILLAR_Y)
        xs = np.arange(ANCHOR_X, dtype=np.float32) * (PILLAR_X * 2.0) + (MIN_X + PILLAR_X)
        yy, xx, rr = np.meshgrid(ys, xs, np.array([0.0, np.pi / 2.0], dtype=np.float32), indexing='ij')
        anchors = np.empty((ANCHOR_Y, ANCHOR_X, 2, 7), dtype=np.float32)
        anchors[..., 0], anchors[..., 1] = xx, yy
        anchors[..., 2] = -1.73  # sensor-height bottom used by the TensorRT implementation
        anchors[..., 3], anchors[..., 4], anchors[..., 5], anchors[..., 6] = 1.6, 3.9, 1.56, rr
        return anchors.reshape(-1, 7)

    def _pillar_inputs(self, points: np.ndarray) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray]:
        """Port the original CPU preprocessing and return PFE inputs + occupancy."""
        pillar_x = np.zeros((MAX_PILLARS, MAX_POINTS), dtype=np.float32)
        pillar_y = np.zeros_like(pillar_x)
        pillar_z = np.zeros_like(pillar_x)
        pillar_i = np.zeros_like(pillar_x)
        x_sub = np.zeros_like(pillar_x)
        y_sub = np.zeros_like(pillar_x)
        mask = np.zeros_like(pillar_x)
        count = np.zeros(MAX_PILLARS, dtype=np.float32)
        x_indices = np.zeros(MAX_PILLARS, dtype=np.int32)
        y_indices = np.zeros(MAX_PILLARS, dtype=np.int32)
        occupancy = np.zeros((GRID_Y, GRID_X), dtype=np.uint8)
        cell_to_pillar: dict[tuple[int, int], int] = {}

        for x, y, z, intensity in points:
            cell_x = int(np.floor((x - MIN_X) / PILLAR_X))
            cell_y = int(np.floor((y - MIN_Y) / PILLAR_Y))
            cell_z = int(np.floor((z - MIN_Z) / (MAX_Z - MIN_Z)))
            if not (0 <= cell_x < GRID_X and 0 <= cell_y < GRID_Y and cell_z == 0):
                continue
            key = (cell_y, cell_x)
            pillar = cell_to_pillar.get(key)
            if pillar is None:
                if len(cell_to_pillar) >= MAX_PILLARS:
                    break
                pillar = len(cell_to_pillar)
                cell_to_pillar[key] = pillar
                x_indices[pillar], y_indices[pillar] = cell_x, cell_y
                # These offsets intentionally match the model's original
                # training/export implementation, not geometric cell centres.
                x_sub[pillar, :] = cell_x * PILLAR_X + 0.1
                y_sub[pillar, :] = cell_y * PILLAR_Y - 39.9
                occupancy[cell_y, cell_x] = 1
            point_index = int(count[pillar])
            if point_index >= MAX_POINTS:
                continue
            pillar_x[pillar, point_index] = x
            pillar_y[pillar, point_index] = y
            pillar_z[pillar, point_index] = z
            pillar_i[pillar, point_index] = intensity
            count[pillar] += 1

        mask = (np.arange(MAX_POINTS)[None, :] < count[:, None]).astype(np.float32)
        arrays = [pillar_x, pillar_y, pillar_z, pillar_i, count, x_sub, y_sub, mask]
        feeds = {item.name: value.reshape((1, MAX_PILLARS)) if value is count else value.reshape((1, 1, MAX_PILLARS, MAX_POINTS))
                 for item, value in zip(self.pfe.get_inputs(), arrays)}
        return feeds, x_indices, y_indices, occupancy

    def _anchor_mask(self, occupancy: np.ndarray) -> np.ndarray:
        """Vectorised integral-image anchor occupancy test from anchor_mask_cuda."""
        integral = occupancy.astype(np.int64).cumsum(axis=0).cumsum(axis=1)
        anchor = self.anchors
        half_x = np.where(np.isclose(np.mod(anchor[:, 6], np.pi), 0.0), anchor[:, 3], anchor[:, 4]) / 2.0
        half_y = np.where(np.isclose(np.mod(anchor[:, 6], np.pi), 0.0), anchor[:, 4], anchor[:, 3]) / 2.0
        x0 = np.clip(np.floor((anchor[:, 0] - half_x - MIN_X) / PILLAR_X).astype(int), 0, GRID_X - 1)
        y0 = np.clip(np.floor((anchor[:, 1] - half_y - MIN_Y) / PILLAR_Y).astype(int), 0, GRID_Y - 1)
        x1 = np.clip(np.floor((anchor[:, 0] + half_x - MIN_X) / PILLAR_X).astype(int), 0, GRID_X - 1)
        y1 = np.clip(np.floor((anchor[:, 1] + half_y - MIN_Y) / PILLAR_Y).astype(int), 0, GRID_Y - 1)
        total = integral[y1, x1]
        total -= np.where(x0 > 0, integral[y1, x0 - 1], 0)
        total -= np.where(y0 > 0, integral[y0 - 1, x1], 0)
        total += np.where((x0 > 0) & (y0 > 0), integral[y0 - 1, x0 - 1], 0)
        return total > 1

    @staticmethod
    def _aabb_nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> np.ndarray:
        """Match the legacy node's fast axis-aligned BEV NMS behaviour."""
        x, y, dx, dy, yaw = (boxes[:, index] for index in (0, 1, 3, 4, 6))
        cos, sin = np.abs(np.cos(yaw)), np.abs(np.sin(yaw))
        half_x, half_y = (cos * dx + sin * dy) / 2.0, (sin * dx + cos * dy) / 2.0
        x0, x1, y0, y1 = x - half_x, x + half_x, y - half_y, y + half_y
        order, keep = scores.argsort()[::-1], []
        while order.size:
            current = order[0]
            keep.append(current)
            rest = order[1:]
            inter_x = np.maximum(0.0, np.minimum(x1[current], x1[rest]) - np.maximum(x0[current], x0[rest]) + 1.0)
            inter_y = np.maximum(0.0, np.minimum(y1[current], y1[rest]) - np.maximum(y0[current], y0[rest]) + 1.0)
            intersection = inter_x * inter_y
            union = ((x1[current] - x0[current] + 1.0) * (y1[current] - y0[current] + 1.0) +
                     (x1[rest] - x0[rest] + 1.0) * (y1[rest] - y0[rest] + 1.0) - intersection)
            order = rest[intersection / np.maximum(union, 1e-6) <= threshold]
        return np.asarray(keep, dtype=int)

    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
        feeds, x_indices, y_indices, occupancy = self._pillar_inputs(points)
        pfe_output = self.pfe.run(None, feeds)[0].reshape(64, MAX_PILLARS)
        scatter = np.zeros((1, 64, GRID_Y, GRID_X), dtype=np.float32)
        pillar_count = int(np.count_nonzero(np.any(feeds[self.pfe.get_inputs()[7].name], axis=(0, 1, 3))))
        if pillar_count == 0:
            return np.empty((0, 7), dtype=np.float32), np.empty(0, dtype=np.float32), 0
        scatter[0, :, y_indices[:pillar_count], x_indices[:pillar_count]] = pfe_output[:, :pillar_count].T
        raw_boxes, raw_scores, raw_dirs = self.rpn.run(None, {self.rpn.get_inputs()[0].name: scatter})
        encoded = raw_boxes.reshape(ANCHOR_Y, ANCHOR_X, 2, 7).reshape(-1, 7)
        scores = 1.0 / (1.0 + np.exp(-raw_scores.reshape(-1)))
        direction = raw_dirs.reshape(ANCHOR_Y, ANCHOR_X, 2, 2).reshape(-1, 2).argmax(axis=1)
        selected = (scores > self.score_threshold) & self._anchor_mask(occupancy)
        encoded, scores, direction, anchors = encoded[selected], scores[selected], direction[selected], self.anchors[selected]
        score_candidate_count = int(np.count_nonzero(selected))
        if not len(scores):
            return np.empty((0, 7), dtype=np.float32), scores, score_candidate_count
        diagonal = np.hypot(anchors[:, 3], anchors[:, 4])
        boxes = np.empty_like(encoded)
        boxes[:, 0] = encoded[:, 0] * diagonal + anchors[:, 0]
        boxes[:, 1] = encoded[:, 1] * diagonal + anchors[:, 1]
        boxes[:, 5] = np.exp(encoded[:, 5]) * anchors[:, 5]
        boxes[:, 2] = encoded[:, 2] * anchors[:, 5] + anchors[:, 2] + anchors[:, 5] / 2.0 - boxes[:, 5] / 2.0
        boxes[:, 3] = np.exp(encoded[:, 3]) * anchors[:, 3]
        boxes[:, 4] = np.exp(encoded[:, 4]) * anchors[:, 4]
        boxes[:, 6] = encoded[:, 6] + anchors[:, 6] + np.where(direction == 0, np.pi, 0.0)
        keep = self._aabb_nms(boxes, scores, self.nms_threshold)
        return boxes[keep], scores[keep], score_candidate_count

    def _output_header(self, message: PointCloud2) -> Header:
        """Return a 0-based evaluation header from this bag's first input seq."""
        source_frame = int(message.header.seq)
        if self._input_frame_base is None:
            self._input_frame_base = source_frame
            rospy.loginfo('PointPillars frame-index base: input Header.seq=%d -> evaluation frame 0', source_frame)
        frame_idx = source_frame - self._input_frame_base
        # Assign fields explicitly.  This makes the emitted Header sequence
        # visible both to downstream ROS subscribers and to the manifest.
        output = Header()
        output.seq = frame_idx
        output.stamp = message.header.stamp
        output.frame_id = message.header.frame_id
        return output

    def _write_pipeline_row(self, message: PointCloud2, output_header: Header, *, status: str, num_points: int,
                            inference_ran: bool, num_score_candidates: int,
                            num_after_nms: int, num_published: int) -> None:
        """Write one row per received point-cloud callback, including zero-box frames."""
        if self.pipeline_csv_writer is None:
            return
        self.pipeline_csv_writer.writerow({
            'source_frame_idx': message.header.seq, 'frame_idx': output_header.seq,
            'source_frame_base': self._input_frame_base,
            'frame_index_mode': 'first_input_header_seq_is_zero',
            'timestamp': message.header.stamp.to_sec(),
            'status': status, 'num_points': num_points, 'inference_ran': int(inference_ran),
            'num_score_candidates': num_score_candidates, 'num_after_nms': num_after_nms,
            'num_published': num_published,
        })
        self.pipeline_csv_file.flush()

    def callback(self, message: PointCloud2) -> None:
        output_header = self._output_header(message)
        names = [field.name for field in message.fields]
        if not {'x', 'y', 'z'}.issubset(names):
            rospy.logerr_throttle(5, 'PointPillars requires PointCloud2 fields x, y, z; got %s', names)
            self._write_pipeline_row(message, output_header, status='missing_xyz_fields', num_points=0,
                                     inference_ran=False, num_score_candidates=0,
                                     num_after_nms=0, num_published=0)
            return
        # KITTI's velodyne PointCloud2 uses the compact field name ``i``;
        # other common ROS publishers call the same value ``intensity``.
        intensity_name = 'intensity' if 'intensity' in names else ('i' if 'i' in names else None)
        field_names = ('x', 'y', 'z', intensity_name) if intensity_name else ('x', 'y', 'z')
        rows = list(point_cloud2.read_points(message, field_names=field_names, skip_nans=True))
        if not rows:
            self.publisher.publish(BoundingBoxArray(header=output_header))
            clear_markers = MarkerArray()
            clear = Marker()
            clear.header = output_header
            clear.action = Marker.DELETEALL
            clear_markers.markers.append(clear)
            self.marker_publisher.publish(clear_markers)
            self._write_pipeline_row(message, output_header, status='empty_pointcloud', num_points=0,
                                     inference_ran=False, num_score_candidates=0,
                                     num_after_nms=0, num_published=0)
            return
        points = np.asarray(rows, dtype=np.float32)
        if intensity_name is None:
            points = np.column_stack((points, np.ones(len(points), dtype=np.float32)))
        else:
            # KITTI Raw bags already store reflectance as float [0, 1].
            # Some generic PointCloud2 sources use uint8-style [0, 255]
            # values, so only normalise that representation.  Dividing KITTI
            # intensities again makes the network receive almost all zeros.
            if np.nanmax(points[:, 3]) > 1.0:
                points[:, 3] /= 255.0
        if self.flip_x:
            points[:, 0] *= -1.0
        boxes, scores, score_candidate_count = self.infer(points)
        result = BoundingBoxArray(header=output_header)
        markers = MarkerArray()
        # A frame can contain fewer detections than the previous one.  Remove
        # old marker IDs before publishing current boxes so RViz never leaves
        # stale detector output behind.
        clear = Marker()
        clear.header = output_header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for detection_id, (values, score) in enumerate(zip(boxes, scores)):
            x, y, bottom_z, width, length, height, model_yaw = values
            # Preserve all post-NMS candidates in the raw CSV for offline AP.
            ros_yaw = -float(model_yaw + np.pi / 2.0)
            ros_yaw = float(np.arctan2(np.sin(ros_yaw), np.cos(ros_yaw)))
            if self.detection_csv_writer is not None:
                self.detection_csv_writer.writerow({
                    'frame_idx': output_header.seq, 'detection_id': detection_id,
                    'class_name': 'Car', 'x': float(x), 'y': float(y),
                    'z': float(bottom_z + height / 2.0), 'h': float(height),
                    'w': float(width), 'l': float(length), 'yaw': ros_yaw,
                    'score': float(score),
                })
            if score < self.publish_score_threshold:
                continue
            bbox = BoundingBox()
            bbox.header = output_header
            bbox.pose.position.x, bbox.pose.position.y = float(x), float(y)
            bbox.pose.position.z = float(bottom_z + height / 2.0)
            # Keep the ROS output convention of the original C++ PointPillars
            # node.  The network's box axes use a different x/y convention:
            # rotate by 90 degrees and reverse yaw before publishing a ROS
            # pose.  AB3DMOT then uses this pose directly for its arrows and
            # trajectory markers.
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, ros_yaw)
            bbox.pose.orientation.x, bbox.pose.orientation.y, bbox.pose.orientation.z, bbox.pose.orientation.w = quaternion
            bbox.dimensions.x, bbox.dimensions.y, bbox.dimensions.z = float(length), float(width), float(height)
            bbox.label, bbox.value = 1, float(score)
            result.boxes.append(bbox)
            marker = Marker()
            marker.header = output_header
            marker.ns = 'pointpillars_raw_detections'
            marker.id = detection_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = bbox.pose
            marker.scale = bbox.dimensions
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.2, 0.45
            marker.lifetime = rospy.Duration(0.5)
            markers.markers.append(marker)
        if self.detection_csv_file is not None:
            self.detection_csv_file.flush()
        # rospy may update a published message Header's seq in-place.  Record
        # the intended evaluation index before publishing the ROS message.
        self._write_pipeline_row(message, output_header, status='ok', num_points=len(points), inference_ran=True,
                                 num_score_candidates=score_candidate_count,
                                 num_after_nms=len(boxes), num_published=len(result.boxes))
        self.publisher.publish(result)
        self.marker_publisher.publish(markers)
        rospy.loginfo_throttle(2, 'PointPillars ONNX: %d points -> %d/%d published/raw car boxes',
                               len(points), len(result.boxes), len(boxes))


def main() -> None:
    rospy.init_node('lidar_point_pillars_onnx')
    PointPillarsONNX()
    rospy.spin()


if __name__ == '__main__':
    main()

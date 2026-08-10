#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import csv
import rospy
import numpy as np
import tf
import xml.etree.ElementTree as ET

from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point
from tf import transformations # rotation_matrix(), concatenate_matrices()

from jsk_recognition_msgs.msg import BoundingBox        #sudo apt-get install ros-melodic-jsk-recognition-msgs
from jsk_recognition_msgs.msg import BoundingBoxArray
try:
    from jsk_rviz_plugins.msg import Pictogram
    from jsk_rviz_plugins.msg import PictogramArray
    from jsk_rviz_plugins.msg import OverlayText
    HAS_JSK_RVIZ_PLUGINS = True
except ImportError:
    # RoboStack does not provide jsk_rviz_plugins for macOS/Apple Silicon.
    # Standard RViz MarkerArray outputs remain available.
    HAS_JSK_RVIZ_PLUGINS = False

from visualization_msgs.msg import Marker, MarkerArray
# catkin_install_python runs this script through a wrapper in devel/lib.  Keep
# the sibling tracker modules importable from the actual source directory.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from model import AB3DMOT
import std_msgs

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

from kalman_filter import KalmanBoxTracker
import message_filters

import pandas as pd

# Resolve resources from the repository root so the node works from any clone
# location rather than depending on one developer's absolute path.
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
TRACKLET_PATH = os.path.join(
    PROJECT_ROOT, 'data', '2011_09_26', '2011_09_26_drive_0032_sync',
    'tracklet_labels.xml')
# Use an absolute local resource so RViz does not need the broken macOS
# RoboStack `rospack` binary to resolve package:// resources.
CAR_DAE_PATH = 'file://' + os.path.abspath(
    os.path.join(SCRIPT_DIR, '..', 'dae', 'car.dae')
)


#Global Variables
detectionBoxes = None
prior_trk_xyz = {}      # 속도계산을 위해 이전 frame들의 xyz를 저장하는 데이터구조
current_id_list = []
prior_path_xyz = {}     # 이동경로를 표현하기 위해 이전 frame들의 xyz를 저장하는 데이터구조
d_obj = {}


MIN_WARNING_DIST = 20     # _m


# 데이터분석용 데이터프레임
df_18 = pd.DataFrame(columns=['frame', 'obj_id', 'tx', 'ty', 'tz', 'dx_t', 'dy_t','velo','v_ego_x', 'v_ego_y', 'v_ego'])
df_34 = pd.DataFrame(columns=['frame', 'obj_id', 'tx', 'ty', 'tz', 'dx_t', 'dy_t','velo','v_ego_x', 'v_ego_y', 'v_ego'])
df_35 = pd.DataFrame(columns=['frame', 'obj_id', 'tx', 'ty', 'tz', 'dx_t', 'dy_t','velo','v_ego_x', 'v_ego_y', 'v_ego'])


class MoDetect_N_Track:
    def __init__(self):
        # ``tracklet`` keeps the original ground-truth-detection baseline.
        # ``pointpillars`` consumes the output of exactly one PointPillars node;
        # it never parses tracklet XML for detections.
        self.detection_source = rospy.get_param('~detection_source', 'tracklet').lower()
        # rospy assigns sequential Header.seq values while publishing. Make
        # tracking CSV indices 0-based from the first detection automatically.
        self._source_frame_base = None
        if self.detection_source not in ('tracklet', 'pointpillars'):
            raise ValueError("~detection_source must be 'tracklet' or 'pointpillars'")

        # Subscriber
        if self.detection_source == 'tracklet':
            self.detection_sub = message_filters.Subscriber("/kitti/velo/pointcloud", PointCloud2)
        else:
            pointpillars_topic = rospy.get_param(
                '~pointpillars_topic', '/detection/lidar_detector/boxes')
            self.detection_sub = message_filters.Subscriber(pointpillars_topic, BoundingBoxArray)
            rospy.loginfo('Tracking PointPillars detections from %s', pointpillars_topic)
        self.imu_sub = message_filters.Subscriber("/kitti/oxts/gps/vel",TwistStamped)

        # Publisher
        self.pub_frame_seq = None
        self.pub_boxes = rospy.Publisher('kitti_box_track', BoundingBoxArray, queue_size=1)
        self.pub_pictograms = None
        self.pub_box_markers = rospy.Publisher('kitti_box_track_markers', MarkerArray, queue_size=1)
        self.pub_label_markers = rospy.Publisher('kitti_box_label_markers_track', MarkerArray, queue_size=1)
        if HAS_JSK_RVIZ_PLUGINS:
            self.pub_frame_seq = rospy.Publisher('kitti_frame_seq', OverlayText, queue_size=1)
            self.pub_pictograms = rospy.Publisher('kitti_box_pictogram_track', PictogramArray, queue_size=1)
        self.pub_selfvelo_text = rospy.Publisher('kitti_selfvelo_text_track', Marker, queue_size=1)
        self.pub_selfveloDirection = rospy.Publisher('kitti_selfvelo_direction_track', Marker, queue_size=1)
        self.pub_objs_ori = rospy.Publisher('kitti_objs_ori_track', MarkerArray, queue_size=3)
        self.pub_objs_velo = rospy.Publisher('kitti_objs_velo_track', MarkerArray, queue_size=1)
        self.pub_path = rospy.Publisher('kitti_objs_path_track', MarkerArray, queue_size=1)
        self.pub_warning_lines = rospy.Publisher('kitti_warning_lines_track', MarkerArray, queue_size=1)
        self.pub_ego_outCircle = rospy.Publisher('kitti_ego_outCircle_track', Marker, queue_size=1)
        self.pub_ego_innerCircle = rospy.Publisher('kitti_ego_innerCircle_track', Marker, queue_size=1)
        self.pub_ego_car = rospy.Publisher('kitti_ego_car_track', Marker, queue_size=1)



        # Publisher & Subscriber Wrapper
        # PointPillars inference finishes after the point cloud was received.
        # With the old three-message queue, the matching velocity sample was
        # usually discarded before the detector published its box array, so
        # the tracking callback never ran.  Keep a sufficiently long history
        # of velocity messages and match by the original ROS timestamps.
        sync_queue_size = rospy.get_param('~sync_queue_size', 400)
        sync_slop = rospy.get_param('~sync_slop', 0.1)
        pub_list = [self.detection_sub, self.imu_sub]
        self.ts = message_filters.ApproximateTimeSynchronizer(
            pub_list, sync_queue_size, sync_slop, allow_headerless=True)
        rospy.loginfo('Detection/velocity sync: queue=%d, slop=%.3fs',
                      sync_queue_size, sync_slop)
        self.ts.registerCallback(self.callback)

        # Use the validated profile for PointPillars, but preserve the legacy
        # Tracklet.xml baseline defaults when that independent path is used.
        default_max_age = 2 if self.detection_source == 'pointpillars' else 3
        default_min_hits = 3 if self.detection_source == 'pointpillars' else 2
        self.max_age = int(rospy.get_param('~max_age', default_max_age))
        self.min_hits = int(rospy.get_param('~min_hits', default_min_hits))
        self.association_iou_threshold = float(
            rospy.get_param('~association_iou_threshold', 0.01))
        if self.max_age < 1 or self.min_hits < 1:
            raise ValueError('~max_age and ~min_hits must be positive integers')
        if not 0.0 <= self.association_iou_threshold <= 1.0:
            raise ValueError('~association_iou_threshold must be within [0, 1]')
        # Multi-Objects tracking instance
        self.mot_tracker = AB3DMOT(
            max_age=self.max_age,
            min_hits=self.min_hits,
            association_iou_threshold=self.association_iou_threshold)
        rospy.loginfo('AB3DMOT: max_age=%d, min_hits=%d, association 3D IoU gate=%.3f',
                      self.max_age, self.min_hits, self.association_iou_threshold)
        self.track_csv_file = None
        self.track_csv_writer = None
        tracks_csv_path = rospy.get_param('~tracks_csv', '')
        if tracks_csv_path:
            tracks_csv_path = os.path.abspath(os.path.expanduser(tracks_csv_path))
            os.makedirs(os.path.dirname(tracks_csv_path), exist_ok=True)
            self.track_csv_file = open(tracks_csv_path, 'w', newline='')
            self.track_csv_writer = csv.DictWriter(self.track_csv_file, fieldnames=(
                'frame_idx', 'track_id', 'class_name', 'x', 'y', 'z', 'h', 'w', 'l', 'yaw'))
            self.track_csv_writer.writeheader()
            rospy.on_shutdown(self.track_csv_file.close)
            rospy.loginfo('Writing active AB3DMOT tracks to %s', tracks_csv_path)
        self.pipeline_csv_file = None
        self.pipeline_csv_writer = None
        pipeline_csv_path = rospy.get_param('~frame_pipeline_csv', '')
        if pipeline_csv_path:
            pipeline_csv_path = os.path.abspath(os.path.expanduser(pipeline_csv_path))
            os.makedirs(os.path.dirname(pipeline_csv_path), exist_ok=True)
            self.pipeline_csv_file = open(pipeline_csv_path, 'w', newline='')
            self.pipeline_csv_writer = csv.DictWriter(self.pipeline_csv_file, fieldnames=(
                'source_frame_idx', 'frame_idx', 'source_frame_base', 'frame_index_mode', 'timestamp', 'detection_source',
                'num_input_detections',
                'num_output_tracks'))
            self.pipeline_csv_writer.writeheader()
            rospy.on_shutdown(self.pipeline_csv_file.close)
            rospy.loginfo('Writing AB3DMOT frame pipeline manifest to %s', pipeline_csv_path)



    def callback(self, detection_msg, TwistStamped):
        header = detection_msg.header
        source_frame = int(header.seq)
        if self._source_frame_base is None:
            self._source_frame_base = source_frame
            rospy.loginfo('AB3DMOT frame-index base: detection Header.seq=%d -> evaluation frame 0', source_frame)
        frame = source_frame - self._source_frame_base
        del current_id_list[:]

        if frame == 0:
            self.mot_tracker.__init__(
                max_age=self.max_age,
                min_hits=self.min_hits,
                association_iou_threshold=self.association_iou_threshold)
            KalmanBoxTracker.count = 0
            prior_trk_xyz.clear()
            prior_path_xyz.clear()


        # frame overlay is optional because jsk_rviz_plugins is not built for macOS.
        overlayTxt = None
        if HAS_JSK_RVIZ_PLUGINS:
            overlayTxt = OverlayText()
            overlayTxt.left = 10
            overlayTxt.top = 10
            overlayTxt.width = 1200
            overlayTxt.height = 1200
            overlayTxt.fg_color.a = 1.0
            overlayTxt.fg_color.r = 1.0
            overlayTxt.fg_color.g = 1.0
            overlayTxt.fg_color.b = 1.0
            overlayTxt.text_size = 12
            overlayTxt.text = "Frame_seq : {}".format(frame)


        boxes = BoundingBoxArray() #3D Boxes with JSK
        boxes.header = header     

        texts = None
        if HAS_JSK_RVIZ_PLUGINS:
            texts = PictogramArray() #Labels with JSK
            texts.header = header

        obj_ori_arrows = MarkerArray() #arrow with visualization_msgs 

        velocity_markers = MarkerArray() #text with visualization_msgs 

        obj_path_markers = MarkerArray() # passed path

        warning_line_markers = MarkerArray()
        box_markers = MarkerArray()
        label_markers = MarkerArray()



        # ego-vehicle 사진 출력
        ego_car = Marker(
            type=Marker.MESH_RESOURCE,
            id=0,
            lifetime=rospy.Duration(0.5),
            pose=Pose(Point(0.0, 0.0, -1.6), Quaternion(0,0,0,1)),
            scale=Vector3(1.5, 1.5, 1.5),
            header=header,
            action=Marker.ADD,
            mesh_resource=CAR_DAE_PATH,
            color=ColorRGBA(1.0, 1.0, 1.0, 1.0)
        )



        ### 자기 속도 Publishing Logic    
        # headerImu = TwistStamped.header     
        oxtLinear = TwistStamped.twist.linear
        selfvelo = np.sqrt(oxtLinear.x ** 2 + oxtLinear.y ** 2 + oxtLinear.z ** 2)
        selfvelo = np.round(selfvelo,1)    # m/s
        selfvelo = selfvelo * 3.6           # km/h
        
        oxtAngular = TwistStamped.twist.angular
        q = tf.transformations.quaternion_from_euler(oxtAngular.x, oxtAngular.y, oxtAngular.z)

        text_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(0.1),
                pose=Pose(Point(-5.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(1.5, 1.5, 1.5),
                header=header,
                color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
                text="{}km/h".format(selfvelo))

        selfvelo_scale = convert_velo2scale(selfvelo)
        arrow_marker = Marker(
                type=Marker.ARROW,
                id=0,
                lifetime=rospy.Duration(0.1),
                pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(*q)),
                scale=Vector3(selfvelo_scale, 0.5, 0.5),
                header=header,
                color=ColorRGBA(1.0, 0.0, 0.0, 0.8))



        if self.detection_source == 'tracklet':
            # Original ground-truth path: preserve its established conversion
            # and output convention exactly for baseline comparability.
            raw_bboxinfo = detectionBoxes[detectionBoxes[:,0] == str(frame), 2:9]
            additional_info = detectionBoxes[detectionBoxes[:,0] == str(frame), 0:2]
            bboxinfo = raw_bboxinfo[:, [3, 4, 5, 0, 1, 2, 6]].astype(np.float64)
        else:
            # PointPillars publishes JSK dimensions as (length, width,
            # height) and a pose yaw around +Z.  AB3DMOT expects
            # (height, width, length, x, y, z, yaw).
            pointpillar_rows = []
            pointpillar_info = []
            for object_index, detected_object in enumerate(detection_msg.boxes):
                q = detected_object.pose.orientation
                yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
                pointpillar_rows.append([
                    detected_object.dimensions.z, detected_object.dimensions.y,
                    detected_object.dimensions.x, detected_object.pose.position.x,
                    detected_object.pose.position.y, detected_object.pose.position.z, yaw,
                ])
                # Version 1.0 emits only cars.  Keep the KITTI spelling used by
                # the evaluator so its class filter works for both experiments.
                pointpillar_info.append([str(frame), 'Car'])
            bboxinfo = np.asarray(pointpillar_rows, dtype=np.float64).reshape((-1, 7))
            additional_info = np.asarray(pointpillar_info, dtype=object).reshape((-1, 2))

        dets_all = {'dets': bboxinfo, 'info': additional_info}


        # ObjectTracking from Detection
        trackers = self.mot_tracker.update(dets_all)        # h,w,l,x,y,z,theta
        trackers_bbox = trackers[:,0:7]
        trackers_info = trackers[:,7:10]                    # id, frame, label
        if self.pipeline_csv_writer is not None:
            self.pipeline_csv_writer.writerow({
                'frame_idx': frame, 'timestamp': header.stamp.to_sec(),
                'source_frame_idx': source_frame,
                'source_frame_base': self._source_frame_base,
                'frame_index_mode': 'first_detection_header_seq_is_zero',
                'detection_source': self.detection_source,
                'num_input_detections': len(bboxinfo), 'num_output_tracks': len(trackers_bbox),
            })
            self.pipeline_csv_file.flush()
        if self.detection_source == 'tracklet':
            trackers_bbox = trackers_bbox[:, [3, 4, 5, 0, 1, 2, 6]]
            trackers_bbox = trackers_bbox[:, [2, 0, 1, 3, 4, 5, 6]].astype(np.float64)
            trackers_bbox[:,0] = trackers_bbox[:,0] + 1.3
            trackers_bbox[:,1] = trackers_bbox[:,1]*-1
            trackers_bbox[:,2] = trackers_bbox[:,2]*-1
            trackers_bbox[:,6] = trackers_bbox[:,6]*-1
            marker_yaw_offset = np.pi / 2.0
            marker_z_is_box_bottom = True
        else:
            # Native PointPillars/JSK frame: x/y/z is already a box
            # centre, so do not apply the tracklet camera-to-Velodyne offsets.
            trackers_bbox = trackers_bbox[:, [3, 4, 5, 0, 1, 2, 6]].astype(np.float64)
            marker_yaw_offset = 0.0
            marker_z_is_box_bottom = False


        # for문을 통해 각 objects들의 정보를 추출하여 사용
        for b, info in zip(trackers_bbox, trackers_info):   
            bbox = BoundingBox()
            bbox.header = header


            # parameter 뽑기     [tx,ty,tz,h,w,l,rz]
            tx, ty, tz = float(b[0]), float(b[1]), float(b[2])  
            rz = float(b[6])
            size = Vector3(float(b[5]), float(b[4]), float(b[3]) )       
            obj_id = info[0]
            label = info[2]
            display_z = tz / 2.0 if marker_z_is_box_bottom else tz

            # 이전 x frame 까지 지나온 points들을 저장하여 반환하는 함수
            # obj_id와 bbox.label은 단지 type차이만 날뿐 같은 데이터
            path_points_list = points_path(tx, ty, tz, obj_id)
            bbox_color = colorCategory20(int(obj_id))
            path_marker = Marker(
                    type=Marker.LINE_STRIP,
                    id=int(obj_id),
                    lifetime=rospy.Duration(0.1),
                    # pose=Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),        # origin point position
                    scale=Vector3(0.1, 0.0, 0.0),                           # line width
                    header=header,
                    # color=ColorRGBA(1.0, 1.0, 1.0, 1.0))
                    color=bbox_color)

            # 수정 필요        
            # if len(prior_path_xyz[obj_id]) > 1: 이 문법 쓰지마세요
            path_marker.points = path_points_list
                
            obj_path_markers.markers.append(path_marker)


            # Tracker들의 BoundingBoxArray 설정
            bbox.pose.position = Point(tx, ty, display_z)
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, rz + marker_yaw_offset)
            bbox.pose.orientation = Quaternion(*q)
            bbox.dimensions = size
            # bbox.value = 0.001
            bbox.label = int(obj_id)
            boxes.boxes.append(bbox)

            # CSV uses precisely the pose sent to RViz: Velodyne x/y, marker
            # centre z, h/w/l dimensions, and the final marker yaw.
            if self.track_csv_writer is not None:
                self.track_csv_writer.writerow({
                    'frame_idx': frame, 'track_id': int(obj_id), 'class_name': label,
                    'x': tx, 'y': ty, 'z': display_z,
                    'h': size.z, 'w': size.y, 'l': size.x, 'yaw': rz + marker_yaw_offset,
                })
                self.track_csv_file.flush()

            # Standard RViz replacement for the unavailable JSK BoundingBox display.
            box_marker = Marker(
                type=Marker.CUBE,
                id=int(obj_id),
                ns='kitti_box_track',
                lifetime=rospy.Duration(0.1),
                pose=Pose(Point(tx, ty, display_z), Quaternion(*q)),
                scale=size,
                header=header,
                color=bbox_color)
            box_marker.color.a = 0.35
            box_markers.markers.append(box_marker)

            if not HAS_JSK_RVIZ_PLUGINS:
                label_marker = Marker(
                    type=Marker.TEXT_VIEW_FACING,
                    id=int(obj_id),
                    ns='kitti_box_label_track',
                    lifetime=rospy.Duration(0.1),
                    pose=Pose(Point(tx, ty, display_z + size.z / 2.0 + 1.0), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.0, 0.0, 1.0),
                    header=header,
                    color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
                    text='{} {}'.format(label, int(obj_id)))
                label_markers.markers.append(label_marker)


            # Tracker들의 속도 추정
            obj_velo,dx_t,dy_t,dz_t = obj_velocity([tx,ty,tz], bbox.label, oxtLinear)
            if obj_velo != None:    
                obj_velo = np.round(obj_velo,1)    # m/s
                obj_velo = obj_velo * 3.6           # km/h

            obj_velo_scale = convert_velo2scale(obj_velo)


            # Tracker들의 Orientation
            obj_ori_arrow = Marker(
                type=Marker.ARROW,
                id=bbox.label,
                lifetime=rospy.Duration(0.1),
                pose=Pose(Point(tx, ty, display_z), Quaternion(*q)),
                scale=Vector3(obj_velo_scale, 0.5, 0.5),
                header=header,
                # color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
                color=bbox_color)
            obj_ori_arrows.markers.append(obj_ori_arrow)



            if HAS_JSK_RVIZ_PLUGINS:
                picto_text = Pictogram()
                picto_text.header = header
                picto_text.mode = Pictogram.STRING_MODE
                picto_text.pose.position = Point(tx, ty, -tz)
                q = tf.transformations.quaternion_from_euler(0.7, 0.0, -0.7)
                picto_text.pose.orientation = Quaternion(0.0, -0.5, 0.0, 0.5)
                picto_text.size = 4
                picto_text.color = std_msgs.msg.ColorRGBA(1, 1, 1, 1)
                picto_text.character = label + ' ' + str(bbox.label)
                texts.pictograms.append(picto_text)

            
            obj_velo_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=bbox.label,
                lifetime=rospy.Duration(0.1),
                pose=Pose(Point(tx, ty, tz-2.0), Quaternion(0.0, -0.5, 0.0, 0.5)),
                scale=Vector3(1.5, 1.5, 1.5),
                header=header,
                color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
                text="{}km/h".format(obj_velo))
            velocity_markers.markers.append(obj_velo_marker)

            current_id_list.append(bbox.label)
        


            # Warning object line
            warning_line = Marker(
                    type=Marker.LINE_LIST,
                    id=int(obj_id),
                    lifetime=rospy.Duration(0.05),
                    pose=Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),        # origin point position
                    scale=Vector3(0.2, 0.0, 0.0),                           # line width
                    header=header,
                    color=ColorRGBA(1.0, 0.0, 0.0, 1.0))

            d = dist_from_objBbox(tx,ty,tz,size.x, size.y, size.z)
            if d < MIN_WARNING_DIST:
                # d_obj[bbox.label] = [Point(tx,ty,tz), Point(0.0, 0.0, 0.0)]
                warning_line.points = Point(tx,ty,tz), Point(0.0, 0.0, 0.0)
                warning_line_markers.markers.append(warning_line)


            # Data Check용 로직 
            # global df_18, df_34, df_35
            # if obj_id =='18':
            #     df_18 = df_18.append({'frame' : frame, 'obj_id' : obj_id, 'tx' : tx, 'ty' : ty, 'tz' : tz, 
            #     'dx_t' : dx_t, 'dy_t' : dy_t, 'velo':obj_velo , 'v_ego_x': oxtLinear.x, 'v_ego_y':oxtLinear.y, 'v_ego':selfvelo}, ignore_index = True)
            # if obj_id =='34':
            #     df_34 = df_34.append({'frame' : frame, 'obj_id' : obj_id, 'tx' : tx, 'ty' : ty, 'tz' : tz, 
            #     'dx_t' : dx_t, 'dy_t' : dy_t, 'velo':obj_velo , 'v_ego_x': oxtLinear.x, 'v_ego_y':oxtLinear.y, 'v_ego':selfvelo}, ignore_index = True)
            # if obj_id =='35':
            #     df_35 = df_35.append({'frame' : frame, 'obj_id' : obj_id, 'tx' : tx, 'ty' : ty, 'tz' : tz, 
            #     'dx_t' : dx_t, 'dy_t' : dy_t, 'velo':obj_velo , 'v_ego_x': oxtLinear.x, 'v_ego_y':oxtLinear.y, 'v_ego':selfvelo}, ignore_index = True)
        


        # Change Outer Circle Color
        outer_circle_color = ColorRGBA(1.0*25/255, 1.0, 0.0, 1.0)
        if len(warning_line_markers.markers) > 0 :
            outer_circle_color = ColorRGBA(1.0*255/255, 1.0*0/255, 1.0*0/255, 1.0)

        # ego_vehicle's warning boundary
        outer_circle = Marker(
            type=Marker.CYLINDER,
            id=0,
            lifetime=rospy.Duration(0.5),
            pose=Pose(Point(0.0,0.0,-1.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(8.0, 8.0, 0.1),                           # line width
            header=header,
            color=outer_circle_color
        )

        inner_circle = Marker(
            type=Marker.CYLINDER,
            id=0,
            lifetime=rospy.Duration(0.5),
            pose=Pose(Point(0.0,0.0,-0.8), Quaternion(0, 0, 0, 1)),
            scale=Vector3(7.0, 7.0, 0.2),                           # line width
            header=header,
            color=ColorRGBA(0.22, 0.22, 0.22, 1.0)
        )    


        for i in list(prior_trk_xyz.keys()):
            if i not in current_id_list:
                prior_trk_xyz.pop(i)

        if self.pub_frame_seq is not None:
            self.pub_frame_seq.publish(overlayTxt)
        self.pub_boxes.publish(boxes)
        if self.pub_pictograms is not None:
            self.pub_pictograms.publish(texts)
        self.pub_box_markers.publish(box_markers)
        self.pub_label_markers.publish(label_markers)
        self.pub_selfvelo_text.publish(text_marker)
        self.pub_selfveloDirection.publish(arrow_marker)
        self.pub_objs_ori.publish(obj_ori_arrows)
        self.pub_objs_velo.publish(velocity_markers)
        self.pub_path.publish(obj_path_markers)
        self.pub_warning_lines.publish(warning_line_markers)
        self.pub_ego_outCircle.publish(outer_circle)
        self.pub_ego_innerCircle.publish(inner_circle)
        self.pub_ego_car.publish(ego_car)



def convert_velo2scale(velo):
    '''
    입력속도 : km/h
    입력받은 속도를 화살표 마커의 길이로 변환
    최소길이 : 0.0 (0km/h)/ 최대길이 : 10.0 (100km/h)
    ret : km/h
    '''
    scale_len = None

    if velo != None:
        if velo > 100:
            scale_len = 15.0
        elif velo < 0:
            scale_len = 0.0
        else:
            scale_len = 15 * velo/100
    else:
        scale_len = 0.0

    return scale_len



def points_path(tx,ty,tz,trk_id):
    '''
    input : x,y,z좌표, tracker_boundingbox id
    output : 이전 _frame에서의 x,y,z 좌표를 전역변수에 저장하여 리스트로 출력
    전역변수 prior_path_xyz : tracker_boundingbox id를 key로 가지는 Dictionary
    '''
    if trk_id in prior_path_xyz:
        prior_path_xyz[trk_id].append(Point(tx,ty,tz))

        if len(prior_path_xyz[trk_id]) > 10:
            prior_path_xyz[trk_id].pop(0)
        # ret = prior_path_xyz[trk_id]
    else:
        prior_path_xyz[trk_id] = [Point(tx,ty,tz)]
        
    return prior_path_xyz[trk_id]



def obj_velocity(trk_xyz_list, trk_id, oxtLinear):

    obj_velo = None
    dx_t,dy_t,dz_t = None,None,None
    
    if trk_id in prior_trk_xyz:
        # 계산
        tx,ty,tz = trk_xyz_list                  # 현재 좌표
        x,y,z = prior_trk_xyz[trk_id]            # 이전 좌표
        dx,dy,dz = x - tx, y - ty, z - tz
        dx_t = dx/0.1
        dy_t = dy/0.1
        dz_t = dz/0.1
        vx = oxtLinear.x - dx/0.1
        vy = oxtLinear.y - dy/0.1
        vz = oxtLinear.z - dz/0.1
        obj_velo = np.sqrt(vx ** 2 + vy ** 2)
        prior_trk_xyz[trk_id] = trk_xyz_list
    else:
        prior_trk_xyz[trk_id] = trk_xyz_list

    return obj_velo, dx_t, dy_t, dz_t
    


# Rviz에서 자동으로 색깔을 지정하는 함수를 직접 구현
def colorCategory20(obj_id):
    c = ColorRGBA()
    c.a = 0.6

    if (obj_id % 20)==0:
        c.r = 0.121569
        c.g = 0.466667
        c.b = 0.705882
    elif (obj_id % 20)==1:
        c.r = 0.682353
        c.g = 0.780392
        c.b = 0.909804  
    elif (obj_id % 20)==2:
        c.r = 1.000000
        c.g = 0.498039
        c.b = 0.054902  
    elif (obj_id % 20)==3:
        c.r = 1.000000
        c.g = 0.733333
        c.b = 0.470588  
    elif (obj_id % 20)==4:
        c.r = 0.172549
        c.g = 0.627451
        c.b = 0.172549
    elif (obj_id % 20)==5: 
        c.r = 0.596078
        c.g = 0.874510
        c.b = 0.541176 
    elif (obj_id % 20)==6:
        c.r = 0.839216
        c.g = 0.152941
        c.b = 0.156863  
    elif (obj_id % 20)==7:  
        c.r = 1.000000
        c.g = 0.596078
        c.b = 0.588235  
    elif (obj_id % 20)==8:  
        c.r = 0.580392
        c.g = 0.403922
        c.b = 0.741176
    elif (obj_id % 20)==9:  
        c.r = 0.772549
        c.g = 0.690196
        c.b = 0.835294
    elif (obj_id % 20)==10: 
        c.r = 0.549020
        c.g = 0.337255
        c.b = 0.294118 
    elif (obj_id % 20)==11: 
        c.r = 0.768627
        c.g = 0.611765
        c.b = 0.580392  
    elif (obj_id % 20)==12:  
        c.r = 0.890196
        c.g = 0.466667
        c.b = 0.760784 
    elif (obj_id % 20)==13: 
        c.r = 0.968627
        c.g = 0.713725
        c.b = 0.823529 
    elif (obj_id % 20)==14: 
        c.r = 0.498039
        c.g = 0.498039
        c.b = 0.498039
    elif (obj_id % 20)==15: 
        c.r = 0.780392
        c.g = 0.780392
        c.b = 0.780392
    elif (obj_id % 20)==16:  
        c.r = 0.737255
        c.g = 0.741176
        c.b = 0.133333
    elif (obj_id % 20)==17: 
        c.r = 0.858824
        c.g = 0.858824
        c.b = 0.552941  
    elif (obj_id % 20)==18:   
        c.r = 0.090196
        c.g = 0.745098
        c.b = 0.811765
    elif (obj_id % 20)==19:
        c.r = 0.619608
        c.g = 0.854902
        c.b = 0.898039

    return c



# 원점과 / object들의 BoundingBox의 꼭지점을 둘러싸는 원과의 거리를 반환
def dist_from_objBbox(tx, ty, tz, h, w, l):
    dist2orgin = np.sqrt(tx**2 + ty**2 + tz**2)
    dist2edge = np.sqrt((h/2)**2 + (w/2)**2 + (w/2)**2)
    dist2orgin - dist2edge

    return dist2orgin - dist2edge



# Read Detection info from tracklet.xml
def readXML(file):
    '''
    tracklet.xml 파일로 부터 Object들의 Bounding Box정보들을 읽어와 
    np.array 타입으로 리턴한다.
    - det_all의 index순서 :  frame,type(label),tx,ty,tz,h,w,l,rz
    '''
    tree = ET.parse(file)
    root = tree.getroot()
    
    item = root.findall('./tracklets/item')

    det_all = np.zeros((0,9))     # (None, 9) : 9는 사용할 3d bbox의 파라미터 개수

    for i, v in enumerate(item):
        h = float(v.find('h').text)
        w = float(v.find('w').text)
        l = float(v.find('l').text)
        frame = int(v.find('first_frame').text)
        label = v.find('objectType').text
        pose = v.findall('./poses/item')
        for j, p in enumerate(pose):
            tx = float(p.find('tx').text)
            ty = float(p.find('ty').text)
            tz = float(p.find('tz').text)
            rz = float(p.find('rz').text)
            
            # frame,type(label),h,w,l,tx,ty,tz,rz  9개 파라미터 저장
            # convert coordinate from velodyne to left-cam
            # det_all = np.append(det_all,[[frame+j,label,tx,ty,tz,h,w,l,rz]], axis=0)
            # det_all = np.append(det_all,[[frame+j,label,-ty,-tz,tx-0.27,h,w,l,-rz - (np.pi/2)]], axis=0) 
            det_all = np.append(det_all,[[frame+j,label,-ty,-tz,tx-0.27,h,w,l,-rz + np.pi/2]], axis=0) 
    return det_all     



def main(args):
    global detectionBoxes

    #Initializes and cleanup ros node with node name
    rospy.init_node('mot_ab3dmot_track_node', anonymous=True)

    detection_source = rospy.get_param('~detection_source', 'tracklet').lower()
    if detection_source == 'tracklet':
        tracklet_path = rospy.get_param('~tracklet_path', TRACKLET_PATH)
        detectionBoxes = readXML(tracklet_path)
        rospy.loginfo('Loaded tracklet ground truth from %s', tracklet_path)
    elif detection_source != 'pointpillars':
        rospy.logfatal("~detection_source must be 'tracklet' or 'pointpillars'")
        return

    pcl_obj = MoDetect_N_Track() 

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")

    # df_18.to_csv("df_18.csv", index=False)
    # df_34.to_csv("df_34.csv", index=False)
    # df_35.to_csv("df_35.csv", index=False)

if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy

import numpy as np
import ros_numpy
import colorsys

import message_filters

import tf as tf2
from tf import TransformListener

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from utils import *
import open3d

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

from sklearn import linear_model
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


pc_stack = np.empty((0, 3), float)
Lane_markers_array = MarkerArray()
Plane_markers_array = MarkerArray()
frame_stack = 10
id_global = 0
weight_past = 0 


class lane_detection_class:

    def __init__(self):
        self.lidar_pub = rospy.Publisher("/frame_stack", PointCloud2, queue_size=5)
        self.lidar_sub = rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, self.callback, queue_size=5)

        self.pub_Lane_marker = rospy.Publisher('/lane_marker', MarkerArray, queue_size=5)
        self.pub_Plane_marker = rospy.Publisher('/Plane_marker', MarkerArray, queue_size=5)
        # self.pub_Lane_marker2 = rospy.Publisher('/lane_marker2', Marker, queue_size=5)

        self.tf = TransformListener()

    def callback(self, PointCloud2):
        global pc_stack, frame_stack, Lane_markers_array, Plane_markers_array, id_global, weight_past
        # print("callback : ", PointCloud2.header.seq)

        pc_np = get_xyzi_points(pointcloud2_to_array(PointCloud2))
        xyz_points = pc_np[:,:3]
        intensity = pc_np[:,3]

        # road_pts = extract_points(pc_np, voxel_size = 0.05, x_range= (-10, 10), y_range= (-10, 15), z_range= (-5, -1.2), i_range= (0.45, 0.9))
        road_pts = extract_points(pc_np, voxel_size = 0.01, x_range= (-5, 5), y_range= (-2.2, 6), z_range= (-5, -1.2),i_range=(2,8))
        # pc_stack = np.append(pc_stack, road_pts, axis=0)

        odom_mat = get_odom(self.tf,"velo_link", "map")
        
        if odom_mat is not None:
            # points = get_transformation(odom_mat,xyz_points)
            points = get_transformation(odom_mat,road_pts)
            pc_stack = np.append(pc_stack, points, axis=0)

        # if PointCloud2.header.seq == 389: 
        #     save_ply(pc_stack,"save.ply")

        if (PointCloud2.header.seq > 0) and (PointCloud2.header.seq % frame_stack == 0): 
            
            pc_stack[...,2] = 0
            X = StandardScaler().fit_transform(pc_stack)

            db = DBSCAN(eps=0.2, min_samples=20).fit(X)

            labels = db.labels_
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
            # n_noise_ = list(labels).count(-1)

            # pc_stack = pc_stack[labels>=0]

            ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(),
                                                    max_trials=100, 
                                                    min_samples=None,
                                                    residual_threshold=None)
                                                    #residual_metric = lamda x: np.sum(np.abs(x), axis=1))


            # Lane_markers_array = MarkerArray() # passed path

            def add_square_feature(X):
                # X = np.concatenate([(X**2).reshape(-1,1), X], axis=1)
                return X

            lane1x = sys.maxsize
            lane1y = sys.maxsize
            lane2x = -sys.maxsize - 1
            lane2y = -sys.maxsize - 1
            for cluster in range(n_clusters_):
                sub_cluster_df = pc_stack[labels == cluster]
          
                Xpoints = sub_cluster_df[...,0]
                Xpoints = Xpoints.reshape(-1,1)
                Ypoints = sub_cluster_df[...,1]
                Ypoints = Ypoints.reshape(-1,1)
                ransac.fit(add_square_feature(Xpoints), Ypoints)
                inlier_mask = ransac.inlier_mask_

                line_X = np.arange(Xpoints.min(), Xpoints.max())[:, np.newaxis]
                line_y_ransac = ransac.predict(add_square_feature(line_X))

                if line_X.min() < lane1x:
                    lane1x = line_X.min()
                if line_y_ransac.min() < lane1y:
                    lane1y = line_y_ransac.min()

                if line_X.max() > lane2x:
                    lane2x = line_X.max()
                if line_y_ransac.max() > lane2y:
                    lane2y = line_y_ransac.max()

                weight = (line_y_ransac.max()- line_y_ransac.min()) / (line_X.max()-line_X.min())

                # print(weight)
                if weight < 0.3:
                    Lane_marker = Marker(type=Marker.LINE_STRIP, 
                                            header = PointCloud2.header,
                                            action = Marker.ADD,
                                            id = id_global,
                                            scale = Vector3(0.5, 0.5, 0.5),
                                            color = ColorRGBA(1.0, 1.0, 1.0, 1.0),
                                            pose= Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),
                                            lifetime=rospy.Duration(300))
                    id_global +=1

                    Lane_marker.header.frame_id = "/map"


                    for i, j in zip(line_X,line_y_ransac):
                        l_points = Point()
                        l_points.x = i
                        l_points.y = j
                        l_points.z = 0.0
                        Lane_marker.points.append(l_points)

                    Lane_markers_array.markers.append(Lane_marker)
                    
                    rect_point1 = Point(lane1x,lane1y,0) 
                    rect_point2 = Point(lane2x,lane2y,0) 

                    Plane_marker = Marker(type=Marker.CUBE, 
                                            header = PointCloud2.header,
                                            action = Marker.ADD,
                                            id = id_global,
                                            scale = Vector3(np.fabs(rect_point1.x - rect_point2.x),
                                                            np.fabs(rect_point1.y - rect_point2.y), 
                                                            np.fabs(rect_point1.z - rect_point2.z)),
                                            color = ColorRGBA(0.0, 0.0, 0.0, 1.0),
                                            pose= Pose(Point((rect_point1.x - rect_point2.x) / 2.0 + rect_point2.x,
                                                             (rect_point1.y - rect_point2.y) / 2.0 + rect_point2.y,
                                                             (rect_point1.z - rect_point2.z) / 2.0 + rect_point2.z), 
                                                             Quaternion(0, 0, 0, 1)),
                                            lifetime=rospy.Duration(300))
                    
                    Plane_marker.header.frame_id = "/map"
                    Plane_markers_array.markers.append(Plane_marker)
                    
                weight_past = weight

            PointCloud2.header.frame_id = "/map"
            point_pc2 = xyzarray_to_pc2(pc_stack, PointCloud2)

            pc_stack = np.empty((0, 3), float)

            self.pub_Lane_marker.publish(Lane_markers_array)
            self.pub_Plane_marker.publish(Plane_markers_array)
            self.lidar_pub.publish(point_pc2)
            

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('Lane_detection_node', anonymous=True)

    lane_detection = lane_detection_class()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)


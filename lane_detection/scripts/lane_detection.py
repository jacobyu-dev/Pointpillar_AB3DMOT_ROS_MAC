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

pc_stack = np.empty((0, 3), float)

frame_stack = 10

class lane_detection_class:

    def __init__(self):
        self.lidar_pub = rospy.Publisher("/frame_stack", PointCloud2, queue_size=4)
        # self.lidar_sub = rospy.Subscriber("/ground_cloud_intensity", PointCloud2, self.callback, queue_size=3)

        self.lidar_sub = rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, self.callback, queue_size=5)
        self.tf = TransformListener()

    def callback(self, PointCloud2):
        global pc_stack, frame_stack
        print("callback : ", PointCloud2.header.seq)

        pc_np = get_xyzi_points(pointcloud2_to_array(PointCloud2))
        xyz_points = pc_np[:,:3]
        intensity = pc_np[:,3]

        road_pts = extract_points(pc_np, voxel_size = 0.05, x_range= (-10, 10), y_range= (-10, 15), z_range= (-5, -1.2), i_range= (0.45, 0.9))

        # pc_stack = np.append(pc_stack, road_pts, axis=0)

        odom_mat = self.get_odom()

        if odom_mat is not None:
            points = get_transformation(odom_mat,road_pts)

            pc_stack = np.append(pc_stack, points, axis=0)


        if (PointCloud2.header.seq > 0) and (PointCloud2.header.seq % frame_stack == 0): 

            PointCloud2.header.frame_id = "/map"
            point_pc2 = xyzarray_to_pc2(pc_stack, PointCloud2)

            pc_stack = np.empty((0, 3), float)

            print("pub : ", PointCloud2.header.seq)
            self.lidar_pub.publish(point_pc2)

    def get_odom(self):
        try:
            t = self.tf.getLatestCommonTime("/map", "/velo_link")
            position, quaternion = self.tf.lookupTransform("/map", "/velo_link", t)

            trans_mat = tf2.transformations.translation_matrix(position)
            rot_mat = tf2.transformations.quaternion_matrix(quaternion)
            # create a 4x4 matrix
            mat = np.dot(trans_mat, rot_mat)
        except(tf2.Exception, tf2.ConnectivityException, tf2.LookupException):
            print("miss")
            return
        return mat


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('Lane_detection_node', anonymous=True)

    lane_detection = lane_detection_class()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

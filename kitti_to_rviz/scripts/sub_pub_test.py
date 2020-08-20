#!/usr/bin/env python
import sys

import cv2

import roslib
import rospy

from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge, CvBridgeError

VERBOSE=True

class image_feature:

    def __init__(self):

        self.image_pub = rospy.Publisher("image_to_rviz", Image, queue_size=1)
        self.bridge = CvBridge()        
        self.image_sub = rospy.Subscriber("/kitti/camera_color_left/image_raw", Image, self.callback, queue_size=1)
        if VERBOSE :
            print "\nsubscribed to /kitti/camera_color_left/image_raw"
            print "\npublish image_to_rviz\n\n"

    def callback(self, ros_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

class lidar_feature:

    def __init__(self):

        self.lidar_pub = rospy.Publisher("point_to_rviz", PointCloud2, queue_size=1)
        self.bridge = CvBridge()        
        self.lidar_sub = rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, self.callback, queue_size=1)
        if VERBOSE :
            print "\nsubscribed to /kitti/velo/pointcloud"
           

    def callback(self, ros_data):
        print "\npublish lidar_to_rviz\n\n"
        self.lidar_pub.publish(ros_data)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('kitti_to_rviz_node', anonymous=True)

    img_class = image_feature()
    lidar_class = lidar_feature()

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

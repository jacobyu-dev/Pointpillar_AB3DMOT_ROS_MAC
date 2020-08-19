#!/usr/bin/env python
import sys

import cv2

import roslib
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

VERBOSE=True

class image_feature:

    def __init__(self):

        self.image_pub = rospy.Publisher("image_to_rviz", Image,queue_size=1)
        self.bridge = CvBridge()        
        self.subscriber = rospy.Subscriber("/kitti/camera_color_left/image_raw", Image, self.callback,queue_size=1)
        if VERBOSE :
            print "\nsubscribed to /kitti/camera_color_left/image_raw"
            print "\npublish image_to_rviz\n\n"

    def callback(self, ros_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50,50), 10, 255)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)

    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

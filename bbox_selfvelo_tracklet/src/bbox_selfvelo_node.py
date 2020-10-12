#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
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
from jsk_rviz_plugins.msg import Pictogram              #sudo apt-get install ros-melodic-jsk-rviz-plugins
from jsk_rviz_plugins.msg import PictogramArray


from visualization_msgs.msg import Marker

import std_msgs
import os.path 


TRACKLET_PATH = '/home/jk/data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml'


#Global Variables
kitti_data = None
pictogram_texts = None

pub_boxes = None
pub_pictograms = None

pub_selfvelo_text = None
pub_selfveloDirection = None

class MoDetect_N_Track:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/kitti/velo/pointcloud",PointCloud2,self.callback,queue_size=1)
        self.subscriber2 = rospy.Subscriber("/kitti/oxts/gps/vel",TwistStamped,self.callback2,queue_size=1)

        self.pub_boxes = rospy.Publisher('kitti_box', BoundingBoxArray, queue_size=1)
        self.pub_pictograms = rospy.Publisher('kitti_box_pictogram', PictogramArray, queue_size=1)

        self.pub_selfvelo_text = rospy.Publisher('kitti_selfvelo_text', Marker, queue_size=1)
        self.pub_selfveloDirection = rospy.Publisher('kitti_selfvelo_direction', Marker, queue_size=1)



    def callback(self, data):

        header = data.header     
        frame = header.seq
        
        boxes = BoundingBoxArray() #3D Boxes with JSK
        boxes.header = header     #
        
        texts = PictogramArray() #Labels with JSK
        texts.header = header


        if kitti_data.has_key(frame) == True:
            for b in kitti_data[frame]:
                b.header = header
                boxes.boxes.append(b)

        if pictogram_texts.has_key(frame) == True:
            for txt in pictogram_texts[frame]:
                txt.header = header
                texts.pictograms.append(txt)

        self.pub_boxes.publish(boxes)
        self.pub_pictograms.publish(texts)


    def callback2(self, data):
        header = data.header     

        oxtLinear = data.twist.linear
        selfvelo = np.sqrt(oxtLinear.x ** 2 + oxtLinear.y ** 2 + oxtLinear.z ** 2)
        selfvelo = np.round_(selfvelo,1)    # m/s
        selfvelo = selfvelo * 3.6           # km/h
        
        oxtAngular = data.twist.angular
        q = tf.transformations.quaternion_from_euler(oxtAngular.x, oxtAngular.y, oxtAngular.z)

        text_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(1.0, 1.0, 1.0),
                header=header,
                color=ColorRGBA(1.0, 1.0, 1.0, 0.8),
                text="{}km/h".format(selfvelo))

        arrow_marker = Marker(
                type=Marker.ARROW,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(*q)),
                scale=Vector3(5.0, 0.5, 0.5),
                header=header,
                color=ColorRGBA(1.0, 0.0, 0.0, 0.8))

        self.pub_selfvelo_text.publish(text_marker)
        self.pub_selfveloDirection.publish(arrow_marker)


def readXML(file):
    tree = ET.parse(file)
    root = tree.getroot()
    
    item = root.findall('./tracklets/item')

    d = {}      
    boxes_2d = {}           #not used
    pictograms = {}

    for i, v in enumerate(item):
        h = float(v.find('h').text)
        w = float(v.find('w').text)
        l = float(v.find('l').text)
        frame = int(v.find('first_frame').text)
        size = Vector3(l, w, h)

        label = v.find('objectType').text

        pose = v.findall('./poses/item')

        for j, p in enumerate(pose):
            tx = float(p.find('tx').text)
            ty = float(p.find('ty').text)
            tz = float(p.find('tz').text)
            rz = float(p.find('rz').text)
            occlusion = float(p.find('occlusion').text)
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, rz)

            b = BoundingBox()
            b.pose.position = Vector3(tx, ty, tz/2.0)
            b.pose.orientation = Quaternion(*q)
            b.dimensions = size
            b.label = i
            
            picto_text = Pictogram()
            picto_text.mode = Pictogram.STRING_MODE
            picto_text.pose.position = Vector3(tx, ty, -tz/2.0)
            q = tf.transformations.quaternion_from_euler(0.7, 0.0, -0.7)
            picto_text.pose.orientation = Quaternion(0.0, -0.5, 0.0, 0.5)
            picto_text.size = 5
            picto_text.color = std_msgs.msg.ColorRGBA(1, 1, 1, 1)
            picto_text.character = label + "\n" + str(b.label)
            

            if d.has_key(frame + j) == True:
                d[frame + j].append(b)
                # boxes_2d[frame + j].append(bbox_2d)
                pictograms[frame + j].append(picto_text)
            else:
                d[frame + j] = [b]
                # boxes_2d[frame + j] = [bbox_2d]
                pictograms[frame + j]= [picto_text]

    return d, boxes_2d, pictograms      #boxes_2d is not used      



def main(args):
    global pub, pub_pictograms, kitti_data, pictogram_texts

    #Initializes and cleanup ros node with node name
    rospy.init_node('bbox_selfvelo_node', anonymous=True)

    kitti_data, _, pictogram_texts = readXML(TRACKLET_PATH)

    pcl_obj = MoDetect_N_Track() 

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module" 


if __name__ == '__main__':
    main(sys.argv)

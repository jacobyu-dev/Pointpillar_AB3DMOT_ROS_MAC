import sys

#import cv2

import roslib
import rospy

from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import pandas as pd
import geopandas as gp
import matplotlib.pyplot as plt

from shapely import wkt
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
from sklearn import linear_model

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import os
import struct
from ParticleFilter import ParticleFilter
import rviz_tools as rviz_tools

cnt = 0


def filter_by_mean_value(pointcloud_df):

    mean = pointcloud_df["Intensity"].mean()
    std = pointcloud_df["Intensity"].std()

    lanes_df = pointcloud_df[pointcloud_df["Intensity"] > mean + 1 * std]
    lanes_df = lanes_df[lanes_df["Intensity"] < mean + 7 * std ]
    
    return lanes_df
    
class lidar_feature:

    def __init__(self):

        #self.lidar_pub = rospy.Publisher("point_to_rviz", PointCloud2, queue_size=1)
        self.pub_line_min_dist = rospy.Publisher('line_to_rviz', Marker, queue_size=1)
        self.pub_line_min_dist2 = rospy.Publisher('line_to_rviz2', Marker, queue_size=1)
        rospy.loginfo('Publishing example line')
        self.bridge = CvBridge()        
        self.lidar_sub = rospy.Subscriber("/ground_cloud_intensity", PointCloud2, self.callback, queue_size=500)
        #if VERBOSE :
        print("\nsubscribed to /ground_cloud_intensity")

    def callback(self, ros_data):
        header = ros_data.header     
        frame = header.seq

        pc = pc2.read_points(ros_data,skip_nans=True,field_names=("x","y","z","intensity"))
        # sys.stdout = open('output_{}.txt'.format(frame),'w')#

        # for p in pc:
        #     print(" ".join(map(str,[p[0], p[1], p[2], p[3]])))#


        data=[]
        for p in pc:
            data.append([p[0], p[1], p[2], p[3]])
            
        print("Get data complete! \n")

        
        a=np.array(data)
        pointcloud_df = pd.DataFrame()
        pointcloud_df["X"] = a[:,0]
        pointcloud_df["Y"] = a[:,1]
        pointcloud_df["Z"] = a[:,2]
        i = a[:,3]*255
        ii=[]
        for x in i:
            ii.append(int(x))
        pointcloud_df["Intensity"] = ii

        def convert_fuse(pointcloud_df, min_x = 0.0, min_y = 0.0, min_z = 0.0):
            pointcloud_df["X"] = pd.to_numeric(pointcloud_df["X"])
            pointcloud_df["Y"] = pd.to_numeric(pointcloud_df["Y"])
            pointcloud_df["Z"] = pd.to_numeric(pointcloud_df["Z"])
            pointcloud_df["Intensity"] = pd.to_numeric(pointcloud_df["Intensity"])

            return pointcloud_df, (min_x, min_y, min_z), (zone_number, zone_letter)

        xyzi_df = pointcloud_df[["X", "Y", "Z", "Intensity"]]

        def filter_by_mean_value(pointcloud_df):

            mean = pointcloud_df["Intensity"].mean()
            std = pointcloud_df["Intensity"].std()

            lanes_df = pointcloud_df[pointcloud_df["Intensity"] > mean + 1 * std]
            lanes_df = lanes_df[lanes_df["Intensity"] < mean + 7 * std ]

            return lanes_df

        lanes_df = xyzi_df.copy()
        lanes_df = filter_by_mean_value(lanes_df)

        
        X = lanes_df[["X", "Y", "Z"]].values
        Easting=[]
        Northing=[]
        group=[]
        for i in X:
            if i[1]>4 and i[1]<6:
                if i[0]>-10 and i[0]<10:
                    Easting.append(i[0])
                    Northing.append(i[1])
                    group.append(1)
            elif i[1]>-1 and i[1]<1:
                if i[0]>-10 and i[0]<10:
                    Easting.append(i[0])
                    Northing.append(i[1])
                    group.append(2)
            elif i[1]>-3 and i[1]<-1:
                if i[0]>-10 and i[0]<10:
                    Easting.append(i[0])
                    Northing.append(i[1])
                    group.append(3)
            else:
                Easting.append(i[0])
                Northing.append(i[1])
                group.append(-1)
                    
        cluster_df2=pd.DataFrame({"X":Easting,"Y":Northing,"group":group})      


        ransac = linear_model.RANSACRegressor()
        ransaclines = []

        labels = cluster_df2["group"]
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        cluster_df2 = cluster_df2[cluster_df2["group"]>=0]

        def add_square_feature(X):
            X = np.concatenate([(X**2).reshape(-1,1), X], axis=1)
            return X

        for cluster in range(n_clusters_):
            sub_cluster_df = cluster_df2[cluster_df2["group"] == cluster]
            try:
                Xpoints = sub_cluster_df[["X"]].values
                Ypoints = sub_cluster_df[["Y"]].values
                Xpoints = Xpoints.reshape(-1,1)

                ransac.fit(add_square_feature(Xpoints), Ypoints)
                line_X = np.arange(Xpoints.min(), Xpoints.max())[:, np.newaxis]
                line_y_ransac = ransac.predict(add_square_feature(line_X))

                ransaclines.append([line_X,line_y_ransac])
            except:
                continue

        # plt.figure()
        # plt.xlim(-30,30)
        # plt.ylim(-30,30)
        # for l in ransaclines:
        #     plt.plot(l[0],l[1])
        # plt.show()   
        global cnt        
        cnt = cnt + 1        
        print("Finish {} Frame! \n".format(cnt))

        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        # marker line points
        marker.points = []

        marker2 = Marker()
        marker2.header.frame_id = "/base_link"
        marker2.type = marker2.LINE_LIST
        marker2.action = marker2.ADD

        # marker scale
        marker2.scale.x = 0.5
        marker2.scale.y = 0.5
        marker2.scale.z = 0.5
        # marker color
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 1.0
        marker2.color.b = 1.0
        # marker orientaiton
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0
        # marker position
        marker2.pose.position.x = 0.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0
        # marker line points
        marker2.points = []

        lane1startx=ransaclines[0][0][0][0]
        #lane1startx=-20
        lane1starty=ransaclines[0][1][0][0]

        first_line_point=Point()
        first_line_point.x=lane1startx
        first_line_point.y=lane1starty
        first_line_point.z=0.0
        marker.points.append(first_line_point)

        lane1endx=ransaclines[0][0][len(ransaclines[0][0])-1][0]
        lane1endy=ransaclines[0][1][len(ransaclines[0][1])-1][0]

        second_line_point=Point()
        second_line_point.x=lane1endx
        second_line_point.y=lane1endy
        second_line_point.z=0.0
        marker.points.append(second_line_point)

        lane2startx=ransaclines[1][0][0][0]
        lane2starty=ransaclines[1][1][0][0]

        lane2endx=ransaclines[1][0][len(ransaclines[1][0])-1][0]
        lane2endy=ransaclines[1][1][len(ransaclines[1][1])-1][0]

        for i in range(0,int(lane2endx-lane2startx),2):
            line_point=Point()
            line_point.x=lane2startx + i
            line_point.y=(lane2starty+lane2endy)/2
            line_point.z=0.0
            marker2.points.append(line_point)    

        lane3startx=ransaclines[2][0][0][0]
        lane3starty=ransaclines[2][1][0][0]

        fifth_line_point=Point()
        fifth_line_point.x=lane3startx
        fifth_line_point.y=lane3starty
        fifth_line_point.z=0.0
        marker.points.append(fifth_line_point)        

        lane3endx=ransaclines[2][0][len(ransaclines[2][0])-1][0]
        lane3endy=ransaclines[2][1][len(ransaclines[2][1])-1][0]

        sixth_line_point=Point()
        sixth_line_point.x=lane3endx
        sixth_line_point.y=lane3endy
        sixth_line_point.z=0.0
        marker.points.append(sixth_line_point) 

        markers = rviz_tools.RvizMarkers('/base_link', 'visualization_marker')
        point1 = Point(lane3startx,lane3starty,0) 
        point2 = Point(lane1endx,lane1endy,0) 
    
        markers.publishRectangle(point1, point2, 'black', 5.0)

        self.pub_line_min_dist2.publish(marker2)
        self.pub_line_min_dist.publish(marker)
        
            
def main(args):

    rospy.init_node('kitti_to_rviz_node', anonymous=True)

    lidar_class = lidar_feature()
    
    rospy.spin()
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



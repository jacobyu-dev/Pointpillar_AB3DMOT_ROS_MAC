import sys

import cv2

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

import lane_func

VERBOSE=True

class lidar_feature:

    def __init__(self):

        self.lidar_pub = rospy.Publisher("point_to_rviz", PointCloud2, queue_size=1)
        self.bridge = CvBridge()        
        self.lidar_sub = rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, self.callback, queue_size=80)
        if VERBOSE :
            print "\nsubscribed to /kitti/velo/pointcloud"

    def callback(self, ros_data):
        header = ros_data.header     
        frame = header.seq

        pc = pc2.read_points(ros_data,skip_nans=True,field_names=("x","y","z","i"))

        data=[]
        for p in pc:
            data.append([p[0], p[1], p[2], p[3]])
            
        print("Get data complete! \n")

        
        a=np.array(data)

        pointcloud_df = pd.DataFrame()
        pointcloud_df["Latitude"] = a[:,0]
        pointcloud_df["Longitude"] = a[:,1]
        pointcloud_df["Altitude"] = a[:,2]
        pointcloud_df["Intensity"] = a[:,3]

        pointcloud_df, (min_x, min_y, min_z), (number, letter) = lane_func.convert_fuse(pointcloud_df)
        
        print("Finish convert coordinates! \n")
        ## It takes a long time...
        ## The UTM coordinate system offers the following benefits:
        ## 1. A square grid -> UTM지도의 어느 곳에서나 일정한 거리 관계를 제공합니다. 
        ##                  ->위도 및 경도와 같은 각도 좌표계에서 경도가 포함하는 거리는 극점을 향해 이동할 때 달라지며 적도에서 위도에 포함되는 거리와 만 같습니다.
        ## 2. No negative numbers or East-West designators
        ## 3. Coordinates are measured in metric units

        xyzi_df = pointcloud_df[["Easting", "Northing", "Altitude", "Intensity"]]
        xyzi_df.to_csv("./pointcloud.xyz", sep=" ", header=False, index=False)

        lanes_df = xyzi_df.copy()
        lanes_df = lane_func.filter_by_mean_value(lanes_df)
        lanes_df[['Easting', 'Northing', 'Altitude', 'Intensity']].to_csv("./filter_mean.xyz", index=False)

        print("Finish Filtering! \n")

        X = lanes_df[["Easting", "Northing", "Altitude"]].values
        X = StandardScaler().fit_transform(X)

        db = DBSCAN(eps=0.1, min_samples=40).fit(X)

        labels = db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

        lanes_df["Group"] = labels
        cluster_df = lanes_df[["Easting", "Northing", "Altitude", "Group"]]
        cluster_df = cluster_df[cluster_df["Group"]>=0]
        cluster_df.to_csv("./clustering.xyz",index=False)

        lines = []

        for cluster in range(n_clusters_):
            sub_cluster_df = cluster_df[cluster_df["Group"] == cluster]
            points = sub_cluster_df[["Easting","Northing", "Altitude"]].values
            distances = squareform(pdist(points))
            for i in range(0,15):
                max_index = np.argmax(distances)
                i1, i2 = np.unravel_index(max_index, distances.shape)
                distances[i1,i2] = 0.0
            max_dist = np.max(distances)
            max_index = np.argmax(distances)
            i1, i2 = np.unravel_index(max_index, distances.shape)
            p1 = sub_cluster_df.iloc[i1]
            p2 = sub_cluster_df.iloc[i2]
            lines.append(([p1["Easting"], p2["Easting"]],[p1["Northing"], p2["Northing"]], [p1["Altitude"], p2["Altitude"]]))
                
        plt.figure(figsize=(20,10))
        plt.xlim(-1000, 700000), plt.ylim(0,2000000) ## 

        for l in lines:
            plt.plot(l[0], l[1], l[2])
            
        plt.show()
        print("Finish one Frame! \n")
        #print("Number of prototype lane markings: ", len(lines))
        

def main(args):

    rospy.init_node('kitti_to_rviz_node', anonymous=True)

    lidar_class = lidar_feature()

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)




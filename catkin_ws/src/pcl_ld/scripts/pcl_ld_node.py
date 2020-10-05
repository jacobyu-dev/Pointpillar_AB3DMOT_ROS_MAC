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
from sklearn import linear_model



cnt = 1



def convert_fuse(pointcloud_df, min_x = 0.0, min_y = 0.0, min_z = 0.0):
    pointcloud_df["Latitude"] = pd.to_numeric(pointcloud_df["Latitude"])
    pointcloud_df["Longitude"] = pd.to_numeric(pointcloud_df["Longitude"])
    pointcloud_df["Altitude"] = pd.to_numeric(pointcloud_df["Altitude"])
    pointcloud_df["Intensity"] = pd.to_numeric(pointcloud_df["Intensity"])
    
    pointcloud_df["Easting"] = pointcloud_df["Latitude"]
    pointcloud_df["Northing"] = pointcloud_df["Longitude"]
    return pointcloud_df, (min_x, min_y, min_z), (zone_number, zone_letter)

def filter_by_mean_value(pointcloud_df):

    mean = pointcloud_df["Intensity"].mean()
    std = pointcloud_df["Intensity"].std()

    lanes_df = pointcloud_df[pointcloud_df["Intensity"] > mean + 1 * std]
    lanes_df = lanes_df[lanes_df["Intensity"] < mean + 7 * std ]
    
    return lanes_df
    
class lidar_feature:

    def __init__(self):

        self.lidar_pub = rospy.Publisher("point_to_rviz", PointCloud2, queue_size=1)
        self.bridge = CvBridge()        
        self.lidar_sub = rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, self.callback, queue_size=500)
        if VERBOSE :
            print("\nsubscribed to /kitti/velo/pointcloud")

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

        pointcloud_df, (min_x, min_y, min_z), (number, letter) = convert_fuse(pointcloud_df)
        
        print("Finish convert coordinates! \n")

        xyzi_df = pointcloud_df[["Easting", "Northing", "Altitude", "Intensity"]]

        lanes_df = xyzi_df.copy()
        lanes_df = filter_by_mean_value(lanes_df)
        lanes_df[['Easting', 'Northing', 'Altitude', 'Intensity']].to_csv("./filter_mean.xyz", index=False)

        print("Finish Filtering! \n")
        
        X = lanes_df[["Easting", "Northing", "Altitude"]].values

        db = DBSCAN(eps=1, min_samples=30).fit(X)

        labels = db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

        lanes_df["Group"] = labels
        cluster_df = lanes_df[["Easting", "Northing", "Altitude", "Group"]]
        cluster_df = cluster_df[cluster_df["Group"]>=0]

        ransac = linear_model.RANSACRegressor()
        ransaclines = []

        for cluster in range(n_clusters_):
            sub_cluster_df = cluster_df[cluster_df["Group"] == cluster]
            Xpoints = sub_cluster_df[["Easting"]].values
            Ypoints = sub_cluster_df[["Northing"]].values
            ransac.fit(Xpoints, Ypoints)
            line_X = np.arange(Xpoints.min(), Xpoints.max())[:, np.newaxis]
            line_y_ransac = ransac.predict(line_X)

            ransaclines.append([line_X,line_y_ransac])
                
        #plt.figure(figsize=(20,10))
        #plt.xlim(-30,30)
        #plt.ylim(-30,30)

        #for l in ransaclines:
        #    plt.plot(l[0], l[1])
            
        #plt.show()
        print("Number of prototype lane markings: \n", len(ransaclines))
        global cnt
        print("Finish {} Frame! \n".format(cnt))
        cnt = cnt + 1
        #print("Number of prototype lane markings: ", len(lines))
        

def main(args):

    rospy.init_node('kitti_to_rviz_node', anonymous=True)

    lidar_class = lidar_feature()

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)




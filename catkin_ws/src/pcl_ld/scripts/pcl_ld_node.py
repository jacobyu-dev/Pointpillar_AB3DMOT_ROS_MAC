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

import utm
from numba import jit, cuda 

import warnings
warnings.filterwarnings('ignore')
VERBOSE=True
cnt = 1

@jit
def utm_convert_E(x):
    return utm.from_latlon(x["Latitude"], x["Longitude"])[0]

@jit
def utm_convert_N(x):
    return utm.from_latlon(x["Latitude"], x["Longitude"])[1]

def convert_fuse(pointcloud_df, min_x = 0.0, min_y = 0.0, min_z = 0.0):
    pointcloud_df["Easting"] = pointcloud_df.apply(utm_convert_E, axis = 1)
    pointcloud_df["Northing"] = pointcloud_df.apply(utm_convert_N, axis = 1)


    
    min_x = pointcloud_df["Easting"].min()
    min_y = pointcloud_df["Northing"].min()   
    min_z = pointcloud_df["Altitude"].min()    
        
    utm_coords = utm.from_latlon(pointcloud_df.loc[0,"Latitude"], pointcloud_df.loc[0,"Longitude"])

    zone_number = utm_coords[2]
    zone_letter = utm_coords[3]
        
    # negative to positive ? maybe
    pointcloud_df["Easting"] = pointcloud_df["Easting"] - min_x    
    pointcloud_df["Northing"] = pointcloud_df["Northing"] - min_y
    pointcloud_df["Altitude"] = pointcloud_df["Altitude"] - min_z
    
    return pointcloud_df, (min_x, min_y, min_z), (zone_number, zone_letter)

def filter_by_mean_value(pointcloud_df):

    mean = pointcloud_df["Intensity"].mean()
    std = pointcloud_df["Intensity"].std()

    lanes_df = pointcloud_df[pointcloud_df["Intensity"] > mean + 1 * std]
    lanes_df = lanes_df[lanes_df["Intensity"] < mean +  9 * std ]

#    print("MEAN FILTER:")
#    print("============")
#    print("Intensity - Mean value:      ", mean)
#    print("Intensity - Std value:       ", std)
#    print("Intensity - Lower bound:     ", mean + 1 * std)
#    print("Intensity - Upper bound:     ", mean + 9 * std)
#    print("Intensity - Filtered points: ", len(lanes_df))
#    print("Intensity - Original points: ", len(pointcloud_df))
#    print("Intensity - Reduction to %:  ", len(lanes_df)/len(pointcloud_df))
    
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
        xyzi_df.to_csv("./pointcloud.xyz", sep=" ", header=False, index=False)

        lanes_df = xyzi_df.copy()
        lanes_df = filter_by_mean_value(lanes_df)
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
                
        #plt.figure(figsize=(20,10))
        #plt.xlim(-1000, 700000), plt.ylim(0,2000000) ## 

        #for l in lines:
        #    plt.plot(l[0], l[1], l[2])
            
        #plt.show()
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




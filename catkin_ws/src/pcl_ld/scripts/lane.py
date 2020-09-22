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

data=[]
with open('./output_data_26(0027).txt') as f:
    line = f.readline()
    while line:
        d=line.split()
        tmp=[]
        for i in d:
            tmp.append(float(i)) # convert string to float
        data.append(tmp)
        line=f.readline()

a=np.array(data)

### assumption: There's no void space line in txt file.
### intensity: 0~255
pointcloud_df = pd.DataFrame()
pointcloud_df["Latitude"] = a[:,0]
pointcloud_df["Longitude"] = a[:,1]
pointcloud_df["Altitude"] = a[:,2]
i = a[:,3]*255
ii=[]
for x in i:
    ii.append(int(x))
pointcloud_df["Intensity"] = ii


def convert_fuse(pointcloud_df, min_x = 0.0, min_y = 0.0, min_z = 0.0):
    pointcloud_df["Latitude"] = pd.to_numeric(pointcloud_df["Latitude"])
    pointcloud_df["Longitude"] = pd.to_numeric(pointcloud_df["Longitude"])
    pointcloud_df["Altitude"] = pd.to_numeric(pointcloud_df["Altitude"])
    pointcloud_df["Intensity"] = pd.to_numeric(pointcloud_df["Intensity"])
    
    #Coordinate Conversions
    pointcloud_df["Easting"] = pointcloud_df.apply(lambda x: utm.from_latlon(x["Latitude"], x["Longitude"])[0], axis = 1)
    pointcloud_df["Northing"] = pointcloud_df.apply(lambda x: utm.from_latlon(x["Latitude"], x["Longitude"])[1], axis = 1)
    
    if min_x == 0:
        min_x = pointcloud_df["Easting"].min()
    if min_y == 0:
        min_y = pointcloud_df["Northing"].min()   
    if min_z == 0:
        min_z = pointcloud_df["Altitude"].min()    
        
    utm_coords = utm.from_latlon(pointcloud_df.loc[0,"Latitude"], pointcloud_df.loc[0,"Longitude"])

    zone_number = utm_coords[2]
    zone_letter = utm_coords[3]
        
    # negative to positive ? maybe
    pointcloud_df["Easting"] = pointcloud_df["Easting"] - min_x    
    pointcloud_df["Northing"] = pointcloud_df["Northing"] - min_y
    pointcloud_df["Altitude"] = pointcloud_df["Altitude"] - min_z
    
    return pointcloud_df, (min_x, min_y, min_z), (zone_number, zone_letter)

pointcloud_df, (min_x, min_y, min_z), (number, letter) = convert_fuse(pointcloud_df)
xyzi_df = pointcloud_df[["Easting", "Northing", "Altitude", "Intensity"]]
xyzi_df.to_csv("./pointcloud.xyz", sep=" ", header=False, index=False)

def convert_to_shape(row):
    return wkt.loads("POINT (" + str(row["Easting"]) + " " + str(row["Northing"]) +" " + str(row["Altitude"]) + ")")

def filter_by_mean_value(pointcloud_df):

    mean = pointcloud_df["Intensity"].mean()
    std = pointcloud_df["Intensity"].std()

    lanes_df = pointcloud_df[pointcloud_df["Intensity"] > mean + 1 * std]
    lanes_df = lanes_df[lanes_df["Intensity"] < mean +  9 * std ]

    print("MEAN FILTER:")
    print("============")
    print("Intensity - Mean value:      ", mean)
    print("Intensity - Std value:       ", std)
    print("Intensity - Lower bound:     ", mean + 1 * std)
    print("Intensity - Upper bound:     ", mean + 9 * std)
    print("Intensity - Filtered points: ", len(lanes_df))
    print("Intensity - Original points: ", len(pointcloud_df))
    print("Intensity - Reduction to %:  ", len(lanes_df)/len(pointcloud_df))
    
    return lanes_df

lanes_df = xyzi_df.copy()
lanes_df = filter_by_mean_value(lanes_df)
lanes_df[['Easting', 'Northing', 'Altitude', 'Intensity']].to_csv("./filter_mean.xyz", index=False)

X = lanes_df[["Easting", "Northing", "Altitude"]].values
X = StandardScaler().fit_transform(X)

db = DBSCAN(eps=0.1, min_samples=40).fit(X)
#eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other. (default=0.5)
#min_samplesint: The number of samples (or total weight) in a neighborhood for a point to be considered as a core point.(default=5)

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

print("Number of prototype lane markings: ", len(lines))

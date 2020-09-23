## Reference: https://github.com/Lukas-Justen/Lane-Marking-Detection  
## ros workspace: https://github.com/seo-dev/KCSY/tree/hrkim/catkin_ws  

## 0921  
1. modify kitti2bag file: add seq -> https://github.com/seo-dev/KCSY/issues/9  
2. modify pcl_ld_node.py callback function  
```
def callback(self, ros_data):
    header = ros_data.header     
    frame = header.seq

    pc = pc2.read_points(ros_data,skip_nans=True,field_names=("x","y","z","i"))
    sys.stdout = open('output_{}.txt'.format(frame),'w')

    for p in pc:
        print(" ".join(map(str,[p[0], p[1], p[2], p[3]])))

    self.lidar_pub.publish(ros_data)
  
```
3. modify Pointcloud_LaneMarking_Detection.ipynb  
```
data=[]
with open('./output_data_26(0027).txt') as f:
    line = f.readline()
    while line:
        d=line.split()
        tmp=[]
        for i in d:
            tmp.append(float(i))
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
```

## 0922

1. change parameter in Pointcloud_LaneMarking_Detection.ipynb  

```
def filter_by_mean_value(pointcloud_df):
    lanes_df = pointcloud_df[pointcloud_df["Intensity"] > mean + 1 * std]
    lanes_df = lanes_df[lanes_df["Intensity"] < mean +  9 * std ]

```
```
db = DBSCAN(eps=0.1, min_samples=40).fit(X)
#eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other. (default=0.5)
#min_samplesint: The number of samples (or total weight) in a neighborhood for a point to be considered as a core point.(default=5)
```
2. modify pcl_ld_node.py: remove make txt file. It can caculate multiple frames.  
https://github.com/seo-dev/KCSY/tree/hrkim/catkin_ws/src/pcl_ld/scripts  

+ The UTM coordinate system offers the following benefits:  
    + A square grid    
                 -> UTM지도의 어느 곳에서나 일정한 거리 관계를 제공합니다.   
                 -> 위도 및 경도와 같은 각도 좌표계에서 경도가 포함하는 거리는 극점을 향해 이동할 때 달라지며 적도에서 위도에 포함되는 거리와 만 같습니다.  
    + No negative numbers or East-West designators  
    + Coordinates are measured in metric units  

> problem: utm conversion takes too long time.  
> solution:  
> 1. assumption: There's no difference in langitude, longitude in specific dataset.  
> 2. extract road point cloud. (no whole frame point cloud)   

## 0923

1. assumption: There's no difference in langitude, longitude in specific dataset.   
+ Both polar coordinate and cartesian cooridnate makes wrong information like below picture   
+ Changing dbscan parameter makes no difference.   
![image](https://user-images.githubusercontent.com/44723287/94007865-07e0e380-fddd-11ea-9763-a23cec6b7087.png)

2. use gpu computing
+ There's no difference in speed..  
```
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
 ```

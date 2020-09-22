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

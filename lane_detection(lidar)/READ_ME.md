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

*add frame count in call back function   
```
global cnt
print("Finish {} Frame! \n".format(cnt))
cnt = cnt + 1
```
![image](https://user-images.githubusercontent.com/44723287/94008816-9013b880-fdde-11ea-9318-a23476ca21ce.png)

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

## 0924

+ Analyze the code with meshlab: install meshlab and save to ply file.
```
def write_pointcloud(filename,xyz_points,rgb_points=None):

    """ creates a .pkl file of the point clouds generated
    """

    assert xyz_points.shape[1] == 3,'Input XYZ points should be Nx3 float array'
    if rgb_points is None:
        rgb_points = np.ones(xyz_points.shape).astype(np.uint8)*255
    assert xyz_points.shape == rgb_points.shape,'Input RGB colors should be Nx3 float array and have same size as input XYZ points'

    # Write header of .ply file
    fid = open(filename,'wb')
    fid.write(bytes('ply\n', 'utf-8'))
    fid.write(bytes('format binary_little_endian 1.0\n', 'utf-8'))
    fid.write(bytes('element vertex %d\n'%xyz_points.shape[0], 'utf-8'))
    fid.write(bytes('property float x\n', 'utf-8'))
    fid.write(bytes('property float y\n', 'utf-8'))
    fid.write(bytes('property float z\n', 'utf-8'))
    fid.write(bytes('property uchar red\n', 'utf-8'))
    fid.write(bytes('property uchar green\n', 'utf-8'))
    fid.write(bytes('property uchar blue\n', 'utf-8'))
    fid.write(bytes('end_header\n', 'utf-8'))

    # Write 3D points to .ply file
    for i in range(xyz_points.shape[0]):
        fid.write(bytearray(struct.pack("fffccc",xyz_points[i,0],xyz_points[i,1],xyz_points[i,2],
                                        rgb_points[i,0].tostring(),rgb_points[i,1].tostring(),
                                        rgb_points[i,2].tostring())))
    fid.close()


```
> Reference: https://gist.github.com/Shreeyak/9a4948891541cb32b501d058db227fff  
  
1. original point(no utm conversion)    
![image](https://user-images.githubusercontent.com/44723287/94144527-34176580-feac-11ea-927a-d61a0f6404a1.png)  
  
It works well, and good at lane detection like below picture.  we can save time !!   
![image](https://user-images.githubusercontent.com/44723287/94144733-7b055b00-feac-11ea-9c09-6c6c71f51b21.png)  
  
2. Try to add color to clustering  

```
#(0,0,0) , (255/number_of_cluster,255/number_of_cluster,255/number_of_cluster)
# (255/number_of_cluster,255/number_of_cluster,255/number_of_cluster)*2 ,,,

temp_cluster = pd.DataFrame()
for cluster in range(n_clusters_):
    sub_cluster_df = cluster_df[cluster_df["Group"] == cluster]
    tmp = int(255/(n_clusters_+1)*(cluster+1))
    
    sub_cluster_df["r"]=tmp
    sub_cluster_df["g"]=tmp
    sub_cluster_df["b"]=tmp
    cluster_df[cluster_df["Group"] == cluster] = sub_cluster_df
    

sub_cluster_df = cluster_df[cluster_df["Group"] == -1]   
sub_cluster_df["r"]=0
sub_cluster_df["g"]=0
sub_cluster_df["b"]=0
cluster_df[cluster_df["Group"] == -1]=sub_cluster_df
```
+ future work  
  
1. change clustering algorithm(no dbscan)
2. remove noise
3. line fitting algorithm

>> 차선 rviz에 뿌리기, 슬램 결과 포인트 클라우드 이용??  
>> gpu 사용여부 확인, 성능비교할때는 시간측정해서 자료 준비, 각각의 단계별로 이미지사진 준비 

## 1005 
+ change clustering algorithm  

scikit learn 클러스터링 알고리즘 중 클러스터 수를 인자로 받지 않는 알고리즘. 인자는 클러스터 수를 비슷하게 맞추는 방향으로 잡았음.  
  
1. dbscan 인자: eps=1, min_samples=30 시간:0.35초 클러스터 수:21 결과사진  
![dbscan](https://user-images.githubusercontent.com/44723287/95050164-0b9e2f80-0726-11eb-81c5-1eea0dae671b.png)

2. Mean-shift 인자: bandwidth=5 시간: 277초 클러스터 수:15  결과사진 
![image](https://user-images.githubusercontent.com/44723287/95049562-f4ab0d80-0724-11eb-9ac9-8c1974ab04f5.png)

3. OPTICS 인자: min_samples=50 시간: 48초 클러스터 수:19 결과사진  
![OPTICS](https://user-images.githubusercontent.com/44723287/95050052-e3163580-0725-11eb-946c-15fbc67aba5c.png)




## Reference: https://github.com/Lukas-Justen/Lane-Marking-Detection  
## ros workspace: https://github.com/seo-dev/KCSY/tree/hrkim/catkin_ws  

+ 0921  
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

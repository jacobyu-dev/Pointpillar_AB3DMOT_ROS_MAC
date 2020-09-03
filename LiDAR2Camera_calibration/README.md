# Lidar-Camera Projection
This code is used to explain the post [here](https://medium.com/@daryl.tanyj/camera-lidar-projection-navigating-between-2d-and-3d-911c78167a94).

https://www.notion.so/Camera-Lidar-Projection-Navigating-between-2D-and-3D-by-Daryl-Tan-The-Startup-Medium-229089a5c9b14edcb9ba2f8e5c2a871b

## Dependency Installation
Assuming you have installed anaconda. https://www.anaconda.com/distribution/#download-section

Get all dependencies with
```
conda create -n ros_env --file requirement.txt

conda activate ros_env
```

## How to use
copy calib_pkg & ros_numpy to catkin_ws

+ 안하면 ros_numpy 못찾음 
```
cd ros_numpy/  
python setup.py install
```

https://github.com/tomas789/kitti2bag 에서 만들어진 bag 파일 이용

```
roscore

rosbag play -l kitti.bag

rosrun calib_pkg Lidar2cam_calib.py

```
#### rqt_graph

![image](https://user-images.githubusercontent.com/68947288/91969872-c4e99e00-ed51-11ea-9f80-6fe737f0c216.png)

#### rviz
![image (1)](https://user-images.githubusercontent.com/68947288/91969879-c6b36180-ed51-11ea-96ac-5d78976da6c8.png)

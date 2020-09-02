아나콘다 설치

https://www.anaconda.com/distribution/#download-section

conda create -n ros_env --file requirement.txt

conda activate ros_env

calib_pkg 와 ros_numpy 파일을 catkin_ws 폴더로 복사

roscore

rosbag play -l kitti.bag

rosrun calib_pkg Lidar2cam_calib.py

![image](https://user-images.githubusercontent.com/68947288/91969872-c4e99e00-ed51-11ea-9f80-6fe737f0c216.png)
![image (1)](https://user-images.githubusercontent.com/68947288/91969879-c6b36180-ed51-11ea-96ac-5d78976da6c8.png)

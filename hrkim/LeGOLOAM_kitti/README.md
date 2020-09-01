+ 환경설정 

1. cd ~/catkin_ws/src
2. git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
3. cd ~/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include && vim utility.h
4. modifiy code 

extern const string pointCloudTopic = "/kitti/velo/pointcloud"; <- you should check your own bag file topic

//param for vel-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 50;

5. cd ~/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/src && vim featureAssociation.cpp
6. modify code

float s 10 * (pi->intensity - int(pi->intensity)); -> float s = 1;

// to delete all the code that corrects point cloud distortion
TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]); -> removed
TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]); -> removed

7. cd ~/ && git clone https://github.com/Mitchell-Lee-93/kitti-lego-loam.git
8. cp -r ~/kitti-lego-loam/LeGO-LOAM/LeGO-LOAM/src ~/catkin_ws/src/LeGO-LOAM/LeGO-LOAM


+ 키티 데이타셋 다운받고 로스백으로 바꾸기 

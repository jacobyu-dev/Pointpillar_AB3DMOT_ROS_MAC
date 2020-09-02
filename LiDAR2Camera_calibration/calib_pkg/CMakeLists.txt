cmake_minimum_required(VERSION 3.0.2)
project(Calib_pkg)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  LIBRARIES opencv
  CATKIN_DEPENDS cv_bridge sensor_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 3.0.2)
project(pcl_ld)
set(PCL_DIR "/home/mds/Downloads/debian/patches/")
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  PCL
)
catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node src/pcl_ld_node.cpp)
target_link_libraries(${PROJECT_NAME}_node 
	${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
  	${PCL_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(rosbag_kitti_test)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)




add_message_files(
   FILES
   MsgTutorial.msg
 )


generate_messages(
   DEPENDENCIES
   std_msgs 
 )



catkin_package(
   LIBRARIES rosbag_kitti_test
   CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(kitti_subscriber src/kitti_subscriber.cpp)
add_dependencies(kitti_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kitti_subscriber
   ${catkin_LIBRARIES}
)



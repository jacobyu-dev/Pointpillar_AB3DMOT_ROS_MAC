cmake_minimum_required(VERSION 3.0.2)
project(bbox_from_tracklet)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
)


#add_message_files(
#   FILES
#   pcl.msg
#)


generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
  LIBRARIES bbox_from_tracklet
  DEPENDS rviz
  CATKIN_DEPENDS rviz message_generation roscpp rospy std_msgs jsk_rviz_plugins
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)


# Building option for "boundingbox_topic" node
add_executable(bounding_pubsub src/bounding_pubsub.cpp)
add_dependencies(bounding_pubsub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bounding_pubsub
   ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS src/bounding_pubsub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

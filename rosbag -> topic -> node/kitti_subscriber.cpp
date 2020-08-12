#include "ros/ros.h"                          
#include <sensor_msgs/PointCloud2.h>


void msgCallback(const sensor_msgs::PointCloud2& msg)
{
  ROS_INFO("recieve msg = %d", msg.height);   
     
}

int main(int argc, char **argv)                        
{
  ros::init(argc, argv, "kitti_subscriber");           
  ros::NodeHandle nh;                                   

  
  ros::Subscriber ros_tutorial_sub = nh.subscribe("/kitti/velo/pointcloud", 100, msgCallback);

  
  ros::spin();

  return 0;
}

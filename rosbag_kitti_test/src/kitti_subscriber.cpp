#include "ros/ros.h"                          
#include <sensor_msgs/PointCloud2.h>

class pubSub
{
    public:
    pubSub() { 
      pub = nh.advertise<sensor_msgs::PointCloud2>("MsgToRviz", 100); 
      sub = nh.subscribe("/kitti/velo/pointcloud", 100, &pubSub::msgCallback, this); 
    }

    private:
    ros::NodeHandle nh;                                   
    ros::Publisher pub;
    ros::Subscriber sub;

    void msgCallback(const sensor_msgs::PointCloud2& msg)
    {
        ROS_INFO("recieve msg = %d", msg.height);   
	     
        pub.publish(msg);          
    }
};



int main(int argc, char **argv)                        
{
  ros::init(argc, argv, "kitti_subscriber");           
  ros::NodeHandle nh;                                   
  pubSub pubsubOject;
  ros::spin();

  return 0;
}

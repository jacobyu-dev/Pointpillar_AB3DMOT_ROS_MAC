# Message from rosbag(kitti.bag) publish & subscribe to Rviz

- 필요한 코드 : pubsub 패키지 



#### Package 생성
catkin_create_pkg rosbag_kitti_test message_generation std_msgs roscpp

위 문장을 실행하여 rosbag_kitti_test(원하는 이름으로 변경가능) 라는 package를 생성한다.

#### Package.xml 수정
package.xml을 다음과 같이 수정한다.
```xml
<?xml version="1.0"?>
<package format="2">
  <name>rosbag_kitti_test</name>
  <version>0.0.0</version>
  <description>The rosbag_kitti_test package</description>

  
  <maintainer email="jk@todo.todo">jk</maintainer>



  <license>BSD</license>

  
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>


  <export>

  </export>
</package>
```


#### Cmakelist.txt 수정
Cmakelist.txt을 다음과 같이 수정한다.
```txt
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

```

#### rosbag파일을 publish하고 subscribe하는 pubsub노드를 cpp파일로 생성한다.
kitti_subscriber.cpp
```cpp
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

```
#### 실행방법
[터미널 1] : roscore 실행 
- roscore

[터미널 2] rosbag 파일 play
- rosbag play -l kitti.bag

[터미널 3] : rosbag_kitti_test 패키지의 kitti_subscriber노드 실행
- rosrun rosbag_kitti_test kitti_subscriber

[터미널 4] : 
- rviz
![image](https://user-images.githubusercontent.com/59205405/90096685-ed900080-dd6e-11ea-902f-5891ae04c467.png)


[터미널 5]
- rqt_graph ( MsgToRviz 추가 전 )
![image](https://user-images.githubusercontent.com/59205405/90096761-28923400-dd6f-11ea-99cf-bca9c4d6c302.png)

-  rqt_graph (MsgToRviz 추가 후) ![image](https://user-images.githubusercontent.com/59205405/90096874-8de62500-dd6f-11ea-9852-8207a689cfe6.png)



[터미널 6] : Msg 확인용(실행할 필요없음)
- rostopic list

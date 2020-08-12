github.com/ros/ros_tutorials 에 들어가 turtlesim  package를 다운받는다

아래 경로에 turtlesim package를 복사 붙여넣는다.  (이름은 turtlesim -> turtlesim2(바꾸고 싶은 아무 이름) 으로 바꾼다.)
![image](https://user-images.githubusercontent.com/59205405/89876232-535a7c00-dbf9-11ea-877f-e01e58df0fc1.png)


turtlesim2 에서 package.xml 과 CMakelist.txt  파일을 바꾸어 주어야 한다.


package.xml 에서  project(turtlesim)  -> project(turtlesim2)  로 바꾸어준다. (package이름과 project() 내의 인자는 같아야한다.)
![image](https://user-images.githubusercontent.com/59205405/89876475-a92f2400-dbf9-11ea-879a-dba02ad33ce7.png)


CMakelist.txt에서 <name>turtlesim2</name>  -> <name>turtlesim2</name>  로 바꾸어준다. (package이름과 <name>태그 내의 인자는 같아야한다.)
![image](https://user-images.githubusercontent.com/59205405/89876626-df6ca380-dbf9-11ea-87ee-19d1f7f7cbb7.png)

위 과정을 거친 후 ~/catkin_ws  경로에서 catkin_make를 하면 dependency때문에 오류가 난다. ( add_dependencies(turtlesim_node turtlesim_gencpp) 의 turtlesim_gencpp 파일이 없기 때문!!)

**Solution )**
rosdep check turtlesim2 (바꾸어준 pkg)

rosdep install turtlesim2 


위 2개의 명령어를 실행시킨 후에는 catkin_make가 잘된다.

**turtlesim을 Test한다.**
roscd turtlesim2    : (~/catkin_ws/src/turltesim2)
rosrun turtlesim2 turtlesim_node   : (turtlesim_node의 이름이 turtlesim2_node 가 아닌 이유는 복사 한 package에서 따로 수정할 필요가  없기때문에 수정하지 않았다.)
rosrun turtlesim2 turtle_tele_op




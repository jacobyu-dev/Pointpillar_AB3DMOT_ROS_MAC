# Darknet_ros YOLO GPU Setting 
설치 시 그래픽카드 종유에 따라 설치해야하는 버전들이 다를 수 있으니 호환성을 고려하여야 한다.
참조 : https://m.blog.naver.com/sw4r/221744353526
https://coding-groot.tistory.com/87

## Setting
- Ubuntu 18.04
- Nvidia geforce 940MX
- Nvidia graphic driver 450
- Cuda toolkit 10.0 (Tensorflow 1.x 버전을 사용하기 위해 10.0을 선택)
- Cudnn 7.5 (cuda 10.0 호환성에 맞추어 선택)
- ROS Melodic

## Setting 순서

**1. Nvidia graphic driver 설치**
	A. Ubuntu "Additional drivers" 옵션을 사용하여 설치
참조 : https://www.cyberciti.biz/faq/ubuntu-linux-install-nvidia-driver-latest-proprietary-driver/
	   
 B. APT로 드라이버 설치
	   
   C. ubuntu-drivers로 드라이버 자동설치
  - B, C는 다음을 참조 https://codechacha.com/ko/install-nvidia-driver-ubuntu/

D. 드라이버 수동설치(Ubuntu18.04)
	   https://www.nvidia.com/Download/index.aspx 에서 다운로드 받아 설치한다.

위 4가지 설치방법 중 A방법을 선택하여 설치를 진행하였다. 이유는 가장 간단하고 정확하게 Driver 설치를 마칠 수 있기 때문

**2. Cuda toolkit 10.0 설치**
 
**3. Cudnn 7.5 설치**

 https://medium.com/@exesse/cuda-10-1-installation-on-ubuntu-18-04-lts-d04f89287130 다음 사이트에서 **cuda toolkit 10.0**과 **cudnn 7.5 설치**를 진행하였다.
 다만, 
 ![image](https://user-images.githubusercontent.com/59205405/91957072-87304980-ed40-11ea-9bda-610a793a1772.png)
 sudo apt install cuda-10-1 대신에 sudo apt install cuda-10-0 으로 설치를 진행
 
 아래 sudo vi ~/.profile 과정 대신에
![image](https://user-images.githubusercontent.com/59205405/91957587-310fd600-ed41-11ea-9e63-decc2ca7fdff.png)

아래 과정을 진행해 준다. 
참조 : https://greedywyatt.tistory.com/106
![image](https://user-images.githubusercontent.com/59205405/91957815-821fca00-ed41-11ea-9ce5-59624e374da7.png)
이 과정까지 완료가 되었다면, nvidia driver, cuda toolkit, cudnn 설치가 완료된 것이다. 
GPU구동을 위해서 이 세가지의 설치를 확인해주길 바란다. 아래 참조에서 설치확인 방법소개.
참조 : https://crmn.tistory.com/31

**4. ROS Melodic 설치**
참조 : https://github.com/katebrighteyes/jetson_ros_melodic
  

**5. usb_cam package 설치**
usb_cam 은 노트북 내장 캡, 외부 usb cam 연결을 위한 패키지이다. 
참조 : https://m.blog.naver.com/PostView.nhn?blogId=nswve&logNo=221483691234&proxyReferer=https:%2F%2Fwww.google.com%2F

$ ~/catkin_wd/src/  위치에 설치해준다. (간혹 설치 안된것처럼 보이게 설치되는 경우가 있는데 어어쨋든 rosrun usb_cam usb_cam_node 명령어가 된다면 설치가 된 것이다.)
실행 명령어는 
**터미널[1] : roscore**
**터미널[2] : rosrun usb_cam usb_cam_node**
외부 usb_cam을 사용할 경우 usb_cam_node 를 /dev/video1 으로 수정해주어야한다. (환경마다 지정번호가 다르니 확인해보아야 한다. 대체로 video0 이 노트북내장 웹캠, video1 이 외부usb캠을으로 지정되어있다.)

**6. Darknet_ros 설치**
참조 : https://github.com/katebrighteyes/darknet_ros

# 실행 전 수정작업
**1. ~/catkin_ws/src/darknet_ros/darknet_ros/config 의 ros.yaml  수정**
![image](https://user-images.githubusercontent.com/59205405/91963853-67515380-ed49-11ea-818a-038bebff191c.png)
camera_reading의 topic 을 /usb_cam/image_raw 로 수정해준다. (usb_cam을 킨 후  rostopic list 으로 usb_cam의 topic이 들어오는지 미리 확인!! ) 

**2. ~/catkin_ws/src/darknet_ros/darknet_ros/launch 의 darknet_ros.launch 에서 "network_param_file" 이 우리가 실행하고자 하는 model이 맞는지 확인 후 수정. 여기서는 yolov2-tiny.yaml을 사용하였다.** 
![image](https://user-images.githubusercontent.com/59205405/91964307-ef375d80-ed49-11ea-8f41-1863b23ab65e.png)

**3. ~/catkin_ws/src/darknet_ros/darknet 에서 Makefile 을 수정해준다.**
![image](https://user-images.githubusercontent.com/59205405/91964779-8b616480-ed4a-11ea-9c38-beea1038cf1d.png)
GPU=1, OPENCV=1 로 수정해주고 ( GPU를 이용할 것 이기 때문 ), -gencode arch=compute_50,code=[sm_50,compute_50] 을 추가해주었다. (Nvidia geforce 940mx의 compute capability가 5.0 이기 때문에) 본인들의 그래픽카드 환경에 따라 수정해주면 된다.

# 실행
터미널[1] : roscore
터미널[2] : rosrun usb_cam usb_cam_node
터미널[3] : roslaunch darknet_ros darknet_ros.launch
터미널[4] : rviz (add topic)

**- 실행화면**
![image](https://user-images.githubusercontent.com/59205405/91965438-70dbbb00-ed4b-11ea-823e-4eed7ff852ee.png)

20-30 fps 정도의 성능을 보인다. (cpu만을 사용했을 때는 0.7 fps 정도!! )

-------------------------------------------------------------------------------------------------------------------------------------
#  Nvidia Graphic Driver, Cuda Toolkit 10.0, Cudnn 7.5 설치
$ sudo apt update

$ sudo apt install cuda-10-0

$ sudo apt install libcudnn7

$ gedit ~/.bashrc
- 아래 내용을 ~/.bashrc 맨 밑라인에 추가!!

export PATH=/usr/local/cuda-10.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}


$ reboot


- **nvcc --version** 가 실행된다면 cuda설치가 잘된 것.
- **cat /usr/local/cuda/include/cudnn.h | grep CUDNN_MAJOR -A 2**  가 실행된다면 cudnn 설치가 잘된 것.

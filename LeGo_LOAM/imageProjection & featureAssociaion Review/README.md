## 논문 리뷰

> LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain  

요약: 먼저 잡음제거를 위해 포인트 클라우드를 분류한다. 그리고 평면과 에지 특징을 구분하기 위해 특징을 추출한다.  
연속된 스캔으로 6자유도 변환의 다양한 구성요소를 해결하기 위해 2단계 Levenberg-Marquardt 최적화 방법 후 평면과 에지 특징을 이용한다.  
LeGO-LOAM은 LOAM과 비슷한 정확도와 더 적은 계산적 비용을 가진다.  
이동으로 인한 위치추정에러를 제거하기 위해 LeGO-LOAM을 SLAM에 통합했고 KITTI 데이터셋으로 검증되었다.   

## 코드 리뷰

+ imageProjection.cpp

> data(토픽) -> ImageProjection(노드) -> segmented cloud info(토픽), segmented cloud(토픽), outlier cloud(토픽)

```
ImageProjection이라는 클래스와 메인함수로 이루어져있음.

메인함수에서 ImageProjection클래스를 선언하고, ImageProjection에서 생성자가 실행되는데 
utility.h에서 로스백 파일 경로가 적혀진 pointcloudTopi이라는 토픽을 이 노드가 subscribe함.
Callback함수로 cloudHandler함수를 이용함. 
        
cloudHandler함수는 7단계로 이루어져있음.

1. ros 메시지를 pcl 포인트클라우드로 바꿈 
2. 스캔의 시작과 끝의 각도 찾기
3. laser cloud를 full cloud로 velodyne raw cloud를 투영함. !
4. ground point와 no ground point로 나눈 다 !
5. 포인트 클라우드를 segmentation한다 !
6. 포인트 클라우드를 publish한다.
7. 다음 반복을 위해 모든 파라미터들을 초기화한다. 

각각의 단계가 함수로 구현되어있음.

kitti dataset을 적용했을때 차이점은 x,y,z,intensity,ring number가 있는 laserCloudinRing변수를 사용하지 않음.
링 기능을 사용하지 않는다. 
```

+ featureAssociation.cpp

> ImageProjection(노드), imu(토픽) -> featureAssociation(노드) -> laser cloud surf/corner last(토픽), outlier cloud last(토픽), laser odom to init(토픽), laser cloud less sharp/flat(토픽), tf

```
featureAssociation이라는 클래스와 메인함수로 이루어져있음.
메인함수에서 featureAssociation클래스를 선언하고, ros::ok()동안 runFeatureAssociation()함수를 돌림.

ros::ok() will return false if:
1. a SIGINT is received (Ctrl-C)
2. we have been kicked off the network by another node with the same name
3. ros::shutdown() has been called by another part of the application.
4. all ros::NodeHandles have been destroyed

runFeatureAssociation()함수는 크게 feature extraction과 feature association 부분으로 나누어져있다. 

1. Feature Extracion

1) imu를 이용한 왜곡보정
2) segmentedCloudRange로 곡률계산함. (왜지..??)
3) 가려진 포인트들을 표시함.
4) surf 방법으로 포인트 특징 추출함 
5) 시각화를 위한 클라우드 publish ! ! ! 

2. Feature Association
        
1) imu 초기 추측 업데이트
2) 위 업데이트 과정으로 변형된 특징도 업데이트함. 
3) 회전율 축적하고 imu 회전율도 적용한다.
4) odometry 를 publish한다. 
5) map최적화를 위한 클라우드 publish한다. 
		
위의 각단계들이 함수로 구현되어있고 아직 정확한 코드 분석은 멀었다
```

## 실습

https://github.com/seo-dev/KCSY/tree/hrkim/hrkim/LeGOLOAM_kitti

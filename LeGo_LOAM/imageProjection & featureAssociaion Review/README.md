## 논문 리뷰

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

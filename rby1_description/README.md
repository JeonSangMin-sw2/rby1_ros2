# rby1_description package

- 해당 패키지는 rby1_a , rby1_m 모델에 대한 패키지.
- 해당 패키지를 통해 moveit 패키지와 bringup 패키지 등 다양한 패키지와 연관됨

## 모델 유효성 검증방식
- launch 파일을 통해 각 모델의 조인트 상태(종류, 각도범위 등) 확인가능
- 변수
  - model_name :rby1a , rby1m 중 선택
  - model_version : 각 모델에 맞는 버전을 호출
    - rby1a : 1.0, 1.1, 1.2
    - rby1m : 1.0, 1.1, 1.2, 1.3
- 예시
```bash
ros2 launch rby1_description rby1_state_publisher.launch.py model:=a version:=1_1

```
  - 실행 시 rviz와 함께 조인트 컨트롤러가 생성

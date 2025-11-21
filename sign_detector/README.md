# Sign Detector Package

표지판 방향 인식을 위한 ROS 패키지입니다.

## 기능
- 카메라 이미지에서 빨간색 표지판 검출
- 표지판 내 화살표 방향 인식 (좌회전/우회전/직진)
- 검출된 방향을 토픽으로 퍼블리시

## 토픽

### Subscribe
- `/usb_cam/image_rect_color` (sensor_msgs/Image): 카메라 입력 이미지

### Publish
- `/sign/direction` (std_msgs/String): 검출된 방향 ("left", "right", "forward")

## 사용 방법

### 1. 패키지 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 노드 실행
```bash
roslaunch sign_detector sign_recognizer.launch
```

### 3. 방향 토픽 확인
```bash
rostopic echo /sign/direction
```

## 파라미터 조정
`scripts/sign_recognizer_node.py` 파일에서 다음 파라미터를 조정할 수 있습니다:

- `lower_red1`, `upper_red1`: 빨간색 HSV 범위 (하단)
- `lower_red2`, `upper_red2`: 빨간색 HSV 범위 (상단)
- `min_area`: 최소 contour 면적 (노이즈 필터링)

## 알고리즘
1. HSV 색공간으로 변환하여 빨간색 영역 검출
2. 모폴로지 연산으로 노이즈 제거
3. 가장 큰 contour를 표지판으로 인식
4. ROI를 좌우로 나누어 픽셀 밀도 비교
5. 밀도가 높은 쪽으로 화살표 방향 판단

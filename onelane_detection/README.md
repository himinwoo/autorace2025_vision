# OneLane Detection Package

단일 차선 인식 및 추종을 위한 ROS 패키지입니다.

## 기능
- **단일 차선 검출**: Sliding Window 알고리즘을 사용한 차선 검출
- **조향 제어**: 검출된 차선 기반 조향각 계산 및 제어
- **좌/우 차선 선택**: 좌측 또는 우측 차선 선택 가능
- **실시간 디버깅**: OpenCV를 통한 실시간 시각화

## 토픽

### Subscribe
- `/usb_cam/image_rect_color` (sensor_msgs/Image): 카메라 입력 이미지

### Publish
- `/camera/steering` (geometry_msgs/Twist): 조향 제어 명령

## 사용 방법

### 1. 패키지 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 노드 실행
```bash
roslaunch onelane_detection onelane_detector.launch
```

### 3. 개별 실행
```bash
rosrun onelane_detection onelane_detector_node.py
```

## 파라미터 설정

Launch 파일(`launch/onelane_detector.launch`)에서 조정 가능:

### 기본 설정
- `image_topic`: 입력 카메라 토픽 (기본: `/usb_cam/image_rect_color`)
- `cmd_topic`: 출력 제어 토픽 (기본: `/camera/steering`)
- `lane_side`: 차선 선택 ("left" 또는 "right")

### 알고리즘 파라미터
- `window_num`: Sliding window 개수 (기본: 12)
- `window_margin`: Window 마진 (기본: 30)
- `crop_start_ratio`: ROI 시작 비율 (기본: 0.5)

### HSV 색상 필터
- `hsv_lower`: HSV 하한값 (기본: [10, 50, 50])
- `hsv_upper`: HSV 상한값 (기본: [40, 255, 255])

### 조향 제어
- `steering_ratio`: 조향 기준점 비율 (0.0~1.0)
- `steering_gain_left`: 좌측 차선 게인
- `steering_gain_right`: 우측 차선 게인

### 기타
- `enable_drive`: 주행 제어 활성화
- `show_debug`: 디버그 창 표시

## 알고리즘

1. **이미지 전처리**: HSV 변환 및 색상 필터링
2. **ROI 추출**: 이미지 하단 영역만 사용
3. **Sliding Window**: 차선 픽셀 검출
4. **다항식 피팅**: 검출된 픽셀로 차선 곡선 생성
5. **조향각 계산**: 차선 기준점과 현재 위치 오차 계산
6. **제어 명령 발행**: 계산된 조향각 퍼블리시

## 기존 패키지와의 차이
- **이전**: `morai_lane` / `result.py`
- **현재**: `onelane_detection` / `onelane_detector_node.py`
- 더 명확한 이름으로 기능 표현

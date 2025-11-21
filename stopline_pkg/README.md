# Stopline D#### Published Topics
- `/camera/stopline/count` (coss_msgs/Coss): 정지선 감지 시 mission_state 증가

#### Parameters
- `~down_hist_start_line` (int, default: 400): 히스토그램 검색 시작 라인
- `~stopline_threshold` (int, default: 10): 정지선 인식 너비 임계값
- `~count_threshold` (int, default: 10): 정지선 감지 확정을 위한 연속 감지 횟수
- `~cooldown_duration` (float, default: 3.0): 정지선 감지 쿨다운 시간 (초)
- `~show_debug_image` (bool, default: false): 디버그 이미지 표시 여부 (cv2.imshow)
- `~white_threshold` (int, default: 200): 그레이스케일 밝기 임계값 (0-255)
- `~histogram_threshold` (int, default: 600): 히스토그램 임계값kage

정지선 감지 전용 ROS 패키지입니다.

## 노드

### stopline_detection_node
정지선 감지 노드입니다.

#### Subscribed Topics
- `/usb_cam/image_rect_color` (sensor_msgs/Image): 카메라 이미지

#### Published Topics
- `/camera/` (coss_msgs/Coss): 정지선 감지 시 mission_state 증가
- `/camera/stopline/debug_image` (sensor_msgs/Image): 디버그용 이미지 (옵션)

#### Parameters
- `~down_hist_start_line` (int, default: 400): 히스토그램 검색 시작 라인
- `~stopline_threshold` (int, default: 10): 정지선 인식 너비 임계값
- `~count_threshold` (int, default: 10): 정지선 감지 확정을 위한 연속 감지 횟수
- `~cooldown_duration` (float, default: 3.0): 정지선 감지 쿨다운 시간 (초)
- `~publish_debug_image` (bool, default: false): 디버그 이미지 발행 여부
- `~white_threshold` (int, default: 200): 그레이스케일 밝기 임계값 (0-255)
- `~histogram_threshold` (int, default: 600): 히스토그램 임계값

## 사용 방법

### 빌드
```bash
cd ~/catkin_ws
catkin_make --pkg stopline_pkg
source devel/setup.bash
```

### 실행
```bash
roslaunch stopline_pkg stopline_detection.launch
```

### 토픽 확인
```bash
# Coss 메시지 (mission_state 확인)
rostopic echo /camera/stopline/count
```

### 디버그 이미지 확인
`show_debug_image=true`로 설정하면 OpenCV 창에서 디버그 이미지를 확인할 수 있습니다.

## 알고리즘
1. 카메라 이미지를 그레이스케일로 변환
2. 밝기 임계값을 이용하여 흰색 영역 추출 (정지선)
3. 이진화된 이미지에서 y축 히스토그램 계산
4. 이미지 하단부에서 정지선 검색
5. 연속 감지 횟수가 임계값을 초과하면 정지선 감지 확정
6. 쿨다운 시간을 적용하여 중복 감지 방지

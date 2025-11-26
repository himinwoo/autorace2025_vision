# Stopline D#### Published Topics
- `/camera/stopline/count` (coss_msgs/Coss): 정지선 감지 시 mission_state 증가

#### Subscribed Topics
- `/usb_cam/image_rect_color` (sensor_msgs/Image): 카메라 이미지
- `/lidar` (coss_msgs/Coss): 라이다 플래그 및 cone_finish 정보
- `~crop_top` (int, default: 280): 이미지 상단 크롭 위치
- `~down_hist_start_line` (int, default: 0): 히스토그램 검색 시작 라인
- `~stopline_threshold` (int, default: 5): 정지선 인식 너비 임계값
- `~count_threshold` (int, default: 5): 정지선 감지 확정을 위한 연속 감지 횟수
- `~cooldown_durations` (list, default: [3.0, 10.0, 5.0, 15.0, 1.5, 3.0]): mission_state별 쿨다운 시간 (초)
- `~show_debug_image` (bool, default: false): 디버그 이미지 표시 여부 (cv2.imshow)
- `~histogram_threshold` (int, default: 200): 히스토그램 임계값
- `~otsu_threshold_offset` (int, default: 50): Otsu 임계값 조정 오프셋
- `~hough_threshold` (int, default: 50): 허프라인 검출 임계값
- `~hough_min_line_length` (int, default: 40): 허프라인 최소 길이 (픽셀)
- `~hough_max_line_gap` (int, default: 10): 허프라인 최대 간격 (픽셀)
- `~hough_angle_tolerance` (float, default: 25.0): 수평선 기준 각도 허용 범위 (도)
- `~hough_use_morphology` (bool, default: true): 허프라인 결과에 모폴로지 연산 적용 여부
- `~yellow_hsv_lower` (list, default: [0, 40, 50]): 노란색 HSV 하한값 (state 9용)
- `~yellow_hsv_upper` (list, default: [26, 110, 255]): 노란색 HSV 상한값 (state 9용)
- `~yellow_state` (int, default: 9): 노란색 정지선 감지 적용 state

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

### 흰색 정지선 감지 (히스토그램 + 허프라인 OR 방식)

#### 1. 허프라인 기반 검출 (독립적)
- 원본 그레이스케일 이미지에서 직접 검출하여 전처리 손실 최소화
- 가우시안 블러로 경미한 노이즈 제거
- Canny 에지 검출
- 허프라인 변환으로 직선 검출
- 수평선에 가까운 라인만 필터링 (각도 허용 범위 내)
- 충분히 긴 라인만 선택 (이미지 폭의 15% 이상)
- 선택적 모폴로지 연산으로 라인 보강

#### 2. 히스토그램 기반 검출
- 과포화 영역 마스킹 (빛 반사 사전 제거)
- Bilateral 필터로 적응적 노이즈 제거
- CLAHE + 적응형 임계값 결합
- 에지 강조 (정지선 경계선)
- 형태학적 필터링 (원형 객체 제거)
- 원형도 기반 필터링

#### 3. 결과 병합
- 허프라인과 히스토그램 결과를 OR 연산으로 결합
- y축 히스토그램 계산 및 정지선 영역 탐색
- 연속 감지 횟수가 임계값 초과 시 정지선 감지 확정

### 노란색 정지선 감지 (state 9용)
- HSV 색공간 변환
- 노란색 범위 마스킹
- 가우시안 블러 및 모폴로지 연산
- 이진화 결과 반환

### 쿨다운 및 상태 관리
- mission_state별 독립적인 쿨다운 시간 적용
- state 3, 7: cone_finish 플래그 대기
- state 6: 라이다 감지 후 쿨다운 시작
- state 10: 정지선 감지 비활성화

# Color Detector Package

빨간색과 파란색을 감지하는 ROS 패키지입니다.

## 기능
- **통합 모드**: 하나의 노드에서 빨강과 파랑을 동시에 감지 (권장)
- **개별 모드**: 빨강과 파랑을 각각의 노드로 실행

## 토픽

### Subscribe
- `/usb_cam/image_rect_color` (sensor_msgs/Image): 카메라 입력 이미지

### Publish
- `/color/red_detection` (coss_msgs/Coss): 빨간색 감지 결과
- `/color/blue_detection` (coss_msgs/Coss): 파란색 감지 결과

## 사용 방법

### 1. 패키지 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 노드 실행

#### 통합 모드 (권장 - 하나의 노드로 빨강+파랑)
```bash
roslaunch color_detector color_detector.launch
```

#### 개별 모드 (빨강과 파랑을 별도 노드로)
```bash
roslaunch color_detector separate_nodes.launch
```

#### 빨강만 실행
```bash
roslaunch color_detector red_only.launch
# 또는
rosrun color_detector red_detector_node
```

#### 파랑만 실행
```bash
roslaunch color_detector blue_only.launch
# 또는
rosrun color_detector blue_detector_node
```

### 3. 토픽 확인
```bash
# 빨강 감지 결과
rostopic echo /color/red_detection

# 파랑 감지 결과
rostopic echo /color/blue_detection
```

## 파라미터 조정

### Launch 파일에서 조정 (color_detector.launch)
```xml
<param name="red_threshold" value="28000"/>   <!-- 빨강 픽셀 threshold -->
<param name="blue_threshold" value="28000"/>  <!-- 파랑 픽셀 threshold -->
<param name="debug_mode" value="true"/>       <!-- 디버깅 윈도우 표시 -->
```

### 소스 코드에서 조정

#### 통합 버전 (`src/color_detector.cpp`)
- 빨강 HSV: `[0, 120, 40] ~ [10, 255, 255]`, `[170, 120, 40] ~ [180, 255, 255]`
- 파랑 HSV: `[100, 100, 50] ~ [130, 255, 255]`

#### 개별 버전
- Red: `src/red_detector.cpp`
- Blue: `src/blue_detector.cpp`

## 장점

### 통합 모드 (color_detector_node)
- ✅ 하나의 이미지로 두 색상 동시 처리 (효율적)
- ✅ 메모리 사용량 감소
- ✅ 동기화 문제 없음
- ✅ 관리 편리

### 개별 모드
- ✅ 독립적인 프로세스
- ✅ 하나가 죽어도 다른 하나는 동작
- ✅ 개별 디버깅 용이

## 노트
- ROI는 이미지 하단 50%만 사용
- 디버깅 윈도우가 자동으로 표시됨 (debug_mode=true)
- 기본적으로 통합 모드 사용 권장

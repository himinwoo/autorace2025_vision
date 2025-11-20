# autorace2025_vision

2025 자율주행 경주 비전 시스템 ROS 패키지 모음

## 패키지 목록

이 저장소는 다음 ROS 패키지들을 포함합니다:

### 1. line_red
빨간색 라인 감지 패키지

### 2. lane_pkg
차선 감지 및 추적 패키지

### 3. image_preprocessing
이미지 전처리 작업 패키지

## 의존성

- ROS (Robot Operating System)
- OpenCV
- 각 패키지의 `package.xml`에 명시된 기타 ROS 의존성

## 빌드 방법

```bash
cd ~/catkin_ws
catkin_make
```

## 사용 방법

각 패키지는 개별적으로 실행할 수 있습니다:

```bash
# 빨간색 라인 감지
roslaunch line_red <launch_file>.launch

# 차선 감지
roslaunch lane_pkg <launch_file>.launch

# 이미지 전처리
roslaunch image_preprocessing <launch_file>.launch
```

## 라이선스

<!-- 라이선스 정보를 여기에 추가하세요 -->

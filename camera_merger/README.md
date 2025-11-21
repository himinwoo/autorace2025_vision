# Camera Merger Package

여러 카메라 관련 패키지의 Coss 메시지를 하나의 `/camera` 토픽으로 통합하는 ROS 패키지입니다.

## 개요

이 패키지는 다음 패키지들의 출력을 구독하고 통합합니다:
- `stopline_pkg`: 정지선 감지 (mission_state)
- `onelane_detection`: 차선 감지 (cam_steer, lane_steer)
- `color_detector`: 색상 감지 (cam_red_detection, cam_blue_detection)

## 구독 토픽

| 토픽 | 메시지 타입 | 제공 패키지 | 사용 필드 |
|------|------------|------------|----------|
| `/camera/stopline/count` | coss_msgs/Coss | stopline_pkg | mission_state |
| `/camera/lane` | coss_msgs/Coss | onelane_detection | cam_steer, lane_steer |
| `/camera/color` | coss_msgs/Coss | color_detector | cam_red_detection, cam_blue_detection |

## 발행 토픽

| 토픽 | 메시지 타입 | 설명 |
|------|------------|------|
| `/camera` | coss_msgs/Coss | 통합된 카메라 정보 |

## 사용 방법

### 빌드
```bash
cd ~/catkin_ws
catkin_make --pkg camera_merger
source devel/setup.bash
```

### 실행
```bash
roslaunch camera_merger camera_merger.launch
```

### 개별 노드 실행
```bash
rosrun camera_merger camera_merger_node
```

## 파라미터

- `~publish_rate` (double, default: 20.0): 통합 메시지 발행 주기 (Hz)

## 통합 로직

각 패키지에서 받은 최신 값을 유지하고, 주기적으로 통합 메시지를 발행합니다:

```cpp
// Stopline 콜백에서
merged_msg.mission_state = stopline_msg.mission_state;

// Lane 콜백에서
merged_msg.cam_steer = lane_msg.cam_steer;
merged_msg.lane_steer = lane_msg.lane_steer;

// Color 콜백에서
merged_msg.cam_red_detection = color_msg.cam_red_detection;
merged_msg.cam_blue_detection = color_msg.cam_blue_detection;

// 타이머 콜백에서 통합 메시지 발행
publish(merged_msg);
```

## 주의사항

**토픽명 확인 필요**: 
- `onelane_detection`과 `color_detector`의 실제 발행 토픽명을 확인하고 코드를 수정해야 합니다.
- 현재 `/camera/lane`, `/camera/color`로 가정했습니다.

실제 토픽명 확인:
```bash
rostopic list | grep camera
```

## 장점

1. **중앙 집중식 데이터**: 모든 카메라 정보가 `/camera` 하나로 통합
2. **동기화**: mutex를 사용한 스레드 안전성
3. **확장성**: 새로운 센서 정보 추가 용이
4. **성능**: C++로 구현하여 빠른 처리

## 토픽 흐름도

```
stopline_pkg ──> /camera/stopline/count ──┐
                                           │
onelane_detection ──> /camera/lane ────────┼──> camera_merger ──> /camera
                                           │
color_detector ──> /camera/color ──────────┘
```

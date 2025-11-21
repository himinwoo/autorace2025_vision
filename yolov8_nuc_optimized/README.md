# YOLOv8 NUC Optimized

Intel NUC에 최적화된 YOLOv8 객체 검출 ROS 패키지

## ✨ 주요 특징

- **🚀 고속 추론**: Intel OpenVINO 런타임 지원으로 Intel NUC에서 최적화된 성능 (30-35 FPS)
- **💾 메모리 효율**: 경량화된 모델과 효율적인 메모리 관리
- **📖 가독성 높은 코드**: 명확한 구조와 상세한 주석
- **⚙️ 유연한 설정**: YAML 기반 설정 파일로 쉬운 파라미터 조정

## 📁 패키지 구조

```
yolov8_nuc_optimized/
├── config/
│   └── detector_config.yaml      # 설정 파일
├── launch/
│   ├── yolov8_detector.launch    # 기본 launch 파일
│   └── yolov8_with_camera.launch # 카메라 통합 launch 파일
├── scripts/
│   ├── yolov8_detector.py        # 메인 검출 노드
│   ├── yolov8_optimizer.py       # 모델 최적화 도구
│   ├── download_model.sh         # 모델 다운로드 스크립트
│   └── install_dependencies.sh   # 자동 설치 스크립트
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🚀 빠른 시작 (3단계)

### 1단계: 의존성 설치

```bash
# 자동 설치 스크립트 사용 (권장)
cd ~/catkin_ws/src/yolov8_nuc_optimized/scripts
./install_dependencies.sh

# 또는 수동 설치
python3 -m pip install --user ultralytics opencv-python numpy openvino openvino-dev
```

### 2단계: ROS 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport
```

### 3단계: 패키지 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 📖 사용법

### 기본 실행

```bash
# 카메라와 함께 실행
roslaunch yolov8_nuc_optimized yolov8_with_camera.launch

# 검출기만 실행 (외부 이미지 토픽 사용)
roslaunch yolov8_nuc_optimized yolov8_detector.launch
```

### 설정 커스터마이징

`config/detector_config.yaml` 파일을 수정하거나 launch 파일에서 파라미터 오버라이드:

```bash
roslaunch yolov8_nuc_optimized yolov8_detector.launch \
  model_path:=yolov8n.pt \
  confidence_threshold:=0.6 \
  input_size:=416 \
  use_openvino:=true
```

## 💡 사용 예제

### 예제 1: 특정 클래스만 검출

사람(class 0)과 자동차(class 2)만 검출:

```bash
roslaunch yolov8_nuc_optimized yolov8_detector.launch \
  model_path:=yolov8n.pt \
  _classes:="[0, 2]"
```

### 예제 2: 고속 처리 모드

속도 최적화 설정 (50-60 FPS):

```bash
roslaunch yolov8_nuc_optimized yolov8_detector.launch \
  model_path:=yolov8n.pt \
  input_size:=320 \
  use_openvino:=true \
  use_fp16:=true \
  confidence_threshold:=0.6
```

### 예제 3: Python 스크립트에서 검출 정보 사용

```python
#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

def detection_callback(msg):
    data = json.loads(msg.data)
    print(f"검출된 객체 수: {data['num_detections']}")
    for det in data['detections']:
        print(f"- {det['class_name']}: {det['confidence']:.2f}")

rospy.init_node('detection_listener')
rospy.Subscriber('/yolov8/detection_info', String, detection_callback)
rospy.spin()
```

## 📊 예상 성능

**Intel NUC 11th Gen i5 기준:**

| 모델 형식 | 입력 크기 | 예상 FPS | 비고 |
|----------|----------|---------|------|
| **OpenVINO FP16** | 640 | **~30-35** | ⭐ 권장 |
| **OpenVINO FP16** | 416 | **~50-60** | 초고속 |
| **OpenVINO FP16** | 320 | **~70-80** | 최고속 |
| PyTorch | 640 | ~12-15 | 비교용 |

## ⚙️ 주요 파라미터

### 모델 설정
- `model_path`: YOLOv8 모델 경로 (기본값: `yolov8n.pt`)
- `input_size`: 입력 이미지 크기 (기본값: `640`, 옵션: `320`, `416`, `640`)
- `model_format`: 모델 형식 (기본값: `pt`, 옵션: `pt`, `onnx`, `openvino`)

### 검출 설정
- `confidence_threshold`: 신뢰도 임계값 (기본값: `0.5`)
- `iou_threshold`: IoU 임계값 (기본값: `0.45`)
- `max_detections`: 최대 검출 개수 (기본값: `100`)
- `classes`: 검출할 클래스 ID 리스트 (비어있으면 모든 클래스 검출)

### 성능 설정
- `use_openvino`: OpenVINO 사용 여부 (기본값: `true`)
- `use_fp16`: FP16 정밀도 사용 (기본값: `true`)
- `num_threads`: CPU 스레드 수 (기본값: `4`)
- `device`: 실행 디바이스 (기본값: `CPU`, 옵션: `CPU`, `GPU`)

### 시각화 설정
- `enable_visualization`: 시각화 활성화 (기본값: `true`)
- `show_labels`: 라벨 표시 (기본값: `true`)
- `show_confidence`: 신뢰도 표시 (기본값: `true`)

더 자세한 설정은 `config/detector_config.yaml`을 참조하세요.

## 📡 ROS 토픽

### 구독 (Subscribed)
- `/camera/image_raw` (sensor_msgs/Image): 입력 이미지

### 발행 (Published)
- `/yolov8/detections` (sensor_msgs/Image): 검출 결과가 그려진 이미지
- `/yolov8/detection_info` (std_msgs/String): 검출 정보 (JSON 형식)

### 결과 확인

```bash
# 검출 결과 이미지 확인
rqt_image_view /yolov8/detections

# 검출 정보 확인
rostopic echo /yolov8/detection_info
```

## 🚨 문제 해결

### pip 오류 발생 시

```bash
AttributeError: module 'lib' has no attribute 'X509_V_FLAG_NOTIFY_POLICY'
```

**해결 방법 1: 자동 스크립트 사용 (권장)**
```bash
cd ~/catkin_ws/src/yolov8_nuc_optimized/scripts
./install_dependencies.sh
```

**해결 방법 2: 수동 복구**
```bash
# pyOpenSSL 제거
sudo apt-get remove --purge python3-openssl
sudo apt-get autoremove

# pip 재설치
curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
python3 /tmp/get-pip.py --user --force-reinstall
rm /tmp/get-pip.py

# PATH 설정
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# 패키지 설치
python3 -m pip install --user ultralytics opencv-python numpy openvino openvino-dev
```

### 빌드 오류 (cv_bridge 없음)

```bash
# ROS 의존성 설치
sudo apt-get update
sudo apt-get install -y ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport

# 전체 빌드
cd ~/catkin_ws
catkin_make

# 또는 특정 패키지만 빌드
catkin_make --pkg yolov8_nuc_optimized

# 또는 문제 패키지 무시
touch ~/catkin_ws/src/[문제_패키지]/CATKIN_IGNORE
catkin_make
```

### 모델 다운로드 실패

```bash
# 수동으로 모델 다운로드
cd ~/catkin_ws/src/yolov8_nuc_optimized/scripts
./download_model.sh

# 또는 Python에서 자동 다운로드 (노드 실행 시 자동)
roslaunch yolov8_nuc_optimized yolov8_detector.launch
```

### 낮은 FPS

**성능 최적화 체크리스트:**
1. ✅ OpenVINO 활성화: `use_openvino:=true`
2. ✅ 작은 입력 크기: `input_size:=416` 또는 `320`
3. ✅ FP16 사용: `use_fp16:=true`
4. ✅ 특정 클래스만 검출: `_classes:="[0, 2]"`
5. ✅ 신뢰도 임계값 증가: `confidence_threshold:=0.6`

```bash
# 최적화된 설정 예제
roslaunch yolov8_nuc_optimized yolov8_detector.launch \
  input_size:=320 \
  use_openvino:=true \
  use_fp16:=true \
  confidence_threshold:=0.6
```
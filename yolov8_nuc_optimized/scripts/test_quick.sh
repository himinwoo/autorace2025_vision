#!/bin/bash
# YOLOv8 빠른 테스트 스크립트

echo "=== YOLOv8 Quick Test Script ==="
echo ""

# ROS 환경 설정
source ~/catkin_ws/devel/setup.bash

# 테스트 이미지 다운로드
echo "Step 1: 테스트 이미지 다운로드..."
cd ~/catkin_ws/src/yolov8_nuc_optimized/scripts
if [ ! -f test.jpg ]; then
    wget https://ultralytics.com/images/bus.jpg -O test.jpg
    echo "✓ 테스트 이미지 다운로드 완료"
else
    echo "✓ 테스트 이미지가 이미 존재합니다"
fi
echo ""

# Python으로 직접 테스트
echo "Step 2: OpenVINO 모델 테스트..."
echo "----------------------------------------"
python3 << 'EOF'
from ultralytics import YOLO
import time

print("모델 로딩 중...")
model = YOLO('yolov8n_openvino_model')
print("✓ 모델 로드 완료")

print("\n이미지 검출 중...")
start_time = time.time()
results = model('test.jpg', verbose=False)
inference_time = time.time() - start_time

print(f"\n=== 검출 결과 ===")
print(f"추론 시간: {inference_time*1000:.1f} ms")
print(f"예상 FPS: {1/inference_time:.1f}")

boxes = results[0].boxes
print(f"\n검출된 객체 수: {len(boxes)}")

if len(boxes) > 0:
    print("\n검출된 객체:")
    for i, box in enumerate(boxes):
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        cls_name = model.names[cls_id]
        print(f"  {i+1}. {cls_name}: {conf:.2%}")

print("\n✓ 테스트 완료!")
print("결과 이미지가 표시됩니다...")
results[0].show()
EOF

echo ""
echo "========================================"
echo "✓ 테스트 성공!"
echo "========================================"
echo ""
echo "다음 단계:"
echo "1. ROS로 실행:"
echo "   roslaunch yolov8_nuc_optimized yolov8_openvino.launch"
echo ""
echo "2. 카메라와 함께 실행:"
echo "   roslaunch yolov8_nuc_optimized yolov8_with_camera.launch \\"
echo "     model_path:=\$(rospack find yolov8_nuc_optimized)/scripts/yolov8n_openvino_model"
echo ""

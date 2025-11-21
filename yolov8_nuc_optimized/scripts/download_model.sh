#!/bin/bash
# YOLOv8 모델 다운로드 스크립트

echo "=== YOLOv8 Model Download Script ==="
echo ""

# 모델 저장 디렉토리
MODEL_DIR="$HOME/.yolov8_models"
mkdir -p "$MODEL_DIR"

echo "Models will be saved to: $MODEL_DIR"
echo ""

# 사용 가능한 모델 목록
echo "Available models:"
echo "  1. YOLOv8n (Nano)    - Fastest, smallest (6.3 MB)"
echo "  2. YOLOv8s (Small)   - Balanced (21.5 MB)"
echo "  3. YOLOv8m (Medium)  - Accurate (49.7 MB)"
echo "  4. YOLOv8l (Large)   - Very accurate (83.7 MB)"
echo "  5. YOLOv8x (XLarge)  - Most accurate (130.5 MB)"
echo ""

# 사용자 입력
read -p "Select model to download (1-5) [default: 1]: " choice
choice=${choice:-1}

# 모델 선택
case $choice in
    1)
        MODEL="yolov8n.pt"
        echo "Downloading YOLOv8n (Nano)..."
        ;;
    2)
        MODEL="yolov8s.pt"
        echo "Downloading YOLOv8s (Small)..."
        ;;
    3)
        MODEL="yolov8m.pt"
        echo "Downloading YOLOv8m (Medium)..."
        ;;
    4)
        MODEL="yolov8l.pt"
        echo "Downloading YOLOv8l (Large)..."
        ;;
    5)
        MODEL="yolov8x.pt"
        echo "Downloading YOLOv8x (XLarge)..."
        ;;
    *)
        echo "Invalid selection. Downloading YOLOv8n (Nano)..."
        MODEL="yolov8n.pt"
        ;;
esac

# Python 스크립트로 모델 다운로드
python3 << EOF
from ultralytics import YOLO
import os

model_dir = os.path.expanduser("$MODEL_DIR")
model_name = "$MODEL"
model_path = os.path.join(model_dir, model_name)

print(f"Downloading {model_name}...")
model = YOLO(model_name)
print(f"Model downloaded successfully!")
print(f"Model location: {model_path}")
EOF

echo ""
echo "✓ Download complete!"
echo ""
echo "To use this model, update the launch file or config:"
echo "  model_path: $MODEL_DIR/$MODEL"
echo ""
echo "To optimize for Intel NUC:"
echo "  python3 yolov8_optimizer.py $MODEL_DIR/$MODEL --format openvino --fp16"

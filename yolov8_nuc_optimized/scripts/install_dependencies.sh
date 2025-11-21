#!/bin/bash
# pip 복구 및 YOLOv8 의존성 설치 스크립트

echo "=== YOLOv8 Dependencies Installation Script ==="
echo ""
echo "이 스크립트는 pip 문제를 해결하고 필요한 패키지를 설치합니다."
echo ""

# 1. pip 복구
echo "Step 1: pip 복구 중..."
echo "----------------------------------------"

# 주의: 시스템 패키지(python3-openssl 등)를 삭제하면 
# ROS 패키지들(rqt_image_view, rviz 등)이 함께 제거될 수 있습니다!
# 대신 사용자 영역에서만 pip를 재설치합니다.

# pip 재설치 (사용자 영역만)
echo "pip를 사용자 영역에 재설치 중..."
curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
python3 /tmp/get-pip.py --user --force-reinstall
rm /tmp/get-pip.py

# PATH 설정
export PATH="$HOME/.local/bin:$PATH"

echo ""
echo "✓ pip 복구 완료"
echo ""

# 2. pip 업그레이드
echo "Step 2: pip 업그레이드 중..."
echo "----------------------------------------"
python3 -m pip install --user --upgrade pip setuptools wheel

echo ""
echo "✓ pip 업그레이드 완료"
echo ""

# 3. 필수 패키지 설치
echo "Step 3: 필수 패키지 설치 중..."
echo "----------------------------------------"

# OpenCV가 이미 시스템에 설치되어 있는지 확인
if python3 -c "import cv2" 2>/dev/null; then
    echo "OpenCV가 이미 설치되어 있습니다. 건너뛰기..."
    OPENCV_SKIP=true
else
    OPENCV_SKIP=false
fi

# 패키지 설치
echo "ultralytics 설치 중..."
python3 -m pip install --user ultralytics

if [ "$OPENCV_SKIP" = false ]; then
    echo "opencv-python 설치 중..."
    python3 -m pip install --user opencv-python
fi

echo "numpy 설치 중..."
python3 -m pip install --user numpy

echo ""
echo "✓ 필수 패키지 설치 완료"
echo ""

# 4. OpenVINO 설치 (선택사항)
echo "Step 4: OpenVINO 설치 (Intel NUC 최적화)"
echo "----------------------------------------"
read -p "OpenVINO를 설치하시겠습니까? (y/n) [권장: y]: " install_openvino
install_openvino=${install_openvino:-y}

if [ "$install_openvino" = "y" ] || [ "$install_openvino" = "Y" ]; then
    echo "OpenVINO 설치 중... (시간이 걸릴 수 있습니다)"
    python3 -m pip install --user openvino openvino-dev
    echo "✓ OpenVINO 설치 완료"
else
    echo "OpenVINO 설치를 건너뜁니다."
    echo "나중에 다음 명령으로 설치할 수 있습니다:"
    echo "  python3 -m pip install --user openvino openvino-dev"
fi

echo ""
echo "========================================"
echo "✓ 모든 설치 완료!"
echo "========================================"
echo ""

# 5. 설치 확인
echo "Step 5: 설치 확인..."
echo "----------------------------------------"

check_package() {
    if python3 -c "import $1" 2>/dev/null; then
        echo "✓ $1 설치됨"
        return 0
    else
        echo "✗ $1 설치 실패"
        return 1
    fi
}

check_package "ultralytics"
check_package "cv2"
check_package "numpy"

if [ "$install_openvino" = "y" ] || [ "$install_openvino" = "Y" ]; then
    check_package "openvino"
fi

echo ""
echo "========================================"
echo "다음 단계:"
echo "========================================"
echo ""
echo "1. 새 터미널을 열거나 다음 명령을 실행하세요:"
echo "   export PATH=\"\$HOME/.local/bin:\$PATH\""
echo ""
echo "2. YOLOv8 모델을 다운로드하세요:"
echo "   roscd yolov8_nuc_optimized/scripts"
echo "   ./download_model.sh"
echo ""
echo "3. 패키지를 실행하세요:"
echo "   roslaunch yolov8_nuc_optimized yolov8_detector.launch"
echo ""

# bashrc에 PATH 추가 (아직 없다면)
if ! grep -q '$HOME/.local/bin' ~/.bashrc; then
    echo ""
    read -p "PATH를 ~/.bashrc에 영구적으로 추가하시겠습니까? (y/n) [권장: y]: " add_path
    add_path=${add_path:-y}
    
    if [ "$add_path" = "y" ] || [ "$add_path" = "Y" ]; then
        echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
        echo "✓ PATH가 ~/.bashrc에 추가되었습니다."
        echo "  새 터미널에서 자동으로 적용됩니다."
    fi
fi

echo ""
echo "설치가 완료되었습니다! 🎉"

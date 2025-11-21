#!/bin/bash
# python3-openssl 삭제로 인해 제거된 ROS 패키지 복구 스크립트

echo "=========================================="
echo "ROS 패키지 복구 스크립트"
echo "=========================================="
echo ""
echo "이 스크립트는 python3-openssl 삭제로 인해"
echo "제거된 ROS 패키지들을 다시 설치합니다."
echo ""

# ROS 버전 감지
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    ROS_DISTRO="noetic"
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    ROS_DISTRO="melodic"
elif [ -f "/opt/ros/kinetic/setup.bash" ]; then
    ROS_DISTRO="kinetic"
else
    echo "❌ ROS 설치를 찾을 수 없습니다."
    exit 1
fi

echo "감지된 ROS 버전: $ROS_DISTRO"
echo ""

# 1. 시스템 Python 패키지 복구
echo "Step 1: 핵심 Python3 시스템 패키지 복구"
echo "----------------------------------------"

echo "python3-openssl 및 관련 의존성 설치 중..."
sudo apt-get update
sudo apt-get install -y \
    python3-openssl \
    python3-cryptography \
    python3-pyasn1 \
    python3-service-identity \
    python3-twisted \
    python3-urllib3 \
    python3-requests \
    python3-certifi \
    python3-chardet \
    python3-idna

echo ""
echo "✓ Python3 시스템 패키지 복구 완료"
echo ""

# 2. ROS 기본 패키지 복구
echo "Step 2: ROS 기본 패키지 복구"
echo "----------------------------------------"

echo "ROS 기본 도구 설치 중..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-robot-plugins

echo ""
echo "✓ ROS 기본 패키지 복구 완료"
echo ""

# 3. RQT 및 시각화 도구 복구
echo "Step 3: RQT 및 시각화 도구 복구"
echo "----------------------------------------"

echo "RQT 이미지/시각화 도구 설치 중..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-rqt-console \
    ros-${ROS_DISTRO}-rqt-bag \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-image-view

echo ""
echo "✓ RQT 및 시각화 도구 복구 완료"
echo ""

# 4. 카메라 및 이미지 처리 패키지 복구
echo "Step 4: 카메라 및 이미지 처리 패키지 복구"
echo "----------------------------------------"

echo "카메라/이미지 관련 패키지 설치 중..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-theora-image-transport \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-camera-info-manager

echo ""
echo "✓ 카메라 및 이미지 처리 패키지 복구 완료"
echo ""

# 5. 네비게이션 및 센서 패키지 복구
echo "Step 5: 네비게이션 및 센서 패키지 복구"
echo "----------------------------------------"

echo "네비게이션/센서 관련 패키지 설치 중..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-move-base

echo ""
echo "✓ 네비게이션 및 센서 패키지 복구 완료"
echo ""

# 6. Gazebo 및 시뮬레이션 복구
echo "Step 6: Gazebo 및 시뮬레이션 복구"
echo "----------------------------------------"

echo "Gazebo/시뮬레이션 관련 패키지 설치 중..."
sudo apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

echo ""
echo "✓ Gazebo 및 시뮬레이션 복구 완료"
echo ""

# 7. 추가 유틸리티 복구
echo "Step 7: 추가 유틸리티 복구"
echo "----------------------------------------"

echo "추가 유틸리티 설치 중..."
sudo apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-osrf-pycommon

echo ""
echo "✓ 추가 유틸리티 복구 완료"
echo ""

# 8. rosdep 업데이트
echo "Step 8: rosdep 업데이트"
echo "----------------------------------------"

echo "rosdep 초기화 및 업데이트 중..."
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    sudo rosdep init 2>/dev/null || true
fi
rosdep update

echo ""
echo "✓ rosdep 업데이트 완료"
echo ""

# 9. catkin 워크스페이스 재빌드 확인
echo "Step 9: catkin 워크스페이스 확인"
echo "----------------------------------------"

if [ -d "$HOME/catkin_ws" ]; then
    echo "catkin 워크스페이스가 발견되었습니다."
    echo ""
    read -p "catkin 워크스페이스를 재빌드하시겠습니까? (y/n) [권장: y]: " rebuild_ws
    rebuild_ws=${rebuild_ws:-y}
    
    if [ "$rebuild_ws" = "y" ] || [ "$rebuild_ws" = "Y" ]; then
        echo ""
        echo "catkin 워크스페이스 재빌드 중..."
        cd $HOME/catkin_ws
        source /opt/ros/${ROS_DISTRO}/setup.bash
        
        # 빌드 도구 감지 (catkin_make vs catkin build)
        if [ -d ".catkin_tools" ]; then
            echo "catkin build 사용 중..."
            catkin build
        else
            echo "catkin_make 사용 중..."
            catkin_make
        fi
        
        echo ""
        echo "✓ catkin 워크스페이스 재빌드 완료"
    fi
fi

echo ""
echo "=========================================="
echo "✓ 모든 패키지 복구 완료!"
echo "=========================================="
echo ""

# 10. 설치 확인
echo "Step 10: 주요 패키지 설치 확인"
echo "----------------------------------------"

check_ros_package() {
    if dpkg -l | grep -q "ros-${ROS_DISTRO}-$1"; then
        echo "✓ ros-${ROS_DISTRO}-$1 설치됨"
        return 0
    else
        echo "✗ ros-${ROS_DISTRO}-$1 설치 안됨"
        return 1
    fi
}

check_ros_package "rqt-image-view"
check_ros_package "rqt-graph"
check_ros_package "rviz"
check_ros_package "cv-bridge"
check_ros_package "image-transport"
check_ros_package "usb-cam"

echo ""
echo "=========================================="
echo "복구 완료 안내"
echo "=========================================="
echo ""
echo "1. 새 터미널을 열거나 다음 명령을 실행하세요:"
echo "   source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "   source ~/catkin_ws/devel/setup.bash"
echo ""
echo "2. 패키지 확인:"
echo "   rospack list | grep rqt"
echo "   rosrun rqt_image_view rqt_image_view"
echo ""
echo "3. 문제가 계속되면:"
echo "   sudo apt-get install --reinstall ros-${ROS_DISTRO}-desktop-full"
echo ""
echo "복구가 완료되었습니다! 🎉"
echo ""
echo "⚠️  주의: 앞으로 시스템 패키지를 삭제하지 마세요!"
echo "   특히 python3-openssl 같은 핵심 패키지는 절대 제거하지 마세요."

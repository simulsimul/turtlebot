#!/bin/bash

# TurtleBot 자동 배포 스크립트
echo "TurtleBot 배포를 시작합니다."

# 시스템 정보 확인
echo "시스템 정보 확인 중..."
echo "아키텍처: $(uname -m)"
echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "메모리: $(free -h | grep Mem | awk '{print $2}')"

# 라즈베리파이 확인
if [[ $(uname -m) != "aarch64" ]] && [[ $(uname -m) != "armv7l" ]]; then
    echo "경고: 라즈베리파이가 아닌 시스템에서 실행 중입니다."
    echo "개발/테스트 모드로 실행됩니다."
    echo "계속하시겠습니까? (y/N)"
    read -r response
    if [[ ! $response =~ ^[Yy]$ ]]; then
        echo "배포를 취소합니다."
        exit 1
    fi
fi

# Docker 설치 확인 및 설치
if ! command -v docker &> /dev/null; then
    echo "Docker 설치 중..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    rm get-docker.sh
    echo "Docker 설치 완료. 시스템을 재시작하거나 다시 로그인해주세요."
else
    echo "Docker가 이미 설치되어 있습니다."
fi

# 기존 컨테이너 정리
echo "기존 TurtleBot 컨테이너 정리 중..."
sudo docker stop turtlebot-auto 2>/dev/null || true
sudo docker rm turtlebot-auto 2>/dev/null || true

# 멀티 플랫폼 이미지 다운로드
echo "TurtleBot 이미지 다운로드 중..."
echo "플랫폼에 맞는 이미지를 자동으로 선택합니다..."
echo "최신 업데이트를 확인합니다..."
sudo docker pull ybkim4053/simulsimul:latest

# 라즈베리파이 하드웨어 권한 설정 (ARM64에서만)
if [[ $(uname -m) == "aarch64" ]] || [[ $(uname -m) == "armv7l" ]]; then
    echo "라즈베리파이 하드웨어 권한 설정 중..."
    sudo usermod -a -G dialout $USER
    
    # USB 장치 권한 설정
    echo "USB 장치 권한 설정 중..."
    sudo chmod 666 /dev/ttyACM0 2>/dev/null || echo "OpenCR board not connected"
    sudo chmod 666 /dev/ttyUSB0 2>/dev/null || echo "LDS sensor not connected"
    
    DEVICE_OPTIONS="--device=/dev/ttyACM0:/dev/ttyACM0 --device=/dev/ttyUSB0:/dev/ttyUSB0"
    MEMORY_OPTIONS="--memory=1g --memory-swap=2g --oom-kill-disable=false"
else
    echo "개발/테스트 모드: 하드웨어 장치 설정을 건너뜁니다."
    DEVICE_OPTIONS=""
    MEMORY_OPTIONS="--memory=2g"
fi

# TurtleBot 실행
echo "TurtleBot 실행 중..."
sudo docker run -d \
    --name turtlebot-auto \
    --restart unless-stopped \
    --privileged \
    --network host \
    -v /dev:/dev \
    -e TURTLEBOT3_MODEL=burger \
    -e ROS_DOMAIN_ID=0 \
    -e LDS_MODEL=LDS-01 \
    -e RCL_ASSERT_RMW_ID_MATCHES=0 \
    -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    $DEVICE_OPTIONS \
    $MEMORY_OPTIONS \
    ybkim4053/simulsimul:latest

# 실행 상태 확인
echo "TurtleBot 시작 대기 중..."
sleep 20

if sudo docker ps | grep -q turtlebot-auto; then
    echo "TurtleBot이 성공적으로 시작되었습니다."
    echo "컨테이너 상태:"
    sudo docker ps | grep turtlebot-auto
    echo ""
    echo "로그 확인: sudo docker logs -f turtlebot-auto"
    echo "중지: sudo docker stop turtlebot-auto"
    echo "재시작: sudo docker restart turtlebot-auto"
else
    echo "TurtleBot 시작에 실패했습니다."
    echo "로그 확인:"
    sudo docker logs turtlebot-auto
    exit 1
fi

echo "TurtleBot 배포가 완료되었습니다."

#!/bin/bash

# 라즈베리파이 TurtleBot 부팅 시 자동 실행 설정

echo "TurtleBot 자동 실행 설정을 시작합니다."

# 현재 사용자 확인
CURRENT_USER=$(whoami)
echo "현재 사용자: $CURRENT_USER"

# 벽 추종 알고리즘 선택 메뉴
echo ""
echo "======================================"
echo "TurtleBot 벽 추종 알고리즘을 선택하세요:"
echo "======================================"
echo "1) 규칙 기반 (Rule-based)"
echo "   - 단순한 진실표 기반 행동 제어"
echo "   - 빠른 반응, 안정적인 성능"
echo ""
echo "2) PID 제어 (PID-based)" 
echo "   - 일정한 거리 유지 제어"
echo "   - 부드러운 움직임, 정밀한 제어"
echo ""
echo "3) 머신러닝 (Machine Learning)"
echo "   - 학습된 모델 기반 제어"
echo "   - 복잡한 환경 대응 가능"
echo ""
echo "======================================"

while true; do
    read -p "선택하세요 (1-3): " ALGORITHM_CHOICE
    case $ALGORITHM_CHOICE in
        1)
            WALL_FOLLOWER_TYPE="rule"
            ALGORITHM_NAME="규칙 기반"
            break
            ;;
        2)
            WALL_FOLLOWER_TYPE="pid"
            ALGORITHM_NAME="PID 제어"
            break
            ;;
        3)
            WALL_FOLLOWER_TYPE="ml"
            ALGORITHM_NAME="머신러닝"
            break
            ;;
        *)
            echo "잘못된 선택입니다. 1-3 중에서 선택해주세요."
            ;;
    esac
done

echo ""
echo "선택된 알고리즘: $ALGORITHM_NAME ($WALL_FOLLOWER_TYPE)"
echo ""

# systemd 서비스 파일 생성
echo "systemd 서비스 파일 생성 중..."
sudo tee /etc/systemd/system/turtlebot-auto.service > /dev/null <<EOF
[Unit]
Description=TurtleBot Auto Start Service ($ALGORITHM_NAME)
After=docker.service network-online.target
Wants=network-online.target
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
User=$CURRENT_USER
Group=docker
WorkingDirectory=/home/$CURRENT_USER
ExecStartPre=/bin/bash -c 'until docker info; do sleep 1; done'
ExecStartPre=-/usr/bin/docker stop turtlebot-auto
ExecStartPre=-/usr/bin/docker rm turtlebot-auto
ExecStartPre=/usr/bin/docker pull ybkim4053/simulsimul:latest
ExecStartPre=/usr/bin/docker image prune -f
ExecStart=/usr/bin/docker run -d \\
    --name turtlebot-auto \\
    --restart unless-stopped \\
    --privileged \\
    --network host \\
    -v /dev:/dev \\
    -e TURTLEBOT3_MODEL=burger \\
    -e ROS_DOMAIN_ID=0 \\
    -e LDS_MODEL=LDS-01 \\
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \\
    -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \\
    -e WALL_FOLLOWER_TYPE=$WALL_FOLLOWER_TYPE \\
    --device=/dev/ttyACM0:/dev/ttyACM0 \\
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \\
    --memory=1g \\
    --memory-swap=2g \\
    --oom-kill-disable=false \\
    ybkim4053/simulsimul:latest
ExecStop=/usr/bin/docker stop turtlebot-auto
ExecStopPost=/usr/bin/docker rm turtlebot-auto
TimeoutStartSec=1800
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOF

# 알고리즘 설정 저장 (나중에 확인용)
echo "알고리즘 설정 저장 중..."
sudo tee /etc/turtlebot-algorithm.conf > /dev/null <<EOF
# TurtleBot 벽 추종 알고리즘 설정
WALL_FOLLOWER_TYPE=$WALL_FOLLOWER_TYPE
ALGORITHM_NAME=$ALGORITHM_NAME
SETUP_DATE=$(date)
SETUP_USER=$CURRENT_USER
EOF

# 서비스 활성화
echo "서비스 등록 및 활성화 중..."
sudo systemctl daemon-reload
sudo systemctl enable turtlebot-auto.service

# 라즈베리파이 최적화 설정
echo "라즈베리파이 최적화 설정 중..."

# GPU 메모리 분할 최적화
if ! grep -q "gpu_mem=16" /boot/firmware/config.txt; then
    echo "gpu_mem=16" | sudo tee -a /boot/firmware/config.txt
fi

# Docker 로그 로테이션 설정
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
EOF

# udev 규칙 설정 (USB 장치 자동 권한)
echo "USB 장치 udev 규칙 설정 중..."
sudo tee /etc/udev/rules.d/99-turtlebot3.rules > /dev/null <<EOF
# TurtleBot3 OpenCR Board
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", MODE="0666", GROUP="dialout"
# LDS Sensor  
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
EOF
sudo udevadm control --reload-rules

# 스왑 파일 생성 (메모리 부족 방지)
if [ ! -f /swapfile ]; then
    echo "스왑 파일 생성 중..."
    sudo fallocate -l 2G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
fi

# 부팅 시 자동 로그인 설정
echo "자동 로그인 설정 중..."
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
sudo tee /etc/systemd/system/getty@tty1.service.d/override.conf > /dev/null <<EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty -a $CURRENT_USER --noclear %i \$TERM
EOF

# 완료 메시지
echo ""
echo "======================================"
echo "TurtleBot 자동 실행 설정이 완료되었습니다."
echo "======================================"
echo ""
echo "설정 내용:"
echo "  - 선택된 알고리즘: $ALGORITHM_NAME ($WALL_FOLLOWER_TYPE)"
echo "  - systemd 서비스: turtlebot-auto.service"
echo "  - 부팅 시 자동 실행: 활성화"
echo "  - GPU 메모리: 16MB (최적화)"
echo "  - 스왑 파일: 2GB"
echo "  - Docker 로그: 로테이션 설정"
echo ""
echo "재부팅 후 자동으로 TurtleBot이 실행됩니다:"
echo "  sudo reboot"
echo ""
echo "서비스 상태 확인:"
echo "  sudo systemctl status turtlebot-auto"
echo "  sudo docker logs -f turtlebot-auto"
echo ""
echo "알고리즘 설정 확인:"
echo "  cat /etc/turtlebot-algorithm.conf"
echo ""
echo "서비스 중지:"
echo "  sudo systemctl stop turtlebot-auto"
echo ""
echo "서비스 비활성화:"
echo "  sudo systemctl disable turtlebot-auto"
echo ""
echo "알고리즘 변경 시 이 스크립트를 다시 실행하세요."
echo "======================================" 
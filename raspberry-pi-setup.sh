#!/bin/bash

# 라즈베리파이 TurtleBot 부팅 시 자동 실행 설정

echo "TurtleBot 자동 실행 설정을 시작합니다."

# 현재 사용자 확인
CURRENT_USER=$(whoami)
echo "현재 사용자: $CURRENT_USER"

# systemd 서비스 파일 생성
echo "systemd 서비스 파일 생성 중..."
sudo tee /etc/systemd/system/turtlebot-auto.service > /dev/null <<EOF
[Unit]
Description=TurtleBot Auto Start Service
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
echo "TurtleBot 자동 실행 설정이 완료되었습니다."
echo ""
echo "설정 내용:"
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
echo "서비스 중지:"
echo "  sudo systemctl stop turtlebot-auto"
echo ""
echo "서비스 비활성화:"
echo "  sudo systemctl disable turtlebot-auto" 
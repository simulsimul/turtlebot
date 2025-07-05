#!/bin/bash

# TurtleBot 자동 배포 스크립트
DOCKER_IMAGE="simulsimul/turtlebot-auto:latest"

sudo apt update
sudo apt install -y docker.io

sudo usermod -aG docker $USER
sudo docker run -d \
  --name turtlebot-auto \
  --privileged \
  --network host \
  --restart unless-stopped \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  -e ROS_DOMAIN_ID=0 \
  -e TURTLEBOT3_MODEL=burger \
  $DOCKER_IMAGE

sudo tee /etc/systemd/system/turtlebot.service > /dev/null <<EOF
[Unit]
Description=TurtleBot Service
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/docker start turtlebot-auto
ExecStop=/usr/bin/docker stop turtlebot-auto

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable turtlebot

echo "설치 완료. 재부팅 후 자동 시작됩니다."
echo "수동 시작: docker start turtlebot-auto"
echo "로그 확인: docker logs -f turtlebot-auto"

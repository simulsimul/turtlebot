#!/bin/bash

echo "======================================"
echo "TurtleBot3 벽 추종 알고리즘 선택"
echo "======================================"
echo "1) rule  - 규칙 기반 알고리즘"
echo "2) pid   - PID 제어 알고리즘" 
echo "3) ml    - 머신러닝 알고리즘"
echo "======================================"

while true; do
    read -p "선택하세요 (1-3): " choice
    case $choice in
        1)
            ALGORITHM="rule"
            echo "선택된 알고리즘: 규칙 기반 (rule)"
            break
            ;;
        2)
            ALGORITHM="pid"
            echo "선택된 알고리즘: PID 제어 (pid)"
            break
            ;;
        3)
            ALGORITHM="ml"
            echo "선택된 알고리즘: 머신러닝 (ml)"
            break
            ;;
        *)
            echo "잘못된 선택입니다. 1-3 중에서 선택해주세요."
            ;;
    esac
done

echo ""
echo "TurtleBot3 컨테이너를 시작합니다..."

# 기존 컨테이너 정리
docker stop turtlebot-manual 2>/dev/null || true
docker rm turtlebot-manual 2>/dev/null || true

# 선택된 알고리즘으로 컨테이너 실행
docker run -it \
    --name turtlebot-manual \
    --privileged \
    --network host \
    -v /dev:/dev \
    -e TURTLEBOT3_MODEL=burger \
    -e ROS_DOMAIN_ID=0 \
    -e LDS_MODEL=LDS-01 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    -e WALL_FOLLOWER_TYPE=$ALGORITHM \
    --device=/dev/ttyACM0:/dev/ttyACM0 \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \
    --memory=1g \
    --memory-swap=2g \
    ybkim4053/simulsimul:latest 
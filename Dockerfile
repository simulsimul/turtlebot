# 라즈베리파이 ARM64 전용 최적화 이미지
FROM arm64v8/ros:humble-ros-base

# 라즈베리파이 최적화 환경 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# ARM64 최적화 - 메모리 효율적 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-rmw-cyclonedx-cpp \
    ros-humble-nav2-bringup \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-dwb-controller \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-msgs \
    ros-humble-nav2-navfn-planner \
    ros-humble-nav2-planner \
    ros-humble-nav2-recoveries \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-rviz-plugins \
    ros-humble-nav2-simple-commander \
    ros-humble-nav2-smoother \
    ros-humble-nav2-util \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-waypoint-follower \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 라즈베리파이 환경 변수 설정
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# 라즈베리파이 메모리 최적화
ENV MALLOC_ARENA_MAX=2
ENV MALLOC_MMAP_THRESHOLD_=131072
ENV MALLOC_TRIM_THRESHOLD_=131072
ENV MALLOC_TOP_PAD_=131072
ENV MALLOC_MMAP_MAX_=65536

# 작업 디렉토리 설정
WORKDIR /opt/ros/humble

# 라즈베리파이 최적화 엔트리포인트 스크립트
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# ROS 환경 설정\n\
source /opt/ros/humble/setup.bash\n\
\n\
# 라즈베리파이 하드웨어 체크\n\
echo "Starting TurtleBot3 on Raspberry Pi..."\n\
echo "Model: $TURTLEBOT3_MODEL"\n\
echo "ROS Domain ID: $ROS_DOMAIN_ID"\n\
echo "RMW Implementation: $RMW_IMPLEMENTATION"\n\
\n\
# 하드웨어 준비 대기\n\
echo "Waiting for hardware initialization..."\n\
sleep 15\n\
\n\
# 명령 실행\n\
exec "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"] 
# 멀티 플랫폼 지원 (ARM64/AMD64)
FROM ros:humble-ros-core

# 플랫폼 정보 가져오기
ARG TARGETPLATFORM
ARG BUILDPLATFORM

# 환경 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 필수 개발 도구 및 ROS2 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# rosdep 초기화
RUN rosdep init && rosdep update

# ROS2 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-node \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-dynamixel-sdk \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot 환경 변수 설정
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0
ENV LDS_MODEL=LDS-01
ENV OPENCR_PORT=/dev/ttyACM0
ENV OPENCR_MODEL=burger
ENV RCL_ASSERT_RMW_ID_MATCHES=0
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1

# ARM64 (라즈베리파이)용 메모리 최적화 설정
RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
    echo "Setting up ARM64 optimizations..."; \
    echo "export MALLOC_ARENA_MAX=2" >> /etc/environment; \
    echo "export MALLOC_MMAP_THRESHOLD_=131072" >> /etc/environment; \
    echo "export MALLOC_TRIM_THRESHOLD_=131072" >> /etc/environment; \
    echo "export MALLOC_TOP_PAD_=131072" >> /etc/environment; \
    echo "export MALLOC_MMAP_MAX_=65536" >> /etc/environment; \
    fi

# 작업 디렉토리 설정
WORKDIR /opt/ros/humble

# 워크스페이스 생성 및 필요한 패키지들 복사
RUN mkdir -p /opt/ros/humble/src
COPY webots_ros2_turtlebot /opt/ros/humble/src/webots_ros2_turtlebot
COPY webots_ros2_driver /opt/ros/humble/src/webots_ros2_driver
COPY webots_ros2_msgs /opt/ros/humble/src/webots_ros2_msgs

# 의존성 설치 및 패키지 빌드
RUN cd /opt/ros/humble && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-select webots_ros2_msgs webots_ros2_driver webots_ros2_turtlebot --symlink-install

# 플랫폼별 최적화 엔트리포인트 스크립트
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# ROS 환경 설정\n\
source /opt/ros/humble/setup.bash\n\
\n\
# 플랫폼 감지\n\
ARCH=$(uname -m)\n\
echo "Starting TurtleBot3 on $ARCH platform..."\n\
echo "Model: $TURTLEBOT3_MODEL"\n\
echo "LDS Model: $LDS_MODEL"\n\
echo "ROS Domain ID: $ROS_DOMAIN_ID"\n\
echo "OpenCR Port: $OPENCR_PORT"\n\
\n\
# ARM64 (라즈베리파이)에서만 하드웨어 설정\n\
if [ "$ARCH" = "aarch64" ]; then\n\
    echo "Configuring for Raspberry Pi hardware..."\n\
    \n\
    # USB 장치 권한 설정\n\
    if [ -e "$OPENCR_PORT" ]; then\n\
        echo "OpenCR board detected at $OPENCR_PORT"\n\
        chmod 666 "$OPENCR_PORT" || true\n\
    else\n\
        echo "Warning: OpenCR board not found at $OPENCR_PORT"\n\
    fi\n\
    \n\
    # LDS 센서 확인\n\
    if [ -e "/dev/ttyUSB0" ]; then\n\
        echo "LDS sensor detected at /dev/ttyUSB0"\n\
        chmod 666 /dev/ttyUSB0 || true\n\
    fi\n\
    \n\
    # 하드웨어 준비 대기\n\
    echo "Waiting for hardware initialization..."\n\
    sleep 10\n\
else\n\
    echo "Running in development/simulation mode on $ARCH"\n\
fi\n\
\n\
# 명령 실행\n\
exec "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/ros/humble/install/setup.bash && ros2 launch turtlebot3_bringup robot.launch.py & sleep 10 && ros2 run webots_ros2_turtlebot auto_navigator"] 
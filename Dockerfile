# 공식 TurtleBot3 베이스 이미지 사용
FROM ros:humble-ros-base

# 플랫폼 정보 가져오기
ARG TARGETPLATFORM
ARG BUILDPLATFORM

# 환경 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 필수 시스템 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
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

# TurtleBot3 전체 스택 설치 (공식 메타패키지)
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-cartographer \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-teleop \
    ros-humble-turtlebot3-example \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-dynamixel-sdk \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-rclpy \
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

# 자율주행 코드를 위한 워크스페이스 생성
RUN mkdir -p /opt/turtlebot3_ws/src

# 자율주행 코드 복사
COPY webots_ros2_turtlebot/webots_ros2_turtlebot/auto_navigator.py /opt/turtlebot3_ws/src/

# 실행 권한 부여
RUN chmod +x /opt/turtlebot3_ws/src/auto_navigator.py

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
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch turtlebot3_bringup robot.launch.py & sleep 10 && python3 /opt/turtlebot3_ws/src/auto_navigator.py"] 
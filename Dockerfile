# 공식 ROS2 Humble 베이스 이미지 사용 (필요한 패키지만 설치)
FROM ros:humble-ros-base

# 플랫폼 정보 가져오기
ARG TARGETPLATFORM
ARG BUILDPLATFORM

# 환경 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 필수 시스템 패키지 및 TurtleBot3 스택 설치 (한번에 처리)
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
    ros-humble-nav2-bringup \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-rclpy \
    python3-numpy \
    python3-sklearn \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# rosdep 업데이트 (이미 초기화됨)
RUN rosdep update

# xacro 실행 파일 PATH 설정 및 확인
RUN echo 'export PATH="/opt/ros/humble/bin:$PATH"' >> /etc/bash.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && which xacro" && \
    ln -sf /opt/ros/humble/bin/xacro /usr/local/bin/xacro

# TurtleBot 환경 변수 설정
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0
ENV LDS_MODEL=LDS-01
ENV OPENCR_PORT=/dev/ttyACM0
ENV OPENCR_MODEL=burger
ENV RCL_ASSERT_RMW_ID_MATCHES=0
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1

# 벽 추종 알고리즘 선택 환경 변수 (실행 시 사용자 입력으로 결정)
# ENV WALL_FOLLOWER_TYPE=pid
# ENV WALL_FOLLOWER_TYPE=ml
ENV WALL_FOLLOWER_TYPE=rule

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

# TurtleBot 워크스페이스 생성
RUN mkdir -p /opt/turtlebot3_ws/src

# 전체 webots_ros2_turtlebot 패키지 복사
COPY webots_ros2_turtlebot/ /opt/turtlebot3_ws/src/webots_ros2_turtlebot/

# 패키지 빌드
RUN cd /opt/turtlebot3_ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select webots_ros2_turtlebot"

# 실행 권한 부여
RUN chmod +x /opt/turtlebot3_ws/src/webots_ros2_turtlebot/webots_ros2_turtlebot/*.py

# 시작 스크립트 생성
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# ROS 환경 설정\n\
source /opt/ros/humble/setup.bash\n\
source /opt/turtlebot3_ws/install/setup.bash\n\
\n\
# 플랫폼 감지\n\
ARCH=$(uname -m)\n\
echo "Starting TurtleBot3 on $ARCH platform..."\n\
echo "Model: $TURTLEBOT3_MODEL"\n\
echo "LDS Model: $LDS_MODEL"\n\
echo "ROS Domain ID: $ROS_DOMAIN_ID"\n\
echo "OpenCR Port: $OPENCR_PORT"\n\
echo "Wall Follower Type: $WALL_FOLLOWER_TYPE"\n\
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

# 실행 스크립트 생성 (알고리즘 선택 기능 포함)
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /opt/turtlebot3_ws/install/setup.bash\n\
export PATH="/opt/ros/humble/bin:$PATH"\n\
\n\
# xacro 확인\n\
echo "Checking xacro availability..."\n\
which xacro || echo "xacro not found in PATH"\n\
\n\
# 사용자로부터 알고리즘 선택 받기\n\
#echo "================================="\n\
#echo "TurtleBot3 Wall Following Algorithm"\n\
#echo "================================="\n\
#echo "사용할 벽 추종 알고리즘을 선택하세요:"\n\
#echo "1) rule  - 규칙 기반 알고리즘"\n\
#echo "2) pid   - PID 제어 알고리즘"\n\
#echo "3) ml    - 머신러닝 알고리즘"\n\
#echo "================================="\n\
#\n\
#while true; do\n\
#    read -p "선택 (rule/pid/ml): " WALL_FOLLOWER_TYPE\n\
#    case "$WALL_FOLLOWER_TYPE" in\n\
#        "rule"|"pid"|"ml")\n\
#            echo "선택된 알고리즘: $WALL_FOLLOWER_TYPE"\n\
#            break\n\
#            ;;\n\
#        *)\n\
#            echo "잘못된 입력입니다. rule, pid, ml 중 하나를 입력하세요."\n\
#            ;;\n\
#    esac\n\
#done\n\
#\n\
#echo ""\n\
#echo "TurtleBot3 시작 중..."\n\
\n\
# 하드웨어 노드 시작\n\
echo "Starting TurtleBot3 hardware nodes..."\n\
ros2 launch turtlebot3_bringup robot.launch.py &\n\
BRINGUP_PID=$!\n\
\n\
# 하드웨어 노드가 준비될 때까지 대기\n\
echo "Waiting for hardware nodes to be ready..."\n\
sleep 15\n\
\n\
# 선택된 알고리즘에 따라 적절한 노드 시작\n\
echo "Starting wall following algorithm: $WALL_FOLLOWER_TYPE"\n\
case "$WALL_FOLLOWER_TYPE" in\n\
    "rule")\n\
        echo "Starting Rule-based wall follower..."\n\
        ros2 run webots_ros2_turtlebot wall_follower_rule &\n\
        ALGO_PID=$!\n\
        ;;\n\
    "pid")\n\
        echo "Starting PID-based wall follower..."\n\
        ros2 run webots_ros2_turtlebot wall_follower_pid &\n\
        ALGO_PID=$!\n\
        ;;\n\
    "ml")\n\
        echo "Starting ML-based wall follower..."\n\
        ros2 run webots_ros2_turtlebot wall_follower_ml &\n\
        ALGO_PID=$!\n\
        ;;\n\
esac\n\
\n\
echo "TurtleBot3 벽 추종 시작됨 ($WALL_FOLLOWER_TYPE 알고리즘)"\n\
echo "종료하려면 Ctrl+C를 누르세요."\n\
\n\
# 시그널 핸들링\n\
trap "echo \"Shutting down...\"; kill $BRINGUP_PID $ALGO_PID 2>/dev/null; exit 0" SIGTERM SIGINT\n\
\n\
# 백그라운드 프로세스 대기\n\
wait $BRINGUP_PID $ALGO_PID' > /start_turtlebot.sh && \
chmod +x /start_turtlebot.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start_turtlebot.sh"] 
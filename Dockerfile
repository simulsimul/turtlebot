FROM osrf/ros:humble-desktop

# Set environment to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install TurtleBot3 packages
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-rmw-cyclonedx-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
sleep 10\n\
exec "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"] 
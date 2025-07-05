FROM ros:humble-desktop

# Install TurtleBot3 packages
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
sleep 10\n\
exec "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"] 
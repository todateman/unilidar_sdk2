# Unitree LiDAR ROS2 Docker Image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    pcl-tools \
    libboost-all-dev \
    python3-pip \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS2 packages including bridge packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-rosbridge-suite \
    python3-websockets \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace

# Copy the entire SDK
COPY . /workspace/unilidar_sdk2/

# Build the ROS2 package
WORKDIR /workspace/unilidar_sdk2/unitree_lidar_ros2

# Clean existing build files and build
RUN rm -rf build/ install/ log/ && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 environment\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source /workspace/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash\n\
\n\
# Wait for serial device to be available (only for services that need it)\n\
if [ -n "${SERIAL_DEVICE}" ] && [ "${SERIAL_DEVICE}" != "" ]; then\n\
    echo "Waiting for serial device ${SERIAL_DEVICE}..."\n\
    while [ ! -e "${SERIAL_DEVICE}" ]; do\n\
        echo "Serial device ${SERIAL_DEVICE} not found. Waiting..."\n\
        sleep 1\n\
    done\n\
    echo "Serial device ${SERIAL_DEVICE} found. Starting ROS2 node..."\n\
else\n\
    echo "No serial device check required. Starting service..."\n\
fi\n\
\n\
# Execute the command\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# Set default environment variables
ENV SERIAL_DEVICE=/dev/ttyACM0
ENV BAUDRATE=4000000

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["ros2", "launch", "unitree_lidar_ros2", "launch.py"]
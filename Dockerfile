FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav2-msgs \
    python3-colcon-common-extensions \
    python3-pip \
    python3-dev \
    python3-numpy \
    git \
    curl \
    net-tools \
    iputils-ping \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Set ROS 2 environment
ENV ROS_DOMAIN_ID=10
ENV ROS_LOCALHOST_ONLY=0
ENV TURTLEBOT3_MODEL=burger

# Create ROS workspace in container
WORKDIR /home/iki_workspace

# Copy ONLY contents of stage_5 (ROS packages)
COPY . /home/iki_workspace/src

# Build ROS workspace
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Launch the node
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run autorace_real start_signal_real"]

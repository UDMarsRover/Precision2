FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions python3-pip python3-rclpy python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Copy camera_module package
WORKDIR /workspace
COPY src/camera_module /workspace/camera_module

# Install Python dependencies
RUN pip3 install -r /workspace/camera_module/requirements.txt || true

# Source ROS 2 setup and run servo node
CMD ["sleep", "infinity"]

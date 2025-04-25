# Use the official ROS2 Humble base image
FROM ros:humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    ros-humble-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

# Copy the workspace into the container
# COPY ./ /Precision2/
VOLUME /workspace

# Set the working directory to the workspace
WORKDIR /workspace

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"


# Run your application


# Set the entrypoint to the bash shell
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && bash"]
# CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && bash"]
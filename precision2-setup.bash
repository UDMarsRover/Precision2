#!/bin/bash

# Function to install ROS 2 Humble
install_ros2_humble() {
    echo "Installing ROS 2 Humble..."
    sudo apt update
    sudo apt install -y curl gnupg2 lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt update
    sudo apt install -y ros-humble-desktop
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# Function to install ROS 2 Jazzy
install_ros2_jazzy() {
    echo "Installing ROS 2 Jazzy..."
    sudo apt update
    sudo apt install -y curl gnupg2 lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)

if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    install_ros2_humble
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    install_ros2_jazzy
else
    echo "Unsupported Ubuntu version: $UBUNTU_VERSION"
fi

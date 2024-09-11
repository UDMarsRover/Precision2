# Precision2
Welcome to the repository for Precision2. This repository is based on much of the source code developed in Precision1. However, with a reorganization of packages, a switch to ROS2, and much more, this repository is being built from the ground up. 


## Creating Packages
To create a ROS2 package, use the following command from the reposiroty `src/` directory:
```bash
ros2 pkg create --build-type ament_python <package_name>
```

## Creating a Node
To create a node, create a file in the correct location, which follows the format:
```
Precision2/src/<package_name>/<package_name>/<your_script.ext>
```
Here, the script will usually be a `.py` file. 
Then, add the executable to the `setup.py` file: 

## Building Packages
From the repository's root directory, run the command:
```bash
colcon build
```

## Installing Packages
To source the installation of the package (you need to do this to run any of the built ROS nodes), run the command from the repository's root directory:
```bash
source install/setup.bash
```

## Running Nodes

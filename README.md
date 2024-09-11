# Precision2
Welcome to the repository for Precision2. This repository is based on much of the source code developed in Precision1. However, with a reorganization of packages, a switch to ROS2, and much more, this repository is being built from the ground up. 

## ROS2 Version
For Precision2, we will be using [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

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
At the bottom of the `Precision2/src/<package_name>/setup.py` file, you will see something like this
```python
entry_points = {
  'console_scripts': [
    '<node_name> = <package_name>.<script_name>:main'
  ]
}
```
Add a new element to `console_scripts` for your executable. The `<node_name>` should be a name for your executable (the name used by `ros2 run`). The `<package_name>` should match the name of the package, and the `<script_name>` should be the name of your script(if the file is `ros_pub.py`, you would replace `<script_name>` with `ros_pub`). Finally, your script will need a `main` function in it that will be called upon execution. 

Lastly, ROS uses a program called 'rosdep' which manages dependencies. To ensure any external dependencies you are using are accessible across computers, add these to the `Precision2/src/<package_name>/package.xml` file. To do so see [Managing Dependencies With Rosdep](#). TODO: add link
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

Pull the ROS Humble image: 
docker pull osrf/ros:humble-desktop

To run docker container, first build:
docker build -t humble .

Then run: 
docker run -it --net=host --ipc=host humble
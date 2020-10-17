#!/bin/bash

image_name="save_dataset"

xhost +
docker run -it --rm \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--net=host \
	$image_name:latest \
	bash -c "source /home/ros_catkin_ws/devel/setup.bash;roslaunch save_dataset odom_check_diff_and_still.launch"

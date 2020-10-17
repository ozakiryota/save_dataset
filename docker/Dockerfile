# FROM ros:kinetic
FROM osrf/ros:kinetic-desktop-full

########## BASIS ##########
RUN apt-get update && \
	apt-get install -y \
		vim \
		wget \
		unzip \
		git \
		build-essential
########## ROS setup ##########
RUN mkdir -p /home/ros_catkin_ws/src && \
	cd /home/ros_catkin_ws/src && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace" && \
	cd /home/ros_catkin_ws && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
	echo "source /home/ros_catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/ros_catkin_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/home/ros_catkin_ws" >> ~/.bashrc && \
	echo "function cmk(){\n lastpwd=\$OLDPWD \n cpath=\$(pwd) \n cd /home/ros_catkin_ws \n catkin_make \$@ \n cd \$cpath \n OLDPWD=\$lastpwd \n}" >> ~/.bashrc
########## My package ##########
##### pcl 1.9.1 #####
RUN	mkdir -p /home/pcl_ws && \
	cd /home/pcl_ws && \
	wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz && \
	tar -zxvf pcl-1.9.1.tar.gz && \
	cd pcl-pcl-1.9.1 && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make -j $(nproc --all) && \
	make -j $(nproc --all) install
##### NO cache #####
ARG CACHEBUST=1
##### git clone #####
# RUN	cd /home/ros_catkin_ws/src && \
# 	git clone https://github.com/ozakiryota/gyrodometry && \
# 	git clone https://github.com/ozakiryota/save_dataset && \
# 	cd /home/ros_catkin_ws && \
# 	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"
RUN	cd /home/ros_catkin_ws/src && \
	git clone https://github.com/ozakiryota/gyrodometry && \
	git clone https://github.com/ozakiryota/save_dataset
######### Initial position ##########
WORKDIR /home/ros_catkin_ws

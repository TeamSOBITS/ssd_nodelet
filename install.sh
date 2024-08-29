#!/bin/bash

echo "╔══╣ Install: ssd_ros (STARTING) ╠══╗"


# Keep track of the current directory
DIR=$(pwd)

# Clone required packages
cd ..
git clone -b feature/humble-devel https://github.com/TeamSOBITS/sobits_msgs.git
git clone -b feature/humble-devel https://github.com/TeamSOBITS/bbox_to_tf.git
cd bbox_to_tf/
bash install.sh

# Install Camera Packages
sudo apt-get install -y \
    v4l-utils \
    libboost-python-dev


# Install ROS Packages
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs

sudo apt-get install -y \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-common \
    ros-${ROS_DISTRO}-image-proc


cd $DIR


echo "╚══╣ Install: ssd_ros (FINISHED) ╠══╝"
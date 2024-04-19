#!/bin/bash

echo "╔══╣ Install: ssd_nodelet (STARTING) ╠══╗"


# Download SOBITS Msgs
DIR=$(pwd)
cd ..
git clone https://github.com/TeamSOBITS/sobits_msgs.git
cd $DIR


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
    ros-${ROS_DISTRO}-nodelet


# Install Camera Packages
sudo apt-get install -y \
    v4l-utils 

sudo apt-get install -y \
    ros-${ROS_DISTRO}-libuvc-camera \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-image-proc


# Setting uvc_camera (WebCamera)
echo "SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"0458\", ATTRS{idProduct}==\"708c\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-uvc.rules
sudo udevadm control --reload-rules
sudo udevadm trigger


# USB Reload
sudo /etc/init.d/udev reload

v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext


echo "╚══╣ Install: ssd_nodelet (FINISHED) ╠══╝"
#!/bin/sh

echo "╔══╣ Install: ssd_nodelet (STARTING) ╠══╗"


sudo apt-get update

sudo apt-get install -y \
    v4l-utils 

sudo apt-get install -y \
    ros-${ROS_DISTRO}-libuvc-camera \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-image-proc

# Setting dynamixel USB1 (SOBIT PRO arm_pantilt)
echo "SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"0458\", ATTRS{idProduct}==\"708c\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-uvc.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# USB Reload
sudo /etc/init.d/udev reload

v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext


echo "╚══╣ Install: ssd_nodelet (FINISHED) ╠══╝"
#!/bin/sh

sudo apt update
sudo apt install v4l-utils -y
sudo apt install ros-noetic-libuvc-camera -y
sudo apt install ros-noetic-camera-calibration -y
sudo apt install ros-noetic-image-proc -y

# Seting dynamixel USB1 (SOBIT PRO arm_pantilt)
echo "SUBSYSTEMS==\"usb\", ENV{DEVTYPE}==\"usb_device\", ATTRS{idVendor}==\"0458\", ATTRS{idProduct}==\"708c\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-uvc.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# USB Reload
sudo /etc/init.d/udev reload

v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext
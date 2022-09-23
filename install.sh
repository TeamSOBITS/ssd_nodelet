#!/bin/sh

sudo apt update
sudo apt install ros-noetic-libuvc-camera ros-noetic-camera-calibration ros-noetic-image-proc
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext
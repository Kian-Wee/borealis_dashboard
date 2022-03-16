#!/bin/bash

# Setup script for borealis, install dependancies and clones packages into borealis_ws/src folder

pip install PyQt5

cd ~/borealis_ws/src
git clone https://github.com/Kian-Wee/borealis_dashboard
git clone https://github.com/Kian-Wee/borealis_uav_target_publisher
git clone https://github.com/acachathuranga/uwb_msgs
git clone https://github.com/acachathuranga/ROS_pose_to_path
catkin build


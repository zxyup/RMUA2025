#!/bin/bash

## 环境配置
# su &&
# apt update && apt install -y python3-catkin-tools ros-noetic-geographic-msgs \
#  ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
#  net-tools &&

export ROS_DISTRO=noetic
. /opt/ros/${ROS_DISTRO}/setup.sh && 
catkin_make --only-pkg-with-deps airsim_ros && 
. devel/setup.sh && 
catkin_make --only-pkg-with-deps basic_dev &&

# 检查是否传递了参数 "--no-run"
if [[ "$1" != "--no-run" ]]; then
    # 如果没有传递参数 "--no-run"，则执行以下两行代码
    source devel/setup.bash
    rosrun basic_dev basic_dev
fi
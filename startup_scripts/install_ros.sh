#!/bin/bash

# install ros
echo "Start to install the ros, http://wiki.ros.org/$rosversion/Installation/Ubuntu"
echo "Update the software list"
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
# 旧的KEY
# sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

echo "Install the ros from apt"
sudo apt-get install ros-$rosversion-desktop-full -y
sudo rosdep init
rosdep update
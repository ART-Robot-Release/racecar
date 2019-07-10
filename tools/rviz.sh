#!/bin/bash

# source me!
# when it self is master
export ROS_IP=`hostname -I`
export ROS_MASTER_URI="http://car:11311"

roslaunch art_racecar rviz.launch
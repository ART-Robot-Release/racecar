#!/bin/bash
export LANG=C.UTF-8

# source me!
# when it self is master
export ROS_IP=`hostname -I`
export ROS_MASTER_URI="http://localhost:11311"

roslaunch art_racecar go_gmapping.launch
#!/bin/bash
export LANG=C.UTF-8

# source me!
# when it self is master
export ROS_IP=`hostname -I`
# export ROS_MASTER_URI="http://`hostname -I|tr -d '[:space:]'`:11311"
export ROS_MASTER_URI="http://localhost:11311"
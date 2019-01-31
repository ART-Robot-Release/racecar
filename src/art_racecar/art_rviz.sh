#!/bin/bash

#for open rviz and set ROS master ip
#Steven.Zhang
#2018.03.09

function get_ip_address { ifconfig | fgrep -v 127.0.0.1 | fgrep 'Mask:255.255.255.0' | egrep -o 'addr:[^ ]*' | fgrep '.8.'| sed 's/^.*://'; }        

#export ROS_IP=$(get_ip_address) 
export ROS_IP=`hostname -I`
export ROS_MASTER_URI=http://192.168.5.101:11311

##source ../devel/setup.bash
#rosrun rviz rviz 

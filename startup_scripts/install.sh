#!/bin/bash

. ../configure.sh
#ubuntu 16

# ubuntu 18.04
# rosversion="melodic"  # so many bugs
# Install the ros

echo "install git"
sudo apt install git -y

echo "install sshd"
sudo apt install openssh-server

cd $install_path
git clone https://github.com/snomiao/racecar

. ./install_ros.sh

echo "Compile the art_racecar"
catkin_make -j8

echo "configuring the serial udev of the car."
sudo bash $install_path/racecar/src/art_racecar/udev/art_init.sh

echo "--Installing Finished, please reboot the computer."


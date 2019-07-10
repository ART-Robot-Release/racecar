#!/bin/bash

# default
ip_controller="192.168.5.12"
ip_master=`hostname -I`
rosversion="kinetic"
install_path="~"

. ../configure.sh
#ubuntu 16

# ubuntu 18.04
# rosversion="melodic"  # so many bugs
# Install the ros

echo "install git"
sudo apt install git -y

echo "install sshd"
sudo apt install openssh-server

echo "clone this project"
cd $install_path
#git clone https://github.com/snomiao/racecar

if [ `id -u` == 0 ]; then
    echo "Don't running this use root(sudo)."
    exit 0
fi

############################################################################
# Install ros
bash install_ros.sh

echo "Setup the ROS environment variables"
echo -e "if [ -f /opt/ros/$rosversion/setup.bash ]; then\n\tsource /opt/ros/$rosversion/setup.bash\nfi" >> ~/.bashrc
echo "source $install_path/racecar/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Install the rosinstall"
sudo apt-get install python-rosinstall -y

echo "Install the ssh"
sudo apt-get install ssh -y

echo "Install the ntpdate"
sudo apt-get install ntpdate -y

echo "Install the chrony"
sudo apt-get install chrony -y

# Install the dependecies for the project
echo "Start to config for the project"

#echo "Install the python dependecies"
#sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose -y

#echo "Install the eigen3"
#sudo apt install libeigen3-dev -y

#echo "Install the nlopt"
#sudo apt install libnlopt* -y

echo "Install the ROS package for art_racecar"
sudo apt-get install ros-$rosversion-joy -y
sudo apt-get install ros-$rosversion-move-base -y
sudo apt-get install ros-$rosversion-mrpt* -y
sudo apt-get install ros-$rosversion-geographic-msgs -y
sudo apt-get install ros-$rosversion-map-server -y
sudo apt-get install ros-$rosversion-gmapping -y   # missing in melodic
sudo apt-get install ros-$rosversion-amcl -y
sudo apt-get install ros-$rosversion-rviz-imu-plugin -y
sudo apt-get install ros-$rosversion-dwa-local-planner -y

############################################################################

echo "Compile the art_racecar"
catkin_make -j8

echo "configuring the serial udev of the car."
cd $install_path/racecar/src/art_racecar/udev/
. art_init.sh

echo "--Installing Finished, please reboot the computer."



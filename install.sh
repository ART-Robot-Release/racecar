#!/bin/bash

# Varibles
ip_controller="192.168.5.12"
ip_master=`hostname -I`
rosversion="kinetic"
install_path="~"

#ubuntu 16

# ubuntu 18.04
# rosversion="melodic"  # so many bugs
# Install the ros

# config proxy
echo "proxy configuring"
echo "
export http_proxy="http://"$ip_controller":1080/"
export https_proxy="http://"$ip_controller":1080/"
">>~/.bashrc

echo "install git"
sudo apt install git -y

echo "install sshd"
sudo apt install openssh-server

cd $install_path
git clone https://github.com/snomiao/racecar

if [ `id -u` == 0 ]; then
echo "Don't running this use root(sudo)."
exit 0
fi

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

echo "Compile the art_racecar"
catkin_make -j8

echo "configuring the serial udev of the car."
sudo bash $install_path/racecar/src/art_racecar/udev/art_init.sh

echo "--Installing Finished, please reboot the computer."


# turbot3

remotePC:

roscore

 
turtlebot3:

roslaunch turbot3_bringup  robot.launch

turtlebot3:

export TURTLEBOT3_MODEL=burger
roslaunch turbot3_bringup  remote.launch

turtlebot3, cartographer with imu：
roslaunch turbot3_slam cartographer.launch

turbot3 ,cartographer without imu：
roslaunch turbot3_slam cartographer_noimu.launch

remote PC
roslaunch turbot3_rviz cartographer_rviz.launch

remote PC
roslaunch turbot3_teleop keyboard.launch 

mkdir ~/map
rosrun map_server map_saver -f ~/map/test 
 

freenect_stack
==============

Build status on Travis CI [![Build Status](https://travis-ci.org/ros-drivers/freenect_stack.svg?branch=master)](http://travis-ci.org/ros-drivers/freenect_stack)

libfreenect based ROS driver


## tilt motor control usage：

#### 1.launch the driver
```
roslaunch freenect_launch freenect-xyz.launch
```
#### 2.publish tilt motor position topic
```
rostopic pub /set_tilt_degree std_msgs/Int16 '{data: -20}' -r 1
```
{data: -20} is the motor position,you can change -20 to any integer number in [-30 30].

## Made with ❤️ by BlueWhale Tech corp.

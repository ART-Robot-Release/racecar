#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "openni_driver");

  nodelet::Loader manager(true); // Bring up manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // Driver nodelet
  manager.load(ros::this_node::getName(), "openni_camera/driver", remappings, my_argv);

  // Manager service calls are on global callback queue
  ros::spin();
}

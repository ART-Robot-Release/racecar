#ifndef TILTDRIVER_H
#define TILTDRIVER_H

// freenect wrapper

#include <freenect_camera/freenect_driver.hpp>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"

namespace freenect_camera
{

class TiltDriver
{
public:
  TiltDriver(){
    cmd_topic_ = "/set_tilt_degree";
    device_ready_ = false;
    set_degree_ = 0.0;
    current_degree_ = 0.0;
    device_ =NULL;
  }

  TiltDriver(boost::shared_ptr<FreenectDevice> device,bool * close_tiltThread){
    close_tiltThread_=close_tiltThread;
    if(device)
    {
        device_=device->device_;
        device_ready_=true;
        //ROS_INFO("get you!\n");
    }
    else
    {
      device_ =NULL;
    }
    cmd_topic_ = "/set_tilt_degree";
    set_degree_ = 0.0;
    current_degree_ = 0.0;
    //ROS_INFO("get you!\n");
  }

  void run(){

    ros::NodeHandle nodeHandler("tilt");
    //ROS_INFO("get you6!\n");

    mTiltPub_=nodeHandler.advertise<std_msgs::Float64>("/current_tilt_degree",1,true);
    //ROS_INFO("get you2!\n");
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic_, 10, &TiltDriver::sendcmd, this);
    //ROS_INFO("get you7!\n");

    ros::Rate r(1);//发布周期为1hz
    //ROS_INFO("get you3!\n");
    while (!(*close_tiltThread_))
    {
        if(device_ready_)
        {
          publishState();
        }
        r.sleep();
    }

  }

  void sendcmd(const std_msgs::Int16 &set_degree){
    set_degree_=set_degree.data;
    if(device_ready_)
    {
      if(freenect_set_tilt_degs(device_, set_degree_))
      {
        device_ready_=false;
      }
    }
  }

  void publishState(){
    if(-1==freenect_update_tilt_state(device_))
    {
      device_ready_=false;
      ROS_INFO("ERROR: CANNOT UPDATE TILT STATE!\n");
      return;
    }
    current_degree_=freenect_get_tilt_degs(freenect_get_tilt_state(device_));
    std_msgs::Float64 status;
    status.data=current_degree_;
    mTiltPub_.publish(status);
  }
private:
  /** \brief the actual openni device */
  freenect_device * device_;
  double current_degree_;
  double set_degree_;
  std::string cmd_topic_;
  bool device_ready_;
  ros::Publisher mTiltPub_;
  bool* close_tiltThread_;
};


}
#endif // TILTDRIVER_H

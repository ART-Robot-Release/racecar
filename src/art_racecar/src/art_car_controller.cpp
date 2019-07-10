/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "PID.h"
#include "art_car_controller.hpp"

#define PI 3.14159265358979
int start_loop_flag = 0;
int start_speed = 1560;
extern  PID  pid_speed;

L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 0.26);
    pn.param("Lrv", Lrv, 10.0);
    pn.param("Vcmd", Vcmd, 1.0);
    pn.param("lfw", lfw, 0.13);
    pn.param("lrv", lrv, 10.0);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 1575);
    pn.param("baseAngle", baseAngle, 90.0);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    Lfw = goalRadius = getL1Distance(Vcmd);
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    cmd_vel.linear.x = 1500; // 1500 for stop
    cmd_vel.angular.z = baseAngle;

    //Show info
    ROS_INFO("[param] baseSpeed: %d", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();
    car_stop = 0;
}

void L1Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            //ROS_INFO("Goal Reached !");
            car_stop = 100;
        }
    }
}




void L1Controller::controlLoopCB(const ros::TimerEvent&)
{
    int count = 100;
    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = baseAngle;

    if(goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);  
        if(foundForwardPt)
        {

            cmd_vel.angular.z = baseAngle + getSteeringAngle(eta)*Angle_gain;
            /*Estimate Gas Input*/

            if(!goal_reached)
            {
                if(start_loop_flag++ <= 10)
                {

                    double u = getGasInput(carVel.linear.x);
                    
                    cmd_vel.linear.x = start_speed + PIDCal(&pid_speed,u);



                     start_speed += 4;
                     if(cmd_vel.linear.x > baseSpeed)   cmd_vel.linear.x = baseSpeed;
                     ROS_INFO("baseSpeed = %.2f\tSteering angle = %.2f",cmd_vel.linear.x,cmd_vel.angular.z);
                }
                else
                {
                    //ROS_INFO("!goal_reached");
                    double u = getGasInput(carVel.linear.x);                   
                    cmd_vel.linear.x = baseSpeed + PIDCal(&pid_speed,u);
                    
                    ROS_INFO("Gas = %.2f\tSteering angle = %.2f",cmd_vel.linear.x,cmd_vel.angular.z);
                }  
            }

        }
    }
    if(car_stop > 0)
    {
        start_loop_flag = 0;
        if(carVel.linear.x > 0)
        {

            cmd_vel.linear.x = 1300; //反向刹车
            pub_.publish(cmd_vel);
           // for(int i=0;i<20;i++)
           // {
           //     pub_.publish(cmd_vel);
           //     sleep(0.1);
           //     ROS_INFO("cat stop cmd_vel= %f",cmd_vel.linear.x);
           // }
            
        }
        else
        {
            car_stop = 0;
            cmd_vel.linear.x = 1500;
            pub_.publish(cmd_vel);

            //ROS_INFO("cmd_vel= %f",cmd_vel.linear.x);
        }
    }
    else
    {
        pub_.publish(cmd_vel);
        car_stop = 0;
        //ROS_INFO("car run cmd_vel= %f",cmd_vel.linear.x);
    }
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "art_car_controller");
    L1Controller controller;
    controller.PID_init();
    ros::spin();
    return 0;
}

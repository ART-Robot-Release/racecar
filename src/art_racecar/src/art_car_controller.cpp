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
double start_speed = 1580;
extern  PID  pid_speed;
CircleData CD1;
void PIDInit (struct PID *pp)                     //PID参数初始化，都置0
{
    memset(pp, 0, sizeof(PID));
}

double PIDCal(struct PID *pp, double ThisError)
{
    //增量式PID算法（需要控制的不是控制量的绝对值，而是控制量的增量）
    double pError,dError,iError;
    double templ;
    pError = ThisError-pp->LastError;
    iError = ThisError;
    dError = ThisError-2*(pp->LastError)+pp->PreError;
    //增量计算
    templ=pp->Proportion*pError + pp->Integral*iError+pp->Derivative*dError;  //增量

    //存储误差用于下次运算
    pp->PreError  = pp->LastError;
    pp->LastError = ThisError;

    return templ;
}

struct  PID  pid_speed;

void L1Controller::PID_init()
{
    PIDInit(&(pid_speed));
    pid_speed.SetPoint = Vcmd;
    pid_speed.Proportion = 20.0;
    pid_speed.Integral = 5.0;
    pid_speed.Derivative = 2.0;
}

void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}


void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}


void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    double x = carPose.orientation.x;
    double y = carPose.orientation.y;
    double z = carPose.orientation.z;
    double w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);
   //ROS_INFO("yaw = %.2f",yaw);
    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    double car2wayPt_x = wayPt.x - carPose.position.x;
    double car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    double car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    double car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;
    //ROS_INFO("dis:%f",car_car2wayPt_x);
    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else 
    {
       
        return false;

    }          
}


bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}

CircleData findCircle(geometry_msgs::Point pt1,geometry_msgs::Point pt2,geometry_msgs::Point pt3)//pt2为中点
{
    geometry_msgs::Point midpt1,midpt2;//定义两个点，分别表示两个中点
    midpt1.x=(pt2.x+pt1.x)/2;
    midpt1.y=(pt2.y+pt1.y)/2;

    midpt2.x=(pt3.x+pt2.x)/2;
    midpt2.y=(pt3.y+pt2.y)/2;

    float k1=-(pt2.x-pt1.x)/(pt2.y-pt1.y);
    float k2=-(pt3.x-pt2.x)/(pt3.y-pt2.y);

    CircleData CD;

    CD.center.x=(midpt2.y-midpt1.y-k2*midpt2.x+k1*midpt1.x)/(k1-k2);
    CD.center.y=midpt1.y+k1*(midpt2.y-midpt1.y-k2*midpt2.x+k2*midpt1.x)/(k1-k2);
    CD.radius=sqrtf((CD.center.x-pt1.x)*(CD.center.x-pt1.x)+(CD.center.y-pt1.y)*(CD.center.y-pt1.y));
    return CD;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;
    
    //ROS_INFO("carPose_yaw = %2f",carPose_yaw);
    int i_start,i_mid,i_end;
    if(!goal_reached){
        for(int i =0; i< map_path.poses.size(); i++)
        {
            i_start=i+100;
            i_mid=i_start+50;
            i_end=i_mid+50;
            if(i_start>map_path.poses.size())
                i_start=map_path.poses.size();
            if(i_mid>map_path.poses.size())
                i_mid=map_path.poses.size();
            if(i_end>map_path.poses.size())
                i_end=map_path.poses.size();
            geometry_msgs::Point map_path_pose1 = map_path.poses[i_start].pose.position;
            //geometry_msgs::PoseStamped odom_path_pose1;
            geometry_msgs::Point map_path_pose2 = map_path.poses[i_mid].pose.position;
            //geometry_msgs::PoseStamped odom_path_pose2;
            geometry_msgs::Point map_path_pose3 = map_path.poses[i_end].pose.position;
            //geometry_msgs::PoseStamped odom_path_pose3;



            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;
          

            CD1=findCircle(map_path_pose1,map_path_pose2,map_path_pose3);
            //ROS_INFO("radius:%f",CD1.radius);
            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);
                 //ROS_INFO("odom_path_pose.pose.x:%f size:%d i:%f",odom_path_pose.pose.position.x,map_path.poses.size(),carPose_pos.x);//规划路径点的分辨率为2cm左右
                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                    
                }
                
                
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            
        }
        
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
       // ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}


double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    return eta;
}


double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.6)
        L1 = 3 / 3.0;
    else if(_Vcmd > 1.1 && _Vcmd < 5.36)
        L1 = _Vcmd*1.7/ 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steeringAnge = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
   // ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v)*Gas_gain;
   // ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}

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
    pn.param("controller_freq", controller_freq, 30);
    pn.param("AngleGain", Angle_gain, -6.0);
    pn.param("GasGain", Gas_gain, 2.5);
    pn.param("baseSpeed", baseSpeed, 1590);
    pn.param("baseAngle", baseAngle, 90.0);
    pn.param("slow_radius",slow_radius,10.0);//减速半径
    pn.param("slow_time",slow_time,50);//减速时间
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
    Lfw = getL1Distance(Vcmd);
    goalRadius=0.5;
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
    ROS_INFO("[param] GasGain: %f",Gas_gain);
    ROS_INFO("[param] slow_radius: %f", slow_radius);
    ROS_INFO("[param] slow_time: %d",slow_time);
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
            ROS_INFO("Goal Reached !");
            car_stop = 100;
        }
    }
}




void L1Controller::controlLoopCB(const ros::TimerEvent&)
{
    int i_count;
    int count = 100;
    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = baseAngle;
 //   Lfw = goalRadius = getL1Distance(carVel.linear.x);
    //ROS_INFO("%f",start_speed);
    //ROS_INFO("carVel.linear.x = %.2f  cmd_pwm = %f",carVel.linear.x,cmd_vel.linear.x);
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
               // if(start_loop_flag++ <= 250)
               if(CD1.radius>slow_radius)
               {
                i_count++;
                if(i_count>slow_time)
                {
		        if(carVel.linear.x<1.5)
                  {
                  // Lfw = goalRadius = getL1Distance(1);

                    double u = getGasInput(carVel.linear.x);
                    
                    //cmd_vel.linear.x = 1500;
		             cmd_vel.linear.x=(int)start_speed;


                     start_speed += 1.8;
                     if(cmd_vel.linear.x > baseSpeed)   cmd_vel.linear.x = baseSpeed;
                     
		             ROS_INFO("baseSpeed = %.2f\tSteering angle = %.2f\tcarVel.linear.x=%.2f",cmd_vel.linear.x,cmd_vel.angular.z,carVel.linear.x);
                     }
		        //else if(carVel.linear.x>=1.5&&carVel.linear.x<1.8)
		        //{
			    //     double u = getGasInput(carVel.linear.x);
			          //Lfw = goalRadius = getL1Distance(2.0);
                   // cmd_vel.linear.x = start_speed + PIDCal(&pid_speed,u);
                //   cmd_vel.linear.x=(int)start_speed;


                //     start_speed += 0.5;
                //     if(cmd_vel.linear.x > baseSpeed)   cmd_vel.linear.x = baseSpeed;

                 //    ROS_INFO("baseSpeed = %.2f\tcarVel.linear.x=%.2f",cmd_vel.linear.x,carVel.linear.x);

		         //}
                 else 
                 {
           	        //Lfw = goalRadius = getL1Distance(3.0);
		             //start_speed=1580;
                    //ROS_INFO("!goal_reached");
                    double u = getGasInput(carVel.linear.x);                   
                    cmd_vel.linear.x = baseSpeed + PIDCal(&pid_speed,u);
                    //cmd_vel.linear.x = 1500;
                    ROS_INFO("Gas = %.2f\tangle = %.2f\tcarVel.linear.x=%.2f",cmd_vel.linear.x,cmd_vel.angular.z,carVel.linear.x);
                }
                }
                else
                {
                    cmd_vel.linear.x =1578;
                    ROS_INFO("SlowSpeed = %.2f\tangle = %.2f\tcarVel.linear.x=%.2f",cmd_vel.linear.x,cmd_vel.angular.z,carVel.linear.x);
                }
                
            }           
            else
            {
                i_count=0;
                cmd_vel.linear.x =1578;
                ROS_INFO("SlowSpeed = %.2f\tangle = %.2f\tcarVel.linear.x=%.2f",cmd_vel.linear.x,cmd_vel.angular.z,carVel.linear.x);
            }
            
            }
        }
    }
    if(car_stop > 0)
    {
        start_loop_flag = 0;
        if(carVel.linear.x > 0)
        {

            cmd_vel.linear.x = 1440; //反向刹车
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
            cmd_vel.linear.x = 1490;
            pub_.publish(cmd_vel);

            //ROS_INFO("cmd_vel= %f",cmd_vel.linear.x);
        }
    }
    else
    {
        pub_.publish(cmd_vel);
        car_stop = 0;
       // ROS_INFO("car run cmd_vel= %f",cmd_vel.linear.x);
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
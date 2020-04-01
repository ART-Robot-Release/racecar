#include <iostream>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "uart_driver.h"
using namespace std;

int startOrStop = 1;	// 1是start，0是stop
string laser_link = "laser_link";

void publish_scan(ros::Publisher *pub, double *dist, int count, ros::Time start, double scan_time)
{
	static int scan_count = 0;
	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.stamp = start;
	scan_msg.header.frame_id = laser_link;
	scan_count++;

	scan_msg.angle_min = 0;
	scan_msg.angle_max = 2 * M_PI;
	scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(count - 1);
	scan_msg.scan_time = scan_time;
	scan_msg.time_increment = scan_time / (double)(count - 1);
	
	scan_msg.range_min = 0.15;
	scan_msg.range_max = 8.0;

	scan_msg.intensities.resize(count);
	scan_msg.ranges.resize(count);


	for (int i = count - 1; i >= 0; i--)
	{
		if (dist[count - i - 1] == 0.0)
			scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
		else
			scan_msg.ranges[i] = dist[count - i - 1] / 1000.0;
		scan_msg.intensities[i] = 0;
	}
	pub->publish(scan_msg);
}

void startStopCB(const std_msgs::Int32ConstPtr msg)
{
	startOrStop = msg->data;
}

int main(int argv, char **argc)
{
	ros::init(argv, argc, "laser_node");
	ros::NodeHandle n;
	
	string scan_topic = "scan";
	ros::param::get("~scan_topic", scan_topic);
	ros::param::get("~laser_link", laser_link);
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1000);
	ros::Subscriber stop_sub = n.subscribe<std_msgs::Int32>("startOrStop", 10, startStopCB);

	string port = "/dev/ttyUSB0";
	ros::param::get("~serial_port",port);

	io_driver driver;
  	int ret = driver.OpenSerial(port.c_str(),B230400);
	if(ret < 0)
	{
		ROS_ERROR("could not open port:%s",port.c_str());
		return 0;
	}
	
   driver.StartScan();
	bool isStarted = true;
	
	double angle[PACKLEN + 10];
	double distance[PACKLEN + 10];
	double data[PACKLEN + 10];
	double speed;
	int count = 0;
		
	ros::Time starts = ros::Time::now();
	ros::Time ends = ros::Time::now();
	ROS_INFO("talker....");
	while(ros::ok())
	{
		ros::spinOnce();

		if(isStarted && 0 == startOrStop) // 当前正在扫描且要求停止
		{
			ROS_INFO("stop");
			driver.StopScan();
			isStarted = false;
		}
		else if(!isStarted && 1 == startOrStop)	// 当前未扫描且要求开始扫描
		{
			ROS_INFO("start");
			driver.StartScan();
			isStarted = true;
		}

		ROS_INFO("%s", isStarted ? "Started":"Stopped");
		if(!isStarted)
			continue;


		memset(data, 0, sizeof(data));
		int ret = driver.GetScanData(angle, distance, PACKLEN, &speed);
		for (int i = 0; i < ret; i++)
		{
			data[i] = distance[i];
		}
		ends = ros::Time::now();
		float scan_duration = (ends - starts).toSec() * 1e-3;
		publish_scan(&scan_pub, data, ret, starts, scan_duration);
		starts = ends;
	}

	driver.StopScan();
	driver.CloseSerial();
	
	return 0;
}



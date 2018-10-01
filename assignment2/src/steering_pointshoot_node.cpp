#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

std::string name = "steering_pointshoot";
int main(int argc, char **argv)
{
	ros::init(argc, argv, name);
	ros::NodeHandle n;
	
	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}

#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

ros::Publisher velocity;
ros::Subscriber odom;
ros::Subscriber goal;

nav_msgs::Odometry current;

void OdomCallback(const nav_msgs::OdometryConstPtr& message)
{
	current = *message;
}

void GoalCallback(const geometry_msgs::PointConstPtr& message)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "steering_servoing");
	ros::NodeHandle n;
	
	velocity = n.advertise<geometry_msgs::Twist>("/stagesim/cmd_vel", 10);
	odom = n.subscribe("/stagesim/odom", 10, OdomCallback);
	goal = n.subscribe("/stagesim/goal", 10, GoalCallback);

	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}


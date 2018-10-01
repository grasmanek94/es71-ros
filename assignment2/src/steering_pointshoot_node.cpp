#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stageros/Odom.h>

ros::Publisher velocity_Publisher;
ros::Subscriber odom_Subscriber;

std::string name = "steering_pointshoot";

stageros::odom stageros_odom;

void OdomCallBack(const stage_ros::odom::ConstPtr& message ){

stageros_odom.pose = message->pose;
stageros_odom.twist = message->twist;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, name);
	ros::NodeHandle n;
	
	velocity_Publisher = n.advertise<geometry_msgs::Twist>("stage_ros/cmd_vel", 10);
	odom_Subscriber = n.subscribe<"stage_ros/odom", 10, OdomCallback);
	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}

#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

/*
ros::Publisher velocity_publisher;
ros::Subscriber subscriber_draw_triangle;
ros::Subscriber subscriber_move_rotate;
ros::Subscriber pose_subscriber;

*/

//std::string name = "draw_triangle";
int main(int argc, char **argv)
{
	ros::init(argc, argv, name);
	ros::NodeHandle n;
	/*
	velocity_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("turtle1/pose", 10, PoseCallback);
	subscriber_draw_triangle = n.subscribe(name + "/cmd", 10, DrawTriangleCallback);
	subscriber_move_rotate = n.subscribe(name + "/move_rotate", 10, MoveRotateCallback);
	*/
	
	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}
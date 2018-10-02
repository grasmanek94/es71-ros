#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

ros::Publisher velocity_Publisher;
ros::Subscriber odom_Subscriber;
ros::Subscriber goal_Subscriber;

std::string name = "steering_pointshoot";

stageros::odom stageros_odom;

void OdomCallBack(const stage_ros::odom::ConstPtr& message ){

stageros_odom.pose = message->pose;
stageros_odom.twist = message->twist;

}
geometry_msgs::Point goalPoint;
void GoalCallBack(const assignment2::goal::ConstPtr &message)
{
	//goal message type = Point(x,y,z), we use z for angle
	goalPoint.x = message.x;
	goalPoint.y = message.y;
	goalPoint.z = message.angle;

}

string::name = "steering_pointshoot_node";
int main(int argc, char **argv)
{
	ros::init(argc, argv, name);
	ros::NodeHandle n;
	
	velocity_Publisher = n.advertise<geometry_msgs::Twist>("stagesim/stage_ros/cmd_vel", 10);
	odom_Subscriber = n.subscribe<"stage_ros/odom">, 10, OdomCallback);
	goal_Subscriber = n.subscribe <name + "/goal">, 10 , GoalCallback) ;
	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}

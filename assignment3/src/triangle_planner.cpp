#ifdef _MSC_VER
#define __attribute__(a) 
#endif

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <angles/angles.h>
#include <assignment3/Goal.h>
#include <nav_msgs/Path.h>
#include <cmath>

ros::Subscriber odom;
ros::Subscriber goal;
ros::Publisher plan;

tf::Quaternion current_quat;
tf::Vector3 current_pos;

void OdomCallback(const nav_msgs::OdometryConstPtr& message)
{
	tf::quaternionMsgToTF(message->pose.pose.orientation, current_quat);

	current_pos.setX(message->pose.pose.position.x);
	current_pos.setY(message->pose.pose.position.y);
}

void CreatePlan(double size)
{
	geometry_msgs::PoseStamped plannedPath[3];

	plannedPath[0].pose.position.x = size + current_pos.getX();
	plannedPath[0].pose.position.y = current_pos.getY();
	plannedPath[0].pose.orientation = tf::createQuaternionMsgFromYaw(angles::from_degrees(120));

	plannedPath[1].pose.position.x = (size / 2) + current_pos.getX();
	plannedPath[1].pose.position.y = size * sin(angles::from_degrees(60)) + current_pos.getY();
	plannedPath[1].pose.orientation = tf::createQuaternionMsgFromYaw(angles::from_degrees(240));

	plannedPath[2].pose.position.x = current_pos.getX();
	plannedPath[2].pose.position.y = current_pos.getY();
	plannedPath[2].pose.orientation = tf::createQuaternionMsgFromYaw(angles::from_degrees(0));

	nav_msgs::Path targetpath;

	for (int i = 0; i < 3; i++)
	{
		targetpath.poses.push_back(plannedPath[i]);
	}

	plan.publish(targetpath);
}

void GoalCallBack(const assignment3::Goal::ConstPtr& message)
{
	CreatePlan(message->size);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "triangle_planner_node");
	ros::NodeHandle n;

	odom = n.subscribe("/stagesim/odom", 10, OdomCallback);
	goal = n.subscribe("assignment3/goal", 10, GoalCallBack);
	plan = n.advertise<nav_msgs::Path>("assignment3/plan", 10);

	ros::spin();

	return 0;
}

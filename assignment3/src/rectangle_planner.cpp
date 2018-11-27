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
	nav_msgs::Path targetpath;

	geometry_msgs::PoseStamped plannedPath;
	plannedPath.pose.position.x = current_pos.getX();
	plannedPath.pose.position.y = current_pos.getY();

	for (int i = 0; i < 4; i++)
	{
		plannedPath.pose.position.x += size * std::cos(angles::normalize_angle(angles::from_degrees(90.0*i)));
		plannedPath.pose.position.y += size * std::sin(angles::normalize_angle(angles::from_degrees(90.0*i)));
		plannedPath.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(angles::from_degrees(90.0*(i + 1))));

		targetpath.poses.push_back(plannedPath);
	}

	plan.publish(targetpath);
}

void GoalCallBack(const assignment3::Goal::ConstPtr& message)
{
	CreatePlan(message->size);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rectangle_planner_node");
	ros::NodeHandle n;

	odom = n.subscribe("/stagesim/odom", 10, OdomCallback);
	goal = n.subscribe("assignment3/goal", 10, GoalCallBack);
	plan = n.advertise<nav_msgs::Path>("assignment3/plan", 10);

	ros::spin();

	return 0;
}

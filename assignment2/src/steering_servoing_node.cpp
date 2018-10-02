#ifdef _MSC_VER
#define __attribute__(a) 
#endif

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

#include <iostream>
#include <string>

ros::Publisher velocity;
ros::Subscriber odom;
ros::Subscriber goal;

tf::Quaternion initial_quat;
tf::Vector3 initial_pos;
tfScalar initial_distance;

tf::Quaternion current_quat;
tf::Vector3 current_pos;

tf::Quaternion target_quat;
tf::Vector3 target_pos;

void PerformMoveLoop();
bool PerformMoveTick();

void OdomCallback(const nav_msgs::OdometryConstPtr& message)
{
	tf::quaternionMsgToTF(message->pose.pose.orientation, current_quat);

	current_pos.setX(message->pose.pose.position.x);
	current_pos.setY(message->pose.pose.position.y);
}

void GoalCallback(const geometry_msgs::PointConstPtr& message)
{
	geometry_msgs::Point target_point = *message;

	tf::Quaternion quat;
	quat.setRPY(0.0, 0.0, target_point.z);
	quat.normalize();

	target_quat = quat;

	target_pos.setX(target_point.x);
	target_pos.setY(target_point.y);

	PerformMoveLoop();
}

void PerformMoveLoop()
{
	ros::Rate loop_rate(1000);
	bool keep_moving = true;

	initial_quat = current_quat;
	initial_pos = current_pos;
	initial_distance = initial_pos.distance(target_pos);

	do
	{
		keep_moving = PerformMoveTick();
		loop_rate.sleep();
		ros::spinOnce();
	} while (keep_moving);
}

bool PerformMoveTick()
{
	tf::Quaternion change_quat = current_quat.slerp(target_quat, 0.1);
	tfScalar distance = current_pos.distance(target_pos);

	if (distance < 0.1)
	{
		return false;
	}

	tfScalar speed = std::max(10.0 * (distance / initial_distance), 1.0);

	tfScalar yaw, pitch, roll;
	tf::Matrix3x3 mat;
	mat.setRotation(change_quat);
	mat.getEulerYPR(yaw, pitch, roll);

	std::cout << yaw << std::endl;

	geometry_msgs::Twist twist;
	twist.linear.x = speed;
	twist.angular.x = yaw * 4.0;

	velocity.publish(twist);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "steering_servoing");
	ros::NodeHandle n;
	
	velocity = n.advertise<geometry_msgs::Twist>("/stagesim/cmd_vel", 10);
	odom = n.subscribe("/stagesim/odom", 10, OdomCallback);
	goal = n.subscribe("steering_servoing/goal", 10, GoalCallback);

	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}

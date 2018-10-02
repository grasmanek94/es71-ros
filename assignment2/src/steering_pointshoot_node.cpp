#ifdef _MSC_VER
#define __attribute__(a) 
#endif

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

ros::Publisher velocity;
ros::Subscriber odom;
ros::Subscriber goal;

nav_msgs::Odometry current;
void MoveRotate(float x, float y, float angle);

void OdomCallback(const nav_msgs::OdometryConstPtr& message)
{
	current = *message;
}

void GoalCallback(const geometry_msgs::PointConstPtr& message)
{
	std::cout << "received message: " << message->x << " " << message->y << " " << message->z << std::endl;
	float radials = angles::from_degrees(message->z);
	MoveRotate(message->x, message->y, radials);

}

double ClampAngle(double radians)
{
	while (radians > M_PI)
	{
		radians -= 2.0 * M_PI;
	}
	while (radians < -M_PI)
	{
		radians += 2.0 * M_PI;
	}
	return radians;
}

void MoveRotate(float x, float y, float angle)
{
	geometry_msgs::Twist twist;
	float turning_angle = atan2(y - current.pose.pose.position.y, x - current.pose.pose.position.x);

	bool loop;
	do
	{
		tf::Quaternion Qangle;
		tf::quaternionMsgToTF(current.pose.pose.orientation, Qangle);
		double angle_jaw = tf::getYaw(Qangle);
		double delta_angle = turning_angle - ClampAngle(angle_jaw);
		loop = std::abs(delta_angle) > 0.005;
		twist.angular.z = delta_angle;

		velocity.publish(twist);
		ros::spinOnce();

	} while (loop);

	std::cout << "turning done" << std::endl;
	twist.angular.z = 0.0;
	twist.linear.x = 10;
	velocity.publish(twist);

	double tmp = sqrt(pow(x - current.pose.pose.position.x, 2.0) + pow(y - current.pose.pose.position.y, 2.0)) + 1.0;
	
	bool loop2 = true;
	do
	{
		double delta_distance = sqrt(pow(x - current.pose.pose.position.x, 2.0) + pow(y - current.pose.pose.position.y, 2.0));
		if (delta_distance > tmp)
		{
			loop2 = false;
		}
		tmp = delta_distance;
		twist.linear.x = delta_distance;
		velocity.publish(twist);
		ros::spinOnce();
	} while (loop2);

	twist.linear.x = 0.0;
	velocity.publish(twist);

	/*
	stage is now in correct position, needs to turn towards angle provided as argument by this method.
	delta_angle = current angle - target angle(param)
	*/

	std::cout << "start final turn" << std::endl;
	double target_angle = angle;

	bool loop3 = true;
	do
	{
		double speed = 10;
		tf::Quaternion Qangle;
		tf::quaternionMsgToTF(current.pose.pose.orientation, Qangle);
		double angle_jaw = tf::getYaw(Qangle);
		double delta_angle = ClampAngle(target_angle) - ClampAngle(angle_jaw);
		loop = std::abs(delta_angle) > 0.01;
		twist.angular.z = delta_angle;

		velocity.publish(twist);
		ros::spinOnce();

	} while (loop3);
	std::cout << "all finished" << std::endl;
	twist.angular.z = 0.0;
	twist.linear.x = 0;
	velocity.publish(twist);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "steering_pointshoot");
	ros::NodeHandle n;
	velocity = n.advertise<geometry_msgs::Twist>("/stagesim/cmd_vel", 10);
	odom = n.subscribe("/stagesim/odom", 10, OdomCallback);
	goal = n.subscribe("steering_pointshoot/goal", 10, GoalCallback);
	std::cout << "main: Prepared callbacks, spinning..." << std::endl;
	ros::spin();

	return 0;
}
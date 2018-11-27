#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include "assignment5/Triangle.h"
#include "assignment5/MoveRotate.h"

ros::Publisher velocity_publisher;
ros::Subscriber subscriber_draw_triangle;
ros::Subscriber subscriber_move_rotate;
ros::Subscriber pose_subscriber;

turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;

double Sign(double a)
{
	if (a == 0.0)
	{
		return 0.0;
	}

	return (a > 0.0 ? 1.0 : -1.0);
}

double Deg2Rad(double angle_in_degrees)
{
	return angle_in_degrees * PI / 180.0;
}

double ClampAngle(double radians)
{
	while (radians > PI)
	{
		radians -= 2.0 * PI;
	}
	while (radians < -PI)
	{
		radians += 2.0 * PI;
	}
	return radians;
}

double Direction(const turtlesim::Pose& target, const turtlesim::Pose& current)
{
	return std::cos(target.theta - std::atan2(target.y - current.y, target.x - current.x));
}

double AbsDistance(const turtlesim::Pose& target, const turtlesim::Pose& current)
{
	return std::sqrt(pow(target.x - current.x, 2.0) + pow(target.y - current.y, 2.0));
}

double Distance(const turtlesim::Pose& target, const turtlesim::Pose& current)
{
	return Sign(Direction(target, current)) * AbsDistance(target, current);
}

turtlesim::Pose GetTargetPosition(const turtlesim::Pose& start, double distance)
{
	turtlesim::Pose temp = start;
	temp.x += (distance * sin(-temp.theta + PI / 2));
	temp.y += (distance * cos(-temp.theta + PI / 2));
	return temp;
}

void FixBounds(const turtlesim::Pose& current, turtlesim::Pose& target, double& move_distance)
{
	const double max_bounds = 10.50; // beetje speling laten t.o.v. 11.0
	const double min_bounds = 0.50; // beetje speling laten t.o.v. 0.0

	bool updated = false;

	if (target.x > max_bounds)
	{
		target.x = max_bounds;
		updated = true;
	}
	else if (target.x < min_bounds)
	{
		target.x = min_bounds;
		updated = true;
	}

	if (target.y > max_bounds)
	{
		target.y = max_bounds;
		updated = true;
	}
	else if (target.y < min_bounds)
	{
		target.y = min_bounds;
		updated = true;
	}

	if (updated)
	{
		double sign = Sign(move_distance);
		move_distance = AbsDistance(target, current);
	}
}

void MoveRotate(double speed, double move_distance, double rotate_degrees)
{
	geometry_msgs::Twist twist;
	bool loop;

	ros::Rate loop_rate(1000);

	if (rotate_degrees != 0.0)
	{
		ros::spinOnce();
		turtlesim::Pose saved_pose = turtlesim_pose;

		double rotate_radians = ClampAngle(Deg2Rad(rotate_degrees));
		double target_radians = ClampAngle(saved_pose.theta + rotate_radians);

		do
		{
			double delta_angle = target_radians - ClampAngle(turtlesim_pose.theta);

			twist.angular.z = std::max(speed * std::abs(delta_angle / rotate_radians), 0.05) * Sign(delta_angle);

			loop = std::abs(delta_angle) > 0.001;

			velocity_publisher.publish(twist);
			ros::spinOnce();
			loop_rate.sleep();
		} while (loop);

		twist.angular.z = 0.0;
		velocity_publisher.publish(twist);
	}

	if (move_distance != 0.0)
	{
		ros::spinOnce();
		turtlesim::Pose target_pose = GetTargetPosition(turtlesim_pose, move_distance);
		FixBounds(turtlesim_pose, target_pose, move_distance);

		do
		{
			double delta_distance = Distance(target_pose, turtlesim_pose);
			twist.linear.x = std::max(speed * std::abs(delta_distance / move_distance), 0.1) * Sign(delta_distance);

			loop = std::abs(delta_distance) > 0.005;

			velocity_publisher.publish(twist);
			ros::spinOnce();
			loop_rate.sleep();
		} while (loop);

		twist.linear.x = 0.0;
		velocity_publisher.publish(twist);
	}
}

void SetAngle(double radians)
{
	double delta = radians - turtlesim_pose.theta;
	MoveRotate(1.0, 0.0, delta);
}

void PoseCallback(const turtlesim::Pose::ConstPtr& message)
{
	turtlesim_pose.x = message->x;
	turtlesim_pose.y = message->y;
	turtlesim_pose.theta = message->theta;
}

double GetMaxTriangleSide(double move_distance, bool cw)
{
	double sign = Sign(move_distance);

	turtlesim::Pose current = turtlesim_pose;
	turtlesim::Pose target;

	for (size_t i = 0; i < 3; ++i)
	{
		turtlesim::Pose target = GetTargetPosition(current, move_distance);
		FixBounds(current, target, move_distance);
		move_distance = AbsDistance(target, current) * sign;

		target.theta = ClampAngle(target.theta + Deg2Rad((cw ? -1.0 : 1.0) * 120.0));
		current = target;
	}

	return move_distance;
}

void DrawTriangle(float side_length, bool cw)
{
	SetAngle(0.0);
	double fixed_length = GetMaxTriangleSide(side_length, cw);

	for (size_t i = 0; i < 3; ++i)
	{
		MoveRotate(5.0, fixed_length, 0.0);
		MoveRotate(5.0, 0.0, (cw ? -1.0 : 1.0) * 120.0);
	}
}

void DrawTriangleCallback(const assignment5::Triangle::ConstPtr& message)
{
	DrawTriangle(message->sideLength, message->clockwise);
}

void MoveRotateCallback(const assignment5::MoveRotate::ConstPtr& message)
{
	MoveRotate(message->speed, message->distance, message->angle);
}

std::string name = "assignment5_node";
int main(int argc, char **argv)
{
	ros::init(argc, argv, name);
	ros::NodeHandle n;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("turtle1/pose", 10, PoseCallback);
	subscriber_draw_triangle = n.subscribe(name + "/cmd", 10, DrawTriangleCallback);
	subscriber_move_rotate = n.subscribe(name + "/move_rotate", 10, MoveRotateCallback);

	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}
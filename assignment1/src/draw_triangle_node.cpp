#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <assignment1/Triangle.h>
#include <assignment1/MoveRotate.h>

ros::Publisher velocity_publisher;
ros::Publisher publisher_draw_triangle;
ros::Publisher publisher_move_rotate;
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

double Distance(const turtlesim::Pose& target, const turtlesim::Pose& current)
{
	return Sign(Direction(target, current)) * std::sqrt(pow(target.x - current.x, 2.0) + pow(target.y - current.y, 2.0));
}

turtlesim::Pose GetTargetPosition(const turtlesim::Pose& start, double distance)
{
	turtlesim::Pose temp = start;
	temp.x += (distance * sin(-temp.theta + PI / 2));
	temp.y += (distance * cos(-temp.theta + PI / 2));
	return temp;
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

			twist.angular.z = speed * Sign(delta_angle) * std::abs(delta_angle / rotate_radians);

			loop = std::abs(delta_angle) > 0.01;

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

		do
		{
			double delta_distance = Distance(target_pose, turtlesim_pose);
			twist.linear.x = speed * Sign(delta_distance) * std::abs(delta_distance / move_distance);

			loop = std::abs(delta_distance) > 0.05;

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

void DrawTriangle(float side_length, bool cw)
{
	SetAngle(0.0);
	for (size_t i = 0; i < 3; ++i)
	{
		MoveRotate(10.0, side_length, 0.0);
		MoveRotate(10.0, 0.0, (cw ? -1.0 : 1.0) * 120.0);
	}
}

void DrawTriangleCallback(const assignment1::Triangle::ConstPtr& message)
{
	DrawTriangle(message->sideLength, message->clockwise);
}

void MoveRotateCallback(const assignment1::MoveRotate::ConstPtr& message)
{
	MoveRotate(message->speed, message->distance, message->angle);
}

std::string name = "draw_triangle";
int main(int argc, char **argv)
{
	ros::init(argc, argv, name);
	ros::NodeHandle n;

	publisher_draw_triangle = n.advertise<assignment1::Triangle>(name + "/cmd", 10);
	publisher_move_rotate = n.advertise<assignment1::MoveRotate>(name + "/move_rotate", 10);
	velocity_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, PoseCallback);
	subscriber_draw_triangle = n.subscribe(name + "/cmd", 10, DrawTriangleCallback);
	subscriber_move_rotate = n.subscribe(name + "/move_rotate", 10, MoveRotateCallback);

	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}
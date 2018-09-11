#include <iostream>

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

void PoseCallback(const turtlesim::Pose::ConstPtr& message);
void DrawTriangleCallback(const assignment1::Triangle::ConstPtr& message);
void MoveRotateCallback(const assignment1::MoveRotate::ConstPtr& message);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd");
	ros::NodeHandle n;

	publisher_draw_triangle = n.advertise<assignment1::Triangle>("cmd/draw_triangle", 10);
	publisher_move_rotate = n.advertise<assignment1::MoveRotate>("cmd/move_rotate", 10);
	velocity_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, PoseCallback);
	subscriber_draw_triangle = n.subscribe("cmd/draw_triangle", 10, DrawTriangleCallback);
	subscriber_move_rotate = n.subscribe("cmd/move_rotate", 10, MoveRotateCallback);

	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}

double Sign(double a)
{
	return a > 0.0 ? 1.0 : -1.0;
}

double DeltaE(double a, double b)
{
	return (a - b) * 100.0;
}

double Distance(const turtlesim::Pose& a, const turtlesim::Pose& b)
{
	return sqrt(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0));
}

void MoveRotate(double speed, double move_distance, double rotate_radians)
{
	std::cout << "Received MoveRotate(" << speed << ", " << move_distance << ", " << rotate_radians << ")" << std::endl;

	ros::Rate loop_rate(10);
	geometry_msgs::Twist twist;

	turtlesim::Pose saved_pose = turtlesim_pose;
	bool loop = true;

	do
	{
		twist.linear.x = speed * DeltaE(Distance(saved_pose, turtlesim_pose), abs(move_distance)) * Sign(move_distance);
		twist.angular.z = speed * DeltaE(saved_pose.theta + rotate_radians, turtlesim_pose.theta);

		loop = 
			abs(twist.linear.x) > 0.1 || 
			abs(twist.angular.z) > 0.1;
		if (!loop)
		{
			std::cout << "Aborting loop in MoveRotate(" << speed << ", " << move_distance << ", " << rotate_radians << ")" << std::endl;
		}
		else
		{
			std::cout << "(" << saved_pose.x << ", " << saved_pose.y << ") -> (" << turtlesim_pose.x << ", " << turtlesim_pose.y << "): " << (DeltaE(Distance(saved_pose, turtlesim_pose), abs(move_distance)) * Sign(move_distance)) << std::endl;
		}

		velocity_publisher.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();
	} while (loop);

	velocity_publisher.publish(geometry_msgs::Twist());
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

double degrees2radians(double angle_in_degrees) 
{
	const double PI = 3.14159265359;
	return angle_in_degrees * PI / 180.0;
}

void DrawTriangle(float side_length, bool cw)
{
	SetAngle(0.0);
	for (size_t i = 0; i < 3; ++i)
	{
		MoveRotate(1.0, side_length, 0.0);
		MoveRotate(1.0, 0.0, degrees2radians(60.0));
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

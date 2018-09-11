#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <assignment1/Triangle.h>

ros::Publisher velocity_publisher;
ros::Publisher own_publisher;
ros::Subscriber own_subscriber;
ros::Subscriber pose_subscriber;

turtlesim::Pose turtlesim_pose;

void PoseCallback(const turtlesim::Pose::ConstPtr& message);
void DrawTriangleCallback(const assignment1::Triangle::ConstPtr& message);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd");
	ros::NodeHandle n;

	own_publisher = n.advertise<assignment1::Triangle>("cmd/draw_triangle", 10);
	velocity_publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, PoseCallback);
	own_subscriber = n.subscribe("cmd/draw_triangle", 10, DrawTriangleCallback);

	ros::spin();

	return 0;
}

void AbstractKeepSendingMessage(const geometry_msgs::Twist& msg, double target_value, double target_speed)
{
	double t0 = ros::Time::now().toSec();
	double current_value = 0.0;
	ros::Rate loop_rate(1000);

	while (current_value<target_value)
	{
		velocity_publisher.publish(msg);
		double t1 = ros::Time::now().toSec();
		current_value = target_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	}

	velocity_publisher.publish(geometry_msgs::Twist());
}

void Move(double speed, double distance)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = speed;

	AbstractKeepSendingMessage(vel_msg, distance, speed);
}

void Rotate(double angular_speed, double relative_angle) {

	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = angular_speed;

	AbstractKeepSendingMessage(vel_msg, relative_angle, angular_speed);
}

double SetAngle(double radians)
{
	double delta = radians - turtlesim_pose.theta;
	Rotate(delta, delta);
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
		Move(10.0, side_length);
		Rotate(10.0, degrees2radians(60.0));
	}
}

void DrawTriangleCallback(const assignment1::Triangle::ConstPtr& message)
{
	DrawTriangle(message->sideLength, message->clockwise);
}

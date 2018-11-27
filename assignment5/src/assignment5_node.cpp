#include <iostream>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#include <actionlib/client/simple_action_client.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>

#include "assignment5/Triangle.h"

typedef actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> Client;
Client client("turtlebot_move", true); // true -> don't need ros::spin()

ros::Subscriber subscriber_draw_triangle;
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

void SimpleDoneCallback(const actionlib::SimpleClientGoalState& state, const turtlebot_actions::TurtlebotMoveResult::ConstPtr& result)
{

}

void SimpleActiveCallback()
{

}

void SimpleFeedbackCallback(const turtlebot_actions::TurtlebotMoveFeedback::ConstPtr& feedback)
{

}

void MoveRotate(double speed, double move_distance, double rotate_degrees)
{
	// turn first
	turtlebot_actions::TurtlebotMoveGoal goal;

	goal.forward_distance = 0.0f;
	goal.turn_distance = rotate_degrees;
	client.sendGoal(goal, &SimpleDoneCallback, &SimpleActiveCallback, &SimpleFeedbackCallback);
	client.waitForResult();

	// then go forward
	goal.forward_distance = move_distance;
	goal.turn_distance = 0.0f;
	client.sendGoal(goal, &SimpleDoneCallback, &SimpleActiveCallback, &SimpleFeedbackCallback);
	client.waitForResult();
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

std::string name = "assignment5_node";
int main(int argc, char **argv)
{
	ros::init(argc, argv, name);
	ros::NodeHandle n;

	pose_subscriber = n.subscribe("turtle1/pose", 10, PoseCallback);
	subscriber_draw_triangle = n.subscribe(name + "/cmd", 10, DrawTriangleCallback);

	client.waitForServer();

	std::cout << "main: Prepared callbacks, spinning..." << std::endl;

	ros::spin();

	return 0;
}
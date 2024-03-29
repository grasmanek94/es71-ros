#ifdef _MSC_VER
#define __attribute__(a) 
#endif

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <angles/angles.h>

ros::Publisher velocity;
ros::Subscriber odom;
ros::Subscriber goal;

tf::Quaternion current_quat;
tf::Vector3 current_pos;

tf::Quaternion target_quat;
tf::Vector3 target_pos;

void PerformMoveLoop();
bool PerformMoveTick(bool& move, bool& rotate);

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
	quat.setRPY(0.0, 0.0, angles::normalize_angle(angles::from_degrees(target_point.z)));
	quat.normalize();

	target_quat = quat;

	target_pos.setX(target_point.x);
	target_pos.setY(target_point.y);

	PerformMoveLoop();
}

void PerformMoveLoop()
{
	ros::Rate loop_rate(25);

	bool keep_moving = true;

	bool move = true;
	bool rotate = true;

	do
	{
		keep_moving = PerformMoveTick(move, rotate);
		loop_rate.sleep();
		ros::spinOnce();
	} while (keep_moving);

	velocity.publish(geometry_msgs::Twist());
}

bool PerformMoveTick(bool& move, bool& rotate)
{
	geometry_msgs::Twist twist;

	tf::Vector3 xhat = target_pos - current_pos;
	tfScalar p = xhat.length();
	tfScalar kp = 1.0;
	tfScalar v = kp * p;

	if (p < 0.001)
	{
		move = false;
	}

	twist.linear.x = v;

	tfScalar O = tf::getYaw(current_quat);

	tfScalar r = tf::getYaw(target_quat);
	
	tfScalar ka = 2.0;
	tfScalar kb = -1.0;
	tfScalar a = angles::normalize_angle(-O + atan2(xhat.y(), xhat.x()));
	tfScalar b = angles::normalize_angle(-O - a - r);
	tfScalar w = ka * a + kb * b;

	if (abs(w) < 0.0002)
	{
		rotate = false;
	}

	twist.angular.z = w;

	velocity.publish(twist);

	return move || rotate;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "steering_servoing");
	ros::NodeHandle n;
	
	velocity = n.advertise<geometry_msgs::Twist>("/stagesim/cmd_vel", 10);
	odom = n.subscribe("/stagesim/odom", 10, OdomCallback);
	goal = n.subscribe("steering_servoing/goal", 10, GoalCallback);

	ros::spin();

	return 0;
}

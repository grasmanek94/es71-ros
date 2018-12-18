#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <angles/angles.h>

ros::Publisher velocity;
ros::Subscriber odom_sub;
ros::Subscriber tf_sub;
ros::Subscriber plan_sub;

tf::Quaternion current_quat;
tf::Vector3 current_pos;
tf::TransformListener* listener;

void UpdatePosition(const tf::StampedTransform& transform)
{
	current_pos.setX(transform.getOrigin().x());
	current_pos.setY(transform.getOrigin().y());

	current_quat = transform.getRotation();
}

bool GetUpdatedTransform()
{
	tf::StampedTransform transform;

	try
	{
		ros::Time time = ros::Time(0);
		ros::Duration duration;
		duration = duration.fromSec(0.01);
		std::string msg;
		if (listener->waitForTransform("/odom", "/base_link", time, duration, duration, &msg))
		{
			listener->lookupTransform("/odom", "/base_link", time, transform);
			UpdatePosition(transform);
			return true;
		}
		else
		{
			ROS_ERROR("%s", msg.c_str());
			ros::Duration(1.0).sleep();
		}
	}
	catch (const tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	return false;
}

void PerformFollowPoint(const geometry_msgs::PoseStamped& goal)
{
	tf::Vector3 vec_goal(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
	geometry_msgs::Twist twist;

	bool should_break = false;

	const double look_ahead_dist = 3.0;
	const double turn_smoothness = 2.0;
	const double speed = 1.0;
	const double half_look_ahead_dist_squared = 2.0 / (look_ahead_dist * look_ahead_dist);

	double distance = look_ahead_dist + 1.0;
	double prev_dist = 0.0;

	ros::Rate rate(10);

	while (ros::ok() && !should_break)
	{
		if (GetUpdatedTransform())
		{
			prev_dist = distance;
			tf::Vector3 diff = vec_goal - current_pos;
			distance = diff.length();

			double robot_yaw = angles::normalize_angle(tf::getYaw(current_quat));
			double alpha = angles::normalize_angle(std::atan2(diff.getY(), diff.getX()) - robot_yaw);
			double y = angles::normalize_angle(std::sin(alpha)) * distance;

			double w = half_look_ahead_dist_squared * y;

			twist.angular.z = w * speed / turn_smoothness;
			twist.linear.x = speed * std::sqrt(distance / look_ahead_dist);
			velocity.publish(twist);

			rate.sleep();
		}

		should_break |= distance < 0.10;
	}

	twist.angular.z = 0.0;
	twist.linear.x = 0.0;
	velocity.publish(twist);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "seek_behavior");
	ros::NodeHandle n;

	listener = new tf::TransformListener();

	velocity = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	plan_sub = n.subscribe("/move_base_simple/goal", 10, PerformFollowPoint);

	ros::spin();

	delete listener;

	return 0;
}

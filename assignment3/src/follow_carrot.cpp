#ifdef _MSC_VER
#define __attribute__(a) 
#endif

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

tf::TransformListener listener;

tf::Quaternion current_quat;
tf::Vector3 current_pos;

void UpdatePosition(const tf::StampedTransform& transform)
{
	current_pos.setX(transform.getOrigin().x());
	current_pos.setY(transform.getOrigin().y());

	current_quat = transform.getRotation();
}

void GetUpdatedTransform()
{
	tf::StampedTransform transform;
	while (true && ros::ok())
	{
		try
		{
			listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			UpdatePosition(transform);
			return;
		}
		catch (const tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

void PerformFollowPoint(const geometry_msgs::PoseStamped& goal, const geometry_msgs::PoseStamped& next_goal, bool last)
{
	tf::Vector3 vec_goal(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
	geometry_msgs::Twist twist;

	const double look_ahead_dist = 3.0;
	const double speed = 1.0;
	const double half_look_ahead_dist_squared = 2.0 / (look_ahead_dist * look_ahead_dist);
	double distance;

	do
	{
		distance = current_pos.distance(vec_goal);

		tf::Vector3 diff = current_pos - vec_goal;

		double robot_yaw = tf::getYaw(current_quat);
		double alpha = std::atan2(diff.getY(), diff.getX()) - robot_yaw;
		double y = std::sin(alpha) * distance;

		double w = half_look_ahead_dist_squared * y;

		twist.angular.z = w * speed;
		twist.linear.x = speed * (last ? std::sqrt(distance / look_ahead_dist) : 1.0);
		velocity.publish(twist);
	} 
	while ((!last && distance < look_ahead_dist) || (last && distance < 0.005));
}

void PerformPathFollowing(nav_msgs::Path path)
{
	geometry_msgs::PoseStamped goal;
	geometry_msgs::PoseStamped next_goal;

	while (ros::ok())
	{
		if (path.poses.size() > 0)
		{
			goal = path.poses.front();
			path.poses.erase(path.poses.begin());

			next_goal = path.poses.size() ? path.poses.front() : goal;

			PerformFollowPoint(goal, next_goal, path.poses.size() == 0);
		}
		else
		{
			return;
		}
	}
}

void PlanCallback(const nav_msgs::PathConstPtr& message)
{
	PerformPathFollowing(*message);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "follow_carrot_node");
	ros::NodeHandle n;

	velocity = n.advertise<geometry_msgs::Twist>("/stagesim/cmd_vel", 10);
	plan_sub = n.subscribe("/stagesim/plan", 10, PlanCallback);

	ros::spin();

	return 0;
}

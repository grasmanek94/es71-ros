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

void PerformFollowPoint(const geometry_msgs::PoseStamped& goal, const geometry_msgs::PoseStamped& next_goal, bool last)
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

	ros::Rate rate(100);

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
			twist.linear.x = speed * (last ? std::sqrt(distance / look_ahead_dist) : 1.0);
			velocity.publish(twist);

			/*std::cout
				<< "dist: " << distance
				<< ", pos: (" << current_pos.getX() << ", " << current_pos.getY() << ")"
				<< ", goal: (" << vec_goal.getX() << ", " << vec_goal.getY() << ")"
				<< ", diff: (" << diff.getX() << ", " << diff.getY() << ")"
				<< ", yaw: " << angles::to_degrees(robot_yaw)
				<< ", alpha: " << angles::to_degrees(alpha)
				<< ", y: " << y
				<< ", w: " << w
				<< ", vel: (" << twist.linear.x << ", " << twist.angular.z << ")"
				<< ", last: " << last
				<< std::endl;*/
			rate.sleep();
		}

		should_break |= !last && distance < look_ahead_dist;
		should_break |= last && distance < 0.05;
		should_break |= last && distance < 1.0 && distance < prev_dist;
	}

	if (last)
	{
		twist.angular.z = 0.0;
		twist.linear.x = 0.0;
		velocity.publish(twist);
	}
}

void PerformPathFollowing(nav_msgs::Path path)
{
	geometry_msgs::PoseStamped goal;
	geometry_msgs::PoseStamped next_goal;

	size_t size = path.poses.size();
	//std::cout << "PerformPathFollowing size: " << size << std::endl;

	while (ros::ok() && path.poses.size() > 0)
	{
		goal = path.poses.front();
		path.poses.erase(path.poses.begin());

		next_goal = path.poses.size() > 0 ? path.poses.front() : goal;

		/*std::cout
			<< "Goal("
			<< goal.pose.position.x << ", "
			<< goal.pose.position.y << ", "
			<< angles::to_degrees(tf::getYaw(goal.pose.orientation)) << ") "
			<< "Next_Goal("
			<< next_goal.pose.position.x << ", "
			<< next_goal.pose.position.y << ", "
			<< angles::to_degrees(tf::getYaw(next_goal.pose.orientation)) << ")"
			<< std::endl;*/
		
		PerformFollowPoint(goal, next_goal, path.poses.size() == 0);
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

	listener = new tf::TransformListener();

	velocity = n.advertise<geometry_msgs::Twist>("/stagesim/cmd_vel", 10);
	plan_sub = n.subscribe("assignment3/plan", 10, PlanCallback);

	ros::spin();

	delete listener;

	return 0;
}

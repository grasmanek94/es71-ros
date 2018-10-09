#ifdef _MSC_VER
#define __attribute__(a) 
#endif

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <angles/angles.h>

ros::Publisher velocity;
ros::Subscriber odom_sub;
ros::Subscriber tf_sub;
ros::Subscriber plan_sub;

tf::Quaternion current_quat;
tf::Vector3 current_pos;

tf2_msgs::TFMessage transforms;

void PerformPathFollowing(const nav_msgs::Path& path);

void OdomCallback(const nav_msgs::OdometryConstPtr& message)
{
	tf::quaternionMsgToTF(message->pose.pose.orientation, current_quat);

	current_pos.setX(message->pose.pose.position.x);
	current_pos.setY(message->pose.pose.position.y);
}

void TfCallback(const tf2_msgs::TFMessageConstPtr& message)
{
	transforms = *message;
}

void PlanCallback(const nav_msgs::PathConstPtr& message)
{
	PerformPathFollowing(*message);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "follow_carrot");
	ros::NodeHandle n;
	
	velocity = n.advertise<geometry_msgs::Twist>("/stagesim/cmd_vel", 10);
	odom_sub = n.subscribe("/stagesim/odom", 10, OdomCallback);
	tf_sub = n.subscribe("/tf", 10, TfCallback);
	plan_sub = n.subscribe("/stagesim/plan", 10, PlanCallback);

	ros::spin();

	return 0;
}

void PerformPathFollowing(const nav_msgs::Path& path)
{

}

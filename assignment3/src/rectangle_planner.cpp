#ifdef _MSC_VER
#define __attribute__(a) 
#endif

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

ros::Subscriber odom;

tf::Quaternion current_quat;
tf::Vector3 current_pos;

void OdomCallback(const nav_msgs::OdometryConstPtr& message)
{
	tf::quaternionMsgToTF(message->pose.pose.orientation, current_quat);

	current_pos.setX(message->pose.pose.position.x);
	current_pos.setY(message->pose.pose.position.y);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rectangle_planner");
	ros::NodeHandle n;
	
	odom = n.subscribe("/stagesim/odom", 10, OdomCallback);

	ros::spin();

	return 0;
}

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

tf2_msgs::TFMessage transforms;

void PerformPathFollowing(const nav_msgs::Path& path);
void UpdatePosition(const tf::StampedTransform& transform)
{
	current_pos.setX(transform.getOrigin().x());
	current_pos.setY(transform.getOrigin().y());

	current_quat = transform.getRotation();
	std::cout << "ODO: " << current_pos.getX() << ", " << current_pos.getY() << std::endl;
}

void OdomCallback(const nav_msgs::OdometryConstPtr& message)
{
	/*tf::quaternionMsgToTF(message->pose.pose.orientation, current_quat);

	current_pos.setX(message->pose.pose.position.x);
	current_pos.setY(message->pose.pose.position.y);*/
	std::cout << "ODO: " << message->pose.pose.position.x << ", " << message->pose.pose.position.y << std::endl;
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

	tf::TransformListener listener;

	while (ros::ok())
	{
		tf::StampedTransform transform;
		try
		{
			listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			UpdatePosition(transform);
		}
		catch (const tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		ros::spinOnce();
	}

	//ros::spin();

	return 0;
}

void PerformPathFollowing(const nav_msgs::Path& path)
{

}

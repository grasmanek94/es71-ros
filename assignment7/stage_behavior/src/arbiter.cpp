#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>

/**
 * Priority based arbiter, forwards highest priority commands
 */
class Arbiter
{
protected:
	ros::NodeHandle handle;
	ros::NodeHandle parent_handle;

	std::vector<ros::Subscriber> cmd_vel;
	ros::Publisher pub;
	ros::Timer timer;
	int inputs;

	geometry_msgs::Twist vel;
	int priority;

	void tick();
	void velocityUpdate(const geometry_msgs::Twist::ConstPtr& cmd, int prio);
};

Arbiter::Arbiter() :
	parent_handle("~"),
	inputs(3)
{
	const int queue_size = 100; // Items
	double rate = 100.0; // Hz

	parent_handle.param("inputs", inputs, inputs);
	parent_handle.param("publish_rate", rate, rate);

	priority = inputs;

	for (int i = 0; i < inputs; ++i)
	{
		cmd_vel.push_back(handle.subscribe<geometry_msgs::Twist>("cmd_vel" + std::to_string(i), queue_size, boost::bind(&Arbiter::velocityUpdate, this, _1, i)));
	}

	pub = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	timer = handle.createTimer(ros::Duration(1.0 / rate), boost::bind(&Arbiter::tick, this));
}

void Arbiter::tick() 
{
	if (priority != inputs)
	{
		priority = inputs;
		pub.publish(vel);
	}
}

void Arbiter::velocityUpdate(const geometry_msgs::Twist::ConstPtr& cmd, int prio) 
{
	if (prio < priority)
	{
		priority = prio;
		vel = *cmd;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arbiter");
	Arbiter arbiter;
	ros::spin();

	return 0;
}

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>

const static int velocities = 3;
/**
 * Priority based arbiter, forwards highest priority commands
 */
class Arbiter
{
public:
	// members

	// methods
  Arbiter();

protected:
	// members
	ros::NodeHandle handle;
	std::vector<ros::Subscriber> cmd_vel;
	ros::Publisher pub;
	ros::Timer timer;

	geometry_msgs::Twist vel;
	int priority;
	// methods
};

Arbiter::Arbiter()
{
	const double rate = 100.0; // Hz
	const int queue_size = 100; // Items
	
	priority = velocities;

	for (int i = 0; i < velocities; ++i)
	{
		cmd_vel.push_back(handle.subscribe<geometry_msgs::Twist>("cmd_vel" + std::to_string(i), queue_size, boost::bind(&Arbiter::velocityUpdate, this, _1, i)));
	}

	pub = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	timer = handle.createTimer(ros::Duration(1.0 / rate), boost::bind(&Arbiter::tick, this));
}

void Arbiter::tick() 
{
	if (priority != velocities) 
	{
		priority = velocities;
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

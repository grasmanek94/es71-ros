#ifdef _MSC_VER
#define __attribute__(a) 
#endif

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <boost/chrono/chrono.hpp>

ros::Publisher clock_pub;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clock");
	ros::NodeHandle n;
	
	clock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 10, true);

	ros::Rate rate(10000);

	rosgraph_msgs::Clock time;
	time.clock.init();

	boost::chrono::time_point <boost::chrono::system_clock> start = boost::chrono::system_clock::now();
	boost::chrono::time_point <boost::chrono::system_clock> end;

	while (ros::ok())
	{
		rate.sleep();

		end = boost::chrono::system_clock::now();
		boost::chrono::duration<double> diff = end - start;
		time.clock.fromSec(diff.count());

		clock_pub.publish(time);
		ros::spinOnce();
	}

	return 0;
}

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigator");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

	// isn't big so stack is fine
	std::vector <move_base_msgs::MoveBaseGoal> goals;

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.position.x = 0.34;
	goal.target_pose.pose.position.y = 0.07;
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = -0.218410;
	goal.target_pose.pose.orientation.w = 0.975857;
	goals.push_back(goal);
	goal.target_pose.pose.position.x = 0.97;
	goal.target_pose.pose.position.y = -0.82;
	goal.target_pose.pose.orientation.z = -0.171565;
	goal.target_pose.pose.orientation.w = 0.985173;
	goals.push_back(goal);
	goal.target_pose.pose.position.x = 2.4;
	goal.target_pose.pose.position.y = -1.0;
	goal.target_pose.pose.orientation.z = 0.125737;
	goal.target_pose.pose.orientation.w = 0.992064;
	goals.push_back(goal);
	goal.target_pose.pose.position.x = 2.5;
	goal.target_pose.pose.position.y = -0.01;
	goal.target_pose.pose.orientation.z = 0.914425;
	goal.target_pose.pose.orientation.w = 0.404755;
	goals.push_back(goal);
	goal.target_pose.pose.position.x = 1.18;
	goal.target_pose.pose.position.y = -0.11;
	goal.target_pose.pose.orientation.z = 0.996741;
	goal.target_pose.pose.orientation.w = -0.080668;
	goals.push_back(goal);

    for (auto goal : goals) 
	{
		ROS_INFO("Sending goal");
		goal.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
            ROS_WARN("The base failed to move");
        }
    }

    return 0;
}

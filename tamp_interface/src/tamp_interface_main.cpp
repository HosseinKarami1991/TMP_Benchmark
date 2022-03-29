#include "tamp_interface.h"
#include <ros/ros.h>


int main(int argc, char **argv)
{
	ros::init(argc,argv,"tamp_interface_node");
	tamp_interface tampInterface;
	agents_tasks agent;
	agent.agents.push_back("LeftArm"); 
    agents_tasks agent2;
	agent2.agents.push_back("RightArm");
	//cout<<"some grasping tests"<<endl;
	//tampInterface.sendGraspingCommand(agent2);
    //tampInterface.sendUnGraspingCommand(agent2);
    //tampInterface.sendGraspingCommand(agent);
	//tampInterface.sendUnGraspingCommand(agent);
	while(ros::ok()){

		ros::spinOnce();
	}
	return 0;
}
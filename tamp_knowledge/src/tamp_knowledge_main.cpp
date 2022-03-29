#include <iostream>
#include <ros/ros.h>
#include "points.h"
#include "world.h"
#include <geometry_msgs/Vector3.h>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <cmath>
#include "tamp_knowledge.h"



int main(int argc, char **argv)
{
	 ros::init(argc,argv,"tamp_knowledge_node");
     
    tamp_knowledge tampKnowledge;
    
    

    //ros::Rate r(1);
    while(ros::ok()){

    	ros::spinOnce();
    	//r.sleep();
    }



	return 0;
}
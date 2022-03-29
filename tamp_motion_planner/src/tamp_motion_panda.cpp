#include "panda_planner.h"
#include <iostream>
#include <ros/ros.h>
using namespace std;


int main(int argc, char **argv){
   
   ros::init(argc, argv, "tamp_motion_node");
  
   panda_planner pandaPlanner;
   while(ros::ok()){
   
    ros::spinOnce();
    
   }

  return 0;
}
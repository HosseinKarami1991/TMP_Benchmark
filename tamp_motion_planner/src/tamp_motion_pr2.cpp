#include "pr2_planner.h"
#include <iostream>
#include <ros/ros.h>
using namespace std;


int main(int argc, char **argv){
   
   ros::init(argc, argv, "tamp_motion_pr2");
  
   pr2_planner PR2Planner;
 

   while(ros::ok()){
   
    ros::spinOnce();
    
   }

  return 0;
}
#include <iostream>
#include <pr_hw_interface.h>
#include "controller_manager/controller_manager.h"
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/ControllerState.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/ros.h>
//#include <my_robot/my_robot.h>
#include <controller_manager/controller_manager.h>
int main(int argc, char **argv)
{
    // Initialize the ROS node
   ros::init(argc, argv, "MyRobot_hardware_inerface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2); 
    
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    
    return 0;
}
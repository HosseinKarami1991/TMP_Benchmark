/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of CU Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * \brief   Outputs to console the joint values of a chosen planning group
 * \author  Dave Coleman
 */

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>

namespace baxter_moveit_scripts
{

static const std::string ROBOT_DESCRIPTION="robot_description";

class GetJointValues
{
public:

  // our interface with MoveIt

  boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  boost::scoped_ptr<moveit::core::RobotState> start_state_;
  GetJointValues()
  {
    ros::NodeHandle nh;
    
    std::string planning_group;

    std::cout << "Type desired planning group name:\n";
    std::getline(std::cin, planning_group);
    
    // Create MoveGroup for right arm
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group)); // \todo group name

    std::vector<double> joint_values;
    std::vector<std::string> joint_names = move_group_->getJoints();
     std::cout << "move_group_->getJoints();:"<<std::endl;

    geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.3;
  move_group_->setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  start_state_.reset(new moveit::core::RobotState(*move_group_->getCurrentState()));
  std::cout << "start_state_.reset"<<std::endl;
  //moveit::core::RobotState start_state(*move_group_->getCurrentState());
 move_group_->setStartState(*start_state_);
 std::cout << "move_group_->setStartState(*start_state_);"<<std::endl;
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //std::cout<<"planning was"<< success<<std::endl;
    while(ros::ok())
    {
      ROS_INFO_STREAM("SDF Code for joint values pose:\n");

      joint_values = move_group_->getCurrentJointValues();
      
      // Output XML
      std::cout << "<group_state name=\"\" group=\"" << planning_group << "\">\n";
      for (std::size_t i = 0; i < joint_values.size(); ++i)
      {
        std::cout << "  <joint name=\"" << joint_names[i] <<"\" value=\"" << joint_values[i] << "\" />\n";
      }
      std::cout << "</group_state>\n\n\n\n";

      ros::Duration(4.0).sleep();
    }

  }

};

} //namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "get_joint_values");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the pick place node
  baxter_moveit_scripts::GetJointValues();

  ros::shutdown();

  return 0;
}

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <rrbot_control/pr2_hw_interface.h>

namespace rrbot_control
{

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");
 
  subjointpos = nh_.subscribe("/vrep/all_joints/joint_states",10,&RRBotHWInterface::psoeCb,this);
   pubcommand = nh_.advertise<sensor_msgs::JointState>("/vrep/pr2/all_joints",10);
}

void RRBotHWInterface::psoeCb(const sensor_msgs::JointStatePtr & msg){

 // for(std::size_t i=0;i<8;++i){
  //    joint_position_[i] = msg->position[i];
  //    joint_velocity_[i] = msg->velocity[i];
  //}
  joint_position_ = msg->position;
  joint_velocity_ = msg->velocity;
 // joint_position_.push_back(0.0);
  //joint_position_.push_back(0.0);
  //joint_velocity_.push_back(0.0);
  //joint_velocity_.push_back(0.0);
}

void RRBotHWInterface::read(ros::Duration &elapsed_time)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void RRBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);
  sensor_msgs::JointState msg;
  msg.position = joint_position_command_;
  msg.velocity = joint_velocity_command_;
  //msg.velocity[15] = msg.velocity[15] * cos(joint_position_[17]) + msg.velocity[16] * sin(joint_position_[17]);
  //msg.velocity[16] = -1.0 *  msg.velocity[15] * sin(joint_position_[17]) + msg.velocity[16] * cos(joint_position_[17]);
  /*
  if(msg.position[11]>=-0.15 || msg.position[11]<=-2.121){
    msg.position[11] = -0.5;
  }
   if(msg.position[4]>=-0.15 || msg.position[4]<=-2.121){
    msg.position[4] = -0.5;
  }
  if(msg.position[13]>=-0.1 || msg.position[13]<=-2.0){
    msg.position[13] = -0.5;
  }
   if(msg.position[6]>=-0.1 || msg.position[6]<=-2.0){
    msg.position[6] = -0.5;
  }
  */
  pubcommand.publish(msg);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  //for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
   // joint_position_[joint_id] += joint_position_command_[joint_id];
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------

}

void RRBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace


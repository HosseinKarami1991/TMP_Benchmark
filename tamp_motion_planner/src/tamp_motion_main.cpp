#include "tamp_motion.hpp"
#include <iostream>
#include "sensor_msgs/JointState.h"
#include <ros/ros.h>
using namespace std;
sensor_msgs::JointState::ConstPtr topub;
sensor_msgs::JointState::ConstPtr joint_state;
 
void callback(sensor_msgs::JointState &msg);
void printPosition(geometry_msgs::PoseStamped pos){

 std::cout<<"**********this is pos of end effector**********"<<std::endl;
 std::cout<<"x: "<<pos.pose.position.x<<std::endl;
 std::cout<<"y: "<<pos.pose.position.y<<std::endl;
 std::cout<<"z: "<<pos.pose.position.z<<std::endl;

}

void callback(const sensor_msgs::JointState::ConstPtr& msg){

 //double x = msg->position[3];
 //joint_state = msg;
 
        	//topub.header = joint_state->header;
 /*
 topub->name.clear();
 topub->name.push_back(msg->name[11]);
 topub->name.push_back(msg->name[12]);
 topub->name.push_back(msg->name[9]);
 topub->name.push_back(msg->name[10]);
 topub->name.push_back(msg->name[13]);
 topub->name.push_back(msg->name[14]);
 topub->name.push_back(msg->name[15]);
 
 topub.position.clear();
 topub.position.push_back(msg->position[11]);
 topub.position.push_back(msg->position[12]);
 topub.position.push_back(msg->position[9]);
 topub.position.push_back(msg->position[10]);
 topub.position.push_back(msg->position[13]);
 topub.position.push_back(msg->position[14]);
 topub.position.push_back(msg->position[15]);

 topub.velocity.clear();
 topub.velocity.push_back(msg->velocity[11]);
 topub.velocity.push_back(msg->velocity[12]);
 topub.velocity.push_back(msg->velocity[9]);
 topub.velocity.push_back(msg->velocity[10]);
 topub.velocity.push_back(msg->velocity[13]);
 topub.velocity.push_back(msg->velocity[14]);
 topub.velocity.push_back(msg->velocity[15]);
 topub.effort.clear();
  topub.effort.push_back(msg->effort[11]);
 topub.effort.push_back(msg->effort[12]);
 topub.effort.push_back(msg->effort[9]);
 topub.effort.push_back(msg->effort[10]);
 topub.effort.push_back(msg->effort[13]);
 topub.effort.push_back(msg->effort[14]);
 topub.effort.push_back(msg->effort[15]);
 
            */ 
 
 //std::cout<<"**********subsititiron**********"<<std::endl;
 //topub.name.push_back("joint1");
  /*
 
 */

}

int main(int argc, char **argv)
{
	 
	 ros::init(argc, argv, "tamp_motion_node");
	 ros::NodeHandle nh;
	 ros::Subscriber sub;
	  ros::Rate loop_rate(0.5);
	 //sub =nh.subscribe("/robot/joint_states",10,callback);
	 // ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/my_joint_states",50);
	tamp_motion tampMotion;
   // cout<<"current ee poseL:"<<tampMotion.getCurrentPose().translation()<<endl;
     
     geometry_msgs::PoseStamped pose_msg;
     tf2::Quaternion targetQuaternion_;
      targetQuaternion_.setRPY(0.5, 1.9,1.0);

    pose_msg.pose.orientation = tf2::toMsg(targetQuaternion_);

     pose_msg.header.frame_id ="base";
     pose_msg.pose.position.x = 1.0;
    pose_msg.pose.position.y = -0.2;
    pose_msg.pose.position.z = 0.24;
   // pose_msg.pose.orientation.x = 0.384237;
  //  pose_msg.pose.orientation.y = 0.922543;
  //  pose_msg.pose.orientation.z = -0.00470893;
  //  pose_msg.pose.orientation.w = 0.035402;


     tampMotion.sendToPose(pose_msg);

	 while(ros::ok()){
	 
 
 
  
	 	


      
      
        //pub.publish(topub);
		ros::spinOnce();
		//loop_rate.sleep();
	 }

	return 0;
}
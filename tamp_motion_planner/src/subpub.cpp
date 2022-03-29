#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
using namespace std;
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::JointState>("/myjoints", 50);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/robot/joint_states", 50, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    //sensor_msgs::JointState topub new(sensor_msgs::JointState);
    msgreceived_ =true;
   sensor_msgs::JointState topub;


  topub.header.stamp = ros::Time::now();

 //topub.name.clear();
   topub.name.resize(7);
   //topub.name[0]="df";
 //cout<<"5th name is "<<msg->position[5]<<endl;
 topub.name[0]="right_s0";
 topub.name[1]="right_s1";
 topub.name[2]="right_e0";
 topub.name[3]="right_e1";
 topub.name[4]="right_w0";
 topub.name[5]="right_w1";
 topub.name[6]="right_w2";

 topub.position.resize(7);
 topub.position[0] = msg->position[11];
 topub.position[1] = msg->position[12];
 topub.position[2] = msg->position[9];
 topub.position[3] = msg->position[10];
 topub.position[4] = msg->position[13];
 topub.position[5] = msg->position[14];
 topub.position[6] = msg->position[15];
 topub.velocity.resize(7);
 topub.velocity[0] = msg->velocity[11];
 topub.velocity[1] = msg->velocity[12];
 topub.velocity[2] = msg->velocity[9];
 topub.velocity[3] = msg->velocity[10];
 topub.velocity[4] = msg->velocity[13];
 topub.velocity[5] = msg->velocity[14];
 topub.velocity[6] = msg->velocity[15];

  topub.effort.resize(7);
 topub.effort[0] = msg->effort[11];
 topub.effort[1] = msg->effort[12];
 topub.effort[2] = msg->effort[9];
 topub.effort[3] = msg->effort[10];
 topub.effort[4] = msg->effort[13];
 topub.effort[5] = msg->effort[14];
 topub.effort[6] = msg->effort[15];
 /*
 topub.name.push_back(msg->name[12]);
 topub.name.push_back(msg->name[9]);
 topub.name.push_back(msg->name[10]);
 topub.name.push_back(msg->name[13]);
 topub.name.push_back(msg->name[14]);
 topub.name.push_back(msg->name[15]);
 
 //topub.position.clear();
 topub->position.push_back(msg->position[11]);
 topub->position.push_back(msg->position[12]);
 topub->position.push_back(msg->position[9]);
 topub->position.push_back(msg->position[10]);
 topub->position.push_back(msg->position[13]);
 topub->position.push_back(msg->position[14]);
 topub->position.push_back(msg->position[15]);

 //topub.velocity.clear();
 topub->velocity.push_back(msg->velocity[11]);
 topub->velocity.push_back(msg->velocity[12]);
 topub->velocity.push_back(msg->velocity[9]);
 topub->velocity.push_back(msg->velocity[10]);
 topub->velocity.push_back(msg->velocity[13]);
 topub->velocity.push_back(msg->velocity[14]);
 topub->velocity.push_back(msg->velocity[15]);
// topub.effort.clear();
  topub->effort.push_back(msg->effort[11]);
 topub->effort.push_back(msg->effort[12]);
 topub->effort.push_back(msg->effort[9]);
 topub->effort.push_back(msg->effort[10]);
 topub->effort.push_back(msg->effort[13]);
 topub->effort.push_back(msg->effort[14]);
 topub->effort.push_back(msg->effort[15]);
*/



   // output=input;
    //.... do something with the input and generate the output...
 if(msgreceived_){
    pub_.publish(topub);
 }
 msgreceived_=false;
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  bool msgreceived_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");
  
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
  ros::Rate loop_rate(50);
  while(ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }

  

  return 0;
}

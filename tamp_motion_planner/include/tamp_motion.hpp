#ifndef TAMPMOTION_H
#define TAMPMOTION_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include<tamp_msgs/trajquest.h>
#include<tamp_msgs/ackquest.h>
#include<tamp_msgs/knowledge.h>
#include<tamp_msgs/vrepmotion.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include "moveit_msgs/ApplyPlanningScene.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h> // for plan_execution
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit/kinematic_constraints/utils.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>




using namespace std;


static const std::string RIGHT_PLANNING_GROUP="right_arm";
static const std::string LEFT_PLANNING_GROUP="left_arm";
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string JOINT_STATE_TOPIC = "/robot/joint_states";




class tamp_motion
{
public:
	
	tamp_motion(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr());
	~tamp_motion();
	bool ackQuery(tamp_msgs::ackquest::Request& request, tamp_msgs::ackquest::Response& response);
	void setVisualTools();
	void addCollisionObjects(vector<char> objects);
    void printPosition(geometry_msgs::PoseStamped pos);
	void printRotation(std::vector<double> rotationvector);
	bool motionQuery(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response);
	string getEndEffectorLink();
	bool loadPlanningSceneMonitor();
	geometry_msgs::PoseStamped getEECurrentpos(string arm);
	vector<double> getEECurrentRPY(string arm);
	bool sendToPose(const geometry_msgs::PoseStamped& pose);
	bool convertResult(moveit::planning_interface::MoveItErrorCode& code);
	geometry_msgs::Pose  getCurrentPose();
	void printCurrentJointValues();
	moveit::planning_interface::MoveItErrorCode sendToPose(const std::string &pose_name);
	void addColision();
	void removeObject(string objid);
	void stopMotion(const std_msgs::Bool &msg);
	double targetPositionTolerance_;
	double targetOrientationTolerance_;
	tf2::Quaternion targetQuaternion_;
	geometry_msgs::Pose targetPos_;
	moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;
	bool motion_result_,sendtojointpose_;
	moveit_msgs::RobotTrajectory  solvedTraj_;
	double planningtime_;
	std::vector<float> jointangles_;
	double timespentforplanning_,timespentforexecution_;
	int nuofplanning_,nuofexecution_;
	void attachObjectToGripper();
	std::vector<double> getJointPose(string & arm);
	std::vector<double> getEEStateFromJoint(string &arm,std::vector<double> &jont_poses);
	bool sendToSimulation(moveit_msgs::RobotTrajectory&, string&);
    void addSimulationCollision();
    void registerJointState(const sensor_msgs::JointState &msg);
    bool allJointsReached(std::vector<double> poses,string arm);
    boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right_,move_group_left_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    moveit::planning_interface::MoveGroupInterface::Options optsright_,optsleft_;
    







	
private:
	string EELink;
	geometry_msgs::PoseStamped EECurrentpos_;
	vector<double> EECurrentRPY_;
	//moveit::planning_interface::MoveGroupInterface move_group_right;
	//moveit::planning_interface::MoveGroupInterface move_group_left;
	moveit_msgs::PlanningScene planning_scene;
	string PLANNING_GROUP;
	string dominant_arm_;

	vector<moveit_msgs::CollisionObject> collisionObjects_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::NodeHandle node_handle;
    ros::ServiceServer trajservice,ackservice;
    ros::ServiceClient planning_scene_diff_client,tampKnowledgeClient,tampVREPMotionClient;
    ros::Subscriber stop_motion,vrep_joint_state;
    bool toexecute_,addcollision_,simulation_;
    std::vector<string> colisionobjectsids_;
    std::vector<moveit_msgs::ObjectColor> object_colors_;
    ros::Publisher scene_plan_pub,vrep_rightarm_pub,vrep_leftarm_pub;
    std::vector<double> lastrightjoints_,lastleftjoints_;



};





























#endif

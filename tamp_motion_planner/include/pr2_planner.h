#ifndef PR2PLANNER_H
#define PR2PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include<tamp_msgs/trajquest.h>
//#include<tamp_msgs/ackquest.h>
//#include<tamp_msgs/knowledge.h>
//#include<tamp_msgs/vrepmotion.h>
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
#include <sensor_msgs/JointState.h>
#include <tamp_msgs/objectssrv.h>
#include <tamp_msgs/pandafreeplan.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tamp_msgs/objectcolorsrv.h>
#include<math.h>
#include "tamp_msgs/registerdata.h"
#include "tf/LinearMath/Transform.h"
#include "tf/LinearMath/Vector3.h"
#include <tamp_msgs/pandaee.h>
#include <tamp_msgs/executetraj.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Accel.h>
#include <boost/thread/condition.hpp>
#include "my_pid.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tamp_msgs/removeobject.h>
#include <chrono>
#include <thread>

#define PI 3.14 
using namespace std;


static const std::string PLANNING_GROUP="base";
static const std::string PLANNING_GROUP_RIGHT="right_arm_only";
static const std::string PLANNING_GROUP_LEFT="left_arm_only";
static const std::string PLANNING_GROUP_WL="whole_left";
static const std::string PLANNING_GROUP_WR="whole_right";
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string PR2_JOINT_STATE_TOPIC = "/vrep/base_position";
static const std::string PR2_BASE_VELOCITY_STATE_TOPIC = "/vrep/base_velocity";
static const std::string PR2_WHEEL_VELOCITY_STATE_TOPIC = "/vrep/base_wheels";
static const std::string PR2_JOINT_COMMAND_TOPIC = "/vrep/pr2/wheels";
static const std::string PANDA2_JOINT_COMMAND_TOPIC = "/franka2/joint_command";
static const std::string PANDA1_MOTION_SERVICE_TOPIC="tamp_motion_service_pr2";
static const std::string PANDA2_MOTION_SERVICE_TOPIC="tamp_motion_service_panda2";
static const std::string JONIT_STATE_TOPIC="/joint_states";
static const std::string PANDA1_FREE_PLAN_TOPIC = "panda1_free_plan";
static const std::string PANDA2_FREE_PLAN_TOPIC = "panda2_free_plan";
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> pr2_arm_control_client;
typedef boost::shared_ptr<pr2_arm_control_client>  pr2_arm_control_client_Ptr;
using namespace std::chrono_literals;
class pr2_planner
{
public:
	
	 pr2_planner(/*planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr()*/);
	~pr2_planner();
	bool sendRightArmToSimulationAction(moveit_msgs::RobotTrajectory &traj);
	bool sendLeftArmToSimulationAction(moveit_msgs::RobotTrajectory &traj);
	 bool sendPr2BaseToSimulationAction(moveit_msgs::RobotTrajectory &traj);
private:

	void callBackPr2(const geometry_msgs::Pose& joint_state);
	bool allPr2JointsReached(std::vector<double>& poses);
	bool motionQuery(tamp_msgs::trajquest::Request& , tamp_msgs::trajquest::Response&);
	bool loadPlanningSceneMonitor();
	void addCollisionObjects(const int& );
	void removeColission();
    void removeObjectCollision(string &object);
    bool executePr2(tamp_msgs::executetraj::Request &req,tamp_msgs::executetraj::Response &res);
    bool sendPr2ToSimulation(moveit_msgs::RobotTrajectory &traj);
    void callBackBaseVelPr2(const geometry_msgs::AccelPtr & base_vel);
    void callBackWheelVelPr2(const sensor_msgs::JointStatePtr & wheel_vel);
    void publishBaseStates(const sensor_msgs::JointState &jointmsg);
    bool checkIfCastersReached(const std::vector<double> v);
    bool motionQueryArm(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response);
    void rightArmCb(const sensor_msgs::JointStatePtr & msg);
    void leftArmCb(const sensor_msgs::JointStatePtr & msg);
	bool sendRightArmToSimulation(moveit_msgs::RobotTrajectory &traj);
	bool sendLefArmToSimulation(moveit_msgs::RobotTrajectory &traj);
	bool allRightArmJointsReached(std::vector<double>& );
	bool allLeftArmJointsReached(std::vector<double>& poses);
	bool motionQueryWholeBody(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response);
	void wholeRightCb(const sensor_msgs::JointStatePtr & msg);
	void wholeLeftCb(const sensor_msgs::JointStatePtr & msg);
	void addCollisionObjects();
	bool removeObjectFromScene(tamp_msgs::removeobject::Request &req,tamp_msgs::removeobject::Response &res);

	bool sendWholeRightToAction(moveit_msgs::RobotTrajectory &traj);
	//pointtype point;
	double torso_joint_;
	std::vector<double> pr2_joints_,pr2_base_velocities_,pr2_caster_positions_,pr2_wheel_velocities_,joint_states_,leftarm_joints_,rightarm_joints_,
	rightarm_errors_,leftarm_errors_,whole_right_joints_,whole_left_joints_;
	boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_pr2_,move_group_right_arm_,move_group_left_arm_,
	move_group_whole_left_,move_group_whole_right_;
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
	std::vector<string> colisionobjectsids_;
	std::vector<moveit_msgs::ObjectColor> object_colors_;
	sensor_msgs::JointState cmdmsgpr2_;
	sensor_msgs::JointState cmdmsgright_,cmdmsgleft_;
	const robot_state::JointModelGroup* joint_model_group_base;
	const robot_state::JointModelGroup* joint_model_group_right;
	const robot_state::JointModelGroup* joint_model_group_left;
	const robot_state::JointModelGroup* joint_model_group_;
	const robot_state::JointModelGroup* joint_model_group_whole_right;
	moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;
	moveit::planning_interface::MoveGroupInterface::Options optpanda,optrightarm, optleftarm,optwholeright,optwholeleft;
	tf2::Quaternion targetQuaternion_;
	geometry_msgs::Pose targetPos_;
	ros::NodeHandle node_handle;
	ros::ServiceServer trajservice1,pr2execution,armtrajservice,wholebodytrajservice,remove_object_scene_server;
	ros::Subscriber pr2jointsub,pr2basevelocitysub,pr2Wheelvelocitysub,rightarmsub,leftarmsub,wholerightsub,wholeleftsub;
	ros::Publisher vrep_pr2_pub,vrep_right_pub,vrep_left_pub;
	ros::ServiceClient collision_object_client,vrep_ee_client;
	double freeplan1time,freeplan2time,planning1time,planning2time;
	int nuplanning1,nuplanning2;
	bool newcommand_;
	pthread_mutex_t pr2_base_controller_lock_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	std::string trajectory_controller;
	//actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client;
	pr2_arm_control_client_Ptr right_arm_action_client_,left_arm_action_client_,base_action_client_;
	string forbiddenobject_;
	int nuofplrightarm,nuofplleftarm,nuofplbase,nuofexeright,nuofexeleft,nuofexebase;
	double pltrightarm,pltimeleftarm,pltbase,exetrightarm,exetleftarm,exetbase;

	
};

#endif







#ifndef PANDAPLANNER_H
#define PANDAPLANNER_H

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





#define PI 3.14 
using namespace std;


static const std::string PLANNING_GROUP="panda_arm";
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string PANDA1_JOINT_STATE_TOPIC = "/franka1/joint_states";
static const std::string PANDA2_JOINT_STATE_TOPIC = "/franka2/joint_states";
static const std::string PANDA1_JOINT_COMMAND_TOPIC = "/franka1/joint_command";
static const std::string PANDA2_JOINT_COMMAND_TOPIC = "/franka2/joint_command";
static const std::string PANDA1_MOTION_SERVICE_TOPIC="tamp_motion_service_panda1";
static const std::string PANDA2_MOTION_SERVICE_TOPIC="tamp_motion_service_panda2";
static const std::string JONIT_STATE_TOPIC="/joint_states";
static const std::string PANDA1_FREE_PLAN_TOPIC = "panda1_free_plan";
static const std::string PANDA2_FREE_PLAN_TOPIC = "panda2_free_plan";

 struct point
	{
		double x;
		double y;
	};

class panda_planner
{
public:
	
	panda_planner(/*planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr()*/);
	~panda_planner();
private:
	void callBackfrankajoint1(const sensor_msgs::JointStateConstPtr& joint_state);
	void callBackfrankajoint2(const sensor_msgs::JointStateConstPtr& joint_state);
	bool allFranka1JointsReached(std::vector<double>& poses);
	bool allFranka2JointsReached(std::vector<double>& poses);
	bool sendFranka1ToSimulation(moveit_msgs::RobotTrajectory &traj);
	bool sendFranka2ToSimulation(moveit_msgs::RobotTrajectory &traj);
	bool motionQuery1(tamp_msgs::trajquest::Request& , tamp_msgs::trajquest::Response&);
	bool motionQuery2(tamp_msgs::trajquest::Request& , tamp_msgs::trajquest::Response&);
	bool freePlan1(tamp_msgs::pandafreeplan::Request&, tamp_msgs::pandafreeplan::Response&);
	bool freePlan2(tamp_msgs::pandafreeplan::Request&, tamp_msgs::pandafreeplan::Response&);
	bool loadPlanningSceneMonitor();
	void addCollisionObjects(const int& );
	void removeColission();
	std::vector<string> findObjectsInsideTri(const double & alpha, const double & betaconst,
		                                     const std::vector<string> object_names,
                                             const std::vector<double> object_poses,
                                             const string &target,const string &other_target);

 inline double sign(const point &p1, const point &p2, const point &p3);
 bool pointInTriangle (point pt, point v1, point v2, point v3);
 int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
 std::vector<string> isInsidePolygon(const double & alpha, const double & beta,
                                                        const std::vector<string> object_names,
                                                        const std::vector<double> object_poses,
                                                        const string &target,const string &other_target);

  void 	removeObjectCollision(string &object);
  bool executePanda1(tamp_msgs::executetraj::Request &req,tamp_msgs::executetraj::Response &res);
  bool executePanda2(tamp_msgs::executetraj::Request &req,tamp_msgs::executetraj::Response &res);

	
	//pointtype point;
	std::vector<double> franka1joints_,franka2joints_,franka1jointerrors_,franka2jointerrors_;
	boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_panda_;
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
	std::vector<string> colisionobjectsids_;
	std::vector<moveit_msgs::ObjectColor> object_colors_;
	sensor_msgs::JointState cmdmsgpanda1_,cmdmsgpanda2_;
	const robot_state::JointModelGroup* joint_model_group_;
	moveit::planning_interface::MoveGroupInterface::Plan motion_plan1_,motion_plan2_;
	moveit::planning_interface::MoveGroupInterface::Options optpanda;
	tf2::Quaternion targetQuaternion_;
	geometry_msgs::Pose targetPos_;
	ros::NodeHandle node_handle;
	ros::ServiceServer trajservice1,trajservice2,panda1FreePlan,panda2FreePlan,panda1execution,panda2execution;
	ros::Subscriber franka1jointsub,franka2jointsub;
	ros::Publisher vrep_franka1_pub,vrep_franka2_pub;
	ros::ServiceClient collision_object_panda1_client,
	collision_object_panda2_client,object_coloring_client,knowledgeRegisterClient,vrep_ee_client;
	double freeplan1time,freeplan2time,planning1time,planning2time;
	int nuplanning1,nuplanning2;
	




};





























#endif

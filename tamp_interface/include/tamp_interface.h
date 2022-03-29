#include <iostream>
#include <ros/ros.h>
#include "tamp_msgs/tampSimulationReq.h"
#include "tamp_msgs/tampSimulationRes.h"
#include "tamp_msgs/baxterControlCommand.h"
#include "tamp_msgs/knowledge.h"
#include <chrono>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <boost/algorithm/string.hpp>
#include"tamp_msgs/trajquest.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "tamp_msgs/ackquest.h"
#include "tamp_msgs/registerdata.h"
#include"std_msgs/Bool.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "tamp_msgs/registerplace.h"
#include<tamp_msgs/vrepgripper.h>
#include<tamp_msgs/removeobject.h>
#include <tamp_msgs/pandafreeplan.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include "tamp_msgs/lastgraph.h"
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <tamp_msgs/changeplansrv.h>
#include <tamp_msgs/updatescene.h>
#include <tamp_msgs/executetraj.h>
#include <sensor_msgs/JointState.h>
#include <tamp_msgs/objectssrv.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <tamp_msgs/hanoipp.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;
using namespace std::chrono;

static const std::string PANDA1_JOINT_STATE_TOPIC = "/franka1/joint_states";
static const std::string PANDA2_JOINT_STATE_TOPIC = "/franka2/joint_states";
static const std::string PANDA1_JOINT_COMMAND_TOPIC = "/franka1/joint_command";
static const std::string PANDA2_JOINT_COMMAND_TOPIC = "/franka2/joint_command";


class agents_tasks{
public:
	vector<string> agents; 			//! the different agents combination for performing actions like: leftArm, rightArm, leftArm+rightArm.
	int agentsNumber; // Left: 0, right=1, left+right=2
	vector<string> collaborators;
	string lastAssignedAction;
	bool isActionSuccessfullyDone;
	bool isBusy;
	microseconds microSec_StartingTime;
	bool emergencyFlag; // stop robot emergency flag

	agents_tasks(){
//		agents=NULL;
//		collaborators=NULL;
		lastAssignedAction="";
		isActionSuccessfullyDone=false;
		isBusy=false;
		agentsNumber=0;
		microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
		emergencyFlag=false;
	};
	~agents_tasks(){};

	void Print(void){
		cout<<"Agents: ";
		for(int i=0;i<agents.size();i++){
			cout<<agents[i]<<" ";
		}
		cout<<endl;

		cout<<"collaborators: ";
		for(int i=0;i<collaborators.size();i++){
			cout<<collaborators[i]<<" ";
		}
		cout<<endl;

		cout<<"Last Assigned Action: "<<lastAssignedAction<<endl;
		cout<<"Is Action successfully done? "<< isActionSuccessfullyDone<<endl;
		cout<<"Is Agent busy? "<<isBusy<<endl;
		cout<<"emergency Flag: "<<emergencyFlag<<endl;
		cout<<"Agent Number: "<<agentsNumber<<endl;
		cout<<"Last action command time: "<<microSec_StartingTime.count()<<endl;

	};
};


class tamp_interface
{
public:
	tamp_interface();
	~tamp_interface();
private:
    //actuatuion commands
    void callBackfrankajoint1(const sensor_msgs::JointStateConstPtr& joint_state);
	void callBackfrankajoint2(const sensor_msgs::JointStateConstPtr& joint_state);
	bool allFranka1JointsReached(std::vector<double>& poses);
	bool allFranka2JointsReached(std::vector<double>& poses);
	bool sendFranka1ToSimulation(moveit_msgs::RobotTrajectory &traj);
	bool sendFranka2ToSimulation(moveit_msgs::RobotTrajectory &traj);
    void arrivingCommands(const std_msgs::String::ConstPtr& msg);
    void setAgentsList();
	void sendGraspingCommand(agents_tasks& agent);
	void sendUnGraspingCommand(agents_tasks& agent);
	void sendHoldingCommand(agents_tasks& agent);
	void sendStoppingCommand(agents_tasks& agent);
	void sendApproachingCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
	void sendApproachingCommandPanda1(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
	void sendApproachingCommandPanda2(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
	void sendRestingCommand(agents_tasks& agent);
	void controlAckPub(agents_tasks& agent);
    void PublishRobotAck(agents_tasks& agent);
    bool checkIfReachedToTarget(geometry_msgs::PoseStamped rp, std::vector<double> tar);
    void checkrcommand(agents_tasks& agent);
    void checklcommand(agents_tasks& agent);
    void registerCommand(agents_tasks& agent);
    void cuffingCommand();
    void UncuffingCommand();
    void checkFultCommand(agents_tasks& agent);
    void publishHumanActionAck(int );
    void checkPickandPlaceObject(agents_tasks&);
    void SceneUpdatecommand(agents_tasks&);
    void registerPlaceCommand(agents_tasks& agent);
    void sendSimulationGraspingCommand(const std_msgs::String::ConstPtr& commandmsg, agents_tasks& agent);
    void sendSimulationUnGraspingCommand(const std_msgs::String::ConstPtr& commandmsg, agents_tasks& agent);
    void sendVictoryCommand(agents_tasks& agent);
    void sendCheckifZs(agents_tasks& agent);
    void sendRemoveObjectCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
    void objectRegisterCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
    void removeObjectCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
    //simulation commands
	void arrivingSimulationCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateGraspingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateUnGraspingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateHoldingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateStoppingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateApproachingCommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateRestingcommand(const tamp_msgs::tampSimulationReq& msg);
	void simulateArmMotionPlanner(std::vector<string> ResponsibleAgents,std::vector<double> target,  bool results,double time);
    void simulateCheckrcommand(const tamp_msgs::tampSimulationReq& msg);
    void simulateChecklcommand(const tamp_msgs::tampSimulationReq& msg);
    void simulateRegisterCommand(const tamp_msgs::tampSimulationReq& msg);
    void leftCuffCB(const baxter_core_msgs::DigitalIOState &);
    void rightCuffCB(const baxter_core_msgs::DigitalIOState &);
    void simulateCuffCommand(const tamp_msgs::tampSimulationReq&);
    void simulateUnCuffCommand(const tamp_msgs::tampSimulationReq&);
    void simulateFaultyCommand(const tamp_msgs::tampSimulationReq&);
    void simulateNonFaultyCommand(const tamp_msgs::tampSimulationReq&);
    void simulateNACommand(const tamp_msgs::tampSimulationReq&);
    void simulateCheckPickObject(const tamp_msgs::tampSimulationReq&);
    void simulateCheckPlaceObject(const tamp_msgs::tampSimulationReq& );
    void simulateRegisterPlace(const tamp_msgs::tampSimulationReq&);
    void simulateUpdatingScene(const tamp_msgs::tampSimulationReq&);
    void simulateSendVictory(const tamp_msgs::tampSimulationReq& msg);
    void simulateCheckIfZ1(const tamp_msgs::tampSimulationReq& msg);
    void simulateCheckIfZ2(const tamp_msgs::tampSimulationReq& msg);
    void simulateRemoveObject(const tamp_msgs::tampSimulationReq& msg);
    void simulateCheckif(const tamp_msgs::tampSimulationReq& msg);
    void simulateCheckPP(const tamp_msgs::tampSimulationReq& msg);
    void simulateRegisterPP(const tamp_msgs::tampSimulationReq& msg);
	std::vector<agents_tasks> agents_list;
	void humaninsertCB(const std_msgs::Int16 &msg);
	void humanActionCB(const std_msgs::Int16 &msg);
	void lastStateCB(const std_msgs::String &msg);
	void simulateObjectRegister(const tamp_msgs::tampSimulationReq& msg);
	void simluateFingingClosestObject(const tamp_msgs::tampSimulationReq& msg);
    void simulatePlanFree(const tamp_msgs::tampSimulationReq& msg);
    void planFreeCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
    void checkifCommand(agents_tasks& agent);
    void simulateAddingGraph(const tamp_msgs::tampSimulationReq& msg);
    void addGraphCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
 	void simulateApproachingPanda1(const tamp_msgs::tampSimulationReq& msg);
 	void simulateApproachingPanda2(const tamp_msgs::tampSimulationReq& msg);
 	void exitCommand(agents_tasks& agent);
 	void simulateExit(const tamp_msgs::tampSimulationReq& msg);
 	void simulateifResp(const tamp_msgs::tampSimulationReq& msg);
 	void checkifResCommand(agents_tasks& agent);
 	bool sendFranka2ToSimulation(std::vector<double> joint_poses);
 	bool sendFranka1ToSimulation(std::vector<double> joint_poses);
 	int findDifObjects(std::vector<string> &z1,std::vector<string> &z2);
 	void solveHanoi(int n);
 	void towerOfHanoi(int n, char from_rod,char to_rod, char aux_rod);
 	void chekPPCommand(agents_tasks& agent);
 	void simulateUpdateBenchmarkScene(const tamp_msgs::tampSimulationReq& msg);
 	void updateBenchCommand(agents_tasks& agent);
 	void simualteRemovingObj(const tamp_msgs::tampSimulationReq& msg);
 	void registerPPCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent);
 	void simulateCheckifDone(const tamp_msgs::tampSimulationReq& msg);
 	void checkIfDoneTamp(agents_tasks& agent);

 	int diskdisplaced_;
 	int numberofdiks_;
 	string benchmark_;
    bool allready_assigned_ = false;
	string lastagent_,lastState_;
	bool rightarmcuffed_,rightarmuncuffed_,leftarmcuffed_,leftarmuncuffed_;
	int previousleftcuffstate_,previousrightcuffstate_;
	double dis_,drot_,ptol_,rtol_;
	ros::NodeHandle nh;
	ros::Subscriber leftArmSub,humaninsertSub,lastStateSub;
	ros::Subscriber rightArmSub;
	ros::Subscriber simulationCommandSub,humanactionSub;
	ros::Subscriber actuationCommandSub,rightCuffCommandSub,leftCuffCommandSub;
	ros::Publisher  leftArmPub,exitpub1,exitpub2;
	ros::Publisher  rightArmPub,robotdisplaypub;
	ros::Publisher simulationResponsePub1,humanactionpub,simulationResponsePub2,simulationResponsePub;
	ros::Publisher controlCommandPub,doneactionsofkiethtetic,kinecttodisplayPub;
	ros::Publisher robotAckPub1,eliminateObject,taskDone,robotAckPub2,robotAckPub;
	ros::ServiceClient tampKnowledgeClient, tampRegisterClient,lastGraphClient,tampRegisterPlaceClient,panda1freeplan,panda2freeplan,tampKnowledgePandaClient;
	ros::ServiceClient tampMotionPlannerClient,tampMotionAckClient,tampGripperSimulationClient,tampMotionPlannerClientBase,tampRemoveObjectSimulationClient,
	tampRemoveObjectClient,tampRegisterObjectPlaceClient,changePlannerClient1,changePlannerClient2,collision_object_panda1_client,collision_object_panda2_client,
	hanoiPPClient, tampMotionPlannerPanda1,tampMotionPlannerPanda2,tampSceneClientPanda,tampGripperPandaClient,panda1trajexeClient,panda2trajexeClient,tampSceneClientBenchmark;
	microseconds microSec_time;
	std::vector<double> franka1joints_,franka2joints_,franka1jointerrors_,franka2jointerrors_;
	ros::Subscriber franka1jointsub,franka2jointsub;
	ros::Publisher vrep_franka1_pub,vrep_franka2_pub;
	sensor_msgs::JointState cmdmsgpanda1_,cmdmsgpanda2_;
	arm_control_client_Ptr panda1ControlClient;
	bool notrespanymore =false;
	int nuexecutionpanda1,nuexecutionpanda2;
	double executinTpanda1,executinTpanda2;
	map<char,int> keymap;
	map<string,int> keymap_cubeworld;
	std::vector<int> veciter;
	//std::vector<moveit_msgs/RobotTrajectory> solvedtrajectropanda1,solvedtrajectropanda2;
   // actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripperActionClinetRight,gripperActionClinetLeft;
};

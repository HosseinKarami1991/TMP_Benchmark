//#include "tamp_msgs/sceneobjects.h"
#include"world.h"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "tamp_msgs/sceneobjects.h"
#include "tamp_msgs/knowledge.h"
#include "tamp_msgs/registerdata.h"
#include <boost/algorithm/string.hpp>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
#include "tamp_msgs/ackquest.h"
#include "tamp_msgs/registerplace.h"
#include <math.h>
#include <tamp_msgs/objectssrv.h>
#include<std_msgs/Int16.h>
#include <tamp_msgs/targetsrv.h>
#include<tamp_msgs/removeobject.h>
#include <tamp_msgs/registerdata.h>
#include <tamp_msgs/updatescene.h>
#include <tamp_msgs/pandaee.h>
#include <tamp_msgs/hanoipp.h>
#include <map>
#include <stdlib.h>
#include <time.h>

#define RST "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYLW  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KCYN  "\x1B[36m"

#define FRED(x)  KRED x RST
#define FGRN(x)  KGRN x RST
#define FBLU(x)  KBLU x RST
#define FYLW(x)  KYLW x RST
#define FCYN(x)  KCYN x RST
#define BOLD(x)  "\x1B[1m" x RST

using namespace std;


class tamp_knowledge
{
public:
	tamp_knowledge();
	~tamp_knowledge();
private:
	void readDataBase();
	bool knowledgeQuery(tamp_msgs::knowledge::Request& request, tamp_msgs::knowledge::Response& response);
	bool knowledgeQueryPanda(tamp_msgs::knowledge::Request& request, tamp_msgs::knowledge::Response& response);
	bool registerData(tamp_msgs::registerdata::Request& request,tamp_msgs::registerdata::Response& response);
	bool registerPlace(tamp_msgs::registerplace::Request &req,tamp_msgs::registerplace::Response &res);
	bool removeObject(tamp_msgs::removeobject::Request &req,tamp_msgs::removeobject::Response &res);
	bool registerObjectPlace(tamp_msgs::registerplace::Request &req,tamp_msgs::registerplace::Response &res);
	bool sceneQuery();
	bool robotStateQuery();
	void findClosestObject();
	void lookUpInDatabase();
	void updateScene();
	void updateDataBase(std::vector<std_msgs::String> objtypes,std::vector<geometry_msgs::Transform> objposes);
    void updatemetrics();
    int minEucleadan(std::vector<std::vector<float>> objs,std::vector<float> des);
    void findClosestObjects();
    void updatesceneItself();
    void eliminateCB(std_msgs::String::ConstPtr msg);
    void callBackkinect(const std_msgs::String::ConstPtr& msg);
    void kinetcCbObjectId(const std_msgs::String::ConstPtr& msg);
    void updatePlaces();
    void updatesimulationscene();
    void updateSimulationObjectsMetric();
    void findClosests(std::vector<string> &objects);
    void findApprochingpoints();
    void removeDataBaseInstant(string nameins,bool all);
    void humanAcionCB(const std_msgs::Int16 &msg);
    void objectInsertedCB(const std_msgs::Int16 &msg);
    bool updateSceneServer(tamp_msgs::updatescene::Request &req,tamp_msgs::updatescene::Response &res);
    //bool updatescenofsimulation(tamp_msgs::objectssrv::Request&,tamp_msgs::objectssrv::Response&);
    void createobjectCriteria(int robotnu,int targetnu,
	const std::vector<string> &objectsinpolygon,const geometry_msgs::Pose eepos);
	int minEucleadan(std::vector<std::vector<double>> objs,std::vector<double> des);
	void removeAndAddDataBaseInstant(string nameins,string name2);
    void updateGraspApproachingInstant(string name,const std::vector<double> &pos);
    void updatePlacingPositions(int robotnu);
    bool removeObjectPanda(tamp_msgs::removeobject::Request &req,tamp_msgs::removeobject::Response &res);
    std::vector<int> findClosestPanda(std::vector<std::vector<double>> &poses);
    bool updateSceneServerBenchmark(tamp_msgs::updatescene::Request &req,tamp_msgs::updatescene::Response &res);
    void updateGraspingHanoi(int na,int nb,int nc);
    bool tampHanoiPP(tamp_msgs::hanoipp::Request &req,tamp_msgs::hanoipp::Response &res);
    void updateGraspingCubeWorld(std::vector<string> &v1,std::vector<string> &v2,std::vector<string> &v3);
    void printMap(std::map<int,string> themap);
    void updateGraspingSort(std::vector<string> &v1,std::vector<string> &v2);
    void updateGraspingNonMonotonic(std::vector<string> &v1,std::vector<string> &v2);
    void updateGraspingKitchen(std::vector<string> &v1,std::vector<string> &v2);
    std::map<int, std::string> cmap;
    std::map<std::string,int> vlauemap;

    string benchmark_;
	std::vector<world> dataBase_;
	ros::NodeHandle nh;
	ros::Publisher pubToRobotDisplay;
	ros::Subscriber objectelominate,kinectcb,kinetcCbObjectIdSub,humanActionSub,objectInsertedSub;
	ros::ServiceServer knowledgeServer,knowledgeServerPanda,registerServer,registerPlaceServer
	,removeObjectServer,registerObjectPlaceServer,updateSceneServerPanda,removeObjectServerPanda,updateSceneServerbenchmark,hanoiPPServer;
	ros::ServiceClient sceneClient,simulationSceneClient,simulationTargetClient,BenchmarkPoseClinet;
	ros::ServiceClient robotStateClient,objectServiceClient,pandaEEClient,collision_object_panda1_client,collision_object_panda2_client;
	bool sceneupdated_;
	int nuofobj_,numberofdiks_;
	
};

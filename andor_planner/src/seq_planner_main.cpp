//===============================================================================//
// Name          : Hossein Karami
// Author(s)	 :  karami.hossein1991@gamil.com
// Affiliation   : University of Genova, Italy - dept. DIBRIS
// Version		 :
// Description   : This work is based on the codes developed by https://github.com/kouroshD
//===============================================================================//


#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include <boost/shared_ptr.hpp>
#include "seq_planner_class.hpp"
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include "tamp_msgs/resetsrv.h"
#include"std_msgs/Bool.h"



#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST
std::vector<string> actionsoflastgraph;
bool doneorno;
bool switchplannerToColab_,switchplannerToNormal_;
string plannerType = "normal";
bool inflow = true;
bool intransition = false;
using namespace std;
string benchmark;
double offlineElapse,offlineoveralltime,onlineElapse,onlineoveralltime;

bool lastGraphQuery(tamp_msgs::lastgraph::Request &req,tamp_msgs::lastgraph::Response & res){


      if(req.update){
      		ROS_INFO("Last graph data query received");

      		res.results = actionsoflastgraph;
      		return true;

      }
      else{
      		ROS_INFO("Last graph data query received, But couldn't respond it :(");
      		return false;
      }


}



void doneCb(std_msgs::Bool msg);
void doneCb(std_msgs::Bool msg){
	ROS_INFO("Arrived Victory msg :)");
   doneorno = msg.data;
    

}
void switchPlannerCB(const std_msgs::String& msg){

	ROS_INFO("Request for switching planner to %s",msg.data.c_str());
       if(msg.data=="collaborative"){
         switchplannerToColab_=true;
         switchplannerToNormal_=false;
       }
       else{
        
        switchplannerToNormal_=true;
        switchplannerToColab_=false;

       }

	



}

int main(int argc, char **argv)
{


	ros::init(argc, argv, "seq_planner");
	ros::NodeHandle nh;

    shared_ptr<seq_planner_class> plan_obj = make_shared<seq_planner_class>();
	ros::Rate loop_rate(80);

	if( nh.getParam("/benchmark",benchmark)){
    	ROS_INFO("Benchmark %s is taken from ros parameter",benchmark.c_str());
    }
    else{
    	benchmark = argv[1];
    	ROS_INFO("Benchmark %s is taken from input arg",benchmark.c_str());
    }
	
	string benchmarkColab="collaborative";
	const char* home=getenv("HOME");
	string assemblyPath(home);
	string assemblyPathNormal = assemblyPath + "/catkin_ws/src/andor_planner/files/tmp_benchmarks/"+benchmark;
    string assemblyPathcollaborative = assemblyPath + "/catkin_ws/src/andor_planner/files/kinesthetic/collaborative";


	vector<vector<string>> gen_Feasible_state_list;
	vector<int> gen_Feasible_stateCost_list;

    ros::ServiceServer lastgraphdata = nh.advertiseService("last_graph_service",&lastGraphQuery);
	ros::ServiceClient andorSRV_client = nh.serviceClient<andor_msgs::andorSRV>("andorService");
    ros::ServiceClient resetClient = nh.serviceClient<tamp_msgs::resetsrv>("resetandorService");
    ros::Subscriber finishedTask =nh.subscribe("task_done",80,doneCb);
    ros::Subscriber switchPlanner =nh.subscribe("switch_planner",1,switchPlannerCB);
    ros::Publisher publasroptimastate = nh.advertise<std_msgs::String>("last_state",80);
    ros::Publisher robotdisplay = nh.advertise<std_msgs::String>("/robotDisplayText",10);


	int responsibleAgent=0; 
	int ack_agent=0;

	bool isGraphSolved=false;
	std::size_t nutp=1;
	switchplannerToColab_ = switchplannerToNormal_ = false;
	bool firsttime_ = true;
	while (ros::ok())
	{		
            std_msgs::String lastmsg;
            lastmsg.data= plan_obj->lastOptimalState_;
			publasroptimastate.publish(lastmsg);


          if(switchplannerToColab_ ){
          	actionsoflastgraph.clear();
          	actionsoflastgraph = plan_obj->getActionforState(plan_obj->lastOptimalState_);
          	cout<<"last unsuccessful actions of state : "<<plan_obj->lastOptimalState_<<endl;
          	std::string dispdata ="";
          	for(size_t i=0;i<actionsoflastgraph.size();i++){
          		cout<<(i+1)<<" : "<<actionsoflastgraph[i]<<endl;
          		dispdata +=actionsoflastgraph[i];
          		dispdata += "+";
          	}
          	std_msgs::String msgg;
          	msgg.data= dispdata;
            robotdisplay.publish(msgg);
            plan_obj.reset(new seq_planner_class);
            plan_obj->loadFiles(assemblyPathcollaborative, benchmarkColab);
            plannerType = "collaborative";
            switchplannerToColab_ =false;
            cout<<"Now Plane name is: "<<plan_obj->assembly_name<<endl;
          }

          else if(firsttime_||switchplannerToNormal_)
          {
          	double timeNow1 = ros::Time::now().toSec();
            plan_obj.reset(new seq_planner_class);
            plan_obj->loadFiles(assemblyPathNormal, benchmark);
            double timeNow2 = ros::Time::now().toSec();
            plannerType = "normal";
            firsttime_=false;
            switchplannerToNormal_=false;
            std_msgs::String msgg;
          	msgg.data= "NormalGraph";
            robotdisplay.publish(msgg);
            cout<<"Now Plane name is: "<<plan_obj->assembly_name<<endl;
            offlineElapse=timeNow2-timeNow1;
			cout<<"ofline time: "<<offlineElapse<<endl;
            offlineoveralltime +=offlineElapse;
            cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;

          }

	 
		 if(doneorno){
		 	ROS_INFO("TASK AND MOTION PLANNING DONE SUCCESSFULLY");
		 	break;



		 }
         


         if(plan_obj->needtoreset_ ){
            nutp++;
         	tamp_msgs::resetsrv msg;
         	msg.request.reset = true;
         	msg.request.planname = plannerType;
            if(resetClient.call(msg)){

            	ROS_INFO("Resetting andor graph");
             


            }
         
             plan_obj->needtoreset_=false;
             plan_obj->updateAndor=true;
			} 

               
		if (plan_obj->updateAndor==true && count>0)
		{   
			
         	double timeNow1 = ros::Time::now().toSec();
			andor_msgs::andorSRV andor_srv;

			andor_srv.request.graphName=plan_obj->AndOrUpdateName;
			if(plan_obj->nodeSolved==true)
			{

				for(int i=0; i<plan_obj->Solved_node_list.size(); i++)
				{
					andor_msgs::Node solvedNode;
					solvedNode.nodeName=plan_obj->Solved_node_list[i][0];
					solvedNode.graphName=plan_obj->Solved_node_list[i][1];

					andor_srv.request.solvedNodes.push_back(solvedNode);
				}
				plan_obj->Solved_node_list.clear();
				plan_obj->nodeSolved=false;
			}

			if(plan_obj->haSolved==true)
			{
				for(int i=0; i<plan_obj->Solved_hyperarc_list.size(); i++)
				{
					andor_msgs::Hyperarc solvedHA;
					solvedHA.hyperarcName=plan_obj->Solved_hyperarc_list[i][0];
					solvedHA.graphName=plan_obj->Solved_hyperarc_list[i][1];
					andor_srv.request.solvedHyperarc.push_back(solvedHA);
				}
				plan_obj->Solved_hyperarc_list.clear();
				plan_obj->haSolved=false;
			}

			if (andorSRV_client.call(andor_srv))
			{

				isGraphSolved=andor_srv.response.graphSolved;


					for (int i=0;i<andor_srv.response.feasibleNodes.size();i++)
					{

						vector<string>Feasible_state;
						string feasbible_state_name=andor_srv.response.feasibleNodes[i].nodeName;
						string andorNameHierarchy=andor_srv.response.feasibleNodes[i].graphName;

						Feasible_state.push_back(feasbible_state_name);
						Feasible_state.push_back("Node");
						Feasible_state.push_back(andorNameHierarchy);

						int cost=andor_srv.response.feasibleNodes[i].nodeCost;
						gen_Feasible_stateCost_list.push_back(cost);
						gen_Feasible_state_list.push_back(Feasible_state);
					}
					for (int i=0;i<andor_srv.response.feasibleHyperarcs.size();i++)
					{
						vector<string>Feasible_state;
						string feasbible_state_name=andor_srv.response.feasibleHyperarcs[i].hyperarcName;
						string andorNameHierarchy=andor_srv.response.feasibleHyperarcs[i].graphName;

						Feasible_state.push_back(feasbible_state_name);
						Feasible_state.push_back("Hyperarc");
						Feasible_state.push_back(andorNameHierarchy);

						int cost=andor_srv.response.feasibleHyperarcs[i].hyperarcCost;
						gen_Feasible_stateCost_list.push_back(cost);
						gen_Feasible_state_list.push_back(Feasible_state);
					}
					plan_obj->updateAndor=false;
					plan_obj->GenerateStateActionTable(gen_Feasible_state_list,gen_Feasible_stateCost_list,plan_obj->AndOrUpdateName, isGraphSolved );
					gen_Feasible_state_list.clear();
					gen_Feasible_stateCost_list.clear();


			}

			double timeNow2 = ros::Time::now().toSec();
			onlineElapse=timeNow2-timeNow1;
			cout<<"online time: "<<onlineElapse<<endl;
            onlineoveralltime +=onlineElapse;
            cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
		}


         


		


		if (count==0){	
			usleep(0.5e6);
		 }
		loop_rate.sleep();
		count++;
		ros::spinOnce();
	}

	cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;
	cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
	return 1;

}   


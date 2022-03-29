//===============================================================================//
// Name          : Hossein Karami
// Author(s)	 :  karami.hossein1991@gamil.com
// Affiliation   : University of Genova, Italy - dept. DIBRIS
// Version		 : Task planner for multi agent systems.
// Description   : This work is based on the codes developed by https://github.com/kouroshD
//===============================================================================//

#include <boost/thread/thread.hpp>
#include <atomic>
#include <csignal>

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
#include <thread>
#include <std_msgs/Empty.h>
#include <signal.h>
#include <tamp_msgs/change_planner.h>
#include <tamp_msgs/changeplansrv.h>
using namespace std;

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
string plannerType = "panda1";
bool inflow = true;
bool intransition = false;
int planner_index=0;
const char* home=getenv("HOME");
string home_add(home);
double offlineElapse,offlineoveralltime,onlineElapse,onlineoveralltime;
static std::atomic<bool> done(false);
bool addsecondcolomn = false;
bool change_graph = false;
string change_planner_name;
int colomn_number;

ros::ServiceServer lastgraphdata,change_planner_server;

ros::ServiceClient andorSRV_client,change_graphClient ;
ros::ServiceClient resetClient ;
ros::Subscriber finishedTask;
ros::Subscriber switchPlanner;
ros::Publisher publasroptimastate ;
ros::Publisher robotdisplay ;
//boost::mutex mymu;

shared_ptr<seq_planner_class> planner2,planner1;

void changePlannerCB(const tamp_msgs::change_planner & msg){

	if(msg.planner_name!=""){
		change_graph = true;
	}
	change_planner_name = msg.planner_name;
	colomn_number = msg.colomn;
}



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


bool changePlannerServ(tamp_msgs::changeplansrv::Request&req,tamp_msgs::changeplansrv::Response&res){
    ROS_INFO("Received a request to change planner %s in colomn %d",req.plan_name.c_str(),req.colomn);
	if(req.colomn==1){
		string planner1_name=req.plan_name;
		string planner1_path = home_add + "/catkin_ws/src/andor_planner/files/panda/"+req.plan_name;
		tamp_msgs::changeplansrv graphchangesrv;
		graphchangesrv.request = req;
		if(change_graphClient.call(graphchangesrv)){
			if(graphchangesrv.response.result){
				planner1.reset(new seq_planner_class);
	    		planner1->loadFiles(planner1_path, planner1_name);
			}

		}
		
	}
	else if(req.colomn==2){

		string planner2_name=req.plan_name;
		string planner2_path = home_add + "/catkin_ws/src/andor_planner/files/panda/"+req.plan_name;
		tamp_msgs::changeplansrv graphchangesrv;
		graphchangesrv.request = req;
		if(change_graphClient.call(graphchangesrv)){
			if(graphchangesrv.response.result){
				planner2.reset(new seq_planner_class);
	    		planner2->loadFiles(planner2_path, planner2_name);
			}

		}

	}
	
	else{
		ROS_INFO("Requested colomn doesnt exist");
		res.result =false;
		return false;
	}
    
    res.result=true;
	return true;

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







void firstPlanner(int argc, char **argv){
	//mymu.lock();
	ros::init(argc, argv, "first_planner");
	ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>(); // error with std::make_shared<ros::NodeHandle>();
 // ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_b", 10);
	ros::Rate loop_rate(50);
	string planner1_name="main1";
	string planner1_path = home_add + "/catkin_ws/src/andor_planner/files/panda/main1";
    planner1 = make_shared<seq_planner_class>();

    vector<vector<string>> gen_Feasible_state_list;
	vector<int> gen_Feasible_stateCost_list;


	/*! 	AGENTS		*/
	int responsibleAgent=0; //! it defines which agent is responsible at each moment, 0: human, 1: robot (in this example we have two agents)
	int ack_agent=0;		//! it shows which agent returns an acknowledgment at this time:( 0: human, 1: robot )

	/*! ANDOR graph update flags and parameters*/
	bool isGraphSolved=false; // if it is true the hri task is done 6259
	int count1=0;
	std::size_t nutp1=1;
	bool firsttime = true;
	while (ros::ok())
	{
	 // ROS_INFO("first loop*******************");		
			if(doneorno){
			 	ROS_INFO("TASK AND MOTION PLANNING DONE SUCCESSFULLY");
			 	break;
		    }

            std_msgs::String lastmsg;
            lastmsg.data= planner1->lastOptimalState_;
			publasroptimastate.publish(lastmsg);

         if(firsttime){

	          	double timeNow1 = ros::Time::now().toSec();
	            //shared_ptr<seq_planner_class> new_planner = make_shared<seq_planner_class>(*planner1_virgin);
	          	planner1.reset(new seq_planner_class);
	            planner1->loadFiles(planner1_path, planner1_name);
	            planner_index = 0;
	           
	            plannerType = planner1_name;
	            
	            std_msgs::String msgg;
	          	msgg.data= planner1_name;
	            robotdisplay.publish(msgg);
	            cout<<"Now Plane name is: "<<planner1->assembly_name<<endl;
	            double timeNow2 = ros::Time::now().toSec();
	            offlineElapse=timeNow2-timeNow1;
				cout<<"ofline time first time: "<<offlineElapse<<endl;
	            offlineoveralltime +=offlineElapse;
	            cout<<"overall offline time  for first time is: "<<offlineoveralltime<<" Seconds"<<endl;
                firsttime=false;
          }
          /*
          else if(change_graph && colomn_number==1){

          		double timeNow1 = ros::Time::now().toSec();
	          	planner1.reset(new seq_planner_class);
	            planner1->loadFiles(planner1_path, change_planner_name);
	           // planner_index = 0;
	            plannerType = change_planner_name;
	            
	            cout<<"Now Plane name is: "<<planner1->assembly_name<<endl;
	            double timeNow2 = ros::Time::now().toSec();
	            offlineElapse=timeNow2-timeNow1;
				cout<<"ofline time: "<<offlineElapse<<endl;
	            offlineoveralltime +=offlineElapse;
	            cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;
	            change_graph = false;

          } 
		 
         */
         if(planner1->needtoreset_ ){
            nutp1++;
         	tamp_msgs::resetsrv msg;
         	msg.request.reset = true;
         	msg.request.planname = plannerType;
            if(resetClient.call(msg)){
            	ROS_INFO("Resetting andor graph %s",planner1_name);
            }
             
             planner1->needtoreset_=false;
             planner1->updateAndor=false;
			} 

               
		if (planner1->updateAndor==true && count1>0)
		{   
         	double timeNow1 = ros::Time::now().toSec();
			andor_msgs::andorSRV andor_srv;
			andor_srv.request.graphName=planner1->AndOrUpdateName;
			if(planner1->nodeSolved==true)
			{

				for(size_t i=0; i<planner1->Solved_node_list.size(); i++)
				{
					andor_msgs::Node solvedNode;
					solvedNode.nodeName=planner1->Solved_node_list[i][0];
					solvedNode.graphName=planner1->Solved_node_list[i][1];

					andor_srv.request.solvedNodes.push_back(solvedNode);
				}
				planner1->Solved_node_list.clear();
				planner1->nodeSolved=false;
			}

			if(planner1->haSolved==true)
			{
				for(size_t i=0; i<planner1->Solved_hyperarc_list.size(); i++)
				{
					andor_msgs::Hyperarc solvedHA;
					solvedHA.hyperarcName=planner1->Solved_hyperarc_list[i][0];
					solvedHA.graphName=planner1->Solved_hyperarc_list[i][1];
					andor_srv.request.solvedHyperarc.push_back(solvedHA);
				}
				planner1->Solved_hyperarc_list.clear();
				planner1->haSolved=false;
			}

			if (andorSRV_client.call(andor_srv))
			{
				isGraphSolved=andor_srv.response.graphSolved;


					for (int i=0;i<andor_srv.response.feasibleNodes.size();i++)
					{
						vector<string>Feasible_state;
						string feasbible_state_name=andor_srv.response.feasibleNodes[i].nodeName;
						string andorNameHierarchy=andor_srv.response.feasibleNodes[i].graphName;
						cout<<"***Feasible node: "<<feasbible_state_name<<endl;
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
						cout<<"***Feasible Hyperarc: "<<feasbible_state_name<<endl;
						Feasible_state.push_back(feasbible_state_name);
						Feasible_state.push_back("Hyperarc");
						Feasible_state.push_back(andorNameHierarchy);

						int cost=andor_srv.response.feasibleHyperarcs[i].hyperarcCost;
						gen_Feasible_stateCost_list.push_back(cost);
						gen_Feasible_state_list.push_back(Feasible_state);
					}
					planner1->updateAndor=false;
					planner1->GenerateStateActionTable(gen_Feasible_state_list,gen_Feasible_stateCost_list,planner1->AndOrUpdateName, isGraphSolved );
					gen_Feasible_state_list.clear();
					gen_Feasible_stateCost_list.clear();


			}

			double timeNow2 = ros::Time::now().toSec();
			onlineElapse=timeNow2-timeNow1;
			cout<<"online time: "<<onlineElapse<<endl;
            onlineoveralltime +=onlineElapse;
            cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
		}


 
		if (count1==0){	usleep(0.5e6); }
		loop_rate.sleep();
		count1++;
		ros::spinOnce();
		//mymu.unlock();
	}
	
}
void secondPlanner(int argc, char **argv){
	//mymu.lock();
	ros::init(argc, argv, "second_planner");
	ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>(); // error with std::make_shared<ros::NodeHandle>();
  //ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_a", 10);
	ros::Rate loop_rate(50);
	string planner2_name="main2";
    string planner2_path = home_add + "/catkin_ws/src/andor_planner/files/panda/main2";
     planner2 = make_shared<seq_planner_class>();


    vector<vector<string>> gen_Feasible_state_list;
	vector<int> gen_Feasible_stateCost_list;

   

	/*! 	AGENTS		*/
	int responsibleAgent=0; //! it defines which agent is responsible at each moment, 0: human, 1: robot (in this example we have two agents)
	int ack_agent=0;		//! it shows which agent returns an acknowledgment at this time:( 0: human, 1: robot )

	/*! ANDOR graph update flags and parameters*/
	bool isGraphSolved=false; // if it is true the hri task is done
	int count2=0;
	std::size_t nutpa2=1;
	bool firsttime_ = true;
	while (ros::ok())
	{		
			if(doneorno){
			 	ROS_INFO("TASK AND MOTION PLANNING DONE SUCCESSFULLY");
			 	break;
		    }

            std_msgs::String lastmsg;
            lastmsg.data= planner2->lastOptimalState_;
			publasroptimastate.publish(lastmsg);

         if(firsttime_){

	          	double timeNow1 = ros::Time::now().toSec();
	            //shared_ptr<seq_planner_class> new_planner = make_shared<seq_planner_class>(*planner1_virgin);
	          	planner2.reset(new seq_planner_class);
	            planner2->loadFiles(planner2_path, planner2_name);
	            planner_index = 0;
	           
	            plannerType = planner2_name;
	            firsttime_=false;
	           // std_msgs::String msgg;
	          	//msgg.data= planner2_name;
	            //robotdisplay.publish(msgg);
	            cout<<"Now Plane name is: "<<planner2->assembly_name<<endl;
	            double timeNow2 = ros::Time::now().toSec();
	            offlineElapse=timeNow2-timeNow1;
				cout<<"ofline time: "<<offlineElapse<<endl;
	            offlineoveralltime +=offlineElapse;
	            cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;
	            //while(true){
	            //	if(addsecondcolomn){
	           // 		break;
	           // 	}
	            //}
         
          }


         /*
         else if(change_graph && colomn_number==2){

      		double timeNow1 = ros::Time::now().toSec();
          	planner2.reset(new seq_planner_class);
            planner2->loadFiles(planner2_path, change_planner_name);
           // planner_index = 0;
            plannerType = change_planner_name;
            //std_msgs::String msgg;
          	//msgg.data= planner1_name;
            //robotdisplay.publish(msgg);
            cout<<"Now Plane name is: "<<planner2->assembly_name<<endl;
            double timeNow2 = ros::Time::now().toSec();
            offlineElapse=timeNow2-timeNow1;
			cout<<"ofline time: "<<offlineElapse<<endl;
            offlineoveralltime +=offlineElapse;
            cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;
            change_graph = false;

      		}
      		//else{
      		//	exit(0);
      	//	}
	 */
         
         if(planner2->needtoreset_ ){
            nutpa2++;
         	tamp_msgs::resetsrv msg;
         	msg.request.reset = true;
         	msg.request.planname = plannerType;
            if(resetClient.call(msg)){
            	ROS_INFO("Resetting andor graph %s",planner2_name);
            }
             
             planner2->needtoreset_=false;
             planner2->updateAndor=false;
			} 

               
		if (planner2->updateAndor==true && count2>0)
		{   
         	double timeNow1 = ros::Time::now().toSec();
			andor_msgs::andorSRV andor_srv;
			andor_srv.request.graphName=planner2->AndOrUpdateName;
			if(planner2->nodeSolved==true)
			{

				for(size_t i=0; i<planner2->Solved_node_list.size(); i++)
				{
					andor_msgs::Node solvedNode;
					solvedNode.nodeName=planner2->Solved_node_list[i][0];
					solvedNode.graphName=planner2->Solved_node_list[i][1];

					andor_srv.request.solvedNodes.push_back(solvedNode);
				}
				planner2->Solved_node_list.clear();
				planner2->nodeSolved=false;
			}

			if(planner2->haSolved==true)
			{
				for(size_t i=0; i<planner2->Solved_hyperarc_list.size(); i++)
				{
					andor_msgs::Hyperarc solvedHA;
					solvedHA.hyperarcName=planner2->Solved_hyperarc_list[i][0];
					solvedHA.graphName=planner2->Solved_hyperarc_list[i][1];
					andor_srv.request.solvedHyperarc.push_back(solvedHA);
				}
				planner2->Solved_hyperarc_list.clear();
				planner2->haSolved=false;
			}

			if (andorSRV_client.call(andor_srv))
			{
				isGraphSolved=andor_srv.response.graphSolved;


					for (size_t i=0;i<andor_srv.response.feasibleNodes.size();i++)
					{
						vector<string>Feasible_state;
						string feasbible_state_name=andor_srv.response.feasibleNodes[i].nodeName;
						string andorNameHierarchy=andor_srv.response.feasibleNodes[i].graphName;
						cout<<"***Feasible node: "<<feasbible_state_name<<endl;
						Feasible_state.push_back(feasbible_state_name);
						Feasible_state.push_back("Node");
						Feasible_state.push_back(andorNameHierarchy);

						int cost=andor_srv.response.feasibleNodes[i].nodeCost;
						gen_Feasible_stateCost_list.push_back(cost);
						gen_Feasible_state_list.push_back(Feasible_state);
					}
					for (size_t i=0;i<andor_srv.response.feasibleHyperarcs.size();i++)
					{
						vector<string>Feasible_state;
						string feasbible_state_name=andor_srv.response.feasibleHyperarcs[i].hyperarcName;
						string andorNameHierarchy=andor_srv.response.feasibleHyperarcs[i].graphName;
						cout<<"***Feasible Hyperarc: "<<feasbible_state_name<<endl;
						Feasible_state.push_back(feasbible_state_name);
						Feasible_state.push_back("Hyperarc");
						Feasible_state.push_back(andorNameHierarchy);

						int cost=andor_srv.response.feasibleHyperarcs[i].hyperarcCost;
						gen_Feasible_stateCost_list.push_back(cost);
						gen_Feasible_state_list.push_back(Feasible_state);
					}
					planner2->updateAndor=false;
					planner2->GenerateStateActionTable(gen_Feasible_state_list,gen_Feasible_stateCost_list,planner2->AndOrUpdateName, isGraphSolved );
					gen_Feasible_state_list.clear();
					gen_Feasible_stateCost_list.clear();


			}

			double timeNow2 = ros::Time::now().toSec();
			onlineElapse=timeNow2-timeNow1;
			cout<<"online time: "<<onlineElapse<<endl;
            onlineoveralltime +=onlineElapse;
            cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
		}


 
		if (count2==0){	usleep(0.5e6); }
		loop_rate.sleep();
		count2++;
		ros::spinOnce();
			//mymu.unlock();

	}

}


/*
void finish(int signum)
{	cout << "Interrupt signal (" << signum << ") received.\n";
    done = true;
    planner1_thread.detach();
    planner2_thread.detach();
}
*/

int main(int argc, char **argv){
	//if (!done.is_lock_free()) return 10; 
		//std::signal(SIGINT, finish);
	ros::init(argc, argv, "multi_agent_planner");
	
    // ros::AsyncSpinner spinner(3); //Choose how many threads
 //  spinner.start();
    std::thread planner1_thread(firstPlanner,argc,argv);
    std::thread planner2_thread(secondPlanner,argc,argv);
    

    ros::NodeHandle nh ;
    ros::Rate loop_rate(80);
    //ros::NodeHandle nh;
    andorSRV_client = nh.serviceClient<andor_msgs::andorSRV>("andorService");
    change_graphClient = nh.serviceClient<tamp_msgs::changeplansrv>("change_graph_service");
	lastgraphdata = nh.advertiseService("last_graph_service",&lastGraphQuery);
	change_planner_server = nh.advertiseService("change_planner_service",&changePlannerServ);
    resetClient = nh.serviceClient<tamp_msgs::resetsrv>("resetandorService");
    finishedTask =nh.subscribe("task_done",80,doneCb);
    switchPlanner =nh.subscribe("switch_planner",1,switchPlannerCB);
    publasroptimastate = nh.advertise<std_msgs::String>("last_state",80);
    robotdisplay = nh.advertise<std_msgs::String>("/robotDisplayText",10);
    ros::Subscriber changeplannersub = nh.subscribe("change_planner",2,changePlannerCB);

    
    
  
    planner1_thread.join();
    planner2_thread.join();
    

	//while (ros::ok()){
   //     loop_rate.sleep();
	//    ros::spinOnce();
   // }
    

	cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;
	cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
	return 0;

}   


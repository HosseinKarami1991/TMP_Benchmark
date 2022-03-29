//===============================================================================//
// Name          : Hossein Karami
// Author(s)	 :  karami.hossein1991@gamil.com
// Affiliation   : University of Genova, Italy - dept. DIBRIS
// Version		 : Task planner for multi agent systems.
// Description   : This work is based on the codes developed by https://github.com/kouroshD
//===============================================================================//

#include <boost/thread/thread.hpp>

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
#include <tamp_msgs/changeplansrv.h>
#include <tamp_msgs/change_planner.h>
#include <sensor_msgs/JointState.h>

using namespace std;

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST
ros::Publisher pubtopanda2;
std::vector<string> actionsoflastgraph;
bool doneorno;
bool switchplannerToColab_,switchplannerToNormal_;
string plannerType = "normal";
bool inflow = true;
bool intransition = false;
int planner_index=0;
const char* home=getenv("HOME");
string home_add(home);
double offlineElapse,offlineoveralltime,onlineElapse,onlineoveralltime;
shared_ptr<seq_planner_class> planner2;
ros::ServiceClient change_graphClient ;

ros::Publisher pubchngegraph;
bool exitplanner = false;
void exitCB(const std_msgs::Bool& msg){
	if(msg.data){
		ROS_INFO("Exiting planner 1");
		exitplanner = true;
	}
	else{

	}
}
void chagnePlanCB(const tamp_msgs::change_planner &msg){
	ROS_INFO("received change planner1 msg");
	string planner2_name=msg.planner_name;
	string planner2_path = home_add + "/catkin_ws/src/andor_planner/files/panda/"+msg.planner_name;
	planner2.reset(new seq_planner_class);
	planner2 = std::make_shared<seq_planner_class>("robot2_ack","simulation2_response");
	planner2->loadFiles(planner2_path, planner2_name);
}


bool changePlannerServ(tamp_msgs::changeplansrv::Request&req,tamp_msgs::changeplansrv::Response&res){
   // ros::AsyncSpinner spinner(2);
   // spinner.start();
    ROS_INFO("Received a request to change planner %s in colomn %d",req.plan_name.c_str(),req.colomn);
    if(req.plan_name==""){
    	ROS_INFO("Exiting planner 2");
    	sensor_msgs::JointState msg;
    	msg.position = {1.0,0.0,0.0,-1.57,0.0,1.57,0.0};
    	pubtopanda2.publish(msg);
    	exitplanner = true;
    	return false;
    }
    //ros::spinOnce();
	if(req.colomn==2){
		ROS_INFO("insde if colomn 2");
	
		tamp_msgs::change_planner cmsg;
		cmsg.planner_name = req.plan_name;
		cmsg.colomn = req.colomn;

		pubchngegraph.publish(cmsg);
		
		
	}
	
	
	else{
		ROS_INFO("Requested colomn doesnt exist");
		res.result =false;
		
	}
    
    
	return true;

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












int main(int argc, char **argv){

	
	
    //ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
   	ros::init(argc, argv, "panda2_seq_planner");
	ros::NodeHandle nh;
    ros::Rate loop_rate(80);
    //ros::MultiThreadedSpinner spinner(2);
    //ros::AsyncSpinner spinner(4);
   // spinner.start();
    ros::Subscriber subtochageplan = nh.subscribe("change_planner2",1,chagnePlanCB);
    ros::ServiceClient change_graphClient = nh.serviceClient<tamp_msgs::changeplansrv>("change_graph_service");
    ros::ServiceServer change_planner_server = nh.advertiseService("change_planner2_service",&changePlannerServ);
    ros::ServiceClient andorSRV_client = nh.serviceClient<andor_msgs::andorSRV>("andorService");
	ros::ServiceServer lastgraphdata = nh.advertiseService("last_graph_service_panda2",&lastGraphQuery);
    ros::ServiceClient resetClient = nh.serviceClient<tamp_msgs::resetsrv>("resetandorService");
    ros::Subscriber finishedTask =nh.subscribe("task_done_panda1",80,doneCb);
    ros::Subscriber switchPlanner =nh.subscribe("switch_planner_panda2",1,switchPlannerCB);
    ros::Publisher publasroptimastate = nh.advertise<std_msgs::String>("last_state_panda2",80);
    
     pubchngegraph = nh.advertise<tamp_msgs::change_planner>("chage_graph",1);
     ros::Subscriber exitsub = nh.subscribe("exit_plan2",1,exitCB);
    pubtopanda2 = nh.advertise<sensor_msgs::JointState>("/franka2/joint_command",5);
 

	string planner2_name="main2";
	string planner2_path = home_add + "/catkin_ws/src/andor_planner/files/panda/main2";
     planner2 = make_shared<seq_planner_class>("robot2_ack","simulation2_response");

    vector<vector<string>> gen_Feasible_state_list;
	vector<int> gen_Feasible_stateCost_list;


	/*! 	AGENTS		*/
	int responsibleAgent=0; //! it defines which agent is responsible at each moment, 0: human, 1: robot (in this example we have two agents)
	int ack_agent=0;		//! it shows which agent returns an acknowledgment at this time:( 0: human, 1: robot )

	/*! ANDOR graph update flags and parameters*/
	bool isGraphSolved=false; // if it is true the hri task is done
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
		    if(exitplanner){
	        	break;
	        }
            std_msgs::String lastmsg;
            lastmsg.data= planner2->lastOptimalState_;
			publasroptimastate.publish(lastmsg);

         if(firsttime){

	          	double timeNow1 = ros::Time::now().toSec();
	            //shared_ptr<seq_planner_class> new_planner = make_shared<seq_planner_class>(*planner2_virgin);
	          	//planner2.reset(new seq_planner_class);
	          	planner2.reset();
				planner2 = std::make_shared<seq_planner_class>("robot2_ack","simulation2_response");
	            planner2->loadFiles(planner2_path, planner2_name);
	            planner_index = 0;
	           
	            plannerType = planner2_name;
	            
	          
	            cout<<"Now Plane name is: "<<planner2->assembly_name<<endl;
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
	          	planner2.reset(new seq_planner_class);
	            planner2->loadFiles(planner2_path, change_planner_name);
	           // planner_index = 0;
	            plannerType = change_planner_name;
	            
	            cout<<"Now Plane name is: "<<planner2->assembly_name<<endl;
	            double timeNow2 = ros::Time::now().toSec();
	            offlineElapse=timeNow2-timeNow1;
				cout<<"ofline time: "<<offlineElapse<<endl;
	            offlineoveralltime +=offlineElapse;
	            cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;
	            change_graph = false;

          } 
		 
         */
         if(planner2->needtoreset_ ){
            nutp1++;
         	tamp_msgs::resetsrv msg;
         	msg.request.reset = true;
         	msg.request.planname = plannerType;
            if(resetClient.call(msg)){
            	ROS_INFO("Resetting andor graph %s",planner2_name);
            }
             
             planner2->needtoreset_=false;
             planner2->updateAndor=false;
			} 

               
		if (planner2->updateAndor==true && count1>0)
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
					planner2->updateAndor=false;
					planner2->GenerateStateActionTable(gen_Feasible_state_list,gen_Feasible_stateCost_list,planner2->AndOrUpdateName, isGraphSolved );
					gen_Feasible_state_list.clear();
					gen_Feasible_stateCost_list.clear();


			}

			double timeNow2 = ros::Time::now().toSec();
			onlineElapse=timeNow2-timeNow1;
			//cout<<"online time: "<<onlineElapse<<endl;
            onlineoveralltime +=onlineElapse;
           // cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
		}


 
		if (count1==0){	usleep(0.5e6); }
		//loop_rate.sleep();
		count1++;
		ros::spinOnce();
		//spinner.spin();
	}
   

	cout<<"overall offline time is: "<<offlineoveralltime<<" Seconds"<<endl;
	cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
	return 1;

}   



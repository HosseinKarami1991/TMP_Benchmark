/*!
 *===============================================================================//
 * Name			 :  multiple_andor.cpp
 * Author(s)	 :  Hossein Karami
 * Affiliation   :  University of Genoa, Italy - dept. DIBRIS
 * Version       :  Multiple agent iterative AND/OR graphs
 * Description   : Main program using the core AND-OR graph is developed by Kourosh Darvish.
 *===============================================================================//
*/

#include <iostream>
#include <ros/ros.h>
#include "andor_aograph.h"
#include "std_msgs/String.h"
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include <boost/shared_ptr.hpp>
#include "tamp_msgs/resetsrv.h"
#include<std_msgs/Int16.h>
#include<std_msgs/Bool.h>
#include <tamp_msgs/changeplansrv.h>
#include <tamp_msgs/change_planner.h>

double offlineElapse;
ros::Publisher pubplan1change,pubplan2change;
int numofhumanrightarm,numofhumanleftarm,nuofresetzero,nuofresetlast,nuofresetpre;
string andor_path;
using namespace std;
string laststatestring;
shared_ptr<AOgraph> graph1_virgin, graph2_virgin;
vector<shared_ptr<AOgraph>> graphVector1,graphVector2,graphVector;// , OfflineGraphVector;
std::vector<std::vector<shared_ptr<AOgraph>>> graphNetwork;
string graph1_name ;
string graph2_name;
string CurrentAndorName_ = "normal";
double onlineElapse=0.0,kttransitiontime=0,pretransitiontime=0,newgraphtransitiontime=0;
ros::Publisher pub_ctrl_cmnd,switchPlanner,stopRobotMotion,robot_display_cmnd;
double offlineoveraltime,onlineoveralltime;
std::size_t numofandor =1;
std::size_t numofswitch =1;
void printSolveStates(std::vector<string> ,std::vector<string>,std::vector<string> ,std::vector<string>);
bool resetANDOR(tamp_msgs::resetsrv::Request &req, tamp_msgs::resetsrv::Response &res);
void switchGraphTo(string &name);

bool changeGraphSrv(tamp_msgs::changeplansrv::Request&req,tamp_msgs::changeplansrv::Response&res){

	ROS_INFO("Received request to change graph to %s in colomn %d",req.plan_name.c_str(),req.colomn);
		
		shared_ptr<AOgraph> new_graph;
		new_graph = make_shared<AOgraph>(req.plan_name);
		new_graph->loadFromFile(andor_path,req.plan_name);
		graphNetwork[req.colomn-1].push_back(new_graph);
		ROS_INFO("Added %s graph to network",req.plan_name.c_str());
		res.result = true;
		return true;
	
}







void chageGraphCB(const tamp_msgs::change_planner &msg){
		ROS_INFO("Received msg to change graph to %s in colomn %d",msg.planner_name.c_str(),msg.colomn);
		shared_ptr<AOgraph> new_graph;
		new_graph = make_shared<AOgraph>(msg.planner_name);
		new_graph->loadFromFile(andor_path,msg.planner_name);
		graphNetwork[msg.colomn-1].push_back(new_graph);
		ROS_INFO("Added %s graph to network",msg.planner_name.c_str());
		if(msg.colomn==1){
			pubplan1change.publish(msg);
		}
		else if(msg.colomn==2){
			pubplan2change.publish(msg);
		}
		

}






void humanCallBack(const std_msgs::Int16 &msg){
 
	int gesture_id = msg.data;
 	cout<<"the gesture_id is: "<<gesture_id<<endl;
	if(gesture_id==1){
		numofhumanleftarm++;
		if(graphVector.back()->gName=="normal"){
			cout<<"Last State is : "<<laststatestring<<endl;
			//if(laststatestring=="h14" || laststatestring=="h15"){
				
			//}
			//else{
				std::string name("collaborative");
				switchGraphTo(name);
			//}
			//
		}

			
		else if(graphVector.back()->gName=="collaborative"){
			//std::string name("collaborative");
			//switchGraphTo(name);
		}
	}
	else if (gesture_id==6){
		numofhumanrightarm++;
		if(graphVector.back()->gName=="normal"){
			std::string name("normallaststate");
			switchGraphTo(name);
		}
	}
 	 
}

void lastStateCb(const std_msgs::String & msg){
  
  if(msg.data!=""){
  	  laststatestring = msg.data;
  	  //cout<<"Last State is : "<<laststatestring<<endl;

  }

}

void resetGrapghToPreviousState(string &graphname){
	ROS_INFO("");
	nuofresetpre++;
	double timeNow1,timeNow2, transtime;
	timeNow1=ros::Time::now().toSec();
	
	cout<<BOLD(FRED("------------- RESETTING GRAPH TO PREVIOUS STATE -------------# "))<<graphname<<endl;

	size_t gIndex,gSize;
	for (size_t i=0;i<graphNetwork.size();i++){
		if(graphname==graphNetwork[i][0]->gName){
			gIndex=i;
			gSize = graphNetwork[i].size()-1;
		}		
	}

	std::vector<string> feasileNodeVectorstring,feasileHyperarcVectorstring;
	
	std::vector<string> solvednodesvector = graphNetwork[gIndex][gSize]->getSolvedNodes();
	
	std::vector<string> solvedarcsvector = graphNetwork[gIndex][gSize]->getSolvedHyperArcs();
	shared_ptr<AOgraph> new_graph;
	if(gIndex==0){
		new_graph = make_shared<AOgraph>(graph1_name);
		new_graph->loadFromFile(andor_path,graph1_name);
		
	}
	
	else if(gIndex==1){
		new_graph = make_shared<AOgraph>(graph2_name);
		new_graph->loadFromFile(andor_path,graph2_name);
		
	}


	ROS_INFO("New graph files loaded: %s",graphname.c_str());
	bool solvedgraph= graphNetwork[gIndex][gSize]->isGraphSolved();
	ROS_INFO("The graph is %s",(solvedgraph)?" completely solved":" NOT solved");
	if (!solvedgraph){
		//if(laststatestring!="" && withlastfeasible){
		//	ROS_INFO("Solving %s from previous graph", laststatestring.c_str());
		//	new_graph->solveByNameHyperarcMe(graphname,laststatestring);
		//}
		
		if(solvednodesvector.size()>0){
			for (size_t i=0; i<solvednodesvector.size();i++){
				new_graph->solveByNameNodeMe(graphname, solvednodesvector[i]);
			}
		}
		if(solvedarcsvector.size()>0){
			for (size_t i=0; i<solvedarcsvector.size();i++){
				new_graph->solveByNameHyperarcMe(graphname,solvedarcsvector[i]);
			}
		}
		
	}

	graphNetwork[gIndex].push_back(new_graph);
    ROS_INFO("Solved new grpah with previous solved states");
	std_msgs::String dispmsg;
    dispmsg.data = "previousstate";
	robot_display_cmnd.publish(dispmsg);

	ROS_INFO("Graph %s, added to the Tree",graphname.c_str());
	cout<<"Current Graph: "<<graphname<<endl;
	printSolveStates(solvednodesvector,solvedarcsvector,feasileNodeVectorstring,feasileHyperarcVectorstring);
	timeNow2=ros::Time::now().toSec();
	transtime=timeNow2-timeNow1;
	pretransitiontime += transtime;
}







void resetGrapghToZero(string &graphname){
    ROS_INFO("");

    double timeNow1=ros::Time::now().toSec();

    nuofresetzero++;
	cout<<BOLD(FRED("------------- RESETTING GRAPH TO ZERO -------------# "))<<graphname<<endl;

	cout<<"Current Graph: "<<graphname<<endl;
	shared_ptr<AOgraph> new_graph;
	if(graphname==graph1_name){
		new_graph =  make_shared<AOgraph>(graph1_name);
		new_graph->loadFromFile(andor_path,graph1_name);
		graphNetwork[0].push_back(new_graph);
	}
	else if(graphname==graph2_name){
		new_graph =  make_shared<AOgraph>(graph2_name);
		new_graph->loadFromFile(andor_path,graph2_name);
		graphNetwork[1].push_back(new_graph);
	}
	std_msgs::String dispmsg;
    dispmsg.data = (graphname=="normal")?"NormalGraph":"kinestheticTeaching";
	robot_display_cmnd.publish(dispmsg);
	double timeNow2=ros::Time::now().toSec();
	double transtime=timeNow2-timeNow1;
	newgraphtransitiontime += transtime;
}


void resetGrapghToLastState(string &graphname, bool withlastfeasible){
    double timeNow1=ros::Time::now().toSec();
    nuofresetlast++;
    ROS_INFO("");
	cout<<BOLD(FRED("------------- RESETTING GRAPH TO LAST STATE -------------# "))<<graphname<<endl;


	size_t gIndex,gSize;
	
	if(graphname=="r1t1" || graphname=="r1t2"){
		gIndex = 0;
		gSize = graphNetwork[0].size()-1;

	}
	else{
		gIndex = 1;
		gSize = graphNetwork[1].size()-1;
	}
	ROS_INFO("Current graph index %d , %d", gIndex,gSize);
	vector<andor_msgs::Node> feasileNodeVector;
	graphNetwork[gIndex][gSize]->getFeasibleNode(feasileNodeVector);
	vector<andor_msgs::Hyperarc> feasileHyperarcVector;
	graphNetwork[gIndex][gSize]->getFeasibleHyperarc(feasileHyperarcVector, feasileNodeVector);
	std::vector<string> feasileNodeVectorstring,feasileHyperarcVectorstring;
	for(size_t i=0;i<feasileNodeVector.size();i++){
		feasileNodeVectorstring.push_back(feasileNodeVector[i].nodeName);
	}

	for(size_t i=0;i<feasileHyperarcVector.size();i++){
		feasileHyperarcVectorstring.push_back(feasileHyperarcVector[i].hyperarcName);
	}

	std::vector<string> solvednodesvector = graphNetwork[gIndex][gSize]->getSolvedNodes();
	std::vector<string> solvedarcsvector = graphNetwork[gIndex][gSize]->getSolvedHyperArcs();
	//const char* =getenv("HOME");
	//string andor_path(home);
	//andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/";
	shared_ptr<AOgraph> new_graph;
	new_graph = make_shared<AOgraph>(graphname);
	new_graph->loadFromFile(andor_path,graphname);
	
	ROS_INFO("New graph files loaded: %s",graphname.c_str());
	bool solvedgraph= graphNetwork[gIndex][gSize]->isGraphSolved();
	ROS_INFO("The graph is %s",(solvedgraph)?" completely solved":" not solved");
	if (!solvedgraph){
		if(laststatestring!="" && withlastfeasible){
			ROS_INFO("Solving %s from previous graph", laststatestring.c_str());
			new_graph->solveByNameHyperarcMe(graphname,laststatestring);
			if(feasileNodeVectorstring.size()>0){

				for (size_t i=0; i<feasileNodeVectorstring.size();i++){
					new_graph->solveByNameNodeMe(graphname, feasileNodeVectorstring[i]);
				}

			}
			if(feasileHyperarcVectorstring.size()>0){
				for (size_t i=0; i<feasileHyperarcVectorstring.size();i++){
					new_graph->solveByNameHyperarcMe(graphname,feasileHyperarcVectorstring[i]);
				}
			}
		}
		
		if(solvednodesvector.size()>0){
			for (size_t i=0; i<solvednodesvector.size();i++){
				new_graph->solveByNameNodeMe(graphname, solvednodesvector[i]);
			}
		}
		if(solvedarcsvector.size()>0){
			for (size_t i=0; i<solvedarcsvector.size();i++){
				new_graph->solveByNameHyperarcMe(graphname,solvedarcsvector[i]);
			}
		}
		
		
	}
    ROS_INFO("Solved new grpah with previous solved states");

	graphNetwork[gIndex].push_back(new_graph);
	
	
	//std_msgs::String dispmsg;
    //dispmsg.data = (graphNetwork[gIndex].back()->gName=="normal")?"NormalGraph":"kinestheticTeaching";
	//robot_display_cmnd.publish(dispmsg);

	ROS_INFO("Graph %s, added to the Tree",graphname.c_str());
	cout<<"Current Graph: "<<graphname<<endl;
	printSolveStates(solvednodesvector,solvedarcsvector,feasileNodeVectorstring,feasileHyperarcVectorstring);
	double timeNow2=ros::Time::now().toSec();
	double transtime=timeNow2-timeNow1;
	kttransitiontime += transtime;

}


void printSolveStates(std::vector<string> solvednodesvectornew,std::vector<string> solvedarcsvectornew
					,std::vector<string> feasileNodeVectorstringnew,std::vector<string> feasileHyperarcVectorstringnew){

	    cout<<"solved node names: "<<"\t";
		if(solvednodesvectornew.size()>0){
			for(size_t i=0;i<solvednodesvectornew.size();i++){
			cout<<solvednodesvectornew[i]<<",";
			}
		}
      	cout<<" \n";
        
        cout<<"solved arc names: "<<"\t"; 
       	if(solvedarcsvectornew.size()>0){
	      	for(size_t i=0;i<solvedarcsvectornew.size();i++){
	      		cout<<solvedarcsvectornew[i]<<",";
	      	}
      	}
      	cout<<"\n";
     
      	cout<<"Feasible node names: "<<"\t";
		if(feasileNodeVectorstringnew.size()>0){
			for(size_t i=0;i<feasileNodeVectorstringnew.size();i++){
				cout<<feasileNodeVectorstringnew[i]<<",";
			}
		}
		cout<<"\n";
		cout<<"Feasible Hyperarc names: "<<"\t";
		if(feasileHyperarcVectorstringnew.size()>0){
			for(size_t i=0;i<feasileHyperarcVectorstringnew.size();i++){
				cout<<feasileHyperarcVectorstringnew[i]<<",";
			}
		}
		cout<<"\n";
		cout<<"Last Optimal State is: "<<laststatestring<<endl; 
		cout<<"\n";

}


void switchGraphTo(string &graphname){
	ROS_INFO("");
	cout<<BOLD(FBLU("------------- SWITCH GRAPH TO-------------# "))<<graphname<<endl;
	if(graphname=="normal"){
		resetGrapghToLastState(graphname,true);
		//resetGrapghToZero(graphname);
		std_msgs::String sw;
		sw.data= "normal";
		switchPlanner.publish(sw);
	}
	else if(graphname=="collaborative"){
		//publish stop robot
	    std_msgs::Bool motionbool;
	    motionbool.data = true;
	    stopRobotMotion.publish(motionbool);
		resetGrapghToZero(graphname);
		std_msgs::String sw;
		sw.data= "collaborative";
		switchPlanner.publish(sw);

	}
	else if (graphname=="normallaststate"){
		string gn("normal");
		std_msgs::Bool motionbool;
	    motionbool.data = true;
	    stopRobotMotion.publish(motionbool);
		resetGrapghToPreviousState(gn);
		//std_msgs::String sw;
		//sw.data= "normal";
		//switchPlanner.publish(sw);

	}
	else{
		ROS_ERROR("No Graph Found with the name %s",graphname);
	}


}

bool resetANDOR(tamp_msgs::resetsrv::Request &req, tamp_msgs::resetsrv::Response &res){
	cout<<BOLD(FRED("---------Request to reset And/or graph: "))<<req.planname<<endl;
	double timeNow1,timeNow2, offlineElapse;
	timeNow1=ros::Time::now().toSec();
	string requestedgraph = req.planname;
	numofandor++;
	if(requestedgraph == "r1t1" || requestedgraph == "r1t2" ||requestedgraph == "r2t1" ||requestedgraph == "r2t2"){
		if(req.reset){
			resetGrapghToLastState(requestedgraph,false);
		}
		else {

			resetGrapghToZero(requestedgraph);
		}
		
	}
	else if(requestedgraph ==graph2_name){
		if(graphNetwork[1].back()->isGraphSolved()){
			
			switchGraphTo(graph1_name);
		}
		else{
			resetGrapghToZero(requestedgraph);
		}
		
	}

	else{
		ROS_ERROR("Requested Graph to be resetted and current graph name don't conform");
	}


	timeNow2=ros::Time::now().toSec();
	offlineElapse=timeNow2-timeNow1;
	offlineoveraltime += offlineElapse;
	cout<<"Offline Overal Time is "<<offlineoveraltime<<" Seconds"<<endl;

}







bool updateANDOR(andor_msgs::andorSRV::Request &req, andor_msgs::andorSRV::Response &res){
	cout<<FRED("--------- Andor received request for: ")<<req.graphName<<endl;
	
	double timeNow1,timeNow2;
	timeNow1=ros::Time::now().toSec();
	int gIndex=0,gSize=0; //! requested graph index number in graphVector, initialize wit random big value
	string GraphName=req.graphName;
	for (size_t i=0;i<graphNetwork.size();i++)
	{   
       // cout<<"i is "<<i<<endl;
        for(int j=graphNetwork[i].size()-1;j>=0;j--){
        	//cout<<"j is "<<j<<endl;
        	if(GraphName==graphNetwork[i][j]->gName){
		    	gIndex=i;
		    	gSize =j;
		    	break;
			}
        }

       

		
	}

       cout<<"Graph index and size are : "<<gIndex<<" , "<<gSize<<endl;
       cout<<"Graph name is: "<<graphNetwork[gIndex][gSize]->gName<<endl;
	if (gIndex==-1 || gSize==-1){
		cout<<FRED("Graph Name:'"<<req.graphName<<"' Is False, Not Found in Online or Offline Graph Vectors, Check Your Graph Name You Are Requesting")<<endl;
		return false;
	}
	else
	{    //cout<<"I am in first esle"<<endl;
		if(req.solvedNodes.size()>0){

			//cout<<"I am in req.solvedNodes.size()>0"<<endl;
			for (int i=0; i<req.solvedNodes.size();i++){   
				//cout<<"*********Solved Nodes: "<<req.solvedNodes[i].nodeName<<" in graph: "<<req.solvedNodes[i].graphName<<endl;
				graphNetwork[gIndex][gSize]->solveByNameNode(req.solvedNodes[i].graphName, req.solvedNodes[i].nodeName);
			}
			std::vector<string> solvednodesvector = graphNetwork[gIndex][gSize]->getSolvedNodes();
			cout<<"getting solved nodes"<<endl;

			for(size_t i=0;i<<solvednodesvector.size();i++){
				cout<<"solved node: "<<solvednodesvector[i]<<endl;
			}

		}
		if(req.solvedHyperarc.size()>0){

			for (int i=0; i<req.solvedHyperarc.size();i++)
			{   
				//cout<<"*********Solved Hyperarcs: "<<req.solvedHyperarc[i].hyperarcName<<endl;
				graphNetwork[gIndex][gSize]->solveByNameHyperarc(req.solvedHyperarc[i].graphName,req.solvedHyperarc[i].hyperarcName);
			}

		}
		if (graphNetwork[gIndex][gSize]->isGraphSolved()){

			cout<<FGRN(BOLD("An And/Or graph is solved and deleted from vector of And/Or graphs; Name: "))<<graphNetwork[gIndex][gSize]->gName<<endl;
			
			res.graphSolved=true;
			timeNow2=ros::Time::now().toSec();
			onlineElapse+=timeNow2-timeNow1;
			cout<<"online time: "<<onlineElapse<<endl;
             onlineoveralltime +=onlineElapse;
             cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
			std_msgs::String msgData;
			msgData.data="Test_DONE";

			pub_ctrl_cmnd.publish(msgData);
		}
		else{

			//cout<<"I an in else"<<endl;
			res.graphSolved=false;
			vector<andor_msgs::Node> feasileNodeVector;
			vector<andor_msgs::Hyperarc> feasileHyperarcVector;
			graphNetwork[gIndex][gSize]->getFeasibleNode(feasileNodeVector);
			graphNetwork[gIndex][gSize]->getFeasibleHyperarc(feasileHyperarcVector, feasileNodeVector);

			for(int i=0;i<feasileHyperarcVector.size();i++){
				res.feasibleHyperarcs.push_back(feasileHyperarcVector[i]);
				//cout<<"*********Feasible Hyperarcs: "<<feasileHyperarcVector[i].hyperarcName<<endl;
			}
			for(int i=0;i<feasileNodeVector.size();i++){
				res.feasibleNodes.push_back(feasileNodeVector[i]);
				//cout<<"*********Feasible Node: "<<feasileNodeVector[i].nodeName<<endl;
			}

			timeNow2=ros::Time::now().toSec();
			onlineElapse+=timeNow2-timeNow1;
			onlineoveralltime +=onlineElapse;
			cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
			cout<<"andor offline phase time elapse: "<<offlineElapse<<" sec"<<endl;

		}

	}
    //cout<<"I an in return"<<endl;
	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "andor_network");
	ros::NodeHandle nh;

  
    robot_display_cmnd=nh.advertise<std_msgs::String>("/robotDisplayText",1);
	switchPlanner=nh.advertise<std_msgs::String>("switch_planner",80);
	stopRobotMotion=nh.advertise<std_msgs::Bool>("stop_motion",80);
    ros::Subscriber humangesture = nh.subscribe("human_action",10,humanCallBack);
    ros::Subscriber laststate = nh.subscribe("last_state",80,lastStateCb);
    ros::ServiceServer andorservice = nh.advertiseService("andorService",updateANDOR);
    ros::ServiceServer resetservice = nh.advertiseService("resetandorService",resetANDOR);
    ros::ServiceServer changeGraph = nh.advertiseService("change_graph_service",changeGraphSrv);
     pubplan1change = nh.advertise<tamp_msgs::change_planner>("change_planner1",1);
     pubplan2change = nh.advertise<tamp_msgs::change_planner>("change_planner2",1);
    ros::Subscriber subgraphchange = nh.subscribe("chage_graph",2,chageGraphCB);

    double timeNow1,timeNow2;
	timeNow1=ros::Time::now().toSec();
    const char* home=getenv("HOME");
	string andor_home(home);
    andor_path= andor_home + "/catkin_ws/src/andor_graph/andors/files/panda/";
	graph1_name = "main1";
	graph2_name = "main2";
	graph1_virgin = make_shared<AOgraph>(graph1_name);
	graph1_virgin->loadFromFile(andor_path,graph1_name);
	graph2_virgin = make_shared<AOgraph>(graph2_name);
	graph2_virgin->loadFromFile(andor_path,graph2_name);

	shared_ptr<AOgraph> graph1_instant = make_shared<AOgraph>(graph1_name);
	graph1_instant->loadFromFile(andor_path,graph1_name);
	shared_ptr<AOgraph> graph2_instant = make_shared<AOgraph>(graph2_name);
	graph2_instant->loadFromFile(andor_path,graph2_name);
	graphVector1.push_back(graph1_instant);
	graphVector2.push_back(graph2_instant);
	//graphVector1[0]->printGraphInfo();
	graphNetwork.push_back(graphVector1);
	graphNetwork.push_back(graphVector2);

	timeNow2=ros::Time::now().toSec();
	offlineElapse=timeNow2-timeNow1;
	cout<<"andor offline phase time elapse: "<<offlineElapse<<" sec"<<endl;
	cout << FGRN(BOLD("*****************")) << endl;
	cout<<FGRN(BOLD("Graphs Names: "));
	for(size_t i=0;i<graphNetwork.size();i++)
		cout<<graphNetwork[i][0]->gName<<" , ";
	cout<<endl;

	//graphNetwork[0][0]->printGraphInfo();

	
	std_msgs::String dispmsg;
    dispmsg.data = "NormalGraph";
	robot_display_cmnd.publish(dispmsg);

	cout << FGRN(BOLD("AND/OR graph is alive: ")) << endl;
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();
	cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
	cout<<"andor offline phase time elapse: "<<offlineElapse<<" sec"<<endl;
	cout<<"overall number of human left arm # : "<<numofhumanleftarm<<" Times"<<endl;
	cout<<"overall number of human right arm: "<<numofhumanrightarm<<" Times"<<endl;
	cout<<"overall number of nuofresetlast  # : "<<nuofresetlast<<" Times"<<endl;
	cout<<"overall number of nuofresetzero : "<<nuofresetzero<<" Times"<<endl;
	cout<<"overall number of nuofresetpre : "<<nuofresetpre<<" Times"<<endl;
	cout<<"overall transition for kttransitiontime time is: "<<kttransitiontime<<" Seconds"<<endl;
	cout<<"overall transition for pretransitiontime time is: "<<pretransitiontime<<" sec"<<endl;
	cout<<"overall transition for newgraphtransitiontime time is: "<<newgraphtransitiontime<<" sec"<<endl;
	return 1;

}   



/*!
 *===============================================================================//
 * Name			 :  andor_main.cpp
 * Author(s)	 :  Hossein Karami
 * Affiliation   :  University of Genoa, Italy - dept. DIBRIS
 * Version		 :  Branched AND/OR graph
 * Description   : Main program using the AND-OR graph, this program is based on codes develped by https://github.com/kouroshD
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

double offlineElapse;
string benchmark;
int numofhumanrightarm,numofhumanleftarm,nuofresetzero,nuofresetlast,nuofresetpre;
string andor_path;
using namespace std;
string laststatestring;
vector<shared_ptr<AOgraph>> graphVector;// , OfflineGraphVector;
string CurrentAndorName_ = "normal";
double onlineElapse=0.0,kttransitiontime=0,pretransitiontime=0,newgraphtransitiontime=0;
ros::Publisher pub_ctrl_cmnd,switchPlanner,stopRobotMotion,robot_display_cmnd;
double offlineoveraltime,onlineoveralltime;
std::size_t numofandor =1;
std::size_t numofswitch =1;
void printSolveStates(std::vector<string> ,std::vector<string>,std::vector<string> ,std::vector<string>);
bool resetANDOR(tamp_msgs::resetsrv::Request &req, tamp_msgs::resetsrv::Response &res);
void switchGraphTo(string &name);

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

	int lastequalgraph;
	for (int i=0;i<graphVector.size();i++){
		if(graphname==graphVector[i]->gName){
			lastequalgraph=i;
		}		
	}
	//vector<andor_msgs::Node> feasileNodeVector;
	//graphVector[lastequalgraph]->getFeasibleNode(feasileNodeVector);
	//vector<andor_msgs::Hyperarc> feasileHyperarcVector;
	//graphVector[lastequalgraph]->getFeasibleHyperarc(feasileHyperarcVector, feasileNodeVector);
	std::vector<string> feasileNodeVectorstring,feasileHyperarcVectorstring;
	//for(size_t i=0;i<feasileNodeVector.size();i++){
	//	feasileNodeVectorstring.push_back(feasileNodeVector[i].nodeName);
	//}

	//for(size_t i=0;i<feasileHyperarcVector.size();i++){
	//	feasileHyperarcVectorstring.push_back(feasileHyperarcVector[i].hyperarcName);
	//}

	std::vector<string> solvednodesvector = graphVector[lastequalgraph]->getSolvedNodes();
	std::vector<string> solvedarcsvector = graphVector[lastequalgraph]->getSolvedHyperArcs();
	//const char* =getenv("HOME");
	//string andor_path(home);
	//andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/";
	shared_ptr<AOgraph> new_graph = make_shared<AOgraph>(graphname);
	new_graph->loadFromFile(andor_path,graphname);
	ROS_INFO("New graph files loaded: %s",graphname.c_str());
	bool solvedgraph= graphVector[lastequalgraph]->isGraphSolved();
	ROS_INFO("The graph is %s",(solvedgraph)?" completely solved":" not solved");
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
    ROS_INFO("Solved new grpah with previous solved states");
	graphVector.push_back(new_graph);
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

	//const char* home=getenv("HOME");
	//string andor_path(home);
	//andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/";
	cout<<"Current Graph: "<<graphname<<endl;
	shared_ptr<AOgraph> new_graph = make_shared<AOgraph>(graphname);
	new_graph->loadFromFile(andor_path,graphname);
	graphVector.push_back(new_graph);
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


	int lastequalgraph;
	for (int i=0;i<graphVector.size();i++){
		if(graphname==graphVector[i]->gName){
			lastequalgraph=i;
		}		
	}
	vector<andor_msgs::Node> feasileNodeVector;
	graphVector[lastequalgraph]->getFeasibleNode(feasileNodeVector);
	vector<andor_msgs::Hyperarc> feasileHyperarcVector;
	graphVector[lastequalgraph]->getFeasibleHyperarc(feasileHyperarcVector, feasileNodeVector);
	std::vector<string> feasileNodeVectorstring,feasileHyperarcVectorstring;
	for(size_t i=0;i<feasileNodeVector.size();i++){
		feasileNodeVectorstring.push_back(feasileNodeVector[i].nodeName);
	}

	for(size_t i=0;i<feasileHyperarcVector.size();i++){
		feasileHyperarcVectorstring.push_back(feasileHyperarcVector[i].hyperarcName);
	}

	std::vector<string> solvednodesvector = graphVector[lastequalgraph]->getSolvedNodes();
	std::vector<string> solvedarcsvector = graphVector[lastequalgraph]->getSolvedHyperArcs();
	//const char* =getenv("HOME");
	//string andor_path(home);
	//andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/";
	shared_ptr<AOgraph> new_graph = make_shared<AOgraph>(graphname);
	new_graph->loadFromFile(andor_path,graphname);
	ROS_INFO("New graph files loaded: %s",graphname.c_str());
	bool solvedgraph= graphVector[lastequalgraph]->isGraphSolved();
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
	graphVector.push_back(new_graph);
	std_msgs::String dispmsg;
    dispmsg.data = (graphVector.back()->gName=="normal")?"NormalGraph":"kinestheticTeaching";
	robot_display_cmnd.publish(dispmsg);

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
	cout<<BOLD(FRED("------------- RESET ANDOR GRAPH -------------# "))<<numofandor<<endl;
	double timeNow1,timeNow2, offlineElapse;
	timeNow1=ros::Time::now().toSec();
	string requestedgraph = req.planname;
	numofandor++;
	if(requestedgraph ==benchmark && graphVector.back()->gName==benchmark){
		if(req.reset){
			resetGrapghToLastState(requestedgraph,false);
		}
		else {

			resetGrapghToZero(requestedgraph);
		}
		
	}
	else if(requestedgraph =="collaborative" && graphVector.back()->gName=="collaborative"){
		if(graphVector.back()->isGraphSolved()){
			std::string graph("normal");
			switchGraphTo(graph);
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void switchGraph(){

	cout<<BOLD(FBLU("------------- SWITCH ANDOR GRAPH -------------# "))<<numofswitch<<endl;
	double timeNow1,timeNow2, offlineElapse;
	
	timeNow1=ros::Time::now().toSec();
	numofswitch++;
	const char* home = getenv("HOME");
	string andor_path(home), andorName;

    if(CurrentAndorName_=="collaborative"){

        cout<<BOLD(FBLU("------------- SWITCHED TO NORMAL GRAPH  -------------# "))<<endl;
         //////////////copying previous graph data to current
		int sizofvectorgraph = graphVector.size();
		int lastnormalgraph;
		for (int i=0;i<graphVector.size();i++){
			if("normal"==graphVector[i]->gName)
			lastnormalgraph=i;
		}


        vector<andor_msgs::Node> feasileNodeVector;
        graphVector[lastnormalgraph]->getFeasibleNode(feasileNodeVector);
        vector<andor_msgs::Hyperarc> feasileHyperarcVector;
		graphVector[lastnormalgraph]->getFeasibleHyperarc(feasileHyperarcVector, feasileNodeVector);
        std::vector<string> feasileNodeVectorstring,feasileHyperarcVectorstring;
        for(size_t i=0;i<feasileNodeVector.size();i++){
          feasileNodeVectorstring.push_back(feasileNodeVector[i].nodeName);
        }

        for(size_t i=0;i<feasileHyperarcVector.size();i++){
          feasileHyperarcVectorstring.push_back(feasileHyperarcVector[i].hyperarcName);
        }

	    std::vector<string> solvednodesvector = graphVector[lastnormalgraph]->getSolvedNodes();
		std::vector<string> solvedarcsvector = graphVector[lastnormalgraph]->getSolvedHyperArcs();
		ROS_INFO("got solved vectors");
	    andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/";
		andorName="normal";
	    cout<<"Current Graph: "<<andorName<<endl;
	    string name = "normal";
	    shared_ptr<AOgraph> normal_graph = make_shared<AOgraph>(name);
	    normal_graph->loadFromFile(andor_path,andorName);
        CurrentAndorName_="normal";
        std_msgs::String dispmsg;
        dispmsg.data = "NormalGraph";
    	robot_display_cmnd.publish(dispmsg);
      	bool solvedgraph= graphVector[lastnormalgraph]->isGraphSolved();

		if (!solvedgraph){
			normal_graph->solveByNameHyperarcMe(name,laststatestring);
			if(solvednodesvector.size()>0){
				for (size_t i=0; i<solvednodesvector.size();i++){
					normal_graph->solveByNameNodeMe(name, solvednodesvector[i]);
				}
			}
			if(solvedarcsvector.size()>0){
				for (size_t i=0; i<solvedarcsvector.size();i++){
					normal_graph->solveByNameHyperarcMe(name,solvedarcsvector[i]);
				}
			}
			
		}
        vector<andor_msgs::Node> feasileNodeVectornew;
        normal_graph->getFeasibleNode(feasileNodeVectornew);
        vector<andor_msgs::Hyperarc> feasileHyperarcVectornew;
		normal_graph->getFeasibleHyperarc(feasileHyperarcVectornew, feasileNodeVectornew);
        std::vector<string> feasileNodeVectorstringnew,feasileHyperarcVectorstringnew;
        for(size_t i=0;i<feasileNodeVectornew.size();i++){
          feasileNodeVectorstringnew.push_back(feasileNodeVectornew[i].nodeName);
        }

        for(size_t i=0;i<feasileHyperarcVectornew.size();i++){
          feasileHyperarcVectorstringnew.push_back(feasileHyperarcVectornew[i].hyperarcName);
        }   
	    std::vector<string> solvednodesvectornew = normal_graph->getSolvedNodes();
        std::vector<string> solvedarcsvectornew = normal_graph->getSolvedHyperArcs();

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
		graphVector.push_back(normal_graph);


    }
    else if(CurrentAndorName_=="normal"){

    	cout<<BOLD(FBLU("------------- SWITCHED TO COLLABORATIVE GRAPH  -------------# "))<<endl;
        std_msgs::String dispmsg;
        dispmsg.data = "kinestheticTeaching";
    	robot_display_cmnd.publish(dispmsg);
        andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/"; //!< The path to the uploaded AND/OR graph
	    andorName="collaborative";//!< The name of the uploaded AND/OR graph
	    cout<<"Current Graph: "<<andorName<<endl;
	    string name = "collaborative";
	    //publish stop robot
	    std_msgs::Bool motionbool;
	    motionbool.data = true;
	    stopRobotMotion.publish(motionbool);
		graphVector.emplace_back(make_shared <AOgraph>(name));
		graphVector.back()->loadFromFile(andor_path,andorName);
		CurrentAndorName_="collaborative";

    }
    std_msgs::Bool sw;
	sw.data= true;
	switchPlanner.publish(sw);
	timeNow2=ros::Time::now().toSec();
	offlineElapse=timeNow2-timeNow1;
	offlineoveraltime += offlineElapse;
    cout<<"Offline Overal Time is "<<offlineoveraltime<<" Seconds"<<endl;


}
*////////////////////////////////////////////////////////////////////////////////////////////////
















//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*




bool resetANDOR(tamp_msgs::resetsrv::Request &req, tamp_msgs::resetsrv::Response &res){
	cout<<BOLD(FRED("------------- RESET ANDOR GRAPH -------------# "))<<numofandor<<endl;
      double timeNow1,timeNow2, offlineElapse;
       int lastsamegraph;
	timeNow1=ros::Time::now().toSec();
	  string name = req.planname;
      numofandor++;
      if(CurrentAndorName_=="collaborative"){
      	if(graphVector.back()->isGraphSolved()){
          switchGraph();
      	}

      	else if(!graphVector.back()->isGraphSolved()){
      		      	      std::vector<string> solvednodesvector,solvedarcsvector;
      
      if(name==graphVector.back()->gName){

      	solvednodesvector = graphVector.back()->getSolvedNodes();
      	solvedarcsvector = graphVector.back()->getSolvedHyperArcs();
      	ROS_INFO("The last Graph was: %s",graphVector.back()->gName.c_str());
      }
      else{
      	
      	for (int i=0;i<graphVector.size();i++)
		{
			if(name==graphVector[i]->gName)
				lastsamegraph=i;
		}
      	solvednodesvector = graphVector[lastsamegraph]->getSolvedNodes();
      	solvedarcsvector = graphVector[lastsamegraph]->getSolvedHyperArcs();
        ROS_INFO("The last Graph was: %s",graphVector[lastsamegraph]->gName.c_str());

      }
      
      ROS_INFO("got solved vectors");
      if(solvednodesvector.size()>0){
      	for(size_t i=0;i<solvednodesvector.size();i++){
      		cout<<"solved node names: "<<solvednodesvector[i]<<" "<<endl;
      	}
      }
       if(solvedarcsvector.size()>0){
      	for(size_t i=0;i<solvedarcsvector.size();i++){
      		cout<<"solved arc names: "<<solvedarcsvector[i]<<" "<<endl;
      	}
      }
      bool solvedgraph= graphVector.back()->isGraphSolved();

      cout<<"Last graph was "<<((solvedgraph)?"solved":"not completely solved")<<endl;

	 if(req.reset){
       

    cout<<"Restarting Graph : "<<name<<endl;
	graphVector.emplace_back(make_shared <AOgraph>(name));
	CurrentAndorName_=="collaborative";

	const char* home=getenv("HOME");
	string andor_path(home), andorName;
  	

	
	andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/"; //!< The path to the uploaded AND/OR graph
	andorName=name;//!< The name of the uploaded AND/OR graph
    cout<<"Current Graph: "<<andorName<<endl;
	graphVector.back()->loadFromFile(andor_path,andorName);
    
    

    	if(solvednodesvector.size()>0)
		{
			for (size_t i=0; i<solvednodesvector.size();i++)
			{
				graphVector.back()->solveByNameNodeMe(name, solvednodesvector[i]);
			}

		}
		if(solvedarcsvector.size()>0)
		{
			for (size_t i=0; i<solvedarcsvector.size();i++)
			{
				graphVector.back()->solveByNameHyperarcMe(name,solvedarcsvector[i]);
			}

		}


    

      	}
      }
      else if(CurrentAndorName_=="normal"){

      	      std::vector<string> solvednodesvector,solvedarcsvector;
      
      if(name==graphVector.back()->gName){

      	solvednodesvector = graphVector.back()->getSolvedNodes();
      	solvedarcsvector = graphVector.back()->getSolvedHyperArcs();
      	ROS_INFO("The last Graph was: %s",graphVector.back()->gName.c_str());
      }
      else{
      	
      	for (int i=0;i<graphVector.size();i++)
		{
			if(name==graphVector[i]->gName)
				lastsamegraph=i;
		}
      	solvednodesvector = graphVector[lastsamegraph]->getSolvedNodes();
      	solvedarcsvector = graphVector[lastsamegraph]->getSolvedHyperArcs();
        ROS_INFO("The last Graph was: %s",graphVector[lastsamegraph]->gName.c_str());

      }
      
      ROS_INFO("got solved vectors");
      if(solvednodesvector.size()>0){
      	for(size_t i=0;i<solvednodesvector.size();i++){
      		cout<<"solved node names: "<<solvednodesvector[i]<<" "<<endl;
      	}
      }
       if(solvedarcsvector.size()>0){
      	for(size_t i=0;i<solvedarcsvector.size();i++){
      		cout<<"solved arc names: "<<solvedarcsvector[i]<<" "<<endl;
      	}
      }
      bool solvedgraph= graphVector.back()->isGraphSolved();

      cout<<"Last graph was "<<((solvedgraph)?"solved":"not completely solved")<<endl;

	 if(req.reset){
       

    cout<<"Restarting Graph : "<<name<<endl;
	graphVector.emplace_back(make_shared <AOgraph>(name));
	CurrentAndorName_=="normal";

	const char* home=getenv("HOME");
	string andor_path(home), andorName;
  	

	
	andor_path+="/catkin_ws/src/ANDOR/andor/files/kinesthetic/"; //!< The path to the uploaded AND/OR graph
	andorName=name;//!< The name of the uploaded AND/OR graph
    cout<<"Current Graph: "<<andorName<<endl;
	graphVector.back()->loadFromFile(andor_path,andorName);
    
    if (!solvedgraph && andorName=="normal"){

    	if(solvednodesvector.size()>0)
		{
			for (size_t i=0; i<solvednodesvector.size();i++)
			{
				graphVector.back()->solveByNameNodeMe(name, solvednodesvector[i]);
			}

		}
		if(solvedarcsvector.size()>0)
		{
			for (size_t i=0; i<solvedarcsvector.size();i++)
			{
				graphVector.back()->solveByNameHyperarcMe(name,solvedarcsvector[i]);
			}

		}


    }
    



	timeNow2=ros::Time::now().toSec();
	offlineElapse=timeNow2-timeNow1;
	offlineoveraltime += offlineElapse;
     cout<<"Offline Overal Time is "<<offlineoveraltime<<" Seconds"<<endl;
	 }
	 
      }

	 return true;

}

*/////////////////////////////////////////////////////////////////////////////







bool updateANDOR(andor_msgs::andorSRV::Request &req, andor_msgs::andorSRV::Response &res){
	cout<<FRED("------------- Andor received request -------------")<<endl;
	/*! when arrive a request:
	 * 1- check for the graph name, if it is not valid, return and error
	 * 2- if the solved nodes or hyper-arcs set is empty, it responds the query by the feasible nodes/hyper-arcs and costs pair sets
	 * 3- if a node or hyper-arc is solved, it updates the graph, and responds the query by the feasible nodes/hyper-arcs and costs pair sets
	 * 4- if the root node is solved, respond by setting the graphSolved=true
	 */

	double timeNow1,timeNow2;
	// 1- check for the graph name
	timeNow1=ros::Time::now().toSec();
	int gIndex=-1; //! requested graph index number in graphVector, initialize wit random big value
	for (int i=0;i<graphVector.size();i++)
	{
		string GraphName=req.graphName;
		if(GraphName==graphVector[i]->gName)
			gIndex=i;
	}
	// Check if the graph name which is coming from the query is correct
       cout<<"Graph index is: "<<gIndex<<endl;
       cout<<"Graph name is: "<<graphVector[gIndex]->gName<<endl;
	if (gIndex==-1){
		cout<<FRED("Graph Name:'"<<req.graphName<<"' Is False, Not Found in Online or Offline Graph Vectors, Check Your Graph Name You Are Requesting")<<endl;
		return false;
	}
	// graph name is correct!
	else
	{
		if(req.solvedNodes.size()>0)
		{
			for (int i=0; i<req.solvedNodes.size();i++)
			{
				cout<<"*********Solved Nodes: "<<req.solvedNodes[i].nodeName<<endl;
				graphVector[gIndex]->solveByNameNode(req.solvedNodes[i].graphName, req.solvedNodes[i].nodeName);
			}
			std::vector<string> solvednodesvector = graphVector[gIndex]->getSolvedNodes();
			cout<<"getting solved nodes"<<endl;
			for(size_t i=0;i<<solvednodesvector.size();i++){
				cout<<"solved node: "<<solvednodesvector[i]<<endl;
			}

		}
		if(req.solvedHyperarc.size()>0)
		{
			for (int i=0; i<req.solvedHyperarc.size();i++)
			{
				cout<<"*********Solved Hyperarcs: "<<req.solvedHyperarc[i].hyperarcName<<endl;
				graphVector[gIndex]->solveByNameHyperarc(req.solvedHyperarc[i].graphName,req.solvedHyperarc[i].hyperarcName);
			}

		}
		if (graphVector[gIndex]->isGraphSolved())
		{
			cout<<FGRN(BOLD("An And/Or graph is solved and deleted from vector of And/Or graphs; Name: "))<<graphVector[gIndex]->gName<<endl;
			//vector<shared_ptr<AOgraph>>::iterator it =graphVector.begin()+gIndex;
			//graphVector.erase(it);
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
		else
		{

			res.graphSolved=false;
			vector<andor_msgs::Node> feasileNodeVector;
			vector<andor_msgs::Hyperarc> feasileHyperarcVector;
			graphVector[gIndex]->getFeasibleNode(feasileNodeVector);
			graphVector[gIndex]->getFeasibleHyperarc(feasileHyperarcVector, feasileNodeVector);

			for(int i=0;i<feasileHyperarcVector.size();i++){
				res.feasibleHyperarcs.push_back(feasileHyperarcVector[i]);
				cout<<"*********Feasible Hyperarcs: "<<feasileHyperarcVector[i].hyperarcName<<endl;
			}
			for(int i=0;i<feasileNodeVector.size();i++){
				res.feasibleNodes.push_back(feasileNodeVector[i]);
				cout<<"*********Feasible Node: "<<feasileNodeVector[i].nodeName<<endl;
			}

			timeNow2=ros::Time::now().toSec();
			onlineElapse+=timeNow2-timeNow1;
			onlineoveralltime +=onlineElapse;
			cout<<"overall online time is: "<<onlineoveralltime<<" Seconds"<<endl;
			cout<<"andor offline phase time elapse: "<<offlineElapse<<" sec"<<endl;

		}

	}

	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "andor");
	ros::NodeHandle nh;
  	
    robot_display_cmnd=nh.advertise<std_msgs::String>("/robotDisplayText",1);
	pub_ctrl_cmnd=nh.advertise<std_msgs::String>("andorTester",80);
	switchPlanner=nh.advertise<std_msgs::String>("switch_planner",80);
	stopRobotMotion=nh.advertise<std_msgs::Bool>("stop_motion",80);
    ros::Subscriber humangesture = nh.subscribe("human_action",10,humanCallBack);
    ros::Subscriber laststate = nh.subscribe("last_state",80,lastStateCb);
    if( nh.getParam("/benchmark",benchmark)){
    	ROS_INFO("Benchmark %s is taken from ros parameter",benchmark.c_str());
    }
    else{
    	benchmark = argv[1];
    	ROS_INFO("Benchmark %s is taken from input arg",benchmark.c_str());
    }
   
	//string name = "normal";
	string name = benchmark;
	graphVector.emplace_back(make_shared <AOgraph>(name));
	//const char* home=getenv("HOME");
	//string andor_path(home), andorName;
	const char* home=getenv("HOME");
	string andor_home(home);
	
	andor_path= andor_home + "/catkin_ws/src/andor_graph/andors/files/tmp_benchmark/";
	double timeNow1,timeNow2;

	timeNow1=ros::Time::now().toSec();
	//andor_path+="/catkin_ws/src/ANDOR/andor/files/tamp2/";
	//std::string andorName="normal";
	std::string andorName=benchmark;
	graphVector.back()->loadFromFile(andor_path,andorName);

	timeNow2=ros::Time::now().toSec();
	offlineElapse=timeNow2-timeNow1;

	std_msgs::String msgData;
	msgData.data="RUN_TESTER";

	ros::ServiceServer service = nh.advertiseService("andorService",updateANDOR);
    ros::ServiceServer resetservice = nh.advertiseService("resetandorService",resetANDOR);
	cout << FGRN(BOLD("*****************")) << endl;
	cout<<FGRN(BOLD("Graphs Name: "));
	for(int i=0;i<graphVector.size();i++)
		cout<<graphVector[i]->gName<<" , ";
	cout<<endl;

	std_msgs::String dispmsg;
    dispmsg.data = "NormalGraph";
	robot_display_cmnd.publish(dispmsg);



	
	cout<<"andor offline phase time elapse: "<<offlineElapse<<" sec"<<endl;
	cout << FGRN(BOLD("AND/OR graph is alive: ")) << endl;
	int count=0;
	ros::Rate loop_rate(10);
	while(ros::ok() && count<10)
	{
		if (count==8)
			pub_ctrl_cmnd.publish(msgData);
		count++;
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


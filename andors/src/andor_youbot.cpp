/*!
 *===============================================================================//
 * Name			 :  andor_main.cpp
 * Author(s)	 :  Kourosh Darvish
 * Affiliation   :  University of Genoa, Italy - dept. DIBRIS
 * Version		 :  Hierarchical, First-Order-Logic, and Standard AND/OR graph
 * Description   : Main program using the AND-OR graph
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

using namespace std;
vector<shared_ptr<AOgraph>> graphVector;// , OfflineGraphVector;
double onlineElapse=0.0;
ros::Publisher pub_ctrl_cmnd;

bool updateANDOR(andor_msgs::andorSRV::Request &req, andor_msgs::andorSRV::Response &res){
	cout<<FBLU("------------- andor receive request -------------")<<endl;
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
				graphVector[gIndex]->solveByNameNode(req.solvedNodes[i].graphName, req.solvedNodes[i].nodeName);
			}

		}
		if(req.solvedHyperarc.size()>0)
		{
			for (int i=0; i<req.solvedHyperarc.size();i++)
			{
				graphVector[gIndex]->solveByNameHyperarc(req.solvedHyperarc[i].graphName,req.solvedHyperarc[i].hyperarcName);
			}

		}
		if (graphVector[gIndex]->isGraphSolved())
		{
			cout<<FGRN(BOLD("An And/Or graph is solved and deleted from vector of And/Or graphs; Name: "))<<graphVector[gIndex]->gName<<endl;
			vector<shared_ptr<AOgraph>>::iterator it =graphVector.begin()+gIndex;
			graphVector.erase(it);
			res.graphSolved=true;
			timeNow2=ros::Time::now().toSec();
			onlineElapse+=timeNow2-timeNow1;
			cout<<"online time: "<<onlineElapse<<endl;

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

			for(int i=0;i<feasileHyperarcVector.size();i++)
				res.feasibleHyperarcs.push_back(feasileHyperarcVector[i]);
			for(int i=0;i<feasileNodeVector.size();i++)
				res.feasibleNodes.push_back(feasileNodeVector[i]);

			timeNow2=ros::Time::now().toSec();
			onlineElapse+=timeNow2-timeNow1;

		}

	}

	return true;
}


/*!
 * Andor graph service name is "andorService"
 *
 *
 * */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "andor_youbot");
	ros::NodeHandle nh;


	pub_ctrl_cmnd=nh.advertise<std_msgs::String>("andorTester",80);

	string name = "TableAssembly";
	graphVector.emplace_back(make_shared <AOgraph>(name));
	const char* home=getenv("HOME");
	string andor_path(home), andorName;


	double timeNow1,timeNow2, offlineElapse;

	timeNow1=ros::Time::now().toSec();
	andor_path+="/catkin_ws/src/ANDOR/andor/files/youbot/"; //!< The path to the uploaded AND/OR graph
	andorName="simpletask";//!< The name of the uploaded AND/OR graph

	//	andorName="TableAssembly_hierarchical";

	//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TRO-TableAssembly/10Leg/";
	//  andorName="TableAssembly";
	//	andorName="TableAssembly_hierarchical";

	//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TRO-BallBoxPlacement/";
	//	andorName="PL_BallPlacement";
	//	andorName="FOL_BallPlacment";

	//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly.txt";
	graphVector.back()->loadFromFile(andor_path,andorName);

	timeNow2=ros::Time::now().toSec();
	offlineElapse=timeNow2-timeNow1;

	//	name = "Reach_Leg1_Plate_connected";
	//	graphVector.emplace_back(make_shared <AOgraph>(name));
	//	andor_path=home;
	//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg1PlateConnected.txt";
	//	graphVector.back()->loadFromFile(andor_path);

	//	name = "Reach_Leg2_Plate_connected";
	//	graphVector.emplace_back(make_shared <AOgraph>(name));
	//	andor_path=home;
	//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg2PlateConnected.txt";
	//	graphVector.back()->loadFromFile(andor_path);
	//
	//	name = "Reach_Leg3_Plate_connected";
	//	graphVector.emplace_back(make_shared <AOgraph>(name));
	//	andor_path=home;
	//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg3PlateConnected.txt";
	//	graphVector.back()->loadFromFile(andor_path);
	//
	//	name = "Reach_Leg4_Plate_connected";
	//	graphVector.emplace_back(make_shared <AOgraph>(name));
	//	andor_path=home;
	//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg4PlateConnected.txt";
	//	graphVector.back()->loadFromFile(andor_path);


	std_msgs::String msgData;
	msgData.data="RUN_TESTER";
	//	ROS_INFO("publish msg: %s",msgData.data.c_str());

	ros::ServiceServer service = nh.advertiseService("andorService_youbot",updateANDOR);

	cout << FGRN(BOLD("*****************")) << endl;
	cout<<FGRN(BOLD("Graphs Name: "));
	for(int i=0;i<graphVector.size();i++)
		cout<<graphVector[i]->gName<<" , ";
	cout<<endl;
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
	return 1;

}   



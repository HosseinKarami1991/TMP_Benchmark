#include "tamp_interface.h"
/*
tamp_interface::tamp_interface():gripperActionClinetRight(nh,"/robot/end_effector/right_gripper/gripper_action",true),
gripperActionClinetLeft(nh,"/robot/end_effector/left_gripper/gripper_action",true){
cout<<"tamp_interface::tamp_interface()"<<endl;




  lastGraphClient = nh.serviceClient<tamp_msgs::lastgraph>("last_graph_service");
  simulationCommandSub = nh.subscribe("simulation_command",10, &tamp_interface::arrivingSimulationCommand,this);
  actuationCommandSub = nh.subscribe("robot_command",10,&tamp_interface::arrivingCommands,this);
  rightCuffCommandSub = nh.subscribe("robot/digital_io/right_lower_cuff/state",10,&tamp_interface::rightCuffCB,this);
  leftCuffCommandSub = nh.subscribe("robot/digital_io/left_lower_cuff/state",10,&tamp_interface::leftCuffCB,this);
  humanactionSub = nh.subscribe("human_action",10,&tamp_interface::humanActionCB,this);
  humaninsertSub = nh.subscribe("object_id",10,&tamp_interface::humaninsertCB,this);
  lastStateSub = nh.subscribe("last_state",10,&tamp_interface::lastStateCB,this);
  simulationResponsePub = nh.advertise<tamp_msgs::tampSimulationRes>("simulation_response",80);
  controlCommandPub	= nh.advertise<tamp_msgs::baxterControlCommand>("robot_control_command",80);
  doneactionsofkiethtetic = nh.advertise<std_msgs::String>("done_actions",12);
  robotdisplaypub = nh.advertise<std_msgs::String>("robotDisplayText",80);
  kinecttodisplayPub = nh.advertise<std_msgs::Bool>("pub_kinect",80);

  robotAckPub = nh.advertise<std_msgs::String>("robot_ack",80);
  eliminateObject = nh.advertise<std_msgs::String>("eliminate_object",80);
  humanactionpub = nh.advertise<std_msgs::Int16>("human_action",80);
  tampRemoveObjectClient= nh.serviceClient<tamp_msgs::removeobject>("tamp_remove_object_service");
  tampKnowledgeClient= nh.serviceClient<tamp_msgs::knowledge>("tamp_knowledge_service");
  tampRegisterClient= nh.serviceClient<tamp_msgs::registerdata>("tamp_register_service");
  tampRegisterPlaceClient= nh.serviceClient<tamp_msgs::registerplace>("tamp_register_place_service");
  tampRegisterObjectPlaceClient= nh.serviceClient<tamp_msgs::registerplace>("tamp_register_object_service");
  tampMotionPlannerClient= nh.serviceClient<tamp_msgs::trajquest>("tamp_motion_service");
  tampMotionAckClient= nh.serviceClient<tamp_msgs::ackquest>("tamp_ack_service");
  taskDone = nh.advertise<std_msgs::Bool>("task_done",80);

  previousleftcuffstate_=previousrightcuffstate_=0;


 




 setAgentsList();
}
*/




tamp_interface::tamp_interface(){
cout<<"tamp_interface::tamp_interface()"<<endl;

//panda1ControlClient.reset(new arm_control_client("/panda1_controller/follow_joint_trajectory"));
collision_object_panda1_client = nh.serviceClient<tamp_msgs::objectssrv>("/tamp_vrep_objects_wrtopanda1");
  collision_object_panda2_client = nh.serviceClient<tamp_msgs::objectssrv>("/tamp_vrep_objects_wrtopanda2");
 exitpub1 = nh.advertise<std_msgs::Bool>("exit_plan1",1);
 exitpub2 = nh.advertise<std_msgs::Bool>("exit_plan2",1);
 panda1trajexeClient = nh.serviceClient<tamp_msgs::executetraj>("panda1_execute");
 panda2trajexeClient = nh.serviceClient<tamp_msgs::executetraj>("panda2_execute");
  tampRemoveObjectClient= nh.serviceClient<tamp_msgs::removeobject>("tamp_removepanda_object_service");
  tampGripperPandaClient = nh.serviceClient<tamp_msgs::vrepgripper>("tamp_vrep_pandagripper_service");
  tampSceneClientPanda = nh.serviceClient<tamp_msgs::updatescene>("tamp_updatescene_panda_service");
  tampSceneClientBenchmark = nh.serviceClient<tamp_msgs::updatescene>("tamp_updatescene_benchmark_service");
  changePlannerClient1 = nh.serviceClient<tamp_msgs::changeplansrv>("change_planner1_service");
  changePlannerClient2 = nh.serviceClient<tamp_msgs::changeplansrv>("change_planner2_service");
  lastGraphClient = nh.serviceClient<tamp_msgs::lastgraph>("last_graph_service");
  simulationCommandSub = nh.subscribe("simulation_command",10, &tamp_interface::arrivingSimulationCommand,this);
  actuationCommandSub = nh.subscribe("robot_command",10,&tamp_interface::arrivingCommands,this);
  rightCuffCommandSub = nh.subscribe("robot/digital_io/right_lower_cuff/state",10,&tamp_interface::rightCuffCB,this);
  leftCuffCommandSub = nh.subscribe("robot/digital_io/left_lower_cuff/state",10,&tamp_interface::leftCuffCB,this);
  simulationResponsePub = nh.advertise<tamp_msgs::tampSimulationRes>("simulation_response",80);
  simulationResponsePub1 = nh.advertise<tamp_msgs::tampSimulationRes>("simulation1_response",80);
  simulationResponsePub2 = nh.advertise<tamp_msgs::tampSimulationRes>("simulation2_response",80);
  controlCommandPub	= nh.advertise<tamp_msgs::baxterControlCommand>("robot_control_command",80);
  doneactionsofkiethtetic = nh.advertise<std_msgs::String>("done_actions",12);
  robotdisplaypub = nh.advertise<std_msgs::String>("robotDisplayText",80);
  kinecttodisplayPub = nh.advertise<std_msgs::Bool>("pub_kinect",80);
  robotAckPub = nh.advertise<std_msgs::String>("robot_ack",80);
  robotAckPub1 = nh.advertise<std_msgs::String>("robot1_ack",80);
  robotAckPub2 = nh.advertise<std_msgs::String>("robot2_ack",80);
  eliminateObject = nh.advertise<std_msgs::String>("eliminate_object",80);
  humanactionpub = nh.advertise<std_msgs::Int16>("human_action",80);
  tampKnowledgePandaClient= nh.serviceClient<tamp_msgs::knowledge>("tamp_knowledge_panda_service");
  tampKnowledgeClient= nh.serviceClient<tamp_msgs::knowledge>("tamp_knowledge_service");
  tampRegisterClient= nh.serviceClient<tamp_msgs::registerdata>("tamp_register_service");
  tampRegisterPlaceClient= nh.serviceClient<tamp_msgs::registerplace>("tamp_register_place_service");
  tampMotionPlannerClient= nh.serviceClient<tamp_msgs::trajquest>("tamp_motion_arm_service");
  tampMotionPlannerClientBase= nh.serviceClient<tamp_msgs::trajquest>("tamp_motion_service_pr2");
  tampMotionPlannerPanda1= nh.serviceClient<tamp_msgs::trajquest>("tamp_motion_service_panda1");
  tampMotionPlannerPanda2= nh.serviceClient<tamp_msgs::trajquest>("tamp_motion_service_panda2");
  tampMotionAckClient= nh.serviceClient<tamp_msgs::ackquest>("tamp_ack_service");
  tampGripperSimulationClient = nh.serviceClient<tamp_msgs::vrepgripper>("/tamp_vrep_gripper_service");
  tampRemoveObjectSimulationClient = nh.serviceClient<tamp_msgs::removeobject>("/remove_object_scene");
  hanoiPPClient = nh.serviceClient<tamp_msgs::hanoipp>("benchmark_pp_service");
  taskDone = nh.advertise<std_msgs::Bool>("task_done",80);
 
  previousleftcuffstate_=previousrightcuffstate_=0;

  panda1freeplan = nh.serviceClient<tamp_msgs::pandafreeplan>("panda1_free_plan");
  panda2freeplan = nh.serviceClient<tamp_msgs::pandafreeplan>("panda2_free_plan");
 

   franka1jointsub = nh.subscribe(PANDA1_JOINT_STATE_TOPIC,5,&tamp_interface::callBackfrankajoint1,this);
  franka2jointsub = nh.subscribe(PANDA2_JOINT_STATE_TOPIC,5,&tamp_interface::callBackfrankajoint2,this);
  vrep_franka1_pub = nh.advertise<sensor_msgs::JointState>(PANDA1_JOINT_COMMAND_TOPIC,3);
  vrep_franka2_pub = nh.advertise<sensor_msgs::JointState>(PANDA2_JOINT_COMMAND_TOPIC,3);
  nuexecutionpanda1=0;
  nuexecutionpanda2=0;
  executinTpanda1 = 0.0;
  executinTpanda2 = 0.0;


 setAgentsList();

 nh.getParam("/benchmark",benchmark_);
 if(benchmark_=="hanoi"){
 	nh.getParam("/disks",numberofdiks_);
 	cout<<"Solving benchmark "<<benchmark_<<" with " << numberofdiks_<<" disks"<<endl;
 	solveHanoi(numberofdiks_);
 	diskdisplaced_=0;
 }
 else if(benchmark_=="cubeworld"){
 	keymap_cubeworld.insert(pair<std::string, int>("cubeA", 1));
    keymap_cubeworld.insert(pair<std::string, int>("cubeB", 2));
    keymap_cubeworld.insert(pair<std::string, int>("cubeC", 3));
    keymap_cubeworld.insert(pair<std::string, int>("cubeD", 4));
    keymap_cubeworld.insert(pair<std::string, int>("cubeE", 5));
    keymap_cubeworld.insert(pair<std::string, int>("cubeF", 6));
    keymap_cubeworld.insert(pair<std::string, int>("cubeG", 7));
    keymap_cubeworld.insert(pair<std::string, int>("cubeH", 8));
    keymap_cubeworld.insert(pair<std::string, int>("cubeI", 9));
    keymap_cubeworld.insert(pair<std::string, int>("cubeJ", 10));
 }
 else if(benchmark_=="sort"){

 }

	 if(!nh.getParam("position_tolerance",ptol_)){
	 		ptol_ = 0.005;
	 }
	 
	 if(!nh.getParam("orientation_tolerance",rtol_)){
	 	rtol_ = 0.01;
	 }
	 


}

void tamp_interface::solveHanoi(int n){
	keymap.insert(pair<char, int>('A', 1));
    keymap.insert(pair<char, int>('B', 2));
    keymap.insert(pair<char, int>('C', 3));
    towerOfHanoi(n, 'A', 'C', 'B');

}



void tamp_interface::towerOfHanoi(int n, char from_rod,char to_rod, char aux_rod)
{
    if (n == 1)
    {   
        auto itfrom = keymap.find(from_rod);
        auto itto = keymap.find(to_rod);
        cout << "Move disk 1 from rod " << from_rod <<
                            " to rod " << to_rod<<"->"<<itfrom->second<<itto->second<<endl;
        veciter.push_back(itfrom->second *10+itto->second);
        return;
    }
        
    auto itfrom = keymap.find(from_rod);
    auto itto = keymap.find(to_rod);
   
    towerOfHanoi(n - 1, from_rod, aux_rod, to_rod);
    cout << "Move disk " << n << " from rod " << from_rod <<
                                " to rod " << to_rod << "->"<<itfrom->second<<itto->second<<endl;
    veciter.push_back(itfrom->second *10+itto->second);
    towerOfHanoi(n - 1, aux_rod, to_rod, from_rod);
}






















void tamp_interface::setAgentsList(){
	cout<<"tamp_interface::setAgentsList"<<endl;
    agents_tasks agent1;
	agent1.agents.push_back("RightArm");
	agent1.agentsNumber=0;
	agent1.collaborators.clear();
	agents_list.push_back(agent1);

	agents_tasks agent2;
	agent2.agents.push_back("LeftArm");
	agent2.agentsNumber=1;
	agent2.collaborators.clear();
	agents_list.push_back(agent2);

	agents_tasks agent3;
	agent3.agents.push_back("Base");
	agent3.agentsNumber=2;
	agent3.collaborators.clear();
	agents_list.push_back(agent3);

	agents_tasks agent4;
	agent4.agents.push_back("WholeRight");
	agent3.agentsNumber=3;
	agent4.collaborators.clear();
	agents_list.push_back(agent4);

	agents_tasks agent5;
	agent5.agents.push_back("WholeLeft");
	agent5.agentsNumber=4;
	agent5.collaborators.clear();
	agents_list.push_back(agent5);




}

void tamp_interface::lastStateCB(const std_msgs::String &msg){
	if(msg.data !=""){
		lastState_ = msg.data;
	}
}


void tamp_interface::humanActionCB(const std_msgs::Int16 &msg){
	ROS_INFO("Recived human Action %d:",msg.data);
	if(msg.data==int(1) || msg.data==int(6) && (lastState_=="h14" || lastState_=="h15")){
		publishHumanActionAck(8);
	}
	
}


void tamp_interface::humaninsertCB(const std_msgs::Int16 &msg){
	ROS_INFO("Recived human object selection, object %d",msg.data);
	publishHumanActionAck(7);
}

void tamp_interface::rightCuffCB(const baxter_core_msgs::DigitalIOState & msg){
	
	auto currentstate = msg.state;
	if(currentstate!=previousrightcuffstate_){
		ROS_INFO("tamp_interface::rightCuffCB");
		if(previousrightcuffstate_ ==0 && currentstate ==1){
        rightarmcuffed_=true;
         ROS_INFO("Right Arm is Cuffed");
         publishHumanActionAck(4);
		} 
		else if(previousrightcuffstate_ ==1 && currentstate ==0){
	        rightarmuncuffed_=true;  
	        ROS_INFO("Right Arm is UnCuffed");
	        publishHumanActionAck(5);
		}
		else{
	       rightarmcuffed_=false;
	       rightarmuncuffed_=false;
		}
	}

	previousrightcuffstate_ = currentstate;

}

void tamp_interface::leftCuffCB(const baxter_core_msgs::DigitalIOState & msg){
	
	auto currentstate = msg.state;
	if(currentstate!=previousleftcuffstate_){
		ROS_INFO("tamp_interface::leftCuffCB");
		if(previousleftcuffstate_ ==0 && currentstate ==1){
        leftarmcuffed_=true;
         ROS_INFO("Left Arm is Cuffed");
	         publishHumanActionAck(4);
		} 
		else if(previousleftcuffstate_ ==1 && currentstate ==0){
	        leftarmuncuffed_=true;  
	        ROS_INFO("Left Arm is UnCuffed");
	        publishHumanActionAck(5);
		}
		else{
	       leftarmcuffed_=false;
	       leftarmuncuffed_=false;
		}
	}
	
	previousleftcuffstate_ = currentstate;

	
}


//**********************************Simulation Commands*****************************************************//

void tamp_interface::arrivingSimulationCommand(const tamp_msgs::tampSimulationReq& msg){


	cout<<BOLD(FBLU("tamp_interface::arrivingSimulationCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	// arrive the simulation command here
	// base on the arriving command call different functions
	cout<<msg.ActionName<<" ";
	for(int i=0;i<msg.ResponsibleAgents.size();i++)
		cout<<msg.ResponsibleAgents[i]<<" ";
	for(int i=0;i<msg.ColleagueAgents.size();i++)
		cout<<msg.ColleagueAgents[i]<<" ";
	for(int i=0;i<msg.ActionParametersName.size();i++)
		cout<<msg.ActionParametersName[i]<<" ";
	cout<<endl;


	string tempActionName=msg.ActionName;
	if(tempActionName=="Grasp"|| tempActionName=="UnGrasp")
		simulateGraspingCommand(msg);
	else if(tempActionName=="Stop")
		simulateStoppingCommand(msg);
	else if(tempActionName=="HoldOn")
		simulateHoldingCommand(msg);
	else if(tempActionName=="Approach"){
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulateApproachingPanda1(msg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulateApproachingPanda2(msg);
		}
		else{
			simulateApproachingCommand(msg);

		}
		
	}
	else if(tempActionName=="Rest")
		simulateRestingcommand(msg);
	else if(tempActionName=="checkifr")
		simulateCheckrcommand(msg);
	else if(tempActionName=="checkifl")
		simulateChecklcommand(msg);
	else if(tempActionName=="register")
		simulateRegisterCommand(msg);
	else if(tempActionName=="Cuff")
		simulateCuffCommand(msg);
	else if(tempActionName=="UnCuff")
		simulateUnCuffCommand(msg);
	else if(tempActionName=="CheckFlt")
		simulateFaultyCommand(msg);
	else if(tempActionName=="CheckNFlt")
		simulateNonFaultyCommand(msg);
	else if(tempActionName=="CheckNA")
		simulateNACommand(msg);
	else if(tempActionName=="checkifz1")
		simulateCheckIfZ1(msg);
	else if(tempActionName=="checkifz2")
		simulateCheckIfZ2(msg);
	else if(tempActionName=="remove")
		simulateRemoveObject(msg);
	else if(tempActionName=="updatescene")
		simulateUpdatingScene(msg);
	else if(tempActionName=="updatebench")
		simulateUpdateBenchmarkScene(msg);
	else if(tempActionName=="sendvictory")
		simulateSendVictory(msg);
	else if(tempActionName=="registerplace")
		simulateRegisterPlace(msg);
	else if (tempActionName=="objectregister")
		simulateObjectRegister(msg);
	else if(tempActionName=="CheckPi1" || tempActionName=="CheckPi2")
		simulateCheckPickObject(msg);
	else if(tempActionName=="CheckPl1" || tempActionName=="CheckPl2" || tempActionName=="CheckPl3" )
		simulateCheckPlaceObject(msg);
	else if(tempActionName=="findclob")
		simluateFingingClosestObject(msg);
	else if(tempActionName=="planfree")
		simulatePlanFree(msg);
	else if (tempActionName=="checkif")
		simulateCheckif(msg);
	else if(tempActionName=="addgraph")
		simulateAddingGraph(msg);
	else if(tempActionName=="exit"){
		simulateExit(msg);
	}
	else if(tempActionName=="checkifres"){
		simulateifResp(msg);
	}
	else if(tempActionName=="checkpp"){
		simulateCheckPP(msg);
	}
	else if(tempActionName=="removeobject"){
		simualteRemovingObj(msg);
	}
	else if (tempActionName=="registerpp"){
		simulateRegisterPP(msg);
	}
	else if(tempActionName=="checkifdone"){
		simulateCheckifDone(msg);
	}
	else
	{
		cout<<"The arriving msg name is wrong: "<<tempActionName <<endl;
		exit(1);
	}



}
void tamp_interface::simulateCheckifDone(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateCheckifDone "))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;
		
	tempResponseMsg.time=0.002;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	std::vector<string> yesorno;
	boost::split(yesorno, msg.ActionParametersName[0], boost::is_any_of("+"));
	if(yesorno[0]=="hanoi"){
		cout<<"msg.actionname is "<<msg.ActionParametersName[0]<<endl;
		tempResponseMsg.success=true;
		tamp_msgs::knowledge ksrv;
		ksrv.request.reqType= "pigC";
		std::vector<double> vc;
		if(tampKnowledgeClient.call(ksrv)){
			vc = ksrv.response.pose;
			

		}
		cout<<"Recived Vc with size "<<vc.size()<<endl;

		for (int i = 0; i < vc.size(); ++i)
		{	cout<<"vc[ "<<i<<" ]"<<vc[i]<<"\t";
			if((int) vc[i]==0){
				if(yesorno[1]=="yes"){
					tempResponseMsg.success=false;
				}
				else{
					tempResponseMsg.success=true;
				}
				
				break;
			}
		}

	}
	else if(yesorno[0]=="cubeworld"){
		tamp_msgs::knowledge ksrv;
		ksrv.request.reqType= "nextcube";
		std::vector<string> vc;
		if(tampKnowledgeClient.call(ksrv)){
			vc = ksrv.response.names;
			

		}

		if(vc.size()==1 || vc.empty()){
			cout<<"nextcube vector is empty"<<endl;
			if(yesorno[1]=="yes"){
				tempResponseMsg.success=true;
			}
			else{
				tempResponseMsg.success=false;
			}
			
		}
		else{
			cout<<"nextcube vector is NOT empty"<<endl;
			if(yesorno[1]=="no"){
				tempResponseMsg.success=true;
			}
			else{
				tempResponseMsg.success=false;
			}
		}
		
	 
	    

	}
	else if(yesorno[0]=="sort"){
		tamp_msgs::knowledge ksrv;
		ksrv.request.reqType= "table3";
		std::vector<string> v3,v4;
		if(tampKnowledgeClient.call(ksrv)){
			v3 = ksrv.response.names;
		}
		ksrv.request.reqType= "table4";
		if(tampKnowledgeClient.call(ksrv)){
			v4 = ksrv.response.names;
		}

		if(v3.size()==8 && v4.size()==8 ){
			if(yesorno[1]=="yes"){
				tempResponseMsg.success=true;
			}
			else{
				tempResponseMsg.success=false;
			}
			
		}
		else{
			if(yesorno[1]=="yes"){
				tempResponseMsg.success=false;
			}
			else{
				tempResponseMsg.success=true;
			}
		}
	}
	else if(yesorno[0]=="nonmonotonic"){
		tamp_msgs::knowledge ksrv;
		ksrv.request.reqType= "table1";
		std::vector<string> v3;
		if(tampKnowledgeClient.call(ksrv)){
			v3 = ksrv.response.names;
		}
		bool greenexits = false;
		for (int i = 0; i < v3.size(); ++i)
		{
			std::vector<string> stickcolornu;
	  		boost::split(stickcolornu, v3[i], boost::is_any_of("_"));
	  		if(stickcolornu.size()>1){
	  			if(stickcolornu[1]=="green"){
		  			greenexits = true;
		  			break;
	  			}
	  		}
	  		
		}

		if(greenexits){
			if(yesorno[1]=="yes"){
				tempResponseMsg.success=false;
			}
			else{
				tempResponseMsg.success=true;
			}
			
		}
		else{
			if(yesorno[1]=="yes"){
				tempResponseMsg.success=true;
			}
			else{
				tempResponseMsg.success=false;
			}
		}



	}

	else if(yesorno[0]=="kitchen"){
		tamp_msgs::knowledge ksrv;
		ksrv.request.reqType= "table1";
		std::vector<string> v3;
		if(tampKnowledgeClient.call(ksrv)){
			v3 = ksrv.response.names;
		}
		bool greenexits = false;
		for (int i = 0; i < v3.size(); ++i)
		{
			std::vector<string> stickcolornu;
	  		boost::split(stickcolornu, v3[i], boost::is_any_of("_"));
	  		if(stickcolornu.size()>1){
	  			if(stickcolornu[0]=="cabbage"){
		  			greenexits = true;
		  			break;
	  			}
	  		}
	  		
		}

		if(greenexits){
			if(yesorno[1]=="yes"){
				tempResponseMsg.success=false;
			}
			else{
				tempResponseMsg.success=true;
			}
			
		}
		else{
			if(yesorno[1]=="yes"){
				tempResponseMsg.success=true;
			}
			else{
				tempResponseMsg.success=false;
			}
		}



	}


	simulationResponsePub.publish(tempResponseMsg);

}

void tamp_interface::simulateRegisterPP(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateRegisterPP "))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	
	tamp_msgs::tampSimulationRes tempResponseMsg;	
	tempResponseMsg.time=0.002;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	tempResponseMsg.success=true;
	simulationResponsePub.publish(tempResponseMsg);
}

void tamp_interface::simualteRemovingObj(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simualteRemovingObj "))<<msg.ResponsibleAgents[0]<<endl;
		cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;


   //string input=commandmsg-> data.c_str();
	//vector<string> msg, msgAction,msgAgents,msgColleagues;
	//string cmndType, reachingPoint;
	//boost::split(msg, input, boost::is_any_of(" "));
	//boost::split(msgAction, msg[0], boost::is_any_of("_"));

	//boost::split(msgAgents, msg[1], boost::is_any_of("+"));
    tamp_msgs::removeobject vgmsg;
    std::vector<double> pig;
    if(benchmark_=="hanoi"){
    		if(msg.ActionParametersName[0]==""){
		vgmsg.request.object ="";
		}
		else{
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for "<<"pig"+msg.ActionParametersName[0]<<endl;
			kmsg.request.reqType= "pig"+msg.ActionParametersName[0];
			
			if(tampKnowledgeClient.call(kmsg)){
				pig = kmsg.response.pose;
				for (int i = numberofdiks_-1; i >= 0; i--)
				{
					if((int)pig[i]!=0){
						vgmsg.request.object = "disk"+ to_string((int) pig[i]);
						cout<<"last disk is "<<vgmsg.request.object<<endl;
						break;
					}
				}
				
			}

		}

    }
    else if(benchmark_=="cubeworld"){
    	tamp_msgs::knowledge kmsg;
			cout<<"requesting for "<<"removing as collision cube in tray "+msg.ActionParametersName[0]<<endl;
			if(msg.ActionParametersName[0]=="nextcube"){
				kmsg.request.reqType= "nextcube";
			}
			else if(msg.ActionParametersName[0]=="cpr"){
				kmsg.request.reqType= "smallestcube";
			}
			else if(msg.ActionParametersName[0]=="cpb"){
				kmsg.request.reqType= "biggestcube";
			}
			else{
				kmsg.request.reqType= "tray"+msg.ActionParametersName[0];
			}
			
			std::vector<string> vnames;
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				if(msg.ActionParametersName[0]=="nextcube"){
					vgmsg.request.object = vnames[1];
				}
				else{
					vgmsg.request.object = vnames.back();
				}
				
				cout<<"last cube is "<<vgmsg.request.object<<endl;
			}
    }
      else if(benchmark_=="sort"){
      	tamp_msgs::knowledge kmsg;
		cout<<"requesting for "<<"removing as collision stick"+msg.ActionParametersName[0]<<endl;
 		kmsg.request.reqType= msg.ActionParametersName[0];
 		std::vector<string> vnames;
 		if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.object = vnames.back();
				cout<<"stick is "<<vgmsg.request.object<<endl;
		}


    }
    else if(benchmark_=="nonmonotonic"){
    	tamp_msgs::knowledge kmsg;
		cout<<"requesting for "<<"removing as collision stick"+msg.ActionParametersName[0]<<endl;
 		kmsg.request.reqType= msg.ActionParametersName[0];
 		std::vector<string> vnames;
 		if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.object = vnames.back();
				cout<<"stick is "<<vgmsg.request.object<<endl;
		}

    }
    else if(benchmark_=="kitchen"){
    	tamp_msgs::knowledge kmsg;
		cout<<"requesting for "<<"removing as collision stick"+msg.ActionParametersName[0]<<endl;
 		kmsg.request.reqType= msg.ActionParametersName[0];
 		std::vector<string> vnames;
 		if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.object = vnames.back();
				cout<<"object is "<<vgmsg.request.object<<endl;
		}

    }

	
tamp_msgs::tampSimulationRes tempResponseMsg;
	
	if(tampRemoveObjectSimulationClient.call(vgmsg)){
		if(vgmsg.response.result){
			tempResponseMsg.success=true;
		}

		else{
			tempResponseMsg.success=false;
		}

	}
	else{

		tempResponseMsg.success=false;
	}

	
		
	tempResponseMsg.time=0.002;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	tempResponseMsg.success=true;
	simulationResponsePub.publish(tempResponseMsg);
}
void tamp_interface::simulateCheckPP(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateCheckPP for"))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	if(benchmark_=="hanoi"){
		std::vector<double> piga,pigb,pigc;
		tamp_msgs::knowledge ksrv;
		ksrv.request.reqType = "pigA";
		if(tampKnowledgeClient.call(ksrv)){
			piga = ksrv.response.pose;
		}
		ksrv.request.reqType = "pigB";
		if(tampKnowledgeClient.call(ksrv)){
			pigb = ksrv.response.pose;
		}
		ksrv.request.reqType = "pigC";
		if(tampKnowledgeClient.call(ksrv)){
			pigc = ksrv.response.pose;
		}
		
		int  ppnu = std::stoi( msg.ActionParametersName[0]);
		cout<<"checking for pick and place for "<<ppnu<<endl;

		

		if(ppnu==veciter[diskdisplaced_]){
			tempResponseMsg.success=true;
		}
		else{

			tempResponseMsg.success=false;
		}
	}
	else if(benchmark_=="cubeworld"){
		string pp = msg.ActionParametersName[0];
		std::vector<string> traybvec,traycvec,trayrvec,nextobjvec;
	
		tamp_msgs::knowledge ksrv;
		ksrv.request.reqType = "trayb";
		if(tampKnowledgeClient.call(ksrv)){
			traybvec = ksrv.response.names;
		}
		ksrv.request.reqType = "trayc";
		if(tampKnowledgeClient.call(ksrv)){
			traycvec = ksrv.response.names;
		}
		ksrv.request.reqType = "trayr";
		if(tampKnowledgeClient.call(ksrv)){
			trayrvec = ksrv.response.names;
		}
		ksrv.request.reqType = "nextcube";
		if(tampKnowledgeClient.call(ksrv)){
			nextobjvec = ksrv.response.names;
		}

		
		if(pp=="nob"){
			string no = nextobjvec[1];
			tempResponseMsg.success=false;
			for (int i = 0; i < traybvec.size(); ++i)
			{
				if(no==traybvec[i]){
					tempResponseMsg.success=true;
					break;
				}
			}
		}
		else if(pp=="noc"){
			string no = nextobjvec[1];
			tempResponseMsg.success=false;
			for (int i = 0; i < traycvec.size(); ++i)
			{
				if(no==traycvec[i]){
					tempResponseMsg.success=true;
					break;
				}
			}

		}
		else if(pp=="nor"){
			string no = nextobjvec[1];
			tempResponseMsg.success=false;
			for (int i = 0; i < trayrvec.size(); ++i)
			{
				if(no==trayrvec[i]){
					tempResponseMsg.success=true;
					break;
				}
			}
			
		}
		else if(pp=="es"){
			tempResponseMsg.success=false;
			if(traycvec.size()<=6){
				tempResponseMsg.success=true;
			}

		}
		else if(pp=="nes"){
			tempResponseMsg.success=false;
			if(traycvec.size()>6){
				tempResponseMsg.success=true;
			}
			
		}
		else if(pp=="cplr" || pp=="cnplr"){

			std::vector<int> vofrs;
			vofrs.push_back(0);
			bool canplace =false;
			std::vector<string> redvec;
			for (int i = 1; i < trayrvec.size(); ++i)
			{
				redvec.push_back(trayrvec[i]);
			}
			for (int i = 0; i < redvec.size(); ++i)
			{
				auto itt = keymap_cubeworld.find(redvec[i]);
				cout<<"value of cube"<<redvec[i]<<" in red tray is : "<<itt->second<<endl;
				vofrs.push_back(itt->second);
			}
			//auto itt = keymap_cubeworld.find(nextobjvec[1]);
		    //vofrs.push_back(itt->second);
		    for (int i = 0; i < vofrs.size(); ++i)
		    {
		    	cout<<"elemnts in trayc are: "<<vofrs[i]<<"\t";
		    }
		    cout<<endl;
		   // if(std::is_sorted(vofrs.begin(), vofrs.end())){
		    canplace = true;
		   // }
		    for (int i = 0; i < vofrs.size()-1; ++i)
		    {
		    	if(vofrs[i+1]-vofrs[i]!=1){
		    		cout<<"differece of cube values are : "<<vofrs[i+1]-vofrs[i]<<endl;
		    		canplace = false;
		    		break;
		    	}
		    }

		    if(pp=="cnplr"){
		    	canplace = !canplace;
		    }

		    tempResponseMsg.success=canplace;
			
			
	
			
		}

	}
	else if(benchmark_=="sort"){
		string pp = msg.ActionParametersName[0];
		if(pp == "table1" || pp == "table2"){
			std::vector<string> table1obj,table2obj;
			tamp_msgs::knowledge ksrv;
			ksrv.request.reqType = "table1";
			if(tampKnowledgeClient.call(ksrv)){
				table1obj = ksrv.response.names;
			}
			ksrv.request.reqType = "table2";
			if(tampKnowledgeClient.call(ksrv)){
				table2obj = ksrv.response.names;
			}
			if(pp == "table1"){
				if(table1obj.size()>=table2obj.size()){
					tempResponseMsg.success = true;
				}
				else{
					tempResponseMsg.success = false;
				}
			}
			else{
				if(table1obj.size()>=table2obj.size()){
					tempResponseMsg.success = false;
				}
				else{
					tempResponseMsg.success = true;
				}

			}


			

		}

		else if(pp=="blue1"){
			tamp_msgs::knowledge ksrv;
			std::vector<string> objectnam,stickcolornu;
			ksrv.request.reqType = "clob1";
			if(tampKnowledgeClient.call(ksrv)){
				objectnam = ksrv.response.names;
			}
			boost::split(stickcolornu, objectnam[1], boost::is_any_of("_"));
			if(stickcolornu[1]=="blue"){
				tempResponseMsg.success = true;
			}
			else{
				tempResponseMsg.success = false;
			}	

		}
		else if(pp=="green1"){
			tamp_msgs::knowledge ksrv;
			std::vector<string> objectnam,stickcolornu;
			ksrv.request.reqType = "clob1";
			if(tampKnowledgeClient.call(ksrv)){
				objectnam = ksrv.response.names;
			}
			boost::split(stickcolornu, objectnam[1], boost::is_any_of("_"));
			if(stickcolornu[1]=="green"){
				tempResponseMsg.success = true;
			}
			else{
				tempResponseMsg.success = false;
			}
			
		}
		else if(pp=="blue2"){
			tamp_msgs::knowledge ksrv;
			std::vector<string> objectnam,stickcolornu;
			ksrv.request.reqType = "clob2";
			if(tampKnowledgeClient.call(ksrv)){
				objectnam = ksrv.response.names;
			}
			boost::split(stickcolornu, objectnam[1], boost::is_any_of("_"));
			if(stickcolornu[1]=="blue"){
				tempResponseMsg.success = true;
			}
			else{
				tempResponseMsg.success = false;
			}
			
		}
		else if(pp=="green2"){
			tamp_msgs::knowledge ksrv;
			std::vector<string> objectnam,stickcolornu;
			ksrv.request.reqType = "clob2";
			if(tampKnowledgeClient.call(ksrv)){
				objectnam = ksrv.response.names;
			}
			boost::split(stickcolornu, objectnam[1], boost::is_any_of("_"));
			if(stickcolornu[1]=="green"){
				tempResponseMsg.success = true;
			}
			else{
				tempResponseMsg.success = false;
			}
			
		}
		


	}
	

	simulationResponsePub.publish(tempResponseMsg);



	


}

void tamp_interface::simulateifResp(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateifResp fur "))<<msg.ResponsibleAgents[0]<<endl;
		cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
		tamp_msgs::tampSimulationRes tempResponseMsg;
		
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;


		tamp_msgs::knowledge ksrv;
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
			
			ksrv.request.reqType = "task1";
			if(tampKnowledgeClient.call(ksrv)){
				if((ksrv.response.names[1]=="r1t1+r1t2" || ksrv.response.names[1]=="r1t2+r1t1") && notrespanymore==false){
					
					if(msg.ActionParametersName[0]=="not"){
						tempResponseMsg.success=false;
						
					}
					else{
						
						tempResponseMsg.success=true;
					}
					
					notrespanymore = true;
				}
				else{
					if(msg.ActionParametersName[0]=="not"){
						
						tempResponseMsg.success=true;
					}
					else{
						tempResponseMsg.success=false;
						
					}
					
				}
			}
		
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			
			ksrv.request.reqType = "task2";
			if(tampKnowledgeClient.call(ksrv)){
				if((ksrv.response.names[1]=="r2t1+r2t2" || ksrv.response.names[1]=="r2t2+r2t1") && notrespanymore==false){

					if(msg.ActionParametersName[0]=="not"){
						tempResponseMsg.success=false;
					}
					else{
						tempResponseMsg.success=true;
						
					}
					notrespanymore = true;
				}
				else{
					if(msg.ActionParametersName[0]=="not"){
						tempResponseMsg.success=true;
						
					}
					else{
						tempResponseMsg.success=false;

					}
				}
			}

		}


		
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
				simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}


}

void tamp_interface::simulateExit(const tamp_msgs::tampSimulationReq& msg){
		cout<<BOLD(FBLU("tamp_interface::simulateExit for "))<<msg.ResponsibleAgents[0]<<endl;
		cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
		tamp_msgs::tampSimulationRes tempResponseMsg;
		tempResponseMsg.success=true;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
				simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}

}
void tamp_interface::simulateApproachingPanda1(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateApproachingPanda1 for "))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	tamp_msgs::knowledge knowledge_msg;
	knowledge_msg.request.reqType=msg.ActionParametersName[0];
	knowledge_msg.request.Name="";


	std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){
		target=knowledge_msg.response.pose;
	}
	else{
		ROS_INFO("knowledge did not respond");

		tempResponseMsg.success=false;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub1.publish(tempResponseMsg);
	}

	if(target.size()==7){
		ROS_INFO("size ==7 joint space  ");
		tempResponseMsg.success=true;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub1.publish(tempResponseMsg);
	}

	else if(target.size()==6){

		tamp_msgs::trajquest trajsrv;
		trajsrv.request.targetpos.position.x = target[0];
		trajsrv.request.targetpos.position.y = target[1];
		trajsrv.request.targetpos.position.z = target[2];
		trajsrv.request.targetpos.orientation.x = target[3];
		trajsrv.request.targetpos.orientation.y = target[4];
		trajsrv.request.targetpos.orientation.z = target[5];
		trajsrv.request.position_tolerance.data = 0.05;
		trajsrv.request.orientation_tolerance.data = 0.1;
		trajsrv.request.execute = false;
		std::vector<string> graspif,objtype;
	    boost::split(graspif, msg.ActionParametersName[0], boost::is_any_of("-"));
	    if(graspif[0]=="place1"){
	    	trajsrv.request.withcollision = false;
	    }
	    else{
	    	trajsrv.request.withcollision = true;
	    }
		
		trajsrv.request.currentrobot = true;
		trajsrv.request.withendeffector = true;
		     			   		 	
		knowledge_msg.request.reqType=graspif[0];

		if(tampKnowledgeClient.call(knowledge_msg)){
			std::vector<string> objtoremove = knowledge_msg.response.names;
			if(objtoremove.size()>1){
				boost::split(objtype, objtoremove[1], boost::is_any_of("_"));
				trajsrv.request.objecttoremve = objtoremove[1];
			}

		}
		else{
			cout<<" The knowledge base does not responded"<<endl;
		}

		if(tampMotionPlannerPanda1.call(trajsrv)){
			cout<<" planning sent for panda 1"<<endl;
			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			if(trajsrv.response.success){
								
				tempResponseMsg.success=true;

			}
			else{
				tempResponseMsg.success=false;
			}
			simulationResponsePub1.publish(tempResponseMsg);

		}
		
	}

}
void tamp_interface::simulateApproachingPanda2(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateApproachingPanda2 for "))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	tamp_msgs::knowledge knowledge_msg;
	knowledge_msg.request.reqType=msg.ActionParametersName[0];
	knowledge_msg.request.Name="";
	std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){
		target=knowledge_msg.response.pose;
	}
	else{
		ROS_INFO("knowledge did not respond");

		tempResponseMsg.success=false;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub1.publish(tempResponseMsg);
	}

	if(target.size()==7){
		ROS_INFO("Size 7 joint space pose ");
		tempResponseMsg.success=true;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub2.publish(tempResponseMsg);
	}

	else if(target.size()==6)
	{

		tamp_msgs::trajquest trajsrv;
		trajsrv.request.targetpos.position.x = target[0];
		trajsrv.request.targetpos.position.y = target[1];
		trajsrv.request.targetpos.position.z = target[2];
		trajsrv.request.targetpos.orientation.x = target[3];
		trajsrv.request.targetpos.orientation.y = target[4];
		trajsrv.request.targetpos.orientation.z = target[5];
		trajsrv.request.position_tolerance.data = 0.05;
		trajsrv.request.orientation_tolerance.data = 0.1;
		trajsrv.request.execute = false;
		std::vector<string> graspif,objtype;
		 boost::split(graspif, msg.ActionParametersName[0], boost::is_any_of("-"));
	    if(graspif[0]=="place2"){
	    	trajsrv.request.withcollision = false;
	    }
	    else{
	    	trajsrv.request.withcollision = true;
	    }
		
		trajsrv.request.currentrobot = true;
		trajsrv.request.withendeffector = true;

		
			     			   		 	
		knowledge_msg.request.reqType=graspif[0];

		if(tampKnowledgeClient.call(knowledge_msg)){
			std::vector<string> objtoremove = knowledge_msg.response.names;
			if(objtoremove.size()>1){
				boost::split(objtype, objtoremove[1], boost::is_any_of("_"));
				trajsrv.request.objecttoremve = objtoremove[1];
			}

		}
		else{
			cout<<" The knowledge base does not responded"<<endl;
		}

		if(tampMotionPlannerPanda2.call(trajsrv)){
			cout<<" planning sent for panda 2"<<endl;
			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			if(trajsrv.response.success){
				tempResponseMsg.success=true;

			}
			else{
				tempResponseMsg.success=false;
			}
			simulationResponsePub2.publish(tempResponseMsg);

		}
		
	}
}

void tamp_interface::simulateAddingGraph(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateAddingGraph for "))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
		tempResponseMsg.success=true;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
				simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}
}
void tamp_interface::simulateCheckif(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateCheckif for "))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::knowledge knsrv;

	if(msg.ResponsibleAgents[0]=="panda1Arm"){
		knsrv.request.reqType="r1t1";
		bool r1t1,r1t2,result=false;
		
		if(tampKnowledgePandaClient.call(knsrv)){
			if(knsrv.response.pose[0]>0.0 || knsrv.response.names.size()!=2){
				ROS_INFO("r1t1 true *****");
				r1t1 = true;
			}
		}
		knsrv.request.reqType="r1t2";
		
		if(tampKnowledgePandaClient.call(knsrv)){
			if(knsrv.response.pose[0]>0.0 || knsrv.response.names.size()!=2){
				r1t2 = true;
				ROS_INFO("r1t2 true *****");
			}

		}

		if(msg.ActionParametersName[0]=="np"){
			if(!r1t1 && !r1t2){
				result =true;
				ROS_INFO("!r1t1 && !r1t2 *****");
			}
		}

		if(msg.ActionParametersName[0]=="yp"){
			if(r1t1 || r1t2){
				result =true;
				ROS_INFO("r1t1 || r1t2 *****");
			}

		}
		else if(msg.ActionParametersName[0]=="r1t1"){
			if(r1t1){
				result =true;
			}
			
		}
		else if(msg.ActionParametersName[0]=="r1t2"){
			if(r1t2){
				result =true;
			}
			
		}
		else if(msg.ActionParametersName[0]=="bp"){
			if(r1t1 && r1t2){
				result =true;
			}
		}
		
		tamp_msgs::tampSimulationRes tempResponseMsg;
		tempResponseMsg.success=result;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	
		simulationResponsePub1.publish(tempResponseMsg);
		
	}
	else if(msg.ResponsibleAgents[0]=="panda2Arm"){
		knsrv.request.reqType="r2t1";
		bool r2t1,r2t2,result=false;
		
		if(tampKnowledgePandaClient.call(knsrv)){
			if(knsrv.response.pose[0]>0.0 || knsrv.response.names.size()!=2){
				r2t1 = true;
				ROS_INFO("r2t1 true *****");
			}
		}
		knsrv.request.reqType="r2t2";
		
		if(tampKnowledgePandaClient.call(knsrv)){
			if(knsrv.response.pose[0]>0.0 || knsrv.response.names.size()!=2){
				r2t2 = true;
				ROS_INFO("r2t2 true *****");
			}

		}

		if(msg.ActionParametersName[0]=="np"){
			if(!r2t1 && !r2t2){
				result =true;
				ROS_INFO("!r1t1 && !r1t2 *****");
			}
		}

		else if(msg.ActionParametersName[0]=="yp"){
			if(r2t1 || r2t2){
				result =true;
				ROS_INFO("r1t1 || r1t2 *****");
			}

		}
		else if(msg.ActionParametersName[0]=="r2t1"){
			if(r2t1){
				result =true;
			}
			
		}
		else if(msg.ActionParametersName[0]=="r2t2"){
			if(r2t2){
				result =true;
			}
			
		}
		else if(msg.ActionParametersName[0]=="bp"){
			if(r2t1 && r2t2){
				result =true;
			}
		}
		
		tamp_msgs::tampSimulationRes tempResponseMsg;
		tempResponseMsg.success=result;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	
		simulationResponsePub2.publish(tempResponseMsg);
		
		
	}
	else{
		ROS_INFO("Agent is not correct");
	}
	
}
void tamp_interface::simulatePlanFree(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulatePlanFree for "))<<msg.ResponsibleAgents[0]<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	tempResponseMsg.success=1;
	tempResponseMsg.time=0.002;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
	}
	else if(msg.ResponsibleAgents[0]=="panda2Arm"){
		simulationResponsePub2.publish(tempResponseMsg);
	}
	else{
		simulationResponsePub.publish(tempResponseMsg);
	}


}
void tamp_interface::simluateFingingClosestObject(const tamp_msgs::tampSimulationReq& msg){
			cout<<BOLD(FBLU("tamp_interface::simluateFingingClosestObject"))<<endl;
			cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}
}

void tamp_interface::simulateObjectRegister(const tamp_msgs::tampSimulationReq& msg){
			cout<<BOLD(FBLU("tamp_interface::simulateObjectRegister"))<<endl;
			cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}
}

void tamp_interface::simulateRemoveObject(const tamp_msgs::tampSimulationReq& msg){

			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			if(msg.ResponsibleAgents[0]=="panda1Arm"){
				simulationResponsePub1.publish(tempResponseMsg);
			}
			else if(msg.ResponsibleAgents[0]=="panda2Arm"){
				simulationResponsePub2.publish(tempResponseMsg);
			}
			else{
				simulationResponsePub.publish(tempResponseMsg);
			}
		
		
}


void tamp_interface::simulateCheckIfZ1(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::knowledge knmsg;
	knmsg.request.reqType="human_action";
	if(tampKnowledgeClient.call(knmsg)){
		std::vector<double> huamactions = knmsg.response.pose;
		if ((int)huamactions.back()==1){

			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
		}
		

		
		else{
			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.success=0;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
		}
	}
}
void tamp_interface::simulateCheckIfZ2(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::knowledge knmsg;
	knmsg.request.reqType="human_action";
	if(tampKnowledgeClient.call(knmsg)){
		std::vector<double> huamactions = knmsg.response.pose;
		if ((int)huamactions.back()==6){

			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
		}
		

		
		else{
			tamp_msgs::tampSimulationRes tempResponseMsg;
			tempResponseMsg.success=0;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
		}
	}
}

void tamp_interface::simulateSendVictory(const tamp_msgs::tampSimulationReq& msg){

	tamp_msgs::tampSimulationRes tempResponseMsg;
	
		tempResponseMsg.success=1;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}
	

}
void tamp_interface::simulateUpdatingScene(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateUpdatingScene"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::updatescene updsrv;
	updsrv.request.task = msg.ActionParametersName[0];
	//kmsg.request.updatescene =true;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	if(tampSceneClientPanda.call(updsrv)){
		tempResponseMsg.success=updsrv.response.result;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}
	}

}

void tamp_interface::simulateUpdateBenchmarkScene(const tamp_msgs::tampSimulationReq& msg){
	

	cout<<BOLD(FBLU("tamp_interface::simulateUpdateBenchmarkScene"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::updatescene updsrv;
	//updsrv.request.task = msg.ActionParametersName[0];
	//kmsg.request.updatescene =true;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	if(tampSceneClientBenchmark.call(updsrv)){
		cout<<"called the service"<<endl;
		tempResponseMsg.success=updsrv.response.result;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		
		simulationResponsePub.publish(tempResponseMsg);
		
	}
}

void tamp_interface::simulateRegisterPlace(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateRegisterPlace"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string pickandplace=msg.ActionParametersName[0];
	tamp_msgs::registerplace regissrv;
	regissrv.request.pp = pickandplace;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	if(tampRegisterPlaceClient.call(regissrv)){
		ROS_INFO("Updated places");
		tempResponseMsg.success=1;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);

	}
    else{
    	ROS_INFO("Could not update places :(");
		tempResponseMsg.success=0;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);


    }


}








void tamp_interface::simulateCheckPickObject(const tamp_msgs::tampSimulationReq& msg){
	cout<<BOLD(FBLU("tamp_interface::simulateCheckPickObject"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string plname=msg.ActionName;
	std::vector<double> pl1vector,pl3vector;
	tamp_msgs::knowledge knmsg;
	knmsg.request.reqType="place1";
	tamp_msgs::tampSimulationRes tempResponseMsg;
	if(tampKnowledgeClient.call(knmsg)){
		pl1vector = knmsg.response.pose;
	}
	knmsg.request.reqType="place3";
	if(tampKnowledgeClient.call(knmsg)){
		pl3vector = knmsg.response.pose;
	}

	bool pickform2 = false;

	if(pl1vector.empty()){
		pickform2 = true;
	}
	else{
		int nextitem;
		if(pl3vector.empty()){
			nextitem = 1;
		}
		else{
			nextitem = (int) pl3vector.back()+1;
		}

		for(int item:pl1vector){
			if(item==nextitem){
				pickform2 = false;
				break;
			}
			else{
				pickform2 = true;
			}
		}

	
	
	}	
	
    if(plname=="CheckPi1"){
    	if(pickform2){
    		tempResponseMsg.success=0;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
    	}
    	else{
    		tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);

    	}
    	


    }
    else if(plname=="CheckPi2"){
    	if(pickform2){
    		tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);

    	}
    	else{
    		tempResponseMsg.success=0;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);

    	}

    }

    else{
    	ROS_ERROR("The difined action deos't exist");

    }
	



}

void tamp_interface::simulateCheckPlaceObject(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateCheckPlaceObject"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string plname=msg.ActionName;
	ROS_INFO("Checking for object placement in : %s ", plname.c_str());
	tamp_msgs::tampSimulationRes tempResponseMsg;
	std::vector<double> objectinhandvector,pl3vector,pl2vector,pl1vector;
	tamp_msgs::knowledge knmsg;
	knmsg.request.reqType="place1";
	if(tampKnowledgeClient.call(knmsg)){
		ROS_INFO("request knowledge for  %s",knmsg.request.reqType.c_str());
		pl1vector = knmsg.response.pose;
	}
	//int lastobjectof1 = (int) pl1vector.back();
	knmsg.request.reqType="place2";
	if(tampKnowledgeClient.call(knmsg)){
		ROS_INFO("request knowledge for  %s",knmsg.request.reqType.c_str());
		pl2vector = knmsg.response.pose;
	}
	//int lastobjectof2 = (int) pl2vector.back();

	knmsg.request.reqType="place3";
	if(tampKnowledgeClient.call(knmsg)){
		ROS_INFO("request knowledge for  %s",knmsg.request.reqType.c_str());
		pl3vector = knmsg.response.pose;
	}

	int lastobjectof3;
	if(pl3vector.empty()){
		lastobjectof3=0;

	}
	else{
		lastobjectof3 = (int) pl3vector.back();

	}
	ROS_INFO("lastobjectof3 is %d",lastobjectof3);
	//knmsg.request.reqType="object_inhand";
	//if(tampKnowledgeClient.call(knmsg)){
	//	objectinhandvector = knmsg.response.pose;
	//}
	//int objectinhand = (int) objectinhandvector.back();
	bool placein2,placein1,placein3=false;
	int objectinhand;
	if(pl2vector.empty()){
		objectinhand = (int)pl1vector.back();
	}
	else{
		if((int) pl2vector.back()==0 ){
			objectinhand = (int)pl1vector.back();
		}
		else{
			objectinhand = (int)pl2vector.back();
		}
		
	}
	ROS_INFO("objectinhand is %d",objectinhand);



	if(objectinhand==lastobjectof3+1){
		placein3=true;
		ROS_INFO("object will placed id place 3");
	}
	else{

		int nextitem;
		if(pl1vector.empty()){
			placein1 = true;
			ROS_INFO("object will placed id place 1");
		}
		else{

			if(pl3vector.empty()){
				nextitem =1;
			}
			else{
				nextitem = (int) pl3vector.back()+1;
			}
			

			for(int item:pl1vector){
				if(item==nextitem){
					placein2 = true;
					ROS_INFO("object will placed id place 2");
					break;
				}
				else{
					placein1 = true;
					ROS_INFO("object will placed id place 11");
				}
			}
		}	

	}
    ROS_INFO("proceeding to publish");
	if(plname=="CheckPl1"){
		ROS_INFO("checking for placein1");
		tempResponseMsg.success=placein1;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);

	}

	else if(plname=="CheckPl2"){
		ROS_INFO("checking for placein2 ");
		tempResponseMsg.success=placein2;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);

	}

	else if(plname=="CheckPl3"){
		ROS_INFO("checking for placein3 ");
		tempResponseMsg.success=placein3;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);


	}

	else{
		ROS_ERROR("No action defied like this oooops :(");
	}


}




void tamp_interface::simulateFaultyCommand(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::knowledge knmsg;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	knmsg.request.reqType="object_status";
	if(tampKnowledgeClient.call(knmsg)){
		if(knmsg.response.names[1]=="faulty"){
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
            std_msgs::String dispmsg;
            dispmsg.data="Faulty";
			robotdisplaypub.publish(dispmsg);

		}
		else{
			tempResponseMsg.success=0;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);


		}
	}

}
void tamp_interface::simulateNonFaultyCommand(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::knowledge knmsg;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	knmsg.request.reqType="object_status";
	if(tampKnowledgeClient.call(knmsg)){
		if(knmsg.response.names[1]=="not_faulty"){
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
			std_msgs::String dispmsg;
            dispmsg.data="Non-Faulty";
			robotdisplaypub.publish(dispmsg);


		}
		else{
			tempResponseMsg.success=0;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);


		}
	}
	

	
}
void tamp_interface::simulateNACommand(const tamp_msgs::tampSimulationReq& msg){

	tamp_msgs::knowledge knmsg;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	knmsg.request.reqType="object_status";
	if(tampKnowledgeClient.call(knmsg)){
		if(knmsg.response.names[1]=="NA"){
			tempResponseMsg.success=1;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);
			std_msgs::String dispmsg;
            dispmsg.data="NA";
			robotdisplaypub.publish(dispmsg);


		}
		else{
			tempResponseMsg.success=0;
			tempResponseMsg.time=0.002;//sec
			tempResponseMsg.ActionName=msg.ActionName;
			tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
			simulationResponsePub.publish(tempResponseMsg);


		}
	}
	
}

void tamp_interface::simulateCuffCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateCuffCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	if(leftarmcuffed_ && msg.ResponsibleAgents[0]=="LeftArm"){

		tempResponseMsg.success=1;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);

	}
	else if(rightarmcuffed_&& msg.ResponsibleAgents[0]=="RightArm"){

		tempResponseMsg.success=1;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);

	}
}


void tamp_interface::simulateUnCuffCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateUnCuffCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	if(leftarmuncuffed_ && msg.ResponsibleAgents[0]=="LeftArm"){

		tempResponseMsg.success=1;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);

	}
	else if(rightarmuncuffed_ && msg.ResponsibleAgents[0]=="RightArm"){

		tempResponseMsg.success=1;
		tempResponseMsg.time=0.002;//sec
		tempResponseMsg.ActionName=msg.ActionName;
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
		simulationResponsePub.publish(tempResponseMsg);

	}
}


void tamp_interface::simulateRegisterCommand(const tamp_msgs::tampSimulationReq& msg){

	
	cout<<BOLD(FBLU("tamp_interface::simulateRegisterCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	//if(rightarmuncuffed_ || rightarmuncuffed_){
	//	std::vector<string> resag;
	//	resag.push_back("RightArm");
		tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	//}
	//else if(leftarmcuffed_||leftarmuncuffed_){
	//	std::vector<string> resag;
	//	resag.push_back("LeftArm");
	//	tempResponseMsg.ResponsibleAgents=resag;
	//}
	
	simulationResponsePub.publish(tempResponseMsg);



}




void tamp_interface::simulateGraspingCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateGraspingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	tamp_msgs::tampSimulationRes tempResponseMsg;
	tempResponseMsg.success=1;
	tempResponseMsg.time=0.2;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;

	
		if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}


}



void tamp_interface::simulateHoldingCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateHoldingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	simulationResponsePub.publish(tempResponseMsg);


}

void tamp_interface::simulateStoppingCommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateStoppingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	simulationResponsePub.publish(tempResponseMsg);

}

void tamp_interface::simulateApproachingCommand(const tamp_msgs::tampSimulationReq& msg){

    cout<<BOLD(FBLU("tamp_interface::simulateApproachingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
    
	//! parse the input command
	//vector<string> parameter1,parameter2,parameter1Info,parameter2Info;
	/// Transport wTo wTg
	//boost::split(parameter1,msg.ActionParametersName[0], boost::is_any_of("-"));// wTo: Point3, Cylinder2-ConnectionFrame
	//boost::split(parameter2,msg.ActionParametersName[1], boost::is_any_of("-"));// wTg: Point4, Plate1_connectionFrame

   
    //! call the knowledge base
	tamp_msgs::knowledge knowledge_msg;

	knowledge_msg.request.reqType=msg.ActionParametersName[0];
//	if(parameter1.size()>1)
//		knowledge_msg.request.Name=parameter1[1];
//	else
	knowledge_msg.request.Name="";

	//knowledge_msg.request.requestInfo=msg.ActionParameterInfo[0]; // objectPose
    std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){

		//int vectorSize=knowledge_msg.response.pose.size();
         target=knowledge_msg.response.pose;
		if(target.size()==0){
			

			cout<<FBLU("Time2: ")<<to_string(ros::Time::now().toSec())<<endl;
			tamp_msgs::tampSimulationRes tempResponseMsg;

			tempResponseMsg.success=false;
			tempResponseMsg.time=0;//sec
			tempResponseMsg.ActionName=msg.ActionName;

			for(int i=0;i<msg.ResponsibleAgents.size();i++)
				tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
			for(int i=0;i<msg.ActionParameterInfo.size();i++)
				tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
			for(int i=0;i<msg.ActionParametersName.size();i++)
				tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
			for(int i=0;i<msg.ColleagueAgents.size();i++)
				tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);





			if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}
			cout<<FBLU("Time Published: ")<<to_string(ros::Time::now().toSec())<<endl;



		}

		else if(target.size()==6){




				bool simulationResult;
				string arm;
				double  actionTime;
				tamp_msgs::trajquest trajsrv;
				 geometry_msgs::Pose tarpos;
				if(msg.ResponsibleAgents[0]=="LeftArm"){

			       arm="left";
			    tarpos.orientation.x=target[3];
			    tarpos.orientation.y=target[4];
			    tarpos.orientation.z=target[5]; 
				}
				else if(msg.ResponsibleAgents[0]=="RightArm"){

			     arm="right";
			      tarpos.orientation.x=target[3];
			    tarpos.orientation.y=target[4];
			    tarpos.orientation.z=target[5]; 
				}
				
			    cout<<FBLU("Time1: ")<<to_string(ros::Time::now().toSec())<<endl;
				//simulateArmMotionPlanner(arm,target,simulationResult, actionTime);
			    
			    trajsrv.request.arm = arm;
			   
			    tarpos.position.x =target[0]; 
			    tarpos.position.y =target[1]; 
			    tarpos.position.z =target[2];
			   
			    trajsrv.request.execute=false;
			    trajsrv.request.targetpos = tarpos;
			   
			     std::vector<string> graspif,objtype;
			     	boost::split(graspif, msg.ActionParametersName[0], boost::is_any_of("-"));
			     	
			   if(graspif.size()>1){
			   		 if(graspif[1]=="pregrasp"||graspif[1]=="grasp"||graspif[1]=="postgrasp"){
			   		 	
			   		 		   knowledge_msg.request.reqType=graspif[0];

						    	if(tampKnowledgeClient.call(knowledge_msg)){
						         std::vector<string> objtoremove = knowledge_msg.response.names;
						         if(objtoremove.size()>1){
						         	boost::split(objtype, objtoremove[1], boost::is_any_of("_"));
							         if(objtype[0]=="cylinder"){
							         	trajsrv.request.objecttoremve = objtoremove[1];
							     		}
									
						         }
						         
								
							     }
							     else{
								cout<<" The knowledge base does not responded"<<endl;
							     }

			   		 	
			    

			    }
			   }
			  

			  
			        trajsrv.request.position_tolerance.data = ptol_;
			        trajsrv.request.orientation_tolerance.data = 0.01;

			  
			    trajsrv.request.currentrobot = true;
			    trajsrv.request.withendeffector = true;
			    trajsrv.request.simulation=false;
			    trajsrv.request.withcollision=true;
			  bool results;
			  double time;
			  if(tampMotionPlannerClient.call(trajsrv)){


			  	results = trajsrv.response.success;
			  	time = trajsrv.response.time;
			  }







			    cout<<FBLU("Time2: ")<<to_string(ros::Time::now().toSec())<<endl;
				tamp_msgs::tampSimulationRes tempResponseMsg;

				tempResponseMsg.success=results;
				tempResponseMsg.time=time;//sec
				tempResponseMsg.ActionName=msg.ActionName;

				for(int i=0;i<msg.ResponsibleAgents.size();i++)
					tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
				for(int i=0;i<msg.ActionParameterInfo.size();i++)
					tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
				for(int i=0;i<msg.ActionParametersName.size();i++)
					tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
				for(int i=0;i<msg.ColleagueAgents.size();i++)
					tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);





				if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}
				cout<<FBLU("Time Published: ")<<to_string(ros::Time::now().toSec())<<endl;




		}

		else if(target.size()==7){
				cout<<BOLD(FGRN("Simulating joint position Goal"))<<endl;
				ROS_INFO("goint to joint pisition");
				for(auto item:target){
					cout<<item<<" ,";
				}
			
				bool simulationResult;
				string arm;
				double  actionTime;
				tamp_msgs::trajquest trajsrv;
				if(msg.ResponsibleAgents[0]=="LeftArm"){

			       arm="left";
			    
				}
				else if(msg.ResponsibleAgents[0]=="RightArm"){

			     arm="right";
			    
				}
				
			    cout<<FBLU("Time1: ")<<to_string(ros::Time::now().toSec())<<endl;
				//simulateArmMotionPlanner(arm,target,simulationResult, actionTime);
			    
			    trajsrv.request.arm = arm;
			   
			    
			   
			    trajsrv.request.execute=false;
			    trajsrv.request.targetjointpos = target;
			    /*
			     std::vector<string> graspif;
			     	boost::split(graspif, msg.ActionParametersName[0], boost::is_any_of("-"));
			   if(graspif.size()>1){
			   		 if(graspif[1]=="grasp"||graspif[1]=="postgrasp"){
			       knowledge_msg.request.reqType=graspif[0];

			    	if(tampKnowledgeClient.call(knowledge_msg)){
			         std::vector<string> objtoremove = knowledge_msg.response.names;
			         trajsrv.request.objecttoremve = objtoremove[0];
					
					
				     }
				     else{
					cout<<" The knowledge base does not responded"<<endl;
				     }


			    }
			   }
			   

			  */
			        trajsrv.request.position_tolerance.data = ptol_;
			        trajsrv.request.orientation_tolerance.data = rtol_;

			  
			        trajsrv.request.currentrobot = true;
			    
			    trajsrv.request.withcollision=true;
			  bool results;
			  double time;
			  if(tampMotionPlannerClient.call(trajsrv)){


			  	results = trajsrv.response.success;
			  	time = trajsrv.response.time;
			  }







			    cout<<FBLU("Time2: ")<<to_string(ros::Time::now().toSec())<<endl;
				tamp_msgs::tampSimulationRes tempResponseMsg;

				tempResponseMsg.success=true;
				tempResponseMsg.time=time;//sec
				tempResponseMsg.ActionName=msg.ActionName;

				for(int i=0;i<msg.ResponsibleAgents.size();i++)
					tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
				for(int i=0;i<msg.ActionParameterInfo.size();i++)
					tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
				for(int i=0;i<msg.ActionParametersName.size();i++)
					tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
				for(int i=0;i<msg.ColleagueAgents.size();i++)
					tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);





				if(msg.ResponsibleAgents[0]=="panda1Arm"){
					simulationResponsePub1.publish(tempResponseMsg);
				}
				else if(msg.ResponsibleAgents[0]=="panda2Arm"){
					simulationResponsePub2.publish(tempResponseMsg);
				}
				else{
					simulationResponsePub.publish(tempResponseMsg);
				}
						cout<<FBLU("Time Published: ")<<to_string(ros::Time::now().toSec())<<endl;



		}

		else if(target.size()==3){

			cout<<BOLD(FGRN("Simulating Base position Goal"))<<endl;
				
			
				bool simulationResult;
				
				double  actionTime;
				tamp_msgs::trajquest trajsrv;
				
				
			    cout<<FBLU("Time1: ")<<to_string(ros::Time::now().toSec())<<endl;
				//simulateArmMotionPlanner(arm,target,simulationResult, actionTime);
			    
			    trajsrv.request.arm = "Base";
			   
			    
			   
			    trajsrv.request.execute=false;
			    trajsrv.request.targetjointpos = target;
			  
		        trajsrv.request.position_tolerance.data = ptol_;
		        trajsrv.request.orientation_tolerance.data = rtol_;

			  
		        trajsrv.request.currentrobot = true;
			    
			    trajsrv.request.withcollision=true;
			    bool results;
			    double time;
			  if(tampMotionPlannerClientBase.call(trajsrv)){


			  	results = trajsrv.response.success;
			  	time = trajsrv.response.time;
			  }







			    cout<<FBLU("Time2: ")<<to_string(ros::Time::now().toSec())<<endl;
				tamp_msgs::tampSimulationRes tempResponseMsg;

				tempResponseMsg.success=true;
				tempResponseMsg.time=time;//sec
				tempResponseMsg.ActionName=msg.ActionName;

				for(int i=0;i<msg.ResponsibleAgents.size();i++)
					tempResponseMsg.ResponsibleAgents.push_back(msg.ResponsibleAgents[i]);
				for(int i=0;i<msg.ActionParameterInfo.size();i++)
					tempResponseMsg.ActionParameterInfo.push_back(msg.ActionParameterInfo[i]);
				for(int i=0;i<msg.ActionParametersName.size();i++)
					tempResponseMsg.ActionParametersName.push_back(msg.ActionParametersName[i]);
				for(int i=0;i<msg.ColleagueAgents.size();i++)
					tempResponseMsg.ColleagueAgents.push_back(msg.ColleagueAgents[i]);





				if(msg.ResponsibleAgents[0]=="panda1Arm"){
					simulationResponsePub1.publish(tempResponseMsg);
				}
				else if(msg.ResponsibleAgents[0]=="panda2Arm"){
					simulationResponsePub2.publish(tempResponseMsg);
				}
				else{
					simulationResponsePub.publish(tempResponseMsg);
				}
						cout<<FBLU("Time Published: ")<<to_string(ros::Time::now().toSec())<<endl;


		}
		
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

	


	
   

	
}

void tamp_interface::simulateRestingcommand(const tamp_msgs::tampSimulationReq& msg){

	cout<<BOLD(FBLU("tamp_interface::simulateRestingcommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	tamp_msgs::tampSimulationRes tempResponseMsg;

	tempResponseMsg.success=1;
	tempResponseMsg.time=0.2;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	if(msg.ResponsibleAgents[0]=="panda1Arm"){
			simulationResponsePub1.publish(tempResponseMsg);
		}
		else if(msg.ResponsibleAgents[0]=="panda2Arm"){
			simulationResponsePub2.publish(tempResponseMsg);
		}
		else{
			simulationResponsePub.publish(tempResponseMsg);
		}

}


void tamp_interface::simulateCheckrcommand(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::tampSimulationRes tempResponseMsg;

	
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	
	if(lastagent_=="RightArm"){
      tempResponseMsg.success=1;
	}
	else{
		tempResponseMsg.success=0;
	}
	simulationResponsePub.publish(tempResponseMsg);

}
void tamp_interface::simulateChecklcommand(const tamp_msgs::tampSimulationReq& msg){
	tamp_msgs::tampSimulationRes tempResponseMsg;

	
	tempResponseMsg.time=0.02;//sec
	tempResponseMsg.ActionName=msg.ActionName;
	tempResponseMsg.ResponsibleAgents=msg.ResponsibleAgents;
	
	if(lastagent_=="RightArm"){
      tempResponseMsg.success=0;
	}
	else{
		tempResponseMsg.success=1;
	}
	simulationResponsePub.publish(tempResponseMsg);

}

/*void tamp_interface::simulateArmMotionPlanner(std::vector<string> ResponsibleAgents,std::vector<double> target,bool results,double time){

     cout<<BOLD(FBLU("tamp_interface::simulateArmMotionPlanner"))<<endl;

    tamp_msgs::trajquest trajsrv;
    trajsrv.request.arm = ResponsibleAgents[0];
    geometry_msgs::Pose tarpos;

    if(ResponsibleAgents[0]=="RightArm"){
       tarpos.orientation.x=0.5;
    tarpos.orientation.y=1.9;
    tarpos.orientation.z=1.0; 

    }
    else{
          tarpos.orientation.x=-0.8;
    tarpos.orientation.y=1.2;
    tarpos.orientation.z=-1.6; 


    }
    tarpos.position.x =target[0]; 
    tarpos.position.y =target[1]; 
    tarpos.position.z =target[2];
    
    trajsrv.request.execute=false;
    trajsrv.request.targetpos = tarpos;
    trajsrv.request.position_tolerance.data = 0.05;
    trajsrv.request.orientation_tolerance.data = 1.5;
    trajsrv.request.withcollision=true;


  if(tampMotionPlannerClient.call(trajsrv)){


  	results = trajsrv.response.success;
  	time = trajsrv.response.time;
  }





}
*/










//**********************************Actuation Commands*****************************************************//


void tamp_interface::arrivingCommands(const std_msgs::String::ConstPtr& commandmsg){

	cout<<BOLD(FBLU("tamp_interface::arrivingCommands"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	ROS_INFO("Arrived robot command: %s",input.c_str());
	int agentNumber;
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));

	if(msg.size()==3)
	boost::split(msgColleagues, msg[2], boost::is_any_of("+"));
	else if(msg.size()>3)
		cout<<"Error in arriving msg size: "<<msg.size()<<input<<endl;

	//! first find which agents should perform the action and assign it.

	if(msgAgents.size()==1)
	{
		if(msgAgents[0]=="RightArm")
		{
			agentNumber=0;
		}
		else if(msgAgents[0]=="LeftArm")
		{
			agentNumber=1;
		}
		else
		{	agentNumber=2;
			//cout<<"The agents definition is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
		}
	}
	else if(msgAgents.size()==2)
	{
		if((msgAgents[0]=="LeftArm" && msgAgents[1]=="RightArm") ||(msgAgents[1]=="LeftArm" && msgAgents[0]=="RightArm"))
		{
			agentNumber=2;
		}
		else
		{
			cout<<"The agents definition is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
		}
	}
	else
	{
		cout<<"The agents size is not correct: "<<msgAgents.size()<<", "<<msg[1]<<endl;
	}

	if(agents_list[agentNumber].isBusy==false || msg[0]=="Stop")
	{


		agents_list[agentNumber].isBusy=true;

		agents_list[agentNumber].lastAssignedAction=msg[0];

		agents_list[agentNumber].microSec_StartingTime=duration_cast< microseconds >(system_clock::now().time_since_epoch());
		agents_list[agentNumber].collaborators=msgColleagues;

	}
	else
	{
		cout<<"The agent you assigned is busy now and it can not perform a new action "<<endl;
	}


	if(msgAction[0]=="Approach")
	{
		if(msgAgents[0]=="panda1Arm"){
			sendApproachingCommandPanda1(commandmsg,agents_list[agentNumber]);
		}
		else if(msgAgents[0]=="panda2Arm"){
			sendApproachingCommandPanda2(commandmsg,agents_list[agentNumber]);
		}
		else{
			sendApproachingCommand(commandmsg,agents_list[agentNumber]);
		}
		
	}
	
	else if(msgAction[0]=="Rest")
	{
		sendRestingCommand(agents_list[agentNumber]);
	}
	
	
	else if(msgAction[0]=="Grasp")
	{
		//sendGraspingCommand(agents_list[agentNumber]);
		sendSimulationGraspingCommand(commandmsg,agents_list[agentNumber]);
	}
	else if(msgAction[0]=="UnGrasp")
	{
		//sendUnGraspingCommand(agents_list[agentNumber]);
		sendSimulationUnGraspingCommand(commandmsg,agents_list[agentNumber]);
	}
	else if (msgAction[0]=="remove"){
		sendRemoveObjectCommand(commandmsg,agents_list[agentNumber]);
	}
	
	else if(msgAction[0]=="Stop")
	{
		sendStoppingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="HoldOn")
	{
		sendHoldingCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="checkifr")
	{
		checkrcommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="checkifl")
	{
		checklcommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="updatescene")
	{
		SceneUpdatecommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="sendvictory")
	{
		sendVictoryCommand(agents_list[agentNumber]);
	}
	else if(msgAction[0]=="checkifz1"|| msgAction[0]=="checkifz2"){
		sendCheckifZs(agents_list[agentNumber]);

	}
	//else if(msgAction[0]=="askhuman")
		//sendAskHumanCommand();

	else if(msgAction[0]=="register")
		registerCommand(agents_list[agentNumber]);
	else if(msgAction[0]=="registerplace")
		registerPlaceCommand(agents_list[agentNumber]);
	else if(msgAction[0]=="Cuff" || msgAction[0]=="UnCuff" )
		cuffingCommand();
	else if(msgAction[0]=="UnCuff")
		UncuffingCommand();
	else if(msgAction[0]=="objectregister")
		objectRegisterCommand(commandmsg,agents_list[agentNumber]);
	else if(msgAction[0]=="CheckFlt" || msgAction[0]=="CheckNFlt" || msgAction[0]=="CheckNA" )
		checkFultCommand(agents_list[agentNumber]);
    else if(msgAction[0]=="CheckPl1" || msgAction[0]=="CheckPl2" || msgAction[0]=="CheckPl3"
    	     || msgAction[0]=="CheckPi1" || msgAction[0]=="CheckPi2")
    	checkPickandPlaceObject(agents_list[agentNumber]);
    else if(msgAction[0]=="planfree"){
    	agents_list[agentNumber].isBusy = true;
    	planFreeCommand(commandmsg,agents_list[agentNumber]);
    }
    else if(msgAction[0]=="checkif"){
    	checkifCommand(agents_list[agentNumber]);
    }
    else if(msgAction[0]=="addgraph"){
    	addGraphCommand(commandmsg,agents_list[agentNumber]);
    }
    else if(msgAction[0]=="exit"){
    	exitCommand(agents_list[agentNumber]);
    }
    else if(msgAction[0]=="checkifres"){
    	checkifResCommand(agents_list[agentNumber]);
    }

    else if(msgAction[0]=="checkpp"){
    	chekPPCommand(agents_list[agentNumber]);
    }
    else if(msgAction[0]=="updatebench"){
    	updateBenchCommand(agents_list[agentNumber]);
    }
    else if(msgAction[0]=="removeobject"){
    	removeObjectCommand(commandmsg,agents_list[agentNumber]);
    }
    else if(msgAction[0]=="registerpp"){
    	registerPPCommand(commandmsg,agents_list[agentNumber]);
    }
    else if(msgAction[0]=="checkifdone"){
    	checkIfDoneTamp(agents_list[agentNumber]);
    }
	else
	{
		cout<<"Error in arriving msg action:"<<msgAction[0]<<endl;
	}

  

}

void tamp_interface::checkIfDoneTamp(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::checkIfDoneTamp"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
}

void tamp_interface::registerPPCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::registerPPCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues,ppvec;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	string pp = msgAction[1];
	boost::split(ppvec, pp, boost::is_any_of("+"));

	if(benchmark_=="hanoi"){
		tamp_msgs::hanoipp hanoisrv;
		hanoisrv.request.from = ppvec[0];
		hanoisrv.request.to = ppvec[1];

		if(hanoiPPClient.call(hanoisrv)){
			agent.isActionSuccessfullyDone=hanoisrv.response.result;
		}

		diskdisplaced_++;

	}
	else if(benchmark_=="cubeworld"){
		tamp_msgs::hanoipp hanoisrv;
		
		hanoisrv.request.from = ppvec[0];
		hanoisrv.request.to = ppvec[1];

		if(hanoiPPClient.call(hanoisrv)){
			agent.isActionSuccessfullyDone=hanoisrv.response.result;
		}


	}
	else if(benchmark_=="sort"){
		tamp_msgs::hanoipp hanoisrv;
		
		hanoisrv.request.from = ppvec[0];
		hanoisrv.request.to = ppvec[1];

		if(hanoiPPClient.call(hanoisrv)){
			agent.isActionSuccessfullyDone=hanoisrv.response.result;
		}
	}
	else if(benchmark_=="nonmonotonic"){
		tamp_msgs::hanoipp hanoisrv;
		
		hanoisrv.request.from = ppvec[0];
		hanoisrv.request.to = ppvec[1];

		if(hanoiPPClient.call(hanoisrv)){
			agent.isActionSuccessfullyDone=hanoisrv.response.result;
		}
	}
	else if(benchmark_=="kitchen"){
		tamp_msgs::hanoipp hanoisrv;
		
		hanoisrv.request.from = ppvec[0];
		hanoisrv.request.to = ppvec[1];

		if(hanoiPPClient.call(hanoisrv)){
			agent.isActionSuccessfullyDone=hanoisrv.response.result;
		}
	}
	


	agent.isBusy=false;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);


}
void tamp_interface::removeObjectCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::removeObjectCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	agent.isActionSuccessfullyDone=true;
	agent.isBusy=false;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
}




void tamp_interface::updateBenchCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::updateBenchCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);


}
void tamp_interface::chekPPCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::chekPPCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);


}

void tamp_interface::checkifResCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::checkifResCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);


}

void tamp_interface::exitCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::exitCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	std_msgs::Bool msg;
	msg.data = true;
	if(agent.agents[0]=="panda1Arm"){
		exitpub1.publish(msg);
	}	
	else if(agent.agents[0]=="panda2Arm"){
		exitpub2.publish(msg);
	}
	else{
		ROS_INFO("Agent is not valid");
	}

}
void tamp_interface::addGraphCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::addGraphCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction;

	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));
	if(msgAction.size()>1){
		if(msgAction[1]=="r1t1" ||msgAction[1]=="r1t2"){
			tamp_msgs::changeplansrv srvreq;
			srvreq.request.plan_name = msgAction[1];
			srvreq.request.colomn = 1;
			agent.isBusy=false;
			if(changePlannerClient1.call(srvreq)){
				ROS_INFO("changing plan1");
			}
			else{
				ROS_INFO("could not connect to change server plan1");
			}
			

		}
		else if (msgAction[1]=="r2t1" ||msgAction[1]=="r2t2"){
			tamp_msgs::changeplansrv srvreq;
			srvreq.request.plan_name = msgAction[1];
			srvreq.request.colomn = 2;
			agent.isBusy=false;
			if(changePlannerClient2.call(srvreq)){
				ROS_INFO("changing plan2");
			}
			else{
				ROS_INFO("could not connect to change server plan2");
			}

		}

	}
	else{

		if(allready_assigned_){
			ROS_INFO("Already assigned");
			if(agent.agents[0]=="panda1Arm"){
				tamp_msgs::knowledge ksrg;
				ksrg.request.reqType="task1";
				string task1;
				if(tampKnowledgeClient.call(ksrg)){
					task1 = ksrg.response.names[1];
				}
				tamp_msgs::changeplansrv srvreq;
				std::vector<string> taskss;
				boost::split(taskss, task1, boost::is_any_of("+"));
				if(taskss.size()>1){
					srvreq.request.plan_name = taskss[0];
				}
				else{
					srvreq.request.plan_name = task1;
				}
				
				srvreq.request.colomn = 1;
				agent.isBusy=false;
				if(changePlannerClient1.call(srvreq)){
					ROS_INFO("changing plan1 to %s",task1.c_str());
				}
				else{
					ROS_INFO("could not connect to change server plan1");
				}

			}
			else if(agent.agents[0]=="panda2Arm"){
					tamp_msgs::knowledge ksrg;
					ksrg.request.reqType="task2";
					string task2;
					if(tampKnowledgeClient.call(ksrg)){
						task2 = ksrg.response.names[1];
					}

					tamp_msgs::changeplansrv srvreq;
					std::vector<string> taskss;
					boost::split(taskss, task2, boost::is_any_of("+"));
					if(taskss.size()>1){
						srvreq.request.plan_name = taskss[0];
					}
					else{
						srvreq.request.plan_name = task2;
					}
					srvreq.request.colomn = 2;
					agent.isBusy=false;
					if(changePlannerClient2.call(srvreq)){
						ROS_INFO("changing plan2 to %s", task2.c_str());
					}
					else{
						ROS_INFO("could not connect to change server plan2");
					}

			}
			else{
				ROS_INFO("Agent is not valid");
			}

		}
		else{
			std::vector<string> r1t1objects,r1t2objects,r2t1objects,r2t2objects;
			allready_assigned_=true;
			string task1,task2;
			ROS_INFO("assining tasks for first time");
			tamp_msgs::knowledge knsrv1,knsrv2,knsrv3,knsrv4;
			knsrv1.request.reqType="r1t1";
			int a,b,c,d,e,f;
			bool na=false,nb=false,nc=false,nd=false;
			if(tampKnowledgePandaClient.call(knsrv1)){
				a = knsrv1.response.pose[0];
				r1t1objects = knsrv1.response.names;
				ROS_INFO("A=%d",a);
				if(knsrv1.response.names.size()>1){
					if(knsrv1.response.names[1]=="no"){
						ROS_INFO("A : knsrv.response.names[1]==no");
						na=true;
					}
				}
				
			}
			knsrv2.request.reqType="r1t2";
			if(tampKnowledgePandaClient.call(knsrv2)){
				b = knsrv2.response.pose[0];
				r1t2objects = knsrv2.response.names;
				ROS_INFO("B=%d",b);
				if(knsrv2.response.names.size()>1){
					if(knsrv2.response.names[1]=="no"){
						ROS_INFO("B : knsrv.response.names[1]==no");
						nb=true;
					}
				}
				
			}
			knsrv3.request.reqType="r2t1";
			if(tampKnowledgePandaClient.call(knsrv3)){
				c = knsrv3.response.pose[0];
				r2t1objects = knsrv3.response.names;
				ROS_INFO("C=%d",c);
				if(knsrv3.response.names.size()>1){
					if(knsrv3.response.names[1]=="no"){
						ROS_INFO("C : knsrv.response.names[1]==no");
						nc=true;
					}
				}
				
			}
			knsrv4.request.reqType="r2t2";
			if(tampKnowledgePandaClient.call(knsrv4)){
				d = knsrv4.response.pose[0];
				r2t2objects = knsrv4.response.names;
				ROS_INFO("D=%d",d);
				if(knsrv4.response.names.size()>1){
					if(knsrv4.response.names[1]=="no"){
						ROS_INFO("D : knsrv.response.names[1]==no");
						nd=true;
					}
				}
				
			}
			int g = findDifObjects(r1t1objects,r1t2objects);
			int h = findDifObjects(r2t1objects,r2t2objects);
			e = a+b;
			f = c+d;
			cout<<"e is: "<<e<<"and f is: "<<f<<endl;
			cout<<"g is: "<<g<<"and h is: "<<h<<endl;
			tamp_msgs::objectssrv objsrv;
			objsrv.request.inscene =true;
			double targetx11=0;
			double targetx21=0;
			double targetx12=0;
			double targetx22=0;
			if(collision_object_panda1_client.call(objsrv)){
				targetx11 = objsrv.response.position[0];
				targetx21 = objsrv.response.position[3];
			}
			if(collision_object_panda2_client.call(objsrv)){
				targetx12 = objsrv.response.position[0];
				targetx22 = objsrv.response.position[3];
			}
			
			std::array<double,4> all {a,b,c,d};
				auto min = std::min_element(all.begin(),all.end());
				int compmin=0;
				if(*min ==a){
					compmin = d;
				}
				else if(*min==b){
					compmin = c;
				}
				else if(*min==c){
					compmin = b;
				}
				else if(*min==d){
					compmin = a;
				}


			if(na==true && nb==true){
				cout<<"na is: "<<std::boolalpha <<na<<" nb is: "<<std::boolalpha <<nb<<endl;
				ROS_INFO("na: && nb:");
				task1 = "";
				task2 = (targetx22<targetx12)?"r2t2+r2t1":"r2t1+r2t2";
			}
			else if(nc==true && nd==true){
				ROS_INFO("nc && nd");
				task1 = (targetx11<targetx21)?"r1t1+r1t2":"r1t2+r1t1";
				task2 = "";
			}

			else if(g<=(*min+compmin) && na==false && nb ==false && g<h){
				ROS_INFO("g<std::min(c,d)");
					task1 = (targetx11<targetx21)?"r1t1+r1t2":"r1t2+r1t1";
					task2 = "";
			}

			else if(h<=(*min+compmin) && nc==false && nd ==false && h<g){
				ROS_INFO("h<std::min(a,b)");
				task1 = "";
				task2 = (targetx22<targetx12)?"r2t2+r2t1":"r2t1+r2t2";

			}
			else if((na==true || nb==true) && h<(a+b) && nc==false && nd==false){
				ROS_INFO("(na==true || nb==true) && h<(a+b)");
				task1 = "";
				task2 = (targetx22<targetx12)?"r2t2+r2t1":"r2t1+r2t2";
			}

			else if((nc==true || nd==true) && g<(c+d)  && na==false && nb==false){
				ROS_INFO("(nc==true || nd==true) && g<(c+d)");
				task1 = (targetx11<targetx21)?"r1t1+r1t2":"r1t2+r1t1";
				task2 = "";

			}
			else if(e > 3 * f && nc==false && nd==false){
				ROS_INFO("e > 3 * f");
				task1 = "";
				task2 = (targetx22<targetx12)?"r2t2+r2t1":"r2t1+r2t2";
			}
			else if(f > 3 * e  && na==false && nb==false){
				ROS_INFO("f > 3 * e");
				task1 = (targetx11<targetx21)?"r1t1+r1t2":"r1t2+r1t1";
				task2 = "";
			}
			else{
				
				if(*min==a ){
					ROS_INFO("*min==a");
					if(na==false){
						task1 = "r1t1";
						task2 = "r2t2";
					}
					else{
						task1 = "r1t2";
						task2 = "r2t1";
					}
					
				}
				else if(*min==b){
					ROS_INFO("*min==b");
					if(nb==false){
						task1 = "r1t2";
						task2 = "r2t1";
					}
					else{
						task1 = "r1t1";
				     	task2 = "r2t2";
					}
					
				}
				else if(*min==c){
					ROS_INFO("*min==c");
					if(nc==false){
						task2 = "r2t1";
						task1 = "r1t2";
					}
					else{
						task2 = "r2t2";
						task1 = "r1t1";
					}
					
				}
				else if(*min==d ){
					ROS_INFO("*min==d");
					if(nd==false){
						task2 = "r2t2";
						task1 = "r1t1";
					}
					else{
						task2 = "r2t1";
						task1 = "r1t2";
					}
					
				}

				else{
					ROS_INFO("Could not assign tasks to robots");
				}	
			}
			tamp_msgs::changeplansrv reqsrv;
			ROS_INFO("task1 assigned to %s and task2 to %s",task1.c_str(),task2.c_str());
			tamp_msgs::registerdata srvreg;
			std::vector<string> namess{"task1",task1};
			srvreg.request.names = namess;
			if(tampRegisterClient.call(srvreg)){
				ROS_INFO("Registered data task1 to knowledge");

			}
			else{
				ROS_INFO("Could not register task1 to knowledge");
			}
				std::vector<string> namess2{"task2",task2};
			srvreg.request.names = namess2;
			if(tampRegisterClient.call(srvreg)){
				ROS_INFO("Registered data task2 to knowledge");

			}
			else{
				ROS_INFO("Could not register task2 to knowledge");
			}
			std::vector<string> taskss;
			
			if(agent.agents[0]=="panda1Arm"){
				tamp_msgs::changeplansrv srvreq;
				boost::split(taskss, task1, boost::is_any_of("+"));
				if(taskss.size()>1){
					srvreq.request.plan_name = taskss[0];
				}
				else{
					srvreq.request.plan_name = task1;
				}

				
				srvreq.request.colomn = 1;
				agent.isBusy=false;
				if(changePlannerClient1.call(srvreq)){
					ROS_INFO("changing plan1");
				}
				else{
					ROS_INFO("could not connect to change server plan1");
				}

			}
			else if(agent.agents[0]=="panda2Arm"){
				tamp_msgs::changeplansrv srvreq;
				boost::split(taskss, task2, boost::is_any_of("+"));

				if(taskss.size()>1){
					ROS_INFO("taskss.size()>1");
					ROS_INFO("taskss[0] %s and taskss[1] %s",taskss[0].c_str(),taskss[1].c_str());
					srvreq.request.plan_name = taskss[0];
				}
				else{
					ROS_INFO("taskss.size()==1");
					srvreq.request.plan_name = task2;
				}
				//srvreq.request.plan_name = task2;
				srvreq.request.colomn = 2;
				agent.isBusy=false;
				if(changePlannerClient2.call(srvreq)){
					ROS_INFO("changing plan2");
				}
				else{
					ROS_INFO("could not connect to change server plan1");
				}

			}


		}

	}

	

}

int tamp_interface::findDifObjects(std::vector<string> &z1,std::vector<string> &z2){
	cout<<BOLD(FBLU("tamp_interface::findDifObjects"))<<endl;
	int eqobj=0;
	for(int i=0;i<z1.size();i++){
		for(int j=0;j<z2.size();j++){
			if(z1[i]==z2[j]){
				eqobj++;
				break;
			}
		}
	}
	int out = z1.size()+ z2.size() - eqobj;
return out;
}
void tamp_interface::checkifCommand(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::checkifCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
}

void tamp_interface::planFreeCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::planFreeCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction;

	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));
	if(agent.agents[0]=="panda1Arm"){
		//agent.isBusy =true;
		ROS_INFO("Recived planfree for %s",agent.agents[0].c_str() );
		tamp_msgs::pandafreeplan msgsrv;
		if(panda1freeplan.call(msgsrv)){
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false; 
		    PublishRobotAck(agent);
		}
		else{
			ROS_INFO("Unable to connect to servere planf free for %s",agent.agents[0].c_str() );
		}
	}
	else if(agent.agents[0]=="panda2Arm"){
		//agent.isBusy =true;
		ROS_INFO("Recived planfree for %s",agent.agents[0].c_str() );
		tamp_msgs::pandafreeplan msgsrv;
		if(panda2freeplan.call(msgsrv)){
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false; 
		    PublishRobotAck(agent);
			
		}
		else{
			ROS_INFO("Unable to connect to servere planf free for %s",agent.agents[0].c_str() );
		}
	}
}


void tamp_interface::objectRegisterCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::objectRegisterCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));
	tamp_msgs::registerplace rpmsg;
	rpmsg.request.pp = msgAction[1];
	if(leftarmuncuffed_ ||leftarmcuffed_){
		rpmsg.request.arm = "left";
		ROS_INFO("left arm cuffed");
	}
	else if(rightarmuncuffed_ ||rightarmcuffed_){
		rpmsg.request.arm = "right";
		ROS_INFO("right arm cuffed");
	}
	else {
		rpmsg.request.arm = "right";
	}
	if(tampRegisterObjectPlaceClient.call(rpmsg)){

		if(rpmsg.response.update){
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false; 
		    PublishRobotAck(agent);
		}
		else{

			agent.isBusy=false;
			agent.isActionSuccessfullyDone=false;
			agent.emergencyFlag=false; 
		    PublishRobotAck(agent);
		}

	}

}

void tamp_interface::sendRemoveObjectCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::sendRemoveObjectCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));
	tamp_msgs::removeobject remsg;
	remsg.request.object = msgAction[1];
	if(tampRemoveObjectClient.call(remsg)){
		cout<<"sendt a request to remove object from scene " << msgAction[1]<<endl;
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false; 
		    PublishRobotAck(agent);
	}


}

void tamp_interface::sendCheckifZs(agents_tasks& agent){
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
}


void tamp_interface::SceneUpdatecommand(agents_tasks& agent){

	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
}
void tamp_interface::registerPlaceCommand(agents_tasks& agent){

	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
}



void tamp_interface::checkPickandPlaceObject(agents_tasks& agent){

	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);

}




void tamp_interface::checkFultCommand(agents_tasks& agent){

	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);

}

void tamp_interface::cuffingCommand(){
    
	publishHumanActionAck(4);

}
void tamp_interface::UncuffingCommand(){
    
	publishHumanActionAck(5);

}

void tamp_interface::registerCommand(agents_tasks& agent){
    
    cout<<BOLD(FBLU("tamp_interface::registerCommand"))<<endl;

    tamp_msgs::lastgraph msgsrv;
    msgsrv.request.update=true;
    std::vector<string> lastactionsvector,parametersvector;
    if(lastGraphClient.call(msgsrv)){
        lastactionsvector = msgsrv.response.results;
    }
    ROS_INFO("parameters to subsitute are :");
    for(auto data:lastactionsvector){

    	std::vector<string> splitedactions;
	    boost::split(splitedactions,data, boost::is_any_of("_"));
	    if(splitedactions[0]=="Approach"){
	    	parametersvector.push_back(splitedactions[1]);
            
            cout<<splitedactions[1]<<endl;
	    }
        
    }

    string subsitutionparameter;
	std::vector<string> actionvec;
	boost::split(actionvec,agent.lastAssignedAction, boost::is_any_of("_"));

	 for(auto data:parametersvector){

    	std::vector<string> splitedactionspost;
	    boost::split(splitedactionspost,data, boost::is_any_of("-"));
	    if(actionvec[1]==splitedactionspost[1]){

	    	subsitutionparameter = data;
	    	break;
	    	
	    }
        
    }
    
   std::vector<string> cameravec;
   boost::split(cameravec,subsitutionparameter, boost::is_any_of("-"));

   if(cameravec[0]=="camera"){
   		

   		if(cameravec[1] == "pregrasp"){
   			std_msgs::Bool topub;
   			topub.data=true;
   			kinecttodisplayPub.publish(topub);
   		}
   		else if(cameravec[1]=="postgrasp"){
   			std_msgs::Bool topub;
   			topub.data=false;
   			kinecttodisplayPub.publish(topub);

   		}



   }

	tamp_msgs::ackquest ackmsg;
	if(rightarmcuffed_ || rightarmuncuffed_){
			ackmsg.request.arm ="right"; 
	}
	else if(leftarmuncuffed_ || leftarmcuffed_){
			ackmsg.request.arm ="left"; 
	}
	else{
		ackmsg.request.arm ="right";
	}

	bool jointspace =false;
	std::vector<double> position;
	if(jointspace){
		std::vector<double> reachedpoint;
		ackmsg.request.jointpose = true;
		if(tampMotionAckClient.call(ackmsg)){

		reachedpoint = ackmsg.response.jointposes;
		position = reachedpoint;

	}


	}
	else{
		geometry_msgs::PoseStamped reachedpoint;
		if(tampMotionAckClient.call(ackmsg)){
			ackmsg.request.jointpose = false;
			reachedpoint = ackmsg.response.eepos;
			position.push_back(reachedpoint.pose.position.x);
			position.push_back(reachedpoint.pose.position.y);
			position.push_back(reachedpoint.pose.position.z);
			position.push_back(reachedpoint.pose.orientation.x);
			position.push_back(reachedpoint.pose.orientation.y);
			position.push_back(reachedpoint.pose.orientation.z);

	}

	}
	
	

	std::vector<string> names;

	names.push_back(subsitutionparameter);
	
	tamp_msgs::registerdata data;
	data.request.pose = position;
	data.request.names = names;
	if(tampRegisterClient.call(data)){
         if(data.response.update){
         	 ROS_INFO("Position of %s updated",actionvec[1].c_str());
         	 std_msgs::String dispmsg;
         	 dispmsg.data = actionvec[1];
         	 doneactionsofkiethtetic.publish(dispmsg);
         }
         else{ROS_INFO("Position of %s Could not be updated",actionvec[1].c_str());}
        

	}
	else{ROS_INFO("Unable to call to service tamp_register_service");}
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);

}
void tamp_interface::sendSimulationGraspingCommand(const std_msgs::String::ConstPtr& commandmsg, agents_tasks& agent){
		cout<<BOLD(FBLU("tamp_interface::sendSimulationGraspingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));
    tamp_msgs::vrepgripper vgmsg;
    std::vector<double> pig;
    if(benchmark_=="hanoi"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for "<<"pig"+msgAction[1]<<endl;
			kmsg.request.reqType= "pig"+msgAction[1];
			
			if(tampKnowledgeClient.call(kmsg)){
				pig = kmsg.response.pose;
				for (int i = numberofdiks_-1; i >=0; i--)
				{
					if((int) pig[i]!=0){
						vgmsg.request.obj = "disk"+ to_string((int) pig[i]);
						break;
					}
					
				}
				
			}

		}
    }
    else if(benchmark_=="cubeworld"){
    	if(msgAction.size()==1){

			 vgmsg.request.obj ="";
		}
		else{

			std::vector<string> vnames;
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for tray"<<msgAction[1]<<endl;
			if(msgAction[1]=="nextcube"){
				kmsg.request.reqType= "nextcube";
			}
			else if(msgAction[1]=="cpr"){
				kmsg.request.reqType= "smallestcube";
			}
			else if(msgAction[1]=="cpb"){
				kmsg.request.reqType= "biggestcube";
			}
			else{
				kmsg.request.reqType= "tray"+msgAction[1];
			}
			
			
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				
				if(msgAction[1]=="nextcube"){
					vgmsg.request.obj = vnames[1];
				}
				else{
					vgmsg.request.obj = vnames.back();
				}	
			}
		}
    	

    }
    else if(benchmark_=="sort"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{
			std::vector<string> vnames;
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for stick"<<msgAction[1]<<endl;
			kmsg.request.reqType= msgAction[1];
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.obj = vnames.back();	
			}
		}
    

    }
    else if(benchmark_=="nonmonotonic"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{
			std::vector<string> vnames;
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for stick"<<msgAction[1]<<endl;
			kmsg.request.reqType= msgAction[1];
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.obj = vnames.back();	
			}
		}
    }
    else if(benchmark_=="kitchen"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{
			std::vector<string> vnames;
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for object"<<msgAction[1]<<endl;
			kmsg.request.reqType= msgAction[1];
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.obj = vnames.back();	
			}
		}
    }


	
	
	if(agent.agents[0]=="RightArm"){
		vgmsg.request.arm="right";
	}
	else {
		vgmsg.request.arm="left";
	}
	 
	vgmsg.request.grasp = true;

	
	if(tampGripperPandaClient.call(vgmsg)){
		if(vgmsg.response.result){
			agent.isActionSuccessfullyDone=true;
		}

		else{
			agent.isActionSuccessfullyDone=false;
		}

	}
	else{

		agent.isActionSuccessfullyDone=false;
	}
	agent.isBusy=false;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
    
}
void tamp_interface::sendSimulationUnGraspingCommand(const std_msgs::String::ConstPtr& commandmsg, agents_tasks& agent){
		cout<<BOLD(FBLU("tamp_interface::sendSimulationUnGraspingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));
    tamp_msgs::vrepgripper vgmsg;
    std::vector<double> pig;
    if(benchmark_=="hanoi"){
    	if(msgAction.size()==1){
		vgmsg.request.obj ="";
		}
		else{
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for "<<"pig"+msgAction[1]<<endl;
			kmsg.request.reqType= "pig"+msgAction[1];
			
			if(tampKnowledgeClient.call(kmsg)){
				pig = kmsg.response.pose;
				for (int i = numberofdiks_-1; i >=0; i--)
				{
					if((int) pig[i]!=0){
						vgmsg.request.obj = "disk"+ to_string((int) pig[i]);
						break;
					}
					
				}
			}

		}


    }
    else if(benchmark_=="cubeworld"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{

			tamp_msgs::knowledge kmsg;
			cout<<"requesting for tray"<<msgAction[1]<<endl;
			if(msgAction[1]=="nextcube"){
				kmsg.request.reqType= "nextcube";
			}
			else if(msgAction[1]=="cpr"){
				kmsg.request.reqType= "smallestcube";
			}
			else if(msgAction[1]=="cpb"){
				kmsg.request.reqType= "biggestcube";
			}
			else{
				kmsg.request.reqType= "tray"+msgAction[1];
			}
			std::vector<string> vnames;
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				if(msgAction[1]=="nextcube"){
					vgmsg.request.obj = vnames[1];
				}
				else{
					vgmsg.request.obj = vnames.back();
				}		
			}
		}
     
    }
    else if(benchmark_=="sort"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{
			std::vector<string> vnames;
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for stick"<<msgAction[1]<<endl;
			kmsg.request.reqType= msgAction[1];
			if(tampKnowledgeClient.call(kmsg)){
			vnames = kmsg.response.names;
			vgmsg.request.obj = vnames.back();	
		}
		}
    	
    }
     else if(benchmark_=="nonmonotonic"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{
			std::vector<string> vnames;
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for stick"<<msgAction[1]<<endl;
			kmsg.request.reqType= msgAction[1];
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.obj = vnames.back();	
			}
		}
    }
    else if(benchmark_=="kitchen"){
    	if(msgAction.size()==1){
			 vgmsg.request.obj ="";
		}
		else{
			std::vector<string> vnames;
			tamp_msgs::knowledge kmsg;
			cout<<"requesting for object"<<msgAction[1]<<endl;
			kmsg.request.reqType= msgAction[1];
			if(tampKnowledgeClient.call(kmsg)){
				vnames = kmsg.response.names;
				vgmsg.request.obj = vnames.back();	
			}
		}
    }

	
	
	
	if(agent.agents[0]=="RightArm"){
		vgmsg.request.arm="right";
	}
	else {
		vgmsg.request.arm="left";
	}
	 
	vgmsg.request.grasp = false;

	
	if(tampGripperPandaClient.call(vgmsg)){
		if(vgmsg.response.result){
			agent.isActionSuccessfullyDone=true;
		}

		else{
			agent.isActionSuccessfullyDone=false;
		}

	}
	else{

		agent.isActionSuccessfullyDone=false;
	}
	agent.isBusy=false;
	agent.emergencyFlag=false; 
    PublishRobotAck(agent);
    
}

/*
void tamp_interface::sendGraspingCommand(agents_tasks& agent){

	if(agent.agents[0]=="RightArm"){


		control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetRight.waitForServer();
		grippgoal.command.position = 10.0;
		gripperActionClinetRight.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetRight.waitForResult(ros::Duration(10.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetRight.getState();
			ROS_INFO("Right Gripper Action finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
		}
		else
			ROS_INFO("Action did not finish before the time out.");

	}

	else if(agent.agents[0]=="LeftArm"){
         
        control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetLeft.waitForServer();
		grippgoal.command.position = 10.0;
		gripperActionClinetLeft.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetLeft.waitForResult(ros::Duration(10.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetLeft.getState();
			ROS_INFO("Left Gripper Action finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);

			
		}
		else
			ROS_INFO("Action did not finish before the time out.");




	}
    



}

void tamp_interface::sendUnGraspingCommand(agents_tasks& agent){


	if(agent.agents[0]=="RightArm"){


		control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetRight.waitForServer();
		grippgoal.command.position = 90.0;
		gripperActionClinetRight.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetRight.waitForResult(ros::Duration(4.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetRight.getState();
			ROS_INFO("Right Gripper Action finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;
            PublishRobotAck(agent);
		}
		else
			ROS_INFO("Action did not finish before the time out.");

	}

	else if(agent.agents[0]=="LeftArm"){
         
        control_msgs::GripperCommandGoal grippgoal;
		gripperActionClinetLeft.waitForServer();
		grippgoal.command.position = 90.0;
		gripperActionClinetLeft.sendGoal(grippgoal);
		bool finished_before_timeout = gripperActionClinetLeft.waitForResult(ros::Duration(4.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = gripperActionClinetLeft.getState();
			ROS_INFO("Left GripperAction finished: %s",state.toString().c_str());
			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
		}
		else
			ROS_INFO("Action did not finish before the time out.");




	}



}
*/
void tamp_interface::sendVictoryCommand(agents_tasks& agent){

	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false;    
    PublishRobotAck(agent);
	std_msgs::Bool victorymsg;
	victorymsg.data= true;
	taskDone.publish(victorymsg);




}
void tamp_interface::checkrcommand(agents_tasks& agent){

            agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
}
void tamp_interface::checklcommand(agents_tasks& agent){

			agent.isBusy=false;
			agent.isActionSuccessfullyDone=true;
			agent.emergencyFlag=false;    
            PublishRobotAck(agent);
}



void tamp_interface::sendHoldingCommand(agents_tasks& agent){

	ros::Duration(1).sleep();
	agent.isBusy=false;
	agent.isActionSuccessfullyDone=true;
	agent.emergencyFlag=false;    
    PublishRobotAck(agent);

}

void tamp_interface::sendStoppingCommand(agents_tasks& agent){


}

void tamp_interface::sendApproachingCommandPanda1(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::sendApproachingCommandPanda1"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));

 	tamp_msgs::knowledge knowledge_msg;
	knowledge_msg.request.reqType=msgAction[1];
	knowledge_msg.request.Name="";
	std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){
		target=knowledge_msg.response.pose;
	}

	if(target.size()==7){
        bool simres = sendFranka1ToSimulation(target);
        agent.isActionSuccessfullyDone =simres;
		agent.isBusy=false;
		agent.emergencyFlag=false;    
		PublishRobotAck(agent);

	}

	else if(target.size()==6){

		tamp_msgs::trajquest trajsrv;
		trajsrv.request.targetpos.position.x = target[0];
		trajsrv.request.targetpos.position.y = target[1];
		trajsrv.request.targetpos.position.z = target[2];
		trajsrv.request.targetpos.orientation.x = target[3];
		trajsrv.request.targetpos.orientation.y = target[4];
		trajsrv.request.targetpos.orientation.z = target[5];
		trajsrv.request.position_tolerance.data = 0.05;
		trajsrv.request.orientation_tolerance.data = 0.1;
		trajsrv.request.execute = false;
		std::vector<string> graspif,objtype;
	    boost::split(graspif, msgAction[1], boost::is_any_of("-"));
	    	//trajsrv.request.withcollision = true;
	   if(graspif[0]=="place1"){
		    	trajsrv.request.withcollision = false;
		    }
		    else{
		    	trajsrv.request.withcollision = true;
		    }
		
		trajsrv.request.currentrobot = true;
		trajsrv.request.withendeffector = true;
		     			   		 	
		knowledge_msg.request.reqType=graspif[0];

		if(tampKnowledgeClient.call(knowledge_msg)){
			std::vector<string> objtoremove = knowledge_msg.response.names;
			if(objtoremove.size()>1){
				boost::split(objtype, objtoremove[1], boost::is_any_of("_"));
				trajsrv.request.objecttoremve = objtoremove[1];
			}

		}
		else{
			cout<<" The knowledge base does not responded"<<endl;
		}
	 

		if(tampMotionPlannerPanda1.call(trajsrv)){
			cout<<" planning sent for panda 1"<<endl;
			agent.isBusy=false;
			if(trajsrv.response.success){
				//tamp_msgs::executetraj exesrv;
				//exesrv.request.trajectory = trajsrv.response.soltraj;
				bool rocores = sendFranka1ToSimulation(trajsrv.response.soltraj);
				if(rocores){

					agent.isActionSuccessfullyDone =true;
				}
				else{
					agent.isActionSuccessfullyDone =false;
					//ROS_ERROR("trajectory execution server did not respond");
				}
			}
			else{
				agent.isActionSuccessfullyDone = false;
			}
			//agent.isActionSuccessfullyDone=trajsrv.response.executedtrajectory;;
			

		}

			agent.emergencyFlag=false;
		PublishRobotAck(agent);


	}

	    
	

  
 

	 


}
bool tamp_interface::sendFranka1ToSimulation(moveit_msgs::RobotTrajectory &traj){
  
 // ros::AsyncSpinner spinner(1);
   // spinner.start();
	nuexecutionpanda1++;
	double timeinit1 = ros::Time::now().toSec();
  ROS_INFO("panda_planner::sendFranka1ToSimulation");
  trajectory_msgs::JointTrajectory trajmsg = traj.joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jointpointsvector = trajmsg.points;
  std::vector<double> allposes;
  for(size_t i=0;i<jointpointsvector.size();i++){
    for(size_t j=0;j<7;j++){
      allposes.push_back(jointpointsvector[i].positions[j]);
    }

  }
  //ROS_INFO("Coppied all joint positions");
 
  bool finalres;
  
  std::vector<double> poses;
      
for(size_t i=0;i<allposes.size()/7;i++){
      poses.clear();
      cmdmsgpanda1_.position.clear();
      for (size_t j=0;j<7;j++)
      {
       cmdmsgpanda1_.position.push_back(allposes[i*7+j]); 
       poses.push_back(allposes[i*7+j]);
      }
      vrep_franka1_pub.publish(cmdmsgpanda1_);
      double timenow1 = ros::Time::now().toSec();
      /*
      finalres = false;
      while (true){
        if(allFranka2JointsReached(poses)){
          finalres = true;
          break;
        }
        if(ros::Time::now().toSec()-timenow1>2.0){
          finalres = false;
          break;
        }
      }
      */
      while(true){

      	if(ros::Time::now().toSec()-timenow1>0.03){
      		break;
      	}

      }
     // std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }
double timenow2 = ros::Time::now().toSec();
  while(true){

      	if(ros::Time::now().toSec()-timenow2>0.8){
      		break;
      	}

      }
 //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 
 executinTpanda1 += ros::Time::now().toSec() - timeinit1;

  return true;

}


bool tamp_interface::sendFranka1ToSimulation(std::vector<double> joint_poses){
  ros::AsyncSpinner spinner(1);
    spinner.start();
    nuexecutionpanda1++;
    double timeinit1 = ros::Time::now().toSec();

  ROS_INFO("panda_planner::sendFranka1ToSimulation");

 

 
  bool finalres;
  
  std::vector<double> poses;
      

      
       cmdmsgpanda1_.position = joint_poses;

      vrep_franka1_pub.publish(cmdmsgpanda1_);
        double timenow1 = ros::Time::now().toSec();
        while(true){

      	if(ros::Time::now().toSec()-timenow1>0.9){
      		break;
      	}

      }
      /*
      double timenow1 = ros::Time::now().toSec();
      finalres = false;
      while (true){
        if(allFranka1JointsReached(joint_poses)){
          finalres = true;
          break;
        }
        if(ros::Time::now().toSec()-timenow1>2.0){
          finalres = false;
          break;
        }
      }
*/
 //std::this_thread::sleep_for(std::chrono::milliseconds(500));
      executinTpanda1 += ros::Time::now().toSec() - timeinit1;
  return true;

}




bool tamp_interface::allFranka1JointsReached(std::vector<double> &poses){
    //ROS_INFO("panda_planner::allFranka1JointsReached");
    if(!franka1jointerrors_.empty()){

      franka1jointerrors_.clear();
    }
    for(size_t i =0;i<7;i++){
      double err = poses[i]-franka1joints_[i];
      //cout<<"error cmdpos: "<<poses[i] <<" subjoint: "<<franka1joints_[i]<<endl;
      franka1jointerrors_.push_back(err);
    }
      
   

    double threshold = 0.05;
    bool condpass = true;
    for(size_t i =0;i<7;i++){
      if(fabs(franka1jointerrors_[i])>threshold){
        condpass = false;
      }
    }

return condpass;

}


void tamp_interface::sendApproachingCommandPanda2(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::sendApproachingCommandPanda2"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));
	boost::split(msgAgents, msg[1], boost::is_any_of("+"));


	tamp_msgs::knowledge knowledge_msg;
	knowledge_msg.request.reqType=msgAction[1];
	knowledge_msg.request.Name="";


	std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){
		target=knowledge_msg.response.pose;
	}

	if(target.size()==7){

	    bool simres = sendFranka2ToSimulation(target);
        agent.isActionSuccessfullyDone =simres;
		agent.isBusy=false;
		agent.emergencyFlag=false;    
		PublishRobotAck(agent);

	}

	else if(target.size()==6){

		tamp_msgs::trajquest trajsrv;
		trajsrv.request.targetpos.position.x = target[0];
		trajsrv.request.targetpos.position.y = target[1];
		trajsrv.request.targetpos.position.z = target[2];
		trajsrv.request.targetpos.orientation.x = target[3];
		trajsrv.request.targetpos.orientation.y = target[4];
		trajsrv.request.targetpos.orientation.z = target[5];
		trajsrv.request.position_tolerance.data = 0.05;
		trajsrv.request.orientation_tolerance.data = 0.1;
		trajsrv.request.execute = false;
		std::vector<string> graspif,objtype;
	    boost::split(graspif, msgAction[1], boost::is_any_of("-"));
	    if(graspif[0]=="place2"){
		    	trajsrv.request.withcollision = false;
		    }
		    else{
		    	trajsrv.request.withcollision = true;
		    }
		
		trajsrv.request.currentrobot = true;
		trajsrv.request.withendeffector = true;
		     			   		 	
		knowledge_msg.request.reqType=graspif[0];

		if(tampKnowledgeClient.call(knowledge_msg)){
			std::vector<string> objtoremove = knowledge_msg.response.names;
			if(objtoremove.size()>1){
				boost::split(objtype, objtoremove[1], boost::is_any_of("_"));
				trajsrv.request.objecttoremve = objtoremove[1];
			}

		}
		else{
			cout<<" The knowledge base does not responded"<<endl;
		}

		if(tampMotionPlannerPanda2.call(trajsrv)){
			cout<<" planning sent for panda 2"<<endl;
			agent.isBusy=false;
			if(trajsrv.response.success){
				//tamp_msgs::executetraj exesrv;
				//exesrv.request.trajectory = trajsrv.response.soltraj;
				if(sendFranka2ToSimulation(trajsrv.response.soltraj)){

					agent.isActionSuccessfullyDone =true;
				}
				else{
					agent.isActionSuccessfullyDone =false;
					//ROS_ERROR("trajectory execution server did not respond");
				}
			}
			else{
				agent.isActionSuccessfullyDone = false;
			}
		//agent.isActionSuccessfullyDone=trajsrv.response.executedtrajectory;;
		

		}	
		agent.emergencyFlag=false;    
		PublishRobotAck(agent);

	}


	


}

bool tamp_interface::sendFranka2ToSimulation(moveit_msgs::RobotTrajectory &traj){
  nuexecutionpanda2++;
  double timeinit1 = ros::Time::now().toSec();
  ROS_INFO("panda_planner::sendFranka2ToSimulation");
  ros::AsyncSpinner spinner(1);
    spinner.start();
  trajectory_msgs::JointTrajectory trajmsg = traj.joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jointpointsvector = trajmsg.points;
  std::vector<double> allposes;
  for(size_t i=0;i<jointpointsvector.size();i++){
    for(size_t j=0;j<7;j++){
      allposes.push_back(jointpointsvector[i].positions[j]);
    }

  }
  //ROS_INFO("Coppied all joint positions");
 
  bool finalres;
  
  std::vector<double> poses;
      

  for(size_t i=0;i<allposes.size()/7;i++){
      poses.clear();
      cmdmsgpanda2_.position.clear();
      for (size_t j=0;j<7;j++)
      {
       cmdmsgpanda2_.position.push_back(allposes[i*7+j]); 
       poses.push_back(allposes[i*7+j]);
      }
      vrep_franka2_pub.publish(cmdmsgpanda2_);
      double timenow1 = ros::Time::now().toSec();

       while(true){

      	if(ros::Time::now().toSec()-timenow1>0.03){
      		break;
      	}

      }
      /*
      finalres = false;
      while (true){
        if(allFranka2JointsReached(poses)){
          finalres = true;
          break;
        }
        if(ros::Time::now().toSec()-timenow1>2.0){
          finalres = false;
          break;
        }
      }
      */
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }
double timenow2 = ros::Time::now().toSec();
 while(true){

      	if(ros::Time::now().toSec()-timenow2>0.5){
      		break;
      	}

      }
 
 //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

 
executinTpanda2 += ros::Time::now().toSec() - timeinit1;
  return true;

}

bool tamp_interface::sendFranka2ToSimulation(std::vector<double> joint_poses){
 ros::AsyncSpinner spinner(1);
   spinner.start();
   nuexecutionpanda2++;
   double timeinit1 = ros::Time::now().toSec();
  ROS_INFO("panda_planner::sendFranka2ToSimulation");

 

 
  bool finalres;
  
  std::vector<double> poses;
      

      
       cmdmsgpanda2_.position = joint_poses;

      vrep_franka2_pub.publish(cmdmsgpanda2_);
      double timenow1 = ros::Time::now().toSec();
      finalres = false;
      /*
      while (true){
        if(allFranka2JointsReached(joint_poses)){
          finalres = true;
          break;
        }
        if(ros::Time::now().toSec()-timenow1>2.0){
          finalres = false;
          break;
        }
      }*/

       while(true){

      	if(ros::Time::now().toSec()-timenow1>0.5){
      		break;
      	}

      }

      

    
executinTpanda2 += ros::Time::now().toSec() - timeinit1;

  return true;

}

bool tamp_interface::allFranka2JointsReached(std::vector<double>& poses){
    //ROS_INFO("panda_planner::allFranka2JointsReached");
     if(!franka2jointerrors_.empty()){
      
      franka2jointerrors_.clear();
    }
    
    for(size_t i =0;i<7;i++){
      double err = poses[i]-franka2joints_[i];
     // cout<<"error cmdpos: "<<poses[i] <<" subjoint: "<<franka2joints_[i]<<endl;
      franka2jointerrors_.push_back(err);
    }
      
   

    double threshold = 0.05;
    bool condpass = true;
    for(size_t i =0;i<7;i++){
      if(fabs(franka2jointerrors_[i])>threshold){
        condpass = false;
      }
    }

return condpass;

}


void tamp_interface::callBackfrankajoint1(const sensor_msgs::JointStateConstPtr& msg){
  if(!franka1joints_.empty()){

      franka1joints_.clear();
    }
  

  for(size_t i=0;i<7;i++){
     franka1joints_.push_back(msg->position[i]);
     
  }
 

}


void tamp_interface::callBackfrankajoint2(const sensor_msgs::JointStateConstPtr& msg){
  if(!franka2joints_.empty()){

      franka2joints_.clear();
    }

  for(size_t i=0;i<7;i++){
     franka2joints_.push_back(msg->position[i]);
     
  }
 

}
void tamp_interface::sendApproachingCommand(const std_msgs::String::ConstPtr& commandmsg,agents_tasks& agent){

	cout<<BOLD(FBLU("tamp_interface::sendApproachingCommand"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string input=commandmsg-> data.c_str();
	vector<string> msg, msgAction,msgAgents,msgColleagues;
	string cmndType, reachingPoint;
	boost::split(msg, input, boost::is_any_of(" "));
	boost::split(msgAction, msg[0], boost::is_any_of("_"));

	boost::split(msgAgents, msg[1], boost::is_any_of("+"));
    




    tamp_msgs::knowledge knowledge_msg;

	knowledge_msg.request.reqType=msgAction[1];

	knowledge_msg.request.Name="";

    std::vector<double> target;
	if(tampKnowledgeClient.call(knowledge_msg)){

         target=knowledge_msg.response.pose;
		cout<<" The knowledge responded"<<endl;
		
	}
	else
	{
		cout<<" The knowledge base does not responded"<<endl;
	}

    tamp_msgs::trajquest trajsrv;
    string arm;
    if(target.size()==6){
    	cout<<BOLD(FBLU("moving to point: "))<<endl;
	    cout<<target[0]<<" "<<target[1]<<" "<<target[2]<<" "<<target[3]<<" "<<
	    target[4]<<" "<<target[5]<<" "<<endl;
		
		
		
		geometry_msgs::Pose tarpos;
		if(msgAgents[0]=="LeftArm"){
	        arm="left";

		}
		else if(msgAgents[0]=="RightArm"){

			arm="right";

		}
        tarpos.position.x =target[0]; 
	    tarpos.position.y =target[1]; 
	    tarpos.position.z =target[2];
		tarpos.orientation.x=target[3];
		tarpos.orientation.y=target[4];
		tarpos.orientation.z=target[5]; 
	    trajsrv.request.arm = arm;
	 
	    trajsrv.request.execute=true;
	    trajsrv.request.targetpos = tarpos;
    	    

    }
    else if(target.size()==7){
    	cout<<BOLD(FBLU("moving to joint point: "))<<endl;
    	cout<<target[0]<<" "<<target[1]<<" "<<target[2]<<" "<<target[3]<<" "<<
	    target[4]<<" "<<target[5]<<" "<<target[6]<<" "<<endl;
	
		if(msgAgents[0]=="LeftArm"){
	        arm="left";
		}
		else if(msgAgents[0]=="RightArm"){
			arm="right";
		}
		 
	    trajsrv.request.arm = arm;
	    trajsrv.request.execute=true;
	    trajsrv.request.targetjointpos = target;

    }
    else if(target.size()==3){

			cout<<BOLD(FGRN("sending Base position Goal"))<<endl;
				
				
			    cout<<FBLU("Time1: ")<<to_string(ros::Time::now().toSec())<<endl;
			    
			    trajsrv.request.arm = "Base";
			   
			    
			   
			    trajsrv.request.execute=true;
			    trajsrv.request.targetjointpos = target;
			  
		        trajsrv.request.position_tolerance.data = ptol_;
		        trajsrv.request.orientation_tolerance.data = rtol_;

			  
		        trajsrv.request.currentrobot = true;
			    
			    trajsrv.request.withcollision=true;
			    bool results;
			    double time;
			  if(tampMotionPlannerClientBase.call(trajsrv)){


			  	results = trajsrv.response.success;
			  	time = trajsrv.response.time;
			  }

			  	agent.isBusy=false;
				agent.isActionSuccessfullyDone=trajsrv.response.executedtrajectory;;
				agent.emergencyFlag=false;    
				PublishRobotAck(agent);


				return;




		}






    std::vector<string> graspif,objtype;
    boost::split(graspif, msgAction[1], boost::is_any_of("-"));
    
    if(graspif.size()>1){
    	
    	  if(graspif[1]=="pregrasp"||graspif[1]=="grasp"||graspif[1]=="postgrasp"){
    	  		
    	  				tamp_msgs::knowledge kmsg;
			        	  kmsg.request.reqType=graspif[0];
			        	  
			        	 // kmsg.request.reqType=graspif[0];

						    	if(tampKnowledgeClient.call(kmsg)){
							         std::vector<string> objtoremove = kmsg.response.names;
							         if(objtoremove.size()>1){
							         	boost::split(objtype, objtoremove[1], boost::is_any_of("_"));
								         if(objtype[0]=="cylinder"){
								         	trajsrv.request.objecttoremve = objtoremove[1];

								         }
							         }
							         
								
							     }
							     else{	
								cout<<" The knowledge base does not responded"<<endl;
							     }
	
    	  		
    		
					
    		}

   		
	}
   
   if(graspif[0]=="camera"){
   		
   		if(graspif[1] == "pregrasp"){
   			std_msgs::Bool topub;
   			topub.data=true;
   			kinecttodisplayPub.publish(topub);
   		}
   		else if(graspif[1]=="postgrasp"){
   			std_msgs::Bool topub;
   			topub.data=false;
   			kinecttodisplayPub.publish(topub);
   		}
   }




    //ptol_ = 0.005;
    //rtol_=0.01;
    trajsrv.request.position_tolerance.data = ptol_;
    trajsrv.request.orientation_tolerance.data = rtol_;
    trajsrv.request.withcollision=true;
    trajsrv.request.simulation=false;
     trajsrv.request.currentrobot = true;
   trajsrv.request.withendeffector = true;

	geometry_msgs::PoseStamped reachedpoint;
	std::size_t j=0;
	if(tampMotionPlannerClient.call(trajsrv)){

	  	bool results = trajsrv.response.success;
	  	double time = trajsrv.response.time;

	   if(trajsrv.response.executedtrajectory){
	   	  ROS_INFO("Executed trajrctory.");
	   	  tamp_msgs::ackquest ackmsg;
		  ackmsg.request.arm =arm; 
		  
			  if(tampMotionAckClient.call(ackmsg)){

			    reachedpoint = ackmsg.response.eepos;

			  }
   		}

		
		if(trajsrv.request.simulation){
			ros::Duration(1.0).sleep();
				bool result =checkIfReachedToTarget(reachedpoint,target);
		cout<<"Reached to target as planned"<<" Eculedan distanse: "<<dis_<<"rotation dis "<<drot_<<endl;
		string logstr = trajsrv.response.executedtrajectory?FBLU("reached the goal"):FRED(" did not reach the goal");
		cout<<FGRN("the simulation trajrctory")<<logstr<<endl;
			agent.isBusy=false;
		agent.isActionSuccessfullyDone=trajsrv.response.executedtrajectory;;
		agent.emergencyFlag=false;    
		PublishRobotAck(agent);
		}

		else{

			agent.isBusy=false;
		agent.isActionSuccessfullyDone=trajsrv.response.executedtrajectory;
		agent.emergencyFlag=false;    
		PublishRobotAck(agent);
		}
		

}
}






void tamp_interface::sendRestingCommand(agents_tasks& agent){




}



//**************************************Acknowledgement**************************************/
void tamp_interface::controlAckPub(agents_tasks& agent){

    cout<<BOLD(FBLU("tamp_interface::controlAckPub"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	std_msgs::String ackMsg;
	cout<<"Last assigned action: "<<agent.lastAssignedAction<<endl;
	ackMsg.data=agent.lastAssignedAction+" "; // Appraoch_Point2, Grasp, Transport_Cylinder2-GraspingPose1_Point7

	for(int i=0;i<agent.agents.size();i++){
		ackMsg.data+=agent.agents[i];
		if(i<agent.agents.size()-1)
			ackMsg.data=ackMsg.data+"+";
	}
	if(agent.isActionSuccessfullyDone==true)
		ackMsg.data+=" true";
	else
		ackMsg.data+=" false";

	if (agent.emergencyFlag==false)
		robotAckPub.publish(ackMsg);
	else
	{
		cout<<"The agent emergency flag is true, therefore we do not give ack to the planner"<<endl;
		agent.Print();
	}



}

void tamp_interface::publishHumanActionAck(int acnum){

	cout<<BOLD(FBLU("tamp_interface::publishHumanActionAck"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	std_msgs::Int16 actionnum;
	actionnum.data = acnum;
	humanactionpub.publish(actionnum);

}
bool  tamp_interface::checkIfReachedToTarget(geometry_msgs::PoseStamped rp, std::vector<double> tar){

     dis_=0;
     drot_=0;
     double dx = rp.pose.position.x - tar[0];
     double dy = rp.pose.position.y - tar[1];
     double dz = rp.pose.position.z - tar[2];
      dis_ = sqrt(dx*dx+dy*dy+dz*dz);
    ROS_INFO("robot reache to goal with a difference of %f", dis_);
     double droll = rp.pose.orientation.x - tar[3];
     double dpitch = rp.pose.orientation.y - tar[4];
     double dyaw = rp.pose.orientation.z - tar[5];
      drot_ = sqrt(droll*droll+dpitch*dpitch+dyaw*dyaw);

     if(dis_<0.9 /*&& drot_<rtol_*/){

     	return true;
     }
     else{

     	return false;
     }






}


void tamp_interface::PublishRobotAck(agents_tasks& agent){
	cout<<BOLD(FBLU("tamp_interface::PublishRobotAck"))<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	std_msgs::String ackMsg;
	cout<<"Last assigned action: "<<agent.lastAssignedAction<<endl;
	ackMsg.data=agent.lastAssignedAction+" "; // Appraoch_Point2, Grasp, Transport_Cylinder2-GraspingPose1_Point7

	for(int i=0;i<agent.agents.size();i++){
		ackMsg.data+=agent.agents[i];
		if(i<agent.agents.size()-1)
			ackMsg.data=ackMsg.data+"+";
	}
	if(agent.isActionSuccessfullyDone==true)
		{    //ROS_INFO("Publishing task accomplishment to planner1");


            lastagent_ = agent.agents[0];
           







			ackMsg.data+=" true";
          /*
	       std::vector<string> actionvec,graspvec;
	     boost::split(actionvec,agent.lastAssignedAction, boost::is_any_of("_"));
         if(actionvec[0]=="Approach"){
         	ROS_INFO("Publishing task accomplishment to planner2");
               boost::split(graspvec,actionvec[1], boost::is_any_of("-"));
               if(graspvec.size()>1){ 
                 	if(graspvec[1]=="postgrasp"){
         		ROS_INFO("Publishing task accomplishment to planner3");
             
            
              tamp_msgs::knowledge knmsg;
              knmsg.request.reqType=graspvec[0];
               if(tampKnowledgeClient.call(knmsg)){
               	ROS_INFO("Publishing task accomplishment to planner4");
               		string res =knmsg.response.names[0];
               		std::vector<string> vec;
               		boost::split(vec,res, boost::is_any_of("_"));
               		ROS_INFO("vec[1] is *****%s:",vec[1]);
               	 if(vec[1]=="target"){
               	 	ROS_INFO("Publishing task accomplishment to planner5");

               	 	   std_msgs::Bool donemsg;
		                 donemsg.data=true;
		                 ROS_INFO("Publishing task accomplishment to planner");
		                 taskDone.publish(donemsg);

               	 }
               	
              

               }
                std_msgs::String msg;
             msg.data=graspvec[0];
             eliminateObject.publish(msg);

         	}
         }
      


         }

         */



        }
	else
		ackMsg.data+=" false";

	if (agent.emergencyFlag==false){

		if(agent.agents[0]=="panda1Arm"){
			robotAckPub1.publish(ackMsg);
		}
		else if(agent.agents[0]=="panda2Arm"){
			robotAckPub2.publish(ackMsg);
		}
		else{
			robotAckPub.publish(ackMsg);
		}

		
	}

		
	else
	{
		cout<<"The agent emergency flag is true, therefore we do not give ack to the planner"<<endl;
		agent.Print();
	}
}

tamp_interface::~tamp_interface(){

	cout<<"tamp_interface::~tamp_interface"<<endl;
	cout<<"#execution ag1: "<<nuexecutionpanda1<<" #execution ag2: "<<nuexecutionpanda2<<endl;
	cout<<"executionT ag1: "<<executinTpanda1<<" executionT ag2: "<<executinTpanda2<<endl;
}
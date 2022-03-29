#include "tamp_knowledge.h"



tamp_knowledge::tamp_knowledge(){

    pubToRobotDisplay=nh.advertise<std_msgs::String>("robotDisplayText",20);
   
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	collision_object_panda1_client = nh.serviceClient<tamp_msgs::objectssrv>("/tamp_vrep_objects_wrtopanda1");
  collision_object_panda2_client = nh.serviceClient<tamp_msgs::objectssrv>("/tamp_vrep_objects_wrtopanda2");
	updateSceneServerbenchmark = nh.advertiseService("tamp_updatescene_benchmark_service",&tamp_knowledge::updateSceneServerBenchmark,this);
	updateSceneServerPanda = nh.advertiseService("tamp_updatescene_panda_service",&tamp_knowledge::updateSceneServer,this);
    knowledgeServerPanda = nh.advertiseService("tamp_knowledge_panda_service",&tamp_knowledge::knowledgeQueryPanda,this);
	knowledgeServer = nh.advertiseService("tamp_knowledge_service",&tamp_knowledge::knowledgeQuery,this);
	registerServer = nh.advertiseService("tamp_register_service",&tamp_knowledge::registerData,this);
	registerPlaceServer = nh.advertiseService("tamp_register_place_service",&tamp_knowledge::registerPlace,this);
	removeObjectServer = nh.advertiseService("tamp_remove_object_service",&tamp_knowledge::removeObject,this);
	removeObjectServerPanda = nh.advertiseService("tamp_removepanda_object_service",&tamp_knowledge::removeObjectPanda,this);
	registerObjectPlaceServer = nh.advertiseService("tamp_register_object_service",&tamp_knowledge::registerObjectPlace,this);
    hanoiPPServer = nh.advertiseService("benchmark_pp_service",&tamp_knowledge::tampHanoiPP,this);

    BenchmarkPoseClinet = nh.serviceClient<tamp_msgs::objectssrv>("tamp_vrep_objects_benchmarks");
	kinetcCbObjectIdSub = nh.subscribe("qr_id",3,&tamp_knowledge::kinetcCbObjectId,this);
	humanActionSub = nh.subscribe("human_action",3,&tamp_knowledge::humanAcionCB,this);
	objectInsertedSub = nh.subscribe("object_id",3,&tamp_knowledge::objectInsertedCB,this);
	sceneClient= nh.serviceClient<tamp_msgs::sceneobjects>("tamp_scene_service");
	robotStateClient=nh.serviceClient<tamp_msgs::ackquest>("tamp_ack_service");
	simulationSceneClient= nh.serviceClient<tamp_msgs::objectssrv>("tamp_vrep_objects_service");
	simulationTargetClient= nh.serviceClient<tamp_msgs::targetsrv>("tamp_vrep_target_service");
	pandaEEClient= nh.serviceClient<tamp_msgs::pandaee>("tamp_vrep_pandaee");
    //objectServiceClient = nh.serviceClient<tamp_msgs::objectstatus>("object_status_service");

	//objectelominate = nh.subscribe("eliminate_object",80,&tamp_knowledge::eliminateCB,this);
	kinectcb = nh.subscribe("obj_status",10,&tamp_knowledge::callBackkinect,this);
   


    if( nh.getParam("/benchmark",benchmark_)){
    	ROS_INFO("Benchmark %s is taken from ros parameter",benchmark_.c_str());
    }
    else{
    	//benchmark_ = argv[1];
    	ROS_INFO("Benchmark %s is taken from input arg",benchmark_.c_str());
    }


 	if(benchmark_=="hanoi"){
	 	nh.getParam("/disks",numberofdiks_);
	 	cout<<"Solving benchmark "<<benchmark_<<" with " << numberofdiks_<<" disks"<<endl;
 	
 	}
 	else if(benchmark_=="cubeworld"){
 		cmap.insert(pair<int,std::string>(1, "cubeA"));
 		vlauemap.insert(pair<std::string,int>("cubeA",1));
 		vlauemap.insert(pair<std::string,int>("cubeB",2));
 		vlauemap.insert(pair<std::string,int>("cubeC",3));
 		vlauemap.insert(pair<std::string,int>("cubeD",4));
 		vlauemap.insert(pair<std::string,int>("cubeE",5));
 		vlauemap.insert(pair<std::string,int>("cubeF",6));
 		vlauemap.insert(pair<std::string,int>("cubeG",7));
 		vlauemap.insert(pair<std::string,int>("cubeH",8));
 		vlauemap.insert(pair<std::string,int>("cubeI",9));
 		vlauemap.insert(pair<std::string,int>("cubeJ",10));
 		printMap(cmap);
 	
 	}
 	 readDataBase();
	cout<<BOLD("******************************************************")<<endl;
	



}


void tamp_knowledge::readDataBase(){

	ROS_INFO("Reading Database");

    const char* home=getenv("HOME");
	string pointPath(home);
	cout<< "Benchmark is: "<<benchmark_<<endl;
	string pointsPath=pointPath+"/catkin_ws/src/TAMP/tamp_knowledge/files/tamp_"+benchmark_+".txt";
	ROS_INFO("Reading Database from %s",pointsPath.c_str());
	ifstream file_path_ifStr(pointsPath.c_str());
	string line;
	vector<string> line_list;

	string delim_type=" ";
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			boost::split(line_list, line, boost::is_any_of(delim_type));
			if(line_list[0]!="#")
			{
				vector<float> Pose;
				vector<string> Name;
				for(int i=1;i<line_list.size();i++)
				{
					Pose.push_back( stof(line_list[i]) );
				}

				Name.push_back(line_list[0]);
				//Name.push_back("Pose");
				world temp_point(Name,Pose);
				dataBase_.push_back(temp_point);
			}
		}
	}


	//int right_ee_pose=dataBase_.size();
   /// int left_ee_pose = right_ee_pose+1;
   // int closest_object_to_rightee = left_ee_pose+1;
    //int closest_object_to_leftee = closest_object_to_rightee+1;
    //int closest_object_to_base = closest_object_to_leftee+1;
   // int num_of_objects = closest_object_to_base+1;
  //  int closest_object_to_target =num_of_objects+1;
   /// int largest_object = closest_object_to_target+1;
   // int object_status = largest_object+1;
   // int place_1 = dataBase_.size();;
   // int place_2 = place_1+1;
   /// int place_3 = place_2+1;
  //  int obj = place_3+1;
    //int object_status = obj+1;
    
    //int nextobject  = dataBase_.size();
   // int humanactions = nextobject+1;
   // int next = humanactions+1;
   // int object_status = next+1;
   // int placecount1 = object_status+1;
   // int placecount2 = placecount1+1;
   // dataBase_.resize(placecount2+1);
   // dataBase_[right_ee_pose].name.push_back("righteepose");
    //dataBase_[left_ee_pose].name.push_back("lefteepose");
    //dataBase_[closest_object_to_rightee].name.push_back("clobree");
    //dataBase_[closest_object_to_leftee].name.push_back("cloblee");
  //  dataBase_[closest_object_to_base].name.push_back("clobb");
    //dataBase_[num_of_objects].name.push_back("num_of_objects");
   // dataBase_[closest_object_to_target].name.push_back("clobt");
    //dataBase_[largest_object].name.push_back("largestobject");
   
   // dataBase_[place_1].name.push_back("place1");
   // dataBase_[place_2].name.push_back("place2");
   // dataBase_[place_3].name.push_back("place3");
    //dataBase_[obj].name.push_back("object");
   // dataBase_[obj].name.push_back("object");
	/*
    dataBase_[object_status].name.push_back("object_status");
    dataBase_[object_status].name.push_back("NA");
    dataBase_[nextobject].name.push_back("nextobject");
    dataBase_[humanactions].name.push_back("human_action");
    dataBase_[next].name.push_back("next");
    dataBase_[placecount1].name.push_back("place1");
    dataBase_[placecount1].value.push_back(0);
    dataBase_[placecount2].name.push_back("place2");
    dataBase_[placecount2].value.push_back(0);
*/
    

  //  int numofobjectsinplace2 = 4;
  //  for(size_t i=0;i<numofobjectsinplace2;i++){
  //  	dataBase_[place_2].value.push_back(0);
  //  }


    
    //updatesceneItself();

	//updatesimulationscene();
    if(benchmark_=="hanoi"){
    	ROS_INFO("Adding pig disks");
    	std::vector<float> pig1,pig2,pig3;
    	for (int i = numberofdiks_; i >0 ; i--)
    	{
    		pig1.push_back(i);
    		pig2.push_back(0);
    		pig3.push_back(0);
    	}
    	vector<string> pigAname,pigBname,pigCname;
    	pigAname.push_back("pigA");
    	pigBname.push_back("pigB");
    	pigCname.push_back("pigC");
    	world piga(pigAname,pig1);
    	world pigb(pigBname,pig2);
    	world pigc(pigCname,pig3);
		dataBase_.push_back(piga);
		dataBase_.push_back(pigb);
		dataBase_.push_back(pigc);

    }
    else if(benchmark_=="cubeworld"){
    	std::vector<float> nexc;
    	vector<string> traybnames,centernames,trayrnames;
    	traybnames.push_back("trayb");
    	traybnames.push_back("cubeB");
    	traybnames.push_back("cubeD");
    	traybnames.push_back("cubeF");
    	traybnames.push_back("cubeH");
    	traybnames.push_back("cubeJ");
    	world trayb(traybnames,nexc);
    	dataBase_.push_back(trayb);


    	centernames.push_back("trayc");
    	centernames.push_back("cubeA");
    	world trayc(centernames,nexc);
    	dataBase_.push_back(trayc);



    	trayrnames.push_back("trayr");
    	trayrnames.push_back("cubeC");
    	trayrnames.push_back("cubeE");
    	trayrnames.push_back("cubeG");
    	trayrnames.push_back("cubeI");
    	world trayr(trayrnames,nexc);
    	dataBase_.push_back(trayr);



    	vector<string> nextcubenames;
    	nextcubenames.push_back("nextcube");
    	nextcubenames.push_back("cubeA");
    	nextcubenames.push_back("cubeB");
    	nextcubenames.push_back("cubeC");
    	nextcubenames.push_back("cubeD");
    	nextcubenames.push_back("cubeE");
    	nextcubenames.push_back("cubeF");
    	nextcubenames.push_back("cubeG");
    	nextcubenames.push_back("cubeH");
    	nextcubenames.push_back("cubeI");
    	nextcubenames.push_back("cubeJ");
    	
    	world nextcu(nextcubenames,nexc);
    	dataBase_.push_back(nextcu);
    }
    else if(benchmark_=="sort"){
    	vector<string> table3objects{"table3"};
    	vector<string> table4objects{"table4"};
    	std::vector<float> pos;
    	std::vector<string> table1objecs{"table1","stick_blue_1","stick_blue_2","stick_blue_3","stick_green_1","stick_green_2",
    									"stick_green_3","stick_green_4","stick_red_1","stick_red_2","stick_red_3","stick_red_4"
    									,"stick_red_5","stick_red_6","stick_red_7"};

		std::vector<string> table2objects{"table2","stick_blue_4","stick_blue_5","stick_blue_6","stick_blue_7","stick_green_5",
										"stick_green_6","stick_green_7","stick_red_8","stick_red_9","stick_red_10","stick_red_11",
										"stick_red_12","stick_red_13","stick_red_14"};

    	

		world inst1(table1objecs,pos);
		world inst2(table2objects,pos);
		world inst3(table3objects,pos);
		world inst4(table4objects,pos);
		dataBase_.push_back(inst1);
		dataBase_.push_back(inst2);
		dataBase_.push_back(inst3);
		dataBase_.push_back(inst4);
    }
    else if(benchmark_=="nonmonotonic"){
    
    	std::vector<float> pos;
    	std::vector<string> table1objecs{"table1","stick_green_1","stick_green_2","stick_green_3","stick_red_1","stick_red_2",
    									"stick_red_3","stick_red_4"
    									};

		std::vector<string> table2objects{"table2","stick_blue_1","stick_blue_2","stick_blue_3","stick_blue_4"};

    	

		world inst1(table1objecs,pos);
		world inst2(table2objects,pos);
		dataBase_.push_back(inst1);
		dataBase_.push_back(inst2);

    }
    else if(benchmark_=="kitchen"){
    	std::vector<float> pos;
    	std::vector<string> table1objecs{"table1","cabbage_1","cabbage_2","raddish_1","raddish_2","raddish_3","raddish_4"};

		std::vector<string> table2objects{"table2","glass_1","glass_2"};

    	

		world inst1(table1objecs,pos);
		world inst2(table2objects,pos);
		dataBase_.push_back(inst1);
		dataBase_.push_back(inst2);
    }


	for(int i=0;i<dataBase_.size();i++)
		dataBase_[i].print();



	

}

bool tamp_knowledge::removeObjectPanda(tamp_msgs::removeobject::Request &req,tamp_msgs::removeobject::Response &res){
	ROS_INFO("tamp_knowledge::removeObjectPanda");
	std::vector<string> ppvec;
  	boost::split(ppvec, req.object, boost::is_any_of("*"));
    string object = ppvec[0];
    string zone = ppvec[1];
    std::vector<string> objectsinpolygon;
    int databaseindex = 0;
    string objectname;

    for(auto i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]==ppvec[0]){
    		 
             objectname = dataBase_[i].name[1];
             break;
		}
	}

	for(auto it=dataBase_.begin(); it!=dataBase_.end();){
    	if(it->name[0]==ppvec[1]){
    		objectsinpolygon = it->name;
    		 dataBase_.erase(it);
             
             break;
		}
		else
			it++;
	}

	for(auto it=objectsinpolygon.begin(); it!=objectsinpolygon.end();){
		if(ppvec[1]==*it){
			objectsinpolygon.erase(it);
			break;
		}
		else
			it++;
	}

	for(auto it=objectsinpolygon.begin(); it!=objectsinpolygon.end();){
		if(objectname==*it){
			ROS_INFO("Erased %s : %s from  %s",ppvec[0].c_str(),it->c_str(),ppvec[1].c_str());
			objectsinpolygon.erase(it);
			break;
		}
		else
			it++;
	}

	world inst;
	inst.name.push_back(zone);
	for(auto object:objectsinpolygon){
		inst.name.push_back(object);

	}

	dataBase_.push_back(inst);
	ROS_INFO("Updated %s",ppvec[1].c_str());

	

	string complemtaryzone;
	if(ppvec[1] =="r1t1"){
		complemtaryzone = "r1t2";
	}
	else if(ppvec[1] =="r1t2"){
		complemtaryzone = "r1t1";
	}
	else if(ppvec[1] =="r2t1"){
		complemtaryzone = "r2t2";
	}
	else if(ppvec[1] =="r2t2"){
		complemtaryzone = "r2t1";
	}


	objectsinpolygon.clear();

	for(auto it=dataBase_.begin(); it!=dataBase_.end();){
	    	if(it->name[0]==complemtaryzone){
	    		objectsinpolygon = it->name;
	    		 dataBase_.erase(it);
	             
	             break;
			}
			else
				it++;
	}

	for(auto it=objectsinpolygon.begin(); it!=objectsinpolygon.end();){
		if(complemtaryzone==*it){
			objectsinpolygon.erase(it);
			break;
		}
		else
			it++;
	}

	for(auto it=objectsinpolygon.begin(); it!=objectsinpolygon.end();){
		if(objectname==*it){
			ROS_INFO("Erased %s : %s from  %s",ppvec[0].c_str(),it->c_str(),complemtaryzone.c_str());
			objectsinpolygon.erase(it);
			break;
		}
		else
			it++;
	}

	world inst2;
	inst2.name.push_back(complemtaryzone);
	for(auto object:objectsinpolygon){
		inst2.name.push_back(object);

	}

	dataBase_.push_back(inst2);
	ROS_INFO("Updated %s",complemtaryzone.c_str());






		

	res.result = true;


	
return res.result;

}






/*

bool tamp_knowledge::removeObjectPanda(tamp_msgs::removeobject::Request &req,tamp_msgs::removeobject::Response &res){
	ROS_INFO("tamp_knowledge::removeObjectPanda");
	std::vector<string> ppvec;
  	boost::split(ppvec, req.object, boost::is_any_of("*"));
    string object = ppvec[0];
  
    string objectname;

    for(auto i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]==ppvec[0]){
    		 
             objectname = dataBase_[i].name[1];
             break;
		}
	}

	for(auto it=dataBase_.begin(); it!=dataBase_.end();){
    	if(it->name[0]=="r1t1" || it->name[0]=="r1t2" || it->name[0]=="r2t1" || it->name[0]=="r2t2"){
    		ROS_INFO("Inside %s",it->name[0].c_str());
    		 for(auto it2=it->name.begin();it2!=it->name.end();){
    		 	if(*it2==objectname){
    		 		ROS_INFO("Inside %s",*it2->c_str());
    		 		it->name.erase(it2);
    		 		break;
    		 	}
    		 	else{
    		 		it2++;
    		 	}
    		 }
             
             
		}
		else{

			it++;
		}
	}


	


		

	res.result = true;


	
return res.result;

}

*/

bool tamp_knowledge::tampHanoiPP(tamp_msgs::hanoipp::Request &req,tamp_msgs::hanoipp::Response &res){
	ROS_INFO("tamp_knowledge::tampHanoiPP");

	if(benchmark_=="hanoi"){
			string frompig = "pig"+req.from;
		string topig = "pig"+ req.to;

		std::vector<float> v1,v2;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]==frompig){
		             v1 = dataBase_[i].value;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]==topig){
		             v2 = dataBase_[i].value;
		             break;
			}
		}
		int lastfrom;
		for(int i =numberofdiks_-1;i>=0;i--){
			if((int)v1[i]!=0){
				lastfrom = v1[i];
				v1[i] = 0;
				cout<<frompig<<" :found non zero value in frompig with index: "<<i+1<<endl;
				break;
			}
		}
		for (int i = 0; i < numberofdiks_; ++i)
		{
			if((int)v2[i]==0){
				v2[i] = lastfrom;
				cout<<topig<<" :found zero value in topig with index: "<<i+1<<endl;
				break;
			}
		}

		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]==frompig){
		             dataBase_[i].value = v1;
		             dataBase_[i].print();
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]==topig){
		             dataBase_[i].value = v2;
		             dataBase_[i].print();
		             break;
			}
		}
		res.result = true;
	}
	else if(benchmark_=="cubeworld"){
		std::vector<string> redtray,redtraynew,bluetray,bluetraynew,traycenter,traycenternew,nextcube,nextcubenew;

		for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="nextcube"){
			             nextcube = dataBase_[i].name;
			             break;
				}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="trayr"){
			             redtray = dataBase_[i].name;
			             break;
				}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="trayb"){
			             bluetray = dataBase_[i].name;
			             break;
				}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="trayc"){
			             traycenter = dataBase_[i].name;
			             break;
				}
		}
		string frompig = "tray"+req.from;
		string topig = "tray"+ req.to;
		vector<float> Pose;

		if(req.to=="next"){
			
			for (int i = 0; i < nextcube.size(); ++i)
			{
				if(i!=1){
					nextcubenew.push_back(nextcube[i]);
				}
			}
			removeDataBaseInstant("nextcube",false);
			
			world temp_point(nextcubenew,Pose);
			dataBase_.push_back(temp_point);

			for (auto element:cmap)
			{
				if(element.second==nextcube[1]){
					removeDataBaseInstant("trayr",false);
					redtray.push_back(element.second);

					cmap.erase(element.first);
					break;
				}
			}
			printMap(cmap);
			
			world temp_pointred(redtray,Pose);
			dataBase_.push_back(temp_pointred);
			
			for (int i = 0; i < traycenter.size(); ++i)
			{
				if(traycenter[i]!=nextcube[1]){
					traycenternew.push_back(traycenter[i]);
				}
			}

			removeDataBaseInstant("trayc",false);
			world temp_pointvc(traycenternew,Pose);
			dataBase_.push_back(temp_pointvc);


		}
		else{

			if(req.from=="c"){
				if(req.to=="b"){
					int maxvalue = 2;
					string elementst;
					for (auto element:cmap)
					{
						if(vlauemap[element.second]>maxvalue && element.second!=nextcube[1]){
							maxvalue = vlauemap[element.second];
							elementst = element.second;
						}
					}

					for (int i = 0; i < traycenter.size(); ++i)
					{
						if(traycenter[i]!=elementst){
							traycenternew.push_back(traycenter[i]);
						}
					}
					removeDataBaseInstant("trayc",false);
					world temp_pointvc(traycenternew,Pose);
					dataBase_.push_back(temp_pointvc);

					for (auto it = cmap.begin();it != cmap.end();++it)
					{
						if(it->second==elementst){
							cmap.erase(it);
							break;
						}

					}
					printMap(cmap);

					bluetray.push_back(elementst);

					removeDataBaseInstant("trayb",false);
					world temp_pointvb(bluetray,Pose);
					dataBase_.push_back(temp_pointvb);


				}
				else if(req.to=="r"){
					int minvalue = 11;
					string elementst;
					for (auto element:cmap)
					{
						if(vlauemap[element.second]<=minvalue && element.second!=nextcube[1]){
							minvalue = vlauemap[element.second];
							elementst = element.second;
						}
					}

					for (int i = 0; i < traycenter.size(); ++i)
					{
						if(traycenter[i]!=elementst){
							traycenternew.push_back(traycenter[i]);
						}
					}
					removeDataBaseInstant("trayc",false);
					world temp_pointvc(traycenternew,Pose);
					dataBase_.push_back(temp_pointvc);

					for (auto it = cmap.begin();it != cmap.end();++it)
					{
						if(it->second==elementst){
							cmap.erase(it);
							break;
						}

					}
					printMap(cmap);

					redtray.push_back(elementst);

					removeDataBaseInstant("trayr",false);
					world temp_pointvb(redtray,Pose);
					dataBase_.push_back(temp_pointvb);

				}

			}
			if(req.to=="c"){
				std::vector<string> v1,v2;
				for(std::size_t i=0;i<dataBase_.size();i++){
					if(dataBase_[i].name[0]==frompig){
				        v1 = dataBase_[i].name;
				        break;
					}
				}
				int lastele;
				for (int i = 1; i <7; ++i)
				{	bool exits = false;
					for (const auto element:cmap){
						if(element.first==i){
								exits = true;
						}
					}
					if(!exits){
						lastele = i;
						break;

					}	
				}
				cmap.insert({lastele,v1.back()});
				printMap(cmap);
				for (int i = 0; i < v1.size()-1; ++i)
				{
					v2.push_back(v1[i]);
				}

				removeDataBaseInstant(frompig,false);
				world temp_pointvc(v2,Pose);
				dataBase_.push_back(temp_pointvc);
				traycenter.push_back(v1.back());
				removeDataBaseInstant("trayc",false);
				world temp_pointvec(traycenter,Pose);
				dataBase_.push_back(temp_pointvec);

			}
			
			

		}
	

		tamp_msgs::updatescene updsrv;
		updateSceneServerBenchmark(updsrv.request,updsrv.response);
		res.result = true;
	}
	else if(benchmark_=="sort"){
		std::vector<string> clob,table,newtable,table2,newtable2;
		std::vector<float> fakepos;
		if(req.from=="a"){
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="clob1"){
			             clob = dataBase_[i].name;
			             break;
				}
			}
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="table1"){
			             table = dataBase_[i].name;
			             break;
				}
			}
			//newtable.push_back("table1");
			for (int i = 0; i < table.size(); ++i)
			{
				if(table[i]!=clob[1]){
					newtable.push_back(table[i]);
				}
			}
			removeDataBaseInstant("table1",false);
			world inst(newtable,fakepos);
			dataBase_.push_back(inst);
			if(req.to=="c"){
				for(std::size_t i=0;i<dataBase_.size();i++){
					if(dataBase_[i].name[0]=="table3"){
				             table2 = dataBase_[i].name;
				             break;
					}
				}
				table2.push_back(clob[1]);
				removeDataBaseInstant("table3",false);
				world inst(table2,fakepos);
				dataBase_.push_back(inst);
			}
			else if(req.to=="d"){
				for(std::size_t i=0;i<dataBase_.size();i++){
					if(dataBase_[i].name[0]=="table4"){
				             table2 = dataBase_[i].name;
				             break;
					}
				}
				table2.push_back(clob[1]);
				removeDataBaseInstant("table4",false);
				world inst(table2,fakepos);
				dataBase_.push_back(inst);

			}

		}
		else if(req.from=="b"){
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="clob2"){
			             clob = dataBase_[i].name;
			             break;
				}
			}
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="table2"){
			             table = dataBase_[i].name;
			             break;
				}
			}
			//newtable.push_back("table2");
			for (int i = 0; i < table.size(); ++i)
			{
				if(table[i]!=clob[1]){
					newtable.push_back(table[i]);
				}
			}
			removeDataBaseInstant("table2",false);
			world inst(newtable,fakepos);
			dataBase_.push_back(inst);
			if(req.to=="c"){
				for(std::size_t i=0;i<dataBase_.size();i++){
					if(dataBase_[i].name[0]=="table3"){
				             table2 = dataBase_[i].name;
				             break;
					}
				}
				table2.push_back(clob[1]);
				removeDataBaseInstant("table3",false);
				world inst(table2,fakepos);
				dataBase_.push_back(inst);
			}
			else if(req.to=="d"){
				for(std::size_t i=0;i<dataBase_.size();i++){
					if(dataBase_[i].name[0]=="table4"){
				             table2 = dataBase_[i].name;
				             break;
					}
				}
				table2.push_back(clob[1]);
				removeDataBaseInstant("table4",false);
				world inst(table2,fakepos);
				dataBase_.push_back(inst);

			}
		}
		res.result = true;
	}
	else if(benchmark_=="nonmonotonic"){
		std::vector<string> table1,table2,table1new,allgreenobjects;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table1"){
		             table1 = dataBase_[i].name;
		             break;
			}
		}
		ROS_INFO("receieved table1");
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table2"){
		             table2 = dataBase_[i].name;
		             break;
			}
		}
		ROS_INFO("receieved table2");
		for (int i = 0; i < table1.size(); ++i)
		{
			std::vector<string> stickcolornu;
	  		boost::split(stickcolornu, table1[i], boost::is_any_of("_"));
	  		if(stickcolornu.size()>1){
	  			if(stickcolornu[1]=="green"){
	  				allgreenobjects.push_back(table1[i]);
	  			}
	  		}
	  		
		}
		ROS_INFO("receieved all green  objects");
		int minio =1;
		string bigestgreen;
		for (int i = 0; i < allgreenobjects.size(); ++i)
		{
			std::vector<string> stickcolornu;
	  		boost::split(stickcolornu, allgreenobjects[i], boost::is_any_of("_"));
	  		if(stoi(stickcolornu[2])>=minio){
	  			minio = stoi(stickcolornu[2]);
	  			bigestgreen = allgreenobjects[i];
	  		}
		}
		ROS_INFO("receieved biggest green object %s",bigestgreen.c_str());
		table2.push_back(bigestgreen);
		removeDataBaseInstant("table2",false);
		std::vector<float> fakepos;
		world inst(table2,fakepos);
		dataBase_.push_back(inst);

		for (int i = 0; i < table1.size(); ++i)
		{
			if(table1[i]!=bigestgreen){
				table1new.push_back(table1[i]);
			}
		}

		removeDataBaseInstant("table1",false);
		world inst2(table1new,fakepos);
		dataBase_.push_back(inst2);


		res.result = true;

	}
	else if(benchmark_=="kitchen"){

		std::vector<string> table1,table2,table1new,allgreenobjects;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table1"){
		             table1 = dataBase_[i].name;
		             break;
			}
		}
		ROS_INFO("receieved table1");
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table2"){
		             table2 = dataBase_[i].name;
		             break;
			}
		}
		ROS_INFO("receieved table2");
		for (int i = 0; i < table1.size(); ++i)
		{
			std::vector<string> stickcolornu;
	  		boost::split(stickcolornu, table1[i], boost::is_any_of("_"));
	  		if(stickcolornu.size()>1){
	  			if(stickcolornu[0]=="cabbage"){
	  				allgreenobjects.push_back(table1[i]);
	  				
	  			}
	  		}
	  		
		}
		ROS_INFO("receieved all green  objects");
		int minio =2;
		string bigestgreen;
		for (int i = 0; i < allgreenobjects.size(); ++i)
		{
			std::vector<string> stickcolornu;
	  		boost::split(stickcolornu, allgreenobjects[i], boost::is_any_of("_"));
	  		if(stoi(stickcolornu[1])<=minio){

	  			minio = stoi(stickcolornu[1]);
	  			cout<<"min: "<<allgreenobjects[i]<<endl;
	  			bigestgreen = allgreenobjects[i];
	  		}
		}
		ROS_INFO("receieved smallest cabbage %s",bigestgreen.c_str());
		table2.push_back(bigestgreen);
		removeDataBaseInstant("table2",false);
		std::vector<float> fakepos;
		world inst(table2,fakepos);
		dataBase_.push_back(inst);

		for (int i = 0; i < table1.size(); ++i)
		{
			if(table1[i]!=bigestgreen){
				table1new.push_back(table1[i]);
			}
		}

		removeDataBaseInstant("table1",false);
		world inst2(table1new,fakepos);
		dataBase_.push_back(inst2);
		//tamp_msgs::updatescene updsrv;
		//updateSceneServerBenchmark(updsrv.request,updsrv.response);

		res.result = true;
	}




return true;

}

bool tamp_knowledge::updateSceneServerBenchmark(tamp_msgs::updatescene::Request &req,tamp_msgs::updatescene::Response &res){
	ROS_INFO("tamp_knowledge::updateSceneServerBenchmark");
	nh.getParam("/benchmark",benchmark_);
	int numofdisks;
	if(benchmark_=="hanoi"){
		ROS_INFO("updating scene for hanoi benchmark");
		nh.getParam("/disks",numofdisks);
		std::vector<float> v1,v2,v3;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="pigA"){
		             v1 = dataBase_[i].value;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="pigB"){
		             v2 = dataBase_[i].value;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="pigC"){
		             v3 = dataBase_[i].value;
		             break;
			}
		}
		int na = 0;
		int nb =0;
		int nc=0;
		for (int i = 0; i < numofdisks; ++i)
		{	if((int)v1[i]!=0){
				na++;
							
			}
			if((int)v2[i]!=0){
				nb++;
							
			}
			if((int)v3[i]!=0){
				nc++;
							
			}
		}

		cout<<"na: "<<na<<" nb: "<<nb<<" nc: "<<nc<<endl;

		updateGraspingHanoi(na,nb,nc);
		res.result = true;
		



	}
	else if(benchmark_=="cubeworld"){

		std::vector<string> v1,v2,v3;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="trayb"){
		             v1 = dataBase_[i].name;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="trayc"){
		             v2 = dataBase_[i].name;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="trayr"){
		             v3 = dataBase_[i].name;
		             break;
			}
		}
		updateGraspingCubeWorld(v1,v2,v3);
		res.result = true;
	}
	else if(benchmark_=="sort"){
		std::vector<string> v1,v2;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table1"){
		             v1 = dataBase_[i].name;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table2"){
		             v2 = dataBase_[i].name;
		             break;
			}
		}

		updateGraspingSort(v1,v2);
		res.result = true;
	}
	else if(benchmark_=="nonmonotonic"){
		std::vector<string> v1,v2;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table1"){
		             v1 = dataBase_[i].name;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table2"){
		             v2 = dataBase_[i].name;
		             break;
			}
		}

		updateGraspingNonMonotonic(v1,v2);
		res.result = true;
	}
	else if(benchmark_=="kitchen"){
		std::vector<string> v1,v2;
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table1"){
		             v1 = dataBase_[i].name;
		             break;
			}
		}
		for(std::size_t i=0;i<dataBase_.size();i++){
			if(dataBase_[i].name[0]=="table2"){
		             v2 = dataBase_[i].name;
		             break;
			}
		}

		updateGraspingKitchen(v1,v2);
		res.result = true;
	}
		
    	return true;

}
void tamp_knowledge::updateGraspingKitchen(std::vector<string> &v1,std::vector<string> &v2){
	ROS_INFO("tamp_knowledge::updateGraspingKitchen");
	
	int nuofgreeninv1=0;
	for (int i = 1; i < v1.size(); ++i)
	{
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, v1[i], boost::is_any_of("_"));
  		if(stickcolornu[0]=="cabbage"){
  			nuofgreeninv1++;
  		}
  		
	}
	cout<<"There are "<<nuofgreeninv1<<" green objects on table 1"<<endl;

	string raddishobject = "raddish_"+to_string(1+nuofgreeninv1);
	string cabbageobject = "cabbage_"+to_string(3-nuofgreeninv1);
	string glassobject = "glass_"+to_string(3-nuofgreeninv1);
	cout<<"Raddish object: "<<raddishobject<<endl;
	cout<<"cabbage object: "<<cabbageobject<<endl;
	cout<<"glass object: "<<glassobject<<endl;
	std::vector<double> raddishpos,cabbagepos,glasspos;
	std::vector<double> objposes,objectorie;
	tamp_msgs::objectssrv obsr;
	std::vector<string> objnames;
	if(BenchmarkPoseClinet.call(obsr)){
		objnames = obsr.response.names;
		objposes = obsr.response.position;
		objectorie = obsr.response.orientation;

	}
	for (int i = 0; i < objnames.size(); ++i)
	{
		if(objnames[i]==raddishobject){
			raddishpos.push_back(objposes[3*i]-0.04);
			raddishpos.push_back(objposes[3*i+1]);
			raddishpos.push_back(objposes[3*i+2]);
		}
		if(objnames[i]==cabbageobject){
			cabbagepos.push_back(objposes[3*i]);
			cabbagepos.push_back(objposes[3*i+1]);
			cabbagepos.push_back(objposes[3*i+2]);
		}
		if(objnames[i]==glassobject){
			glasspos.push_back(objposes[3*i]);
			glasspos.push_back(objposes[3*i+1]);
			glasspos.push_back(objposes[3*i+2]);
		}
	
	}
	std::vector<float> vpose;
	std::vector<string> redvec,greenvec,bluevec;
	removeDataBaseInstant("raddish",false);
	redvec.push_back("raddish");
	redvec.push_back(raddishobject);
	removeDataBaseInstant("glass",false);
	bluevec.push_back("glass");
	bluevec.push_back(glassobject);
	removeDataBaseInstant("cabbage",false);
	greenvec.push_back("cabbage");
	greenvec.push_back(cabbageobject);
	world inst1(redvec,vpose);
	world inst2(bluevec,vpose);
	world inst3(greenvec,vpose);
	dataBase_.push_back(inst1);
	dataBase_.push_back(inst2);
	dataBase_.push_back(inst3);



	std::vector<string> places{"raddishr","raddishl","raddishrpl","raddishlpl",
								"cabbager","cabbagel","cabbagelpl","cabbagerpl","cabbagelplw","cabbagerplw",
								"cabbagelplc","cabbagerplc","cabbagelplt","cabbagerplt",
								"glassr","glassl","glassrplw","glasslplw","glassrplt","glasslplt"};

   



	for (int i = 0; i < places.size(); ++i){
		string pregraspname = places[i]+"-pregrasp";
		string graspname = places[i]+"-grasp";
		string postgraspname = places[i]+"-postgrasp";
		removeDataBaseInstant(pregraspname,false);
		removeDataBaseInstant(graspname,false);
		removeDataBaseInstant(postgraspname,false);
		double preandgraspz,postgraspz,Xg,Yg,Xpg,Ypg,graspz,psi,X,Y;
		double dispregrasp = 0.1;
		double disgrasp = 0.01;
		
		postgraspz = 1.1;
		double Yo = 0.3;
		if(i==0 || i==1){
			X = raddishpos[0];
			Y = raddishpos[1];
			preandgraspz=graspz=0.84; 
			psi = 0.0;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==2 || i==3){
			X = raddishpos[0];
			if(nuofgreeninv1==2){
				Y = raddishpos[1] - 0.15;
			}
			else{
				Y = raddishpos[1] + 0.15;
			}
			
			preandgraspz=graspz=0.84; 
			psi = 0.0;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==4 || i==5){
			X = cabbagepos[0];
			Y = cabbagepos[1];
			preandgraspz=graspz=0.84; 
			psi = 0.0;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
			
		}
		else if(i==6 || i==7){
			X = cabbagepos[0]-0.2;
			if(nuofgreeninv1==2){
				Y = cabbagepos[1]+0.1;
			}
			else{
				Y = cabbagepos[1]-0.1;
			}
			preandgraspz=graspz=0.84; 
			psi = 0.0;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==8 ){
			X = 2.1;
			if(nuofgreeninv1==2){
				Y = -1.3;
			}
			else{
				Y = -1.3;
			}
			preandgraspz=graspz=0.96;
			psi = -0.4;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==9){
			X = 2.1;
			if(nuofgreeninv1==2){
				Y = -1.3;
			}
			else{
				Y = -1.3;
			}
			preandgraspz=graspz=0.96;
			psi = 0.4;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==10 || i==11){
			X = 2.0;
			if(nuofgreeninv1==2){
				Y = -0.5;
			}
			else{
				Y = -0.3;
			}
			preandgraspz=graspz=1.11;
			psi = 0.0;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==12 || i==13){
			Y = -2.0;
			if(nuofgreeninv1==2){
				X = 0.7;
			}
			else{
				X = 1.3;
			}
			preandgraspz=graspz=0.84;
			psi = -1.57;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==14 || i==15){
			X = glasspos[0];
			Y = glasspos[1] ; 
			psi = -1.57;
			preandgraspz=graspz=0.84;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==16 || i==17){
			X = 2.0;
			if(nuofgreeninv1==2){
				Y = -1.4;
			}
			else{
				Y = -1.2;
			}
			preandgraspz=graspz=0.96;
			psi = 0.0;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==18 || i==19){
			Y = -1.9;
			if(nuofgreeninv1==2){
				X = 0.7;
			}
			else{
				X = 1.3;
			}
			preandgraspz=graspz=0.84;
			psi = -1.57;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		

		world inst,instgrasp,instpostgrasp;
		inst.name.push_back(pregraspname);
		inst.value.push_back(Xpg);
		inst.value.push_back(Ypg);
		inst.value.push_back(graspz);
		inst.value.push_back(0.0);
		inst.value.push_back(0.0);
		inst.value.push_back(psi);
		inst.print();
		dataBase_.push_back(inst);

		instgrasp.name.push_back(graspname);
		instgrasp.value.push_back(Xg);
		instgrasp.value.push_back(Yg);
		instgrasp.value.push_back(graspz);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(psi);
		instgrasp.print();
		dataBase_.push_back(instgrasp);

		instpostgrasp.name.push_back(postgraspname);
		instpostgrasp.value.push_back(Xpg);
		instpostgrasp.value.push_back(Ypg);
		instpostgrasp.value.push_back(graspz);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(psi);
		instpostgrasp.print();
		dataBase_.push_back(instpostgrasp);
	
	}




}
void tamp_knowledge::updateGraspingNonMonotonic(std::vector<string> &v1,std::vector<string> &v2){
	ROS_INFO("tamp_knowledge::updateGraspingNonMonotonic");
	int nuofgreeninv1=0;
	for (int i = 1; i < v1.size(); ++i)
	{
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, v1[i], boost::is_any_of("_"));
  		if(stickcolornu[1]=="green"){
  			nuofgreeninv1++;
  		}
  		
	}
	cout<<"There are "<<nuofgreeninv1<<" green objects on table 1"<<endl;

	string redobject = "stick_red_"+to_string(nuofgreeninv1+1);
	string greenobject = "stick_green_"+to_string(nuofgreeninv1);
	string blueobject = "stick_blue_"+to_string(nuofgreeninv1+1);
	string greentarget = "target_green_"+to_string(nuofgreeninv1);
	cout<<"Red object: "<<redobject<<endl;
	cout<<"Green object: "<<greenobject<<endl;
	cout<<"Blue object: "<<blueobject<<endl;
	cout<<"Green Target object: "<<greentarget<<endl;
	std::vector<double> redpos,greenpos,bluepos,greentargetpos;
	std::vector<double> objposes,objectorie;
	tamp_msgs::objectssrv obsr;
	std::vector<string> objnames;
	if(BenchmarkPoseClinet.call(obsr)){
		objnames = obsr.response.names;
		objposes = obsr.response.position;
		objectorie = obsr.response.orientation;

	}
	for (int i = 0; i < objnames.size(); ++i)
	{
		if(objnames[i]==redobject){
			redpos.push_back(objposes[3*i]-0.04);
			redpos.push_back(objposes[3*i+1]);
			redpos.push_back(objposes[3*i+2]);
		}
		if(objnames[i]==greenobject){
			greenpos.push_back(objposes[3*i]);
			greenpos.push_back(objposes[3*i+1]);
			greenpos.push_back(objposes[3*i+2]);
		}
		if(objnames[i]==blueobject){
			bluepos.push_back(objposes[3*i]);
			bluepos.push_back(objposes[3*i+1]);
			bluepos.push_back(objposes[3*i+2]);
		}
		if(objnames[i]==greentarget){
			greentargetpos.push_back(objposes[3*i]);
			greentargetpos.push_back(objposes[3*i+1]);
			greentargetpos.push_back(objposes[3*i+2]);
		}

	}
	std::vector<float> vpose;
	std::vector<string> redvec,greenvec,bluevec;
	removeDataBaseInstant("red",false);
	redvec.push_back("red");
	redvec.push_back(redobject);
	removeDataBaseInstant("blue",false);
	bluevec.push_back("blue");
	bluevec.push_back(blueobject);
	removeDataBaseInstant("green",false);
	greenvec.push_back("green");
	greenvec.push_back(greenobject);
	world inst1(redvec,vpose);
	world inst2(bluevec,vpose);
	world inst3(greenvec,vpose);
	dataBase_.push_back(inst1);
	dataBase_.push_back(inst2);
	dataBase_.push_back(inst3);

	std::vector<string> places{"redr","redl","redlpll","redrpll","redlplr","redrplr",
								"greenr","greenl","greenlpll","greenrpll","greenlplr","greenrplr",
								"greenlpll2","greenrpll2","greenlplr2","greenrplr2",
								"bluer","bluel","bluerplr","bluelplr","bluerpll","bluelpll",
								"targetgreenr","targetgreenl"};

for (int i = 0; i < places.size(); ++i)
{
	string pregraspname = places[i]+"-pregrasp";
	string graspname = places[i]+"-grasp";
	string postgraspname = places[i]+"-postgrasp";
	removeDataBaseInstant(pregraspname,false);
	removeDataBaseInstant(graspname,false);
	removeDataBaseInstant(postgraspname,false);
	double preandgraspz,postgraspz,Xg,Yg,Xpg,Ypg,graspz,psi,X,Y;
	double dispregrasp = 0.1;
	double disgrasp = 0.02;
	preandgraspz=graspz=0.86;
	postgraspz = 1.1;
	double Yo = 0.3;
	if(i==0 || i==1){
		X = redpos[0];
		Y = redpos[1]; 
		psi = 0.0;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==2 || i==3){
		X = redpos[0];
		Y = redpos[1]+0.15; 
		psi = 0.0;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==4 || i==5){
		X = redpos[0];
		Y = redpos[1]-0.15; 
		psi = 0.0;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
		
	}
	else if(i==6 || i==7){
		X = greenpos[0];
		Y = greenpos[1]; 
		psi = 0.0;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==8 || i==9){
		X = greenpos[0];
		Y = greenpos[1] +0.15; 
		psi = 0.0;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==10 || i==11){
		X = greenpos[0];
		Y = greenpos[1] -0.15; 
		psi = 0.0;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==12 || i==13){
		X = bluepos[0]+0.15;
		Y = bluepos[1] ; 
		psi = -1.57;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==14 || i==15){
		X = bluepos[0]-0.15;
		Y = bluepos[1] ; 
		psi = -1.57;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==16 || i==17){
		X = bluepos[0];
		Y = bluepos[1]; 
		psi = -1.57;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==18 || i==19){
		X = bluepos[0]-0.15;
		Y = bluepos[1]; 
		psi = -1.57;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}
	else if(i==20 || i==21){
		X = bluepos[0]+0.15;
		Y = bluepos[1]; 
		psi = -1.57;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
		
	}
	else if(i==22 || i==23){
		X = greentargetpos[0];
		Y = greentargetpos[1]; 
		psi = -1.57;
		Xpg = X - dispregrasp * cos(psi);
		Ypg = Y - dispregrasp * sin(psi);
		Xg = X - disgrasp * cos(psi);
		Yg = Y - disgrasp * sin(psi);
	}

	world inst,instgrasp,instpostgrasp;
	inst.name.push_back(pregraspname);
	inst.value.push_back(Xpg);
	inst.value.push_back(Ypg);
	inst.value.push_back(preandgraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(0.0);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);

	instgrasp.name.push_back(graspname);
	instgrasp.value.push_back(Xg);
	instgrasp.value.push_back(Yg);
	instgrasp.value.push_back(graspz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);

	instpostgrasp.name.push_back(postgraspname);
	instpostgrasp.value.push_back(Xpg);
	instpostgrasp.value.push_back(Ypg);
	instpostgrasp.value.push_back(preandgraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);
	
}

removeDataBaseInstant("table1pos",false);
removeDataBaseInstant("table2pos",false);
std::vector<float> v11,v22;
float newpos1 = -0.25 + (3-nuofgreeninv1)* 0.25;
v11.push_back(1.35);
v11.push_back(newpos1);
v11.push_back(0.0);
float newpos2 = 0.75 + (3-nuofgreeninv1)* 0.25;
v22.push_back(newpos2);
v22.push_back(-1.25);
v22.push_back(-1.57);
std::vector<string> name1,name2;
name1.push_back("table1pos");
name2.push_back("table2pos");
world insttable1(name1,v11);
world insttable2(name2,v22);
dataBase_.push_back(insttable1);
dataBase_.push_back(insttable2);





}
void tamp_knowledge::updateGraspingSort(std::vector<string> &v1,std::vector<string> &v2){

	ROS_INFO("tamp_knowledge::updateGraspingSort");
	tamp_msgs::objectssrv obsr;
	std::vector<string> objnames;
	std::vector<double> objposes,objectorie,objeposontable1,objeposontable2;
	if(BenchmarkPoseClinet.call(obsr)){
		objnames = obsr.response.names;
		objposes = obsr.response.position;
		objectorie = obsr.response.orientation;

	}
	ROS_INFO("BenchmarkPoseClinet.call(obsr)");
	for (int i = 1; i < v1.size(); ++i)
	{
		for (int j = 0; j < objnames.size(); ++j)
		{
			if(v1[i]==objnames[j]){
				objeposontable1.push_back(objposes[3*j]);
				objeposontable1.push_back(objposes[3*j+1]);
				objeposontable1.push_back(objposes[3*j+2]);

			}
		}
	}
	ROS_INFO("v1 objects found");
	for (int i = 1; i < v2.size(); ++i)
	{
		for (int j = 0; j < objnames.size(); ++j)
		{
			if(v2[i]==objnames[j]){
				objeposontable2.push_back(objposes[3*j]);
				objeposontable2.push_back(objposes[3*j+1]);
				objeposontable2.push_back(objposes[3*j+2]);

			}
		}
	}
	ROS_INFO("v2 objects found");
	string clob1,clo1,clob2,clo2;
	std::vector<double> clob1pos,clo1pos,clob2pos,clo2pos;
	double dismin = 1.0;

	for (int i = 1; i < v1.size(); ++i)
	{
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, v1[i], boost::is_any_of("_"));
  		if(stickcolornu[1]!="red"){
  			double dis = sqrt(pow(objeposontable1[3*i-3]-1.4,2)+pow(objeposontable1[3*i-2],2));
  			if(dis<=dismin){
  				dismin = dis;
  				clob1 = v1[i];
  				clob1pos.clear();
  				clob1pos.push_back(objeposontable1[3*i-3]);
  				clob1pos.push_back(objeposontable1[3*i-2]);
  				clob1pos.push_back(objeposontable1[3*i-1]);
  			}
  		}
  		
	}
	if(clob1pos.empty()){
		clob1 = "stick_blue_7";
		clob1pos = {0.0,0.0,0.0};
	}
	ROS_INFO("clob1 object found: %s, pose: %f, %f, %f",clob1.c_str(),clob1pos[0],clob1pos[1],clob1pos[2]);
	double disred=2.5;
	for (int i = 1; i < v1.size(); ++i){
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, v1[i], boost::is_any_of("_"));
  		double xobj = clob1pos[0];
  		double yobj = clob1pos[1];
		if(stickcolornu[1]=="red"){
			double xobs = objeposontable1[3*i-3];
			double yobs = objeposontable1[3*i-2];
			//double dis = sqrt(pow(clob1pos[0]-objeposontable1[3*i-3],2)+pow(clob1pos[1]-objeposontable1[3*i-2],2));
  			if(xobs<=disred && fabs(yobs-yobj)<0.15){
  				disred = xobs;
  				clo1 = v1[i];
  				clo1pos.clear();
  				clo1pos.push_back(objeposontable1[3*i-3]);
  				clo1pos.push_back(objeposontable1[3*i-2]);
  				clo1pos.push_back(objeposontable1[3*i-1]);
  			}
		}
	}
	if(clo1pos.empty()){
		clo1 = clob1;
		clo1pos = clob1pos;
	}
	ROS_INFO("clo1 object found: %s, pose: %f, %f, %f",clo1.c_str(),clo1pos[0],clo1pos[1],clo1pos[2]);

		double dismin2 = 1.0;
	for (int i = 1; i < v2.size(); ++i)
	{
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, v2[i], boost::is_any_of("_"));
  		if(stickcolornu[1]!="red"){
  			double dis = sqrt(pow(objeposontable2[3*i-3]+1.4,2)+pow(objeposontable2[3*i-2],2));
  			if(dis<=dismin2){
  				dismin2 = dis;
  				clob2 = v2[i];
  				clob2pos.clear();
  				clob2pos.push_back(objeposontable2[3*i-3]);
  				clob2pos.push_back(objeposontable2[3*i-2]);
  				clob2pos.push_back(objeposontable2[3*i-1]);
  			}
  		}
  		
	}
	if(clob2pos.empty()){
		clob2 = "stick_blue_7";
		clob2pos = {0.0,0.0,0.0};
	}
	ROS_INFO("clob2 object found: %s, pose: %f, %f, %f",clob2.c_str(),clob2pos[0],clob2pos[1],clob2pos[2]);
	double disred2=-2.5;
	for (int i = 1; i < v2.size(); ++i){
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, v2[i], boost::is_any_of("_"));
  		double xobj = clob2pos[0];
  		double yobj = clob2pos[1];
		if(stickcolornu[1]=="red"){
			double xobs = objeposontable2[3*i-3];
			double yobs = objeposontable2[3*i-2];
			//double dis = sqrt(pow(clob2pos[0]-objeposontable2[3*i-3],2)+pow(clob2pos[1]-objeposontable2[3*i-2],2));
  			if(xobs>=disred2 && fabs(yobs-yobj)<0.15){
  				disred2 = xobs;
  				clo2 = v2[i];
  				clo2pos.clear();
  				clo2pos.push_back(objeposontable2[3*i-3]);
  				clo2pos.push_back(objeposontable2[3*i-2]);
  				clo2pos.push_back(objeposontable2[3*i-1]);
  			}
		}
	}
	if(clo2pos.empty()){
		clo2 = clob2;
		clo2pos = clob2pos;
	}
	ROS_INFO("clo2 object found: %s, pose: %f, %f, %f",clo2.c_str(),clo2pos[0],clo2pos[1],clo2pos[2]);
	std::vector<float> vpose;
	std::vector<string> clob1vec,clo1vec,clob2vec,clo2vec;
	removeDataBaseInstant("clob1",false);
	clob1vec.push_back("clob1");
	clob1vec.push_back(clob1);
	removeDataBaseInstant("clob2",false);
	clob2vec.push_back("clob2");
	clob2vec.push_back(clob2);
	removeDataBaseInstant("clo1",false);
	clo1vec.push_back("clo1");
	clo1vec.push_back(clo1);
	removeDataBaseInstant("clo2",false);
	clo2vec.push_back("clo2");
	clo2vec.push_back(clo2);
	world inst1(clob1vec,vpose);
	world inst2(clob2vec,vpose);
	world inst3(clo1vec,vpose);
	world inst4(clo2vec,vpose);
	dataBase_.push_back(inst1);
	dataBase_.push_back(inst2);
	dataBase_.push_back(inst3);
	dataBase_.push_back(inst4);

	std::vector<string> places{"clob1r","clob1l","clo1rp","clo1lp","clo1rpl","clo1lpl","clob2r","clob2l","clo2rp","clo2lp",
							   "clo2rpl","clo2lpl","table3p","table4p","table3pl","table4pl","clo1rpl2","clo1lpl2","clo2rpl2","clo2lpl2"};

	for (int i = 0; i < places.size(); ++i)
	{
		string pregraspname = places[i]+"-pregrasp";
		string graspname = places[i]+"-grasp";
		string postgraspname = places[i]+"-postgrasp";
		removeDataBaseInstant(pregraspname,false);
		removeDataBaseInstant(graspname,false);
		removeDataBaseInstant(postgraspname,false);
		double preandgraspz,postgraspz,Xg,Yg,Xpg,Ypg,graspz,psi,X,Y;
		double dispregrasp = 0.1;
		double disgrasp = 0.02;
		preandgraspz=graspz=0.86;
		postgraspz = 1.1;
		double Yo = 0.3;
		if(i==0){
			X = clob1pos[0];
			Y = clob1pos[1]; 
			psi = atan((Yo-Y)/X);
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==1){
			X = clob1pos[0];
			Y = clob1pos[1]; 
			psi = -atan((Yo-Y)/X);
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==2){
			X = clo1pos[0];
			Y = clo1pos[1]; 
			psi = fabs(atan((Yo-Y)/(X-1.4)));
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==3){
			X = clo1pos[0];
			Y = clo1pos[1]; 
			psi = -fabs(atan((Yo-Y)/(X-1.4)));
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==4){
			X = clo1pos[0];
			Y = clo1pos[1] - 0.15;
			psi = atan((Yo-Y)/X);
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
			
		}
		else if(i==5){
		
			X = clo1pos[0];
			Y = clo1pos[1] - 0.15;
			
			psi = -atan((Yo-Y)/X);
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);

		}
		else if(i==6){
			X = clob2pos[0];
			Y = clob2pos[1]; 
			psi = -3.14 + fabs(atan((Yo-Y)/(fabs(X+1.4))));
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==7){
			X = clob2pos[0];
			Y = clob2pos[1]; 
			psi = 3.14 - fabs(atan((Yo-Y)/(fabs(X+1.4))));
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
			
		}
		else if(i==8){
			X = clo2pos[0];
			Y = clo2pos[1]; 
			psi = -3.14 + atan((Yo-Y)/(fabs(X+1.4)));
			Xpg = X + dispregrasp * fabs(cos(psi));
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X + disgrasp * fabs(cos(psi));
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==9){
			X = clo2pos[0];
			Y = clo2pos[1]; 
			psi = 3.14 - atan((Yo-Y)/(fabs(X+1.4)));
			Xpg = X + dispregrasp * fabs(cos(psi));
			Ypg = Y + dispregrasp * sin(psi);
			Xg = X + disgrasp * fabs(cos(psi));
			Yg = Y + disgrasp * sin(psi);
		}
		else if(i==10){
			
			X = clo2pos[0] ;
			Y = clo2pos[1] - 0.15;
			psi = -3.14 + atan((Yo-Y)/(fabs(X+1.4)));
			Xpg = X + disgrasp * fabs(cos(psi));
			Ypg = Y - disgrasp * sin(psi);
			Xg = X + disgrasp * fabs(cos(psi));
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==11){
			X = clo2pos[0] ;
			Y = clo2pos[1] - 0.15;
	
			psi = 3.14 - atan((Yo-Y)/(fabs(X+1.4)));
			Xpg = X + disgrasp * fabs(cos(psi));
			Ypg = Y + disgrasp * sin(psi);
			Xg = X + disgrasp * fabs(cos(psi));
			Yg = Y + disgrasp * sin(psi);
		}
		else if(i==12){
			std::vector<string> table3vec;
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="table3"){
			             table3vec = dataBase_[i].name;
			             break;
				}
			}
			int nuofob = table3vec.size()-1;
			if(nuofob<3){
				X = 0.2 - nuofob * 0.15;
				Y = 2.1; 
			}
			else{
				X = 0.2 - (nuofob%4) * 0.15;
				Y = 1.95; 
			}
			
			psi = 1.5 + 0.1 * nuofob;
			Xpg = X + dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X + disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==13){
			std::vector<string> table3vec;
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="table4"){
			             table3vec = dataBase_[i].name;
			             break;
				}
			}
			int nuofob = table3vec.size()-1;
		
			psi = - 1.5 - 0.1* nuofob;
			if(nuofob<3){
				X = 0.2 - nuofob * 0.15;
				Y = -2.1; 
			}
			else{
				X = 0.2 - (nuofob%4) * 0.15;
				Y = -1.95; 
			}
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==14){
			std::vector<string> table3vec;
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="table3"){
			             table3vec = dataBase_[i].name;
			             break;
				}
			}
			int nuofob = table3vec.size()-1;
			if(nuofob<3){
				X = 0.2 - nuofob * 0.15;
				Y = 2.1; 
			}
			else{
				X = 0.2 -(nuofob%4) * 0.15;
				Y = 1.95; 
			}
			psi = 1.2 + 0.1 * nuofob;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);

		}
		else if(i==15){
			std::vector<string> table3vec;
			for(std::size_t i=0;i<dataBase_.size();i++){
				if(dataBase_[i].name[0]=="table4"){
			             table3vec = dataBase_[i].name;
			             break;
				}
			}
			int nuofob = table3vec.size()-1;
			if(nuofob<3){
				X = 0.2 - nuofob * 0.15;
				Y = -2.1; 
			}
			else{
				X = 0.2 - (nuofob%4) * 0.15;
				Y = -1.95; 
			}
			psi = -1.5 - 0.1 * nuofob;
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);

		}
		else if(i==16){
			X = clo1pos[0];
			Y = clo1pos[1] + 0.15;
			psi = atan((Yo-Y)/X);
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==17){
			X = clo1pos[0];
			Y = clo1pos[1] + 0.15;
			
			psi = -atan((Yo-Y)/X);
			Xpg = X - dispregrasp * cos(psi);
			Ypg = Y - dispregrasp * sin(psi);
			Xg = X - disgrasp * cos(psi);
			Yg = Y - disgrasp * sin(psi);
		}
		else if(i==18){
			X = clo2pos[0] ;
			Y = clo2pos[1] + 0.15;
			psi = -3.14 + atan((Yo-Y)/(fabs(X+1.4)));
			Xpg = X + disgrasp * fabs(cos(psi));
			Ypg = Y - disgrasp * sin(psi);
			Xg = X + disgrasp * fabs(cos(psi));
			Yg = Y - disgrasp * sin(psi);

		}
		else if(i==19){
			X = clo2pos[0] ;
			Y = clo2pos[1] + 0.15;
			psi = 3.14 - atan((Yo-Y)/(fabs(X+1.4)));
			Xpg = X + disgrasp * fabs(cos(psi));
			Ypg = Y + disgrasp * sin(psi);
			Xg = X + disgrasp * fabs(cos(psi));
			Yg = Y + disgrasp * sin(psi);
		}


		world inst,instgrasp,instpostgrasp;
		inst.name.push_back(pregraspname);
		inst.value.push_back(Xpg);
		inst.value.push_back(Ypg);
		inst.value.push_back(preandgraspz);
		inst.value.push_back(0.0);
		inst.value.push_back(0.0);
		inst.value.push_back(psi);
		inst.print();
		dataBase_.push_back(inst);

		instgrasp.name.push_back(graspname);
		instgrasp.value.push_back(Xg);
		instgrasp.value.push_back(Yg);
		instgrasp.value.push_back(graspz);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(psi);
		instgrasp.print();
		dataBase_.push_back(instgrasp);

		instpostgrasp.name.push_back(postgraspname);
		instpostgrasp.value.push_back(Xg);
		instpostgrasp.value.push_back(Yg);
		instpostgrasp.value.push_back(postgraspz);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(psi);
		instpostgrasp.print();
		dataBase_.push_back(instpostgrasp);

	}

	std::vector<string> table3vec,table4vec;
	for(std::size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name[0]=="table1"){
	             table3vec = dataBase_[i].name;
	             break;
		}
	}
	int nuofnonred1 = 0 ;
	for (int i = 1; i < table3vec.size(); ++i)
	{
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, table3vec[i], boost::is_any_of("_"));
  		if(stickcolornu[1]!="red"){
  			 nuofnonred1++;
  		}
  		
  		
	}

	if(nuofnonred1<=3){
		removeDataBaseInstant("table1pos",false);
		std::vector<float> v;
		if(nuofnonred1<=2){
			v.push_back(1.45);
		}
		else{
			v.push_back(1.4);
		}
		
		v.push_back(0.0);
		v.push_back(0.0);
		std::vector<string> name;
		name.push_back("table1pos");
		world inst(name,v);
		dataBase_.push_back(inst);

	}


	for(std::size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name[0]=="table2"){
             table4vec = dataBase_[i].name;
             break;
		}
	}
	int nuofnonred2 = 0 ;
	for (int i = 1; i < table4vec.size(); ++i)
	{
		std::vector<string> stickcolornu;
  		boost::split(stickcolornu, table4vec[i], boost::is_any_of("_"));
  		if(stickcolornu[1]!="red"){
  			 nuofnonred2++;
  		}
  		
  		
	}

	if(nuofnonred2<=3){
		removeDataBaseInstant("table2pos",false);
		std::vector<float> v;
		if(nuofnonred2<=2){
			v.push_back(-1.5);
		}
		else{
			v.push_back(-1.4);
		}
		
		v.push_back(0.0);
		v.push_back(-3.14);
		std::vector<string> name;
		name.push_back("table2pos");
		world inst(name,v);
		dataBase_.push_back(inst);

	}





	






	

}
void tamp_knowledge::updateGraspingCubeWorld(std::vector<string> &v1,std::vector<string> &v2,std::vector<string> &v3){
	ROS_INFO("tamp_knowledge::updateGraspingCubeWorld");
	tamp_msgs::objectssrv obsr;
	std::vector<string> objnames;
	std::vector<double> objposes,objectorie;
	std::vector<string> nextcube;
	for(std::size_t j=0;j<dataBase_.size();j++){
		if(dataBase_[j].name[0]=="nextcube"){
	             nextcube = dataBase_[j].name;
	             break;
		}
	}
	if(BenchmarkPoseClinet.call(obsr)){
		objnames = obsr.response.names;
		objposes = obsr.response.position;
		objectorie = obsr.response.orientation;

	}
		std::vector<double> pigapos,pigbpos,pigcpos,slelempos,bgelpos;
		int mine = 6;
		for (auto element:cmap)
		{ 
			if(element.first<=mine){
				mine = element.first;
			}
		}
		string cubecname = cmap[mine];
		int maxvalue = 2;
		string elementbiggest;
		for (auto element:cmap){
			if(vlauemap[element.second]>=maxvalue && element.second!=nextcube[1]){
				maxvalue = vlauemap[element.second];
				elementbiggest = element.second;
			}
		}
		int minvalue = 11;
		string elementst;
		for (auto element:cmap)
		{
			if(vlauemap[element.second]<=minvalue && element.second!=nextcube[1]){
				minvalue = vlauemap[element.second];
				elementst = element.second;
			}
		}
	for (int i = 0; i < objnames.size(); ++i){

			if(objnames[i]=="tray_blue"){
				pigapos.push_back(objposes[3*i]);
				pigapos.push_back(objposes[3*i+1]);
				pigapos.push_back(objposes[3*i+2]);
			}
			if(objnames[i]==cubecname){
				pigbpos.push_back(objposes[3*i]);
				pigbpos.push_back(objposes[3*i+1]);
				pigbpos.push_back(objposes[3*i+2]);
				
			}
			if(objnames[i]=="tray_red"){
				pigcpos.push_back(objposes[3*i]);
				pigcpos.push_back(objposes[3*i+1]);
				pigcpos.push_back(objposes[3*i+2]);
			}
			if(objnames[i]==elementbiggest){
				bgelpos.push_back(objposes[3*i]);
				bgelpos.push_back(objposes[3*i+1]);
				bgelpos.push_back(objposes[3*i+2]);
			}
		
			if(objnames[i]==elementst){
				slelempos.push_back(objposes[3*i]);
				slelempos.push_back(objposes[3*i+1]);
				slelempos.push_back(objposes[3*i+2]);
			}
		


	}
	if(pigapos.empty()){

		for (int i = 0; i < objnames.size(); ++i){

			if(objnames[i]=="tray_blue"){
				pigapos.push_back(objposes[3*i]);
				pigapos.push_back(objposes[3*i+1]);
				pigapos.push_back(objposes[3*i+2]+0.05);
			}
	

		}

	}
	if(pigbpos.empty()){
		for (int i = 0; i < objnames.size(); ++i){

			if(objnames[i]=="table"){
				pigbpos.push_back(objposes[3*i]);
				pigbpos.push_back(objposes[3*i+1]);
				pigbpos.push_back(0.82);
			}
	

		}

	}
	if(pigcpos.empty()){
		for (int i = 0; i < objnames.size(); ++i){

			if(objnames[i]=="tray_red"){
				pigcpos.push_back(objposes[3*i]);
				pigcpos.push_back(objposes[3*i+1]);
				pigcpos.push_back(objposes[3*i+2]+0.05);
			}
	

		}
	}
	if(slelempos.empty()){
		for (int i = 0; i < objnames.size(); ++i){

			if(objnames[i]=="table"){
				slelempos.push_back(objposes[3*i]);
				slelempos.push_back(objposes[3*i+1]);
				slelempos.push_back(objposes[3*i+2]+0.05);
			}
	

		}
	}
	if(bgelpos.empty()){
		for (int i = 0; i < objnames.size(); ++i){

			if(objnames[i]=="table"){
				bgelpos.push_back(objposes[3*i]);
				bgelpos.push_back(objposes[3*i+1]);
				bgelpos.push_back(objposes[3*i+2]+0.05);
			}
	

		}
	}
	removeDataBaseInstant("smallestcube",false);
	removeDataBaseInstant("biggestcube",false);
	std::vector<string> smcube,bgcube;
	std::vector<float> vf;
	smcube.push_back("smallestcube");
	smcube.push_back(elementst);
	bgcube.push_back("biggestcube");
	bgcube.push_back(elementbiggest);
	world sminst(smcube,vf);
	world bginst(bgcube,vf);
	dataBase_.push_back(sminst);
	dataBase_.push_back(bginst);


	std::vector<string> places{"trayb","trayr","traycpr","traycpb","traycpl","nextcube"};
	for (int i = 0; i < places.size(); ++i)
	{
		string pregraspname = places[i]+"-pregrasp";
		string graspname = places[i]+"-grasp";
		string postgraspname = places[i]+"-postgrasp";
		removeDataBaseInstant(pregraspname,false);
		removeDataBaseInstant(graspname,false);
		removeDataBaseInstant(postgraspname,false);
		double preandgraspz,postgraspz,X,Y,graspz;
		if(i==0){
			std::vector<double> v1objpose;
			if(v1.size()==1){
				preandgraspz = pigapos[2] +0.07;
				postgraspz = pigapos[2]+0.12;
				graspz = pigapos[2]+0.06;
				X = pigapos[0];
				Y = pigapos[1];
			
			}
			else{
				
				for (int i = 0; i < objnames.size(); ++i){
					if(objnames[i]==v1.back()){
						v1objpose.push_back(objposes[3*i]);
						v1objpose.push_back(objposes[3*i+1]);
						v1objpose.push_back(objposes[3*i+2]);
					}
				}
				preandgraspz = std::min(v1objpose[2] +0.11,1.0);
				postgraspz = std::min(v1objpose[2]+0.16,1.0);
				graspz = v1objpose[2]+0.07;
				X = v1objpose[0];
				Y = v1objpose[1];
			

			}
			
			
			
		}
		else if(i==1){
			std::vector<double> v3objpose;
			if(v3.size()==1){
				cout<<"vector size of red tray is 1"<<endl;
				preandgraspz = pigcpos[2] +0.07;
				postgraspz = pigcpos[2]+0.12;
				graspz = pigcpos[2]+0.06;
				X = pigcpos[0];
				Y = pigcpos[1];
			}
			else{
				
				for (int i = 0; i < objnames.size(); ++i){
					if(objnames[i]==v3.back()){
						v3objpose.push_back(objposes[3*i]);
						v3objpose.push_back(objposes[3*i+1]);
						v3objpose.push_back(objposes[3*i+2]);
					}
				}

				preandgraspz = std::min(v3objpose[2] +0.11,1.0);
				postgraspz = std::min(v3objpose[2]+0.16,1.0);
				graspz = v3objpose[2]+0.08;
				X = v3objpose[0];
				Y = v3objpose[1];
			}
			
			
	
		}

		else if(i==2){
			preandgraspz = 0.86;
			postgraspz = 0.9;
			graspz = 0.8;
			X = slelempos[0];
			Y = slelempos[1];
		}
		else if(i==3){
			X = bgelpos[0];
			Y = bgelpos[1];
			preandgraspz = 0.86;
			postgraspz = 0.9;
			graspz = 0.8;
		}
		else if(i==4){
			 cout<<"****************printing cmap elements for grasping placement updates"<<endl;
			printMap(cmap);
			int latticenu=7;
			for (int k = 1; k <7; k++){
				bool exits = false;
				for (auto element:cmap){ 
					if(element.first==k){
						exits  = true;
						cout<<"****************Latice "<<k<<" is filled by: "<<element.second<<endl;
						break;
					}
				}
				if(!exits){
					 latticenu = k;
					 cout<<"****************n-th smallest availabe lattice is:(((((((((((()))))))))))) "<<k<<endl;
					 break;
				}

				
			}

			preandgraspz = 0.82;	
			postgraspz = 0.89;
			graspz = 0.77;
			if(latticenu%2==0){
				Y = -0.07;
			}
			else{
				Y = 0.07;
			}
			X = 0.38 + floor((latticenu-1)/2) * 0.1;
			
		
			
		}
		else if(i==5){
			std::vector<string> v1;
			for(std::size_t j=0;j<dataBase_.size();j++){
				if(dataBase_[j].name[0]=="nextcube"){
			             v1 = dataBase_[j].name;
			             break;
				}
			}
			std::vector<double> nextcubepose;
			if(v1.size()>1){
				for (int j = 0; j < objnames.size(); ++j){

					if(objnames[j]==v1[1]){
						nextcubepose.push_back(objposes[3*j]);
						nextcubepose.push_back(objposes[3*j+1]);
						nextcubepose.push_back(objposes[3*j+2]);
					}

				}
			}
			else{
				nextcubepose.push_back(0.0);
				nextcubepose.push_back(0.0);
				nextcubepose.push_back(0.0);
			}
			
			preandgraspz = nextcubepose[2]+ 0.06;	
			postgraspz = nextcubepose[2]+ 0.06;
			graspz = nextcubepose[2] + 0.06;
			X = nextcubepose[0];
			Y = nextcubepose[1];


		}
		
		

		

		world inst,instgrasp,instpostgrasp;
		inst.name.push_back(pregraspname);
		inst.value.push_back(X);
		inst.value.push_back(Y);
		inst.value.push_back(preandgraspz);
		inst.value.push_back(0.0);
		inst.value.push_back(1.57);
		inst.value.push_back(0.0);
		inst.print();
		dataBase_.push_back(inst);
		instgrasp.name.push_back(graspname);
		instgrasp.value.push_back(X);
		instgrasp.value.push_back(Y);
		instgrasp.value.push_back(graspz);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(1.57);
		instgrasp.value.push_back(0.0);
		instgrasp.print();
		dataBase_.push_back(instgrasp);

		instpostgrasp.name.push_back(postgraspname);
		instpostgrasp.value.push_back(X);
		instpostgrasp.value.push_back(Y);
		instpostgrasp.value.push_back(postgraspz);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(1.57);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.print();
		dataBase_.push_back(instpostgrasp);
	}

}
bool tamp_knowledge::updateSceneServer(tamp_msgs::updatescene::Request &req,tamp_msgs::updatescene::Response &res){
	ROS_INFO("receieved a request to update scene for panda robot %s",req.task.c_str());
	int robotnu, targetnu;
	if(req.task=="r1t1"){
		robotnu = 1;
		targetnu = 1;
	}
	else if(req.task=="r1t2"){
		robotnu = 1;
		targetnu = 2;

	}
	else if(req.task=="r2t1"){
		robotnu = 2;
		targetnu = 1;

	}
	else if(req.task=="r2t2"){
		robotnu = 2;
		targetnu = 2;

	}

	std::vector<string> objectsinpolygon;
	
    
	for(std::size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name[0]==req.task){
	             objectsinpolygon = dataBase_[i].name;
	             break;
		}
	}
	if(objectsinpolygon.size()>1){
		objectsinpolygon.erase(objectsinpolygon.begin());
	}
	//if(objectsinpolygon.size()==1){
	///	res.result = false;
    //	return false;
	//}
	geometry_msgs::Pose eepos;
    tamp_msgs::pandaee eesrv;
   
    eesrv.request.robot = robotnu;
    
   

    if(pandaEEClient.call(eesrv)){
    	eepos = eesrv.response.pose;

    }

    createobjectCriteria(robotnu,targetnu,objectsinpolygon,eepos);

    res.result = true;
    return true;


}

void tamp_knowledge::createobjectCriteria(int robotnu,int targetnu,
	const std::vector<string> &objectsinpolygon,const geometry_msgs::Pose eepos){
	ROS_INFO("tamp_knowledge::createobjectCriteria");
	tamp_msgs::objectssrv msg;
	std::vector<string> nameofobjs;
	std::vector<double> poseofobjects;
	string target1,target2;
	if(robotnu==1){
		if(collision_object_panda1_client.call(msg)){
			nameofobjs = msg.response.names;
			poseofobjects = msg.response.position;
			target1 = msg.response.target;
			target2 = msg.response.target2;
		}
	}
	else if(robotnu==2){

		if(collision_object_panda2_client.call(msg)){
			nameofobjs = msg.response.names;
			poseofobjects = msg.response.position;
			target1 = msg.response.target;
			target2 = msg.response.target2;
		}

	}

	std::vector<std::vector<double>> poseofpolygonobjects;

	for(size_t i=0;i<objectsinpolygon.size();i++){
		for(size_t j=0;j<nameofobjs.size();j++){
			if(objectsinpolygon[i]==nameofobjs[j]){
				std::vector<double> v;
				v.push_back(poseofobjects[3*j]);
				v.push_back(poseofobjects[3*j+1]);
				v.push_back(poseofobjects[3*j+2]);
				poseofpolygonobjects.push_back(v);
				break;
			}
		}
	}
    
    string target = (targetnu==1)?target1:target2;
    std::vector<double> targetpos;

	for(size_t j=0;j<nameofobjs.size();j++){
			if(target==nameofobjs[j]){
				targetpos.push_back(poseofobjects[3*j]);
				targetpos.push_back(poseofobjects[3*j+1]);
				targetpos.push_back(poseofobjects[3*j+2]);
				break;
			}
	}

     
    //std::vector<double> posofeed{0.0,0.0,0.0};
   // std::vector<double> eeposee = {eepos.position.x,eepos.position.y,eepos.position.z};
	//int clobeed = minEucleadan(poseofpolygonobjects,posofeed);
	//int clobee = minEucleadan(poseofpolygonobjects,eeposee);
	//int clobt = minEucleadan(poseofpolygonobjects,targetpos);

	string strclobeed = "clobed"+to_string(robotnu)+to_string(targetnu);
	string strclobee = "clobee"+to_string(robotnu)+to_string(targetnu);
	string strclobt = "clobt"+to_string(robotnu)+to_string(targetnu);
	string strtarget = "target"+to_string(targetnu);

	if(poseofpolygonobjects.size()==0){

		ROS_INFO("Zero objects inside polygon");

		removeAndAddDataBaseInstant(strclobeed,target);
		updateGraspApproachingInstant(strclobeed,targetpos);

		removeAndAddDataBaseInstant(strclobee,target);
		updateGraspApproachingInstant(strclobee,targetpos);

		removeAndAddDataBaseInstant(strclobt,target);
		updateGraspApproachingInstant(strclobt,targetpos);

		removeAndAddDataBaseInstant(strtarget,target);
		updateGraspApproachingInstant(strtarget,targetpos);


	}
	else if(poseofpolygonobjects.size()==1){
		ROS_INFO("One object inside polygon");

		removeAndAddDataBaseInstant(strclobeed,objectsinpolygon[0]);
		updateGraspApproachingInstant(strclobeed,poseofpolygonobjects[0]);

		removeAndAddDataBaseInstant(strclobee,objectsinpolygon[0]);
		updateGraspApproachingInstant(strclobee,poseofpolygonobjects[0]);

		removeAndAddDataBaseInstant(strclobt,objectsinpolygon[0]);
		updateGraspApproachingInstant(strclobt,poseofpolygonobjects[0]);


		removeAndAddDataBaseInstant(strtarget,target);
		updateGraspApproachingInstant(strtarget,targetpos);
	}
	else if(poseofpolygonobjects.size()==2){
		ROS_INFO("Two objects inside polygon");

		removeAndAddDataBaseInstant(strclobeed,objectsinpolygon[0]);
		updateGraspApproachingInstant(strclobeed,poseofpolygonobjects[0]);

		removeAndAddDataBaseInstant(strclobee,objectsinpolygon[1]);
		updateGraspApproachingInstant(strclobee,poseofpolygonobjects[1]);

		removeAndAddDataBaseInstant(strclobt,objectsinpolygon[1]);
		updateGraspApproachingInstant(strclobt,poseofpolygonobjects[1]);


		removeAndAddDataBaseInstant(strtarget,target);
		updateGraspApproachingInstant(strtarget,targetpos);

	}
	else{
		ROS_INFO("3 or more objects inside polygon");
		 std::vector<int> sorted = findClosestPanda(poseofpolygonobjects);

		removeAndAddDataBaseInstant(strclobeed,objectsinpolygon[sorted[1]]);
		std::vector<double> strclobeedpos = poseofpolygonobjects[sorted[1]];
		updateGraspApproachingInstant(strclobeed,strclobeedpos);

		removeAndAddDataBaseInstant(strclobee,objectsinpolygon[sorted[0]]);
		std::vector<double> strclobeepos = poseofpolygonobjects[sorted[0]];
		updateGraspApproachingInstant(strclobee,strclobeepos);

		removeAndAddDataBaseInstant(strclobt,objectsinpolygon[sorted[2]]);
		std::vector<double> strclobtpos = poseofpolygonobjects[sorted[2]];
		updateGraspApproachingInstant(strclobt,strclobtpos);

		removeAndAddDataBaseInstant(strtarget,target);
		updateGraspApproachingInstant(strtarget,targetpos);

	}
   


    
	
	//updatePlacingPositions(robotnu);









}

std::vector<int> tamp_knowledge::findClosestPanda(std::vector<std::vector<double>> &poses){
	ROS_INFO("tamp_knowledge::findClosestPanda");
	int clobt = 0;
	int clobee = 0;
	int clobed = 0;
	std::vector<double> xs;
	for(int i=0;i<poses.size();i++){
		xs.push_back(poses[i][0]);
	}
	double minimum3 = 1.0;
	double minimum1 = 1.0;
	double minimum2 = 1.0;

	for(int i=0;i<xs.size();i++){
		if (xs[i] < minimum1){
            minimum1 = xs[i];
            clobee = i;
            cout<<"minimum1 "<<clobee<<" : "<<xs[i]<<endl;
            
  		}
	}

	for(int i=0;i<xs.size();i++){
		if(i!=clobee){
			if (xs[i] < minimum2){
	            minimum2 = xs[i];
	            clobed = i;
	            cout<<"minimum2 "<<clobed<<" : "<<xs[i]<<endl;
	            
  			}
		}	
	}

	for(int i=0;i<xs.size();i++){
		if(i!=clobee && i!=clobed){
			if (xs[i] < minimum3){
	            minimum3 = xs[i];
	            clobt = i;
	            cout<<"minimum3 "<<clobt<<" : "<<xs[i]<<endl;
	            
  			}
		}	
	}

	std::vector<int> sortedindex;
	sortedindex.push_back(clobee);
	sortedindex.push_back(clobed);
	sortedindex.push_back(clobt);
	cout<<"indexs clobee "<<clobee<<" clobed "<<clobed<<" clobt "<<clobt<<endl;

return sortedindex;

}

void tamp_knowledge::updatePlacingPositions(int robotnu){

	ROS_INFO("tamp_knowledge::updatePlacingPositions");
	string placestr = "place"+to_string(robotnu);
	int nuofplacement;
	for(auto item:dataBase_){
		if(item.name[0]==placestr){
			ROS_INFO("Inside if ");
			nuofplacement = (int) item.value[0];
			break;
		}
	}
	ROS_INFO("found place nu");
	
	string pregraspname = placestr+"-pregrasp";
	string graspname = placestr+"-grasp";
	string postgraspname = placestr+"-postgrasp";

	removeDataBaseInstant(pregraspname,false);
	removeDataBaseInstant(graspname,false);
	removeDataBaseInstant(postgraspname,false);


		double x, y,psi;
		double pregraspz = 0.5;
		double graspofz = 0.3;//height of cube + height/2 cylindr
		cout<<"******pregraspz is: "<<pregraspz<<"((((((((((graspz is: "<<graspofz<<endl;
	if(robotnu==1){
		double dx = 0.1;
		double dy = 0.1;
		int nx = 4;
		int ny = 5;
		double x0 = -0.5;//obtain from vrep
		double y0 = -0.5;//obtain from vrep
		x = x0 + dx * fmod(nuofplacement,nx+1);
		y = y0 + dy * floor(nuofplacement/nx+1);
		psi = -1.5 - atan(x/y);
		
		ROS_INFO("Calculated place pose for panda 1");

	}
	else{

		double dx = 0.1;
		double dy = 0.1;
		int nx = 4;
		int ny = 4;
		double x0 = -0.5;
		double y0 = 0.5;
		x = x0 + dx * floor(nuofplacement/ny+1);
		y = y0 - dy * fmod(nuofplacement,ny+1);
		psi = 1.5 + atan(x/y);
		//preandgraspz = 0.5;
		//graspz = 0.1 + 0.1;
		ROS_INFO("Calculated place pose for panda 2");

	}





	world inst,instgrasp,instpostgrasp;
	inst.name.push_back(pregraspname);
	inst.value.push_back(x);
	inst.value.push_back(y);
	inst.value.push_back(pregraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(1.57);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);
	instgrasp.name.push_back(graspname);
	instgrasp.value.push_back(x);
	instgrasp.value.push_back(y);
	instgrasp.value.push_back(graspofz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(1.57);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);

	instpostgrasp.name.push_back(postgraspname);
	instpostgrasp.value.push_back(x);
	instpostgrasp.value.push_back(y);
	instpostgrasp.value.push_back(pregraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(1.57);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);



}
void tamp_knowledge::updateGraspingHanoi(int na,int nb,int nc){

	tamp_msgs::objectssrv obsr;
	std::vector<string> objnames;
	std::vector<double> objposes,pigapos,pigbpos,pigcpos;
	if(BenchmarkPoseClinet.call(obsr)){
		objnames = obsr.response.names;
		objposes = obsr.response.position;

	}
	for (int i = 0; i < objnames.size(); ++i)
	{
		if(objnames[i]=="peg1"){
			pigapos.push_back(objposes[3*i]);
			pigapos.push_back(objposes[3*i+1]);
			pigapos.push_back(objposes[3*i+2]);
		}
		if(objnames[i]=="peg2"){
			pigbpos.push_back(objposes[3*i]);
			pigbpos.push_back(objposes[3*i+1]);
			pigbpos.push_back(objposes[3*i+2]);
			
		}
		if(objnames[i]=="peg3"){
			pigcpos.push_back(objposes[3*i]);
			pigcpos.push_back(objposes[3*i+1]);
			pigcpos.push_back(objposes[3*i+2]);
		}

	}

	//for right arm
	for (int i = 0; i < 3; ++i)
	{
		string pregraspname = "pig"+to_string(i+1)+"R"+"-pregrasp";
		string graspname = "pig"+to_string(i+1)+"R"+"-grasp";
		string postgraspname = "pig"+to_string(i+1)+"R"+"-postgrasp";
		removeDataBaseInstant(pregraspname,false);
		removeDataBaseInstant(graspname,false);
		removeDataBaseInstant(postgraspname,false);
		double preandgraspz,postgraspz,X,Y,graspz;
		if(i==0){
			preandgraspz = pigapos[2]- 0.01+ (na+1.25) * 0.06;	
			postgraspz = pigapos[2]- 0.01 + (na+2) * 0.06;
			graspz = pigapos[2] - 0.01 + (na+0.25) * 0.06;
			X = pigapos[0];
			Y = pigapos[1];
			
		}
		else if(i==1){
			preandgraspz = pigbpos[2]- 0.01 + (nb+1.25) * 0.06;	
			postgraspz = pigbpos[2]- 0.01 + (nb+2) * 0.06 ;
			graspz = pigbpos[2] - 0.01 + (nb+0.25) * 0.06;
			X = pigbpos[0];
			Y = pigbpos[1];
		}
		else{
			preandgraspz = pigcpos[2]- 0.01 + (nc+1.25) * 0.06;	
			postgraspz = pigcpos[2]- 0.01 + (nc+2) * 0.06;
			graspz = pigcpos[2] - 0.01 + (nc+0.25) * 0.06;
			X = pigcpos[0];
			Y = pigcpos[1];
		}
		
		double dispregrasp = 0.06;	
		double Yo = 0.3;
		double disgrasp = 0.01;
		double psi,xp,yp,xg,yg;
		

		psi = atan((Yo-Y)/X);
		xp = X - dispregrasp * cos(psi);
		yp = Y - dispregrasp * sin(psi);
		xg = X - disgrasp * cos(psi);
		yg = Y - disgrasp * sin(psi);
		


		

		world inst,instgrasp,instpostgrasp;
		inst.name.push_back(pregraspname);
		inst.value.push_back(xg);
		inst.value.push_back(yg);
		inst.value.push_back(preandgraspz);
		inst.value.push_back(0.0);
		inst.value.push_back(0.0);
		inst.value.push_back(psi);
		inst.print();
		dataBase_.push_back(inst);
		instgrasp.name.push_back(graspname);
		instgrasp.value.push_back(xg);
		instgrasp.value.push_back(yg);
		instgrasp.value.push_back(graspz);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(psi);
		instgrasp.print();
		dataBase_.push_back(instgrasp);

		instpostgrasp.name.push_back(postgraspname);
		instpostgrasp.value.push_back(xg);
		instpostgrasp.value.push_back(yg);
		instpostgrasp.value.push_back(postgraspz);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(psi);
		instpostgrasp.print();
		dataBase_.push_back(instpostgrasp);
	}

	//for left arm

	for (int i = 0; i < 3; ++i)
	{
		string pregraspname = "pig"+to_string(i+1)+"L"+"-pregrasp";
		string graspname = "pig"+to_string(i+1)+"L"+"-grasp";
		string postgraspname = "pig"+to_string(i+1)+"L"+"-postgrasp";
		removeDataBaseInstant(pregraspname,false);
		removeDataBaseInstant(graspname,false);
		removeDataBaseInstant(postgraspname,false);
		double preandgraspz,postgraspz,X,Y,graspz;
		if(i==0){
			preandgraspz = pigapos[2]- 0.01 + (na+	1.25) * 0.06 ;	
			postgraspz = pigapos[2]- 0.01 + (na+2.5) * 0.06;
			graspz = pigapos[2] - 0.01 + (na+0.25) * 0.06;
			X = pigapos[0];
			Y = pigapos[1];
			
		}
		else if(i==1){
			preandgraspz = pigbpos[2] - 0.01 + (nb+1.25) * 0.06;	
			postgraspz = pigbpos[2]- 0.01 + (nb+2.5) * 0.06;
			graspz = pigapos[2] - 0.01 + (nb+0.25) * 0.06;
			X = pigbpos[0];
			Y = pigbpos[1];
		}
		else{
			preandgraspz = pigcpos[2] - 0.01 + (nc+1.25) * 0.06;	
			postgraspz = pigcpos[2] - 0.01 + (nc+2.5) * 0.06;
			graspz = pigapos[2] - 0.01+ (nc+0.25) * 0.06;
			X = pigcpos[0];
			Y = pigcpos[1];
		}
		
		double dispregrasp = 0.06;	
		double Yo = 0.3;
		double disgrasp = 0.01;
		double psi,xp,yp,xg,yg;

		psi = -atan((Yo-Y)/X);
		xp = X - dispregrasp * cos(psi);
		yp = Y - dispregrasp * sin(psi);
		xg = X - disgrasp * cos(psi);
		yg = Y - disgrasp * sin(psi);
		

		world inst,instgrasp,instpostgrasp;
		inst.name.push_back(pregraspname);
		inst.value.push_back(xg);
		inst.value.push_back(yg);
		inst.value.push_back(preandgraspz);
		inst.value.push_back(0.0);
		inst.value.push_back(0.0);
		inst.value.push_back(psi);
		inst.print();
		dataBase_.push_back(inst);
		instgrasp.name.push_back(graspname);
		instgrasp.value.push_back(xg);
		instgrasp.value.push_back(yg);
		instgrasp.value.push_back(graspz);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(0.0);
		instgrasp.value.push_back(psi);
		instgrasp.print();
		dataBase_.push_back(instgrasp);

		instpostgrasp.name.push_back(postgraspname);
		instpostgrasp.value.push_back(xg);
		instpostgrasp.value.push_back(yg);
		instpostgrasp.value.push_back(postgraspz);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(0.0);
		instpostgrasp.value.push_back(psi);
		instpostgrasp.print();
		dataBase_.push_back(instpostgrasp);
	}



}
void tamp_knowledge::updateGraspApproachingInstant(string name,const std::vector<double> &pos){
	string pregraspname = name+"-pregrasp";
	string graspname = name+"-grasp";
	string postgraspname = name+"-postgrasp";
	removeDataBaseInstant(pregraspname,false);
	removeDataBaseInstant(graspname,false);
	removeDataBaseInstant(postgraspname,false);

	
	double preandgraspz = pos[2]+0.05;	
	double postgraspz = pos[2]+0.3;
	double dispregrasp = 0.15;	
	double Yo = 0.4;
	double disgrasp = 0.04;
	double X = pos[0];
	double Y = pos[1];
	double psi,xp,yp,xg,yg;
	if(Y>0){

		psi = atan((Y-Yo)/X);
		xp = X - dispregrasp * cos(psi);
		yp = Y - dispregrasp * sin(psi);
		xg = X - disgrasp * cos(psi);
		yg = Y - disgrasp * sin(psi);
		

	}
	else{

		psi = atan((Yo-fabs(Y))/X);
		xp = X - dispregrasp * cos(psi);
		yp = Y - dispregrasp * sin(psi);
		xg = X - disgrasp * cos(psi);
		yg = Y - disgrasp * sin(psi);
	}
	

	world inst,instgrasp,instpostgrasp;
	inst.name.push_back(pregraspname);
	inst.value.push_back(xp);
	inst.value.push_back(yp);
	inst.value.push_back(preandgraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(1.57);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);
	instgrasp.name.push_back(graspname);
	instgrasp.value.push_back(xg);
	instgrasp.value.push_back(yg);
	instgrasp.value.push_back(preandgraspz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(1.57);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);

	instpostgrasp.name.push_back(postgraspname);
	instpostgrasp.value.push_back(xp);
	instpostgrasp.value.push_back(yp);
	instpostgrasp.value.push_back(postgraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(1.57);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);



	

}
void tamp_knowledge::removeAndAddDataBaseInstant(string nameins,string name2){
	ROS_INFO("tamp_knowledge::removeAndAddDataBaseInstant");
	for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
		if(it->name[0]==nameins){ 
		    dataBase_.erase(it);		   
		    	break;
		}

		else
			{it++;}
	}
	world inst;
	inst.name.push_back(nameins);
	inst.name.push_back(name2);
	dataBase_.push_back(inst);

}

bool tamp_knowledge::registerObjectPlace(tamp_msgs::registerplace::Request &req,tamp_msgs::registerplace::Response &res){
	string pp = req.pp;
	int nextobject;
	ROS_INFO("Registering object %s",pp.c_str());
	if(pp=="inserted"){
		for(std::size_t i=0;i<dataBase_.size();i++){
	    	if(dataBase_[i].name[0]=="nextobject"){
	             nextobject =(int) dataBase_[i].value[0];
	             
	             res.update=true;
	             break;
			}
		}
		string objname;
		for(std::size_t i=0;i<dataBase_.size();i++){
	    	if(dataBase_[i].name[0]=="next"){
	             dataBase_[i].name.clear();
	             dataBase_[i].name.push_back("next");
	             objname = "cylinder_"+to_string(nextobject);
	             dataBase_[i].name.push_back(objname);
	             cout<<"next object is : "<<objname<<endl;
	             break;
			}
		}
		//tamp_msgs::removeobject::Request reqmsg;
		///tamp_msgs::removeobject::Response resmsg;
		//reqmsg.object ="next";
		//removeObject(reqmsg,resmsg);
		removeDataBaseInstant("next-pregrasp",false);
		removeDataBaseInstant("next-grasp",false);
		removeDataBaseInstant("next-postgrasp",false);
		for(std::size_t i=0;i<dataBase_.size();i++){
			string pregrasp,grasp,postgrasp;
			pregrasp=objname+"-pregrasp";
			grasp=objname+"-grasp";
			postgrasp=objname+"-postgrasp";
	    	if(dataBase_[i].name[0]==pregrasp){
	    		world isnt;
	    		isnt.name.push_back("next-pregrasp");
	    		for(size_t j=0;j<dataBase_[i].value.size();j++){
	    			isnt.value.push_back(dataBase_[i].value[j]);
	    		}
	             ROS_INFO("Adding next-pregrasp");
	             isnt.print();
	             dataBase_.push_back(isnt);
			}
			if(dataBase_[i].name[0]==grasp){
	    		world isnt;
	    		isnt.name.push_back("next-grasp");
	             for(size_t j=0;j<dataBase_[i].value.size();j++){
	    			isnt.value.push_back(dataBase_[i].value[j]);
	    		}
	    		ROS_INFO("Adding next-grasp");
	             isnt.print();
	             dataBase_.push_back(isnt);
			}
			if(dataBase_[i].name[0]==postgrasp){
	    		world isnt;
	    		isnt.name.push_back("next-postgrasp");
	             for(size_t j=0;j<dataBase_[i].value.size();j++){
	    			isnt.value.push_back(dataBase_[i].value[j]);
	    		}
	    		ROS_INFO("Adding next-postgrasp");
	             isnt.print();
	             dataBase_.push_back(isnt);
			}
		}



	}

	else if(pp=="clob"){
		updatemetrics();
		removeDataBaseInstant("next-pregrasp",false);
		removeDataBaseInstant("next-grasp",false);
		removeDataBaseInstant("next-postgrasp",false);
		res.update = true;
		if(req.arm=="right"){
			ROS_INFO("Clobree is the next object");
			for(std::size_t i=0;i<dataBase_.size();i++){
				string pregrasp,grasp,postgrasp;
				pregrasp="clobree-pregrasp";
				grasp="clobree-grasp";
				postgrasp="clobree-postgrasp";
		    	if(dataBase_[i].name[0]==pregrasp){
		    		world isnt;
		    		isnt.name.push_back("next-pregrasp");
		    		for(size_t j=0;j<dataBase_[i].value.size();j++){
		    			isnt.value.push_back(dataBase_[i].value[j]);
		    		}
		             ROS_INFO("Adding next-pregrasp");
		             isnt.print();
		             dataBase_.push_back(isnt);
				}
				if(dataBase_[i].name[0]==grasp){
		    		world isnt;
		    		isnt.name.push_back("next-grasp");
		             for(size_t j=0;j<dataBase_[i].value.size();j++){
		    			isnt.value.push_back(dataBase_[i].value[j]);
		    		}
		    		ROS_INFO("Adding next-grasp");
		             isnt.print();
		             dataBase_.push_back(isnt);
				}
				if(dataBase_[i].name[0]==postgrasp){
		    		world isnt;
		    		isnt.name.push_back("next-postgrasp");
		             for(size_t j=0;j<dataBase_[i].value.size();j++){
		    			isnt.value.push_back(dataBase_[i].value[j]);
		    		}
		    		ROS_INFO("Adding next-postgrasp");
		             isnt.print();
		             dataBase_.push_back(isnt);
				}
			}
		}
		else if(req.arm=="left"){
				 ROS_INFO("Clobleee is the next object");
				for(std::size_t i=0;i<dataBase_.size();i++){
					string pregrasp,grasp,postgrasp;
					pregrasp="cloblee-pregrasp";
					grasp="cloblee-grasp";
					postgrasp="cloblee-postgrasp";
			    	if(dataBase_[i].name[0]==pregrasp){
			    		world isnt;
			    		isnt.name.push_back("next-pregrasp");
			    		for(size_t j=0;j<dataBase_[i].value.size();j++){
			    			isnt.value.push_back(dataBase_[i].value[j]);
			    		}
			             ROS_INFO("Adding next-pregrasp");
			             isnt.print();
			             dataBase_.push_back(isnt);
					}
					if(dataBase_[i].name[0]==grasp){
			    		world isnt;
			    		isnt.name.push_back("next-grasp");
			             for(size_t j=0;j<dataBase_[i].value.size();j++){
			    			isnt.value.push_back(dataBase_[i].value[j]);
			    		}
			    		ROS_INFO("Adding next-grasp");
			             isnt.print();
			             dataBase_.push_back(isnt);
					}
					if(dataBase_[i].name[0]==postgrasp){
			    		world isnt;
			    		isnt.name.push_back("next-postgrasp");
			             for(size_t j=0;j<dataBase_[i].value.size();j++){
			    			isnt.value.push_back(dataBase_[i].value[j]);
			    		}
			    		ROS_INFO("Adding next-postgrasp");
			             isnt.print();
			             dataBase_.push_back(isnt);
					}
				}
		}
		else{
			ROS_ERROR("inserted arm, %s, doesnt exist",req.arm.c_str());
		}
	}
	else{
		cout<<"the object for registration is not defined !!"<<endl;
	}
return res.update;
}


void tamp_knowledge::objectInsertedCB(const std_msgs::Int16 &msg){
	ROS_INFO("tamp_knowledge::objectInsertedCB, %d",msg.data);
	for(std::size_t i=0;i<dataBase_.size();i++){
	    	if(dataBase_[i].name[0]=="nextobject"){
	    		dataBase_[i].value.clear();
	    		dataBase_[i].value.push_back(msg.data);
	    		dataBase_[i].print();
	    		break;
	    	}
	}
}


void tamp_knowledge::humanAcionCB(const std_msgs::Int16 &msg){
	ROS_INFO("tamp_knowledge::humanAcionCB, %d",msg.data);
	for(std::size_t i=0;i<dataBase_.size();i++){
	    	if(dataBase_[i].name[0]=="human_action"){
	             //dataBase_[i].value.pop_back();
	    			if(dataBase_[i].value.size()>1){
	    				dataBase_[i].value.clear();
	    			}
	             dataBase_[i].value.push_back(msg.data);
	             dataBase_[i].print();
	             break;
			}
	    }
}

bool tamp_knowledge::removeObject(tamp_msgs::removeobject::Request &req,tamp_msgs::removeobject::Response &res){
	string object;
	for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]==req.object){
             object = dataBase_[i].name[1];
             cout<<"object to remove is : "<<object<<endl;
             break;
		}
	}
		string name1 = object + "-pregrasp";
		string name2 = object + "-grasp";
		string name3 = object + "-postgrasp";
	for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){

		if(it->name[0]==object || it->name[0]==name1 || it->name[0]==name2 ||it->name[0]==name3){
		   	ROS_INFO("Database %s found and removed",it->name[0].c_str());
		    dataBase_.erase(it);
		    
		    //break;
		}

		else
			{it++;}
	}

	updatesceneItself();

	for(int i=0;i<dataBase_.size();i++)
			dataBase_[i].print();

	res.result = true;
return res.result;

}

bool tamp_knowledge::registerPlace(tamp_msgs::registerplace::Request &req,tamp_msgs::registerplace::Response &res){

  string pp = req.pp;
  std::vector<string> ppvec;
  boost::split(ppvec, pp, boost::is_any_of("-"));
  if(ppvec[0]=="1"){
  	int ob,objindex;
  		for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]=="place1"){
             objindex= i;
             break;
		}
    }
    ob = (int) dataBase_[objindex].value.back();
    dataBase_[objindex].value.pop_back();
    dataBase_[objindex].print();
    if(ppvec[1]=="2"){
    	for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]=="place2"){
             dataBase_[i].value.push_back(ob);
             dataBase_[i].print();
             break;
		}
    }

    }
    else if(ppvec[1]=="3"){
    	for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]=="place3"){
             dataBase_[i].value.push_back(ob);
             dataBase_[i].print();
             break;
		}
    }

    }
    else{
    	ROS_ERROR("No place found for object replacing oops");
    }
  }
  else if(ppvec[0]=="2"){
  	int ob,objindex;
  		for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]=="place2"){
             objindex= i;
             break;
		}
    }
    ob = (int) dataBase_[objindex].value.back();
    dataBase_[objindex].value.pop_back();
    dataBase_[objindex].print();
    if(ppvec[1]=="1"){
    	for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]=="place1"){
             dataBase_[i].value.push_back(ob);
             dataBase_[i].print();
             break;
		}
    }

    }
    else if(ppvec[1]=="3"){
    	for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]=="place3"){
             dataBase_[i].value.push_back(ob);
             dataBase_[i].print();
             break;
		}
    }

    }
    else{
    	ROS_ERROR("No place found for object replacing oops");
    }
  }


return true;
}

void tamp_knowledge::kinetcCbObjectId(const std_msgs::String::ConstPtr& msg){
	string objectid = msg->data;
	int id;
	if(objectid!=""){
		if(objectid=="ar_marker_11"){
		id=1;

		}
		else if(objectid=="ar_marker_22"){
			id=2;

		}
			
		else if(objectid=="ar_marker_33"){
			id=3;
			
		}
		else if(objectid=="ar_marker_44"){
			id=4;
			
		}
		
		

		for(std::size_t i=0;i<dataBase_.size();i++){
	    	if(dataBase_[i].name[0]=="place2"){
	             dataBase_[i].value.pop_back();
	             dataBase_[i].value.push_back(id);
	             dataBase_[i].print();
	             break;
			}
	    }

   	 ROS_INFO("added object %d to place2",id);


	}
	


}






void tamp_knowledge::callBackkinect(const std_msgs::String::ConstPtr& msg){
	string objstatus  = msg->data.c_str();
    std::size_t objindex;
   

    for(std::size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name[0]=="object_status"){
             objindex= i;
             dataBase_[i].name.pop_back();
             dataBase_[i].name.push_back(objstatus);
             break;

		}

    }
       
    cout<<"*************New Data of Object Status****************"<<endl;
    dataBase_[objindex].print();
		
    //    cout<<"objectstatus : "<<singleobjstatus<<endl;
   // cout<<"lastobjstatus: "<<lastobjstatus<<endl;
   // cout<<"objstatus: "<<objstatus<<endl;


}

bool tamp_knowledge::registerData(tamp_msgs::registerdata::Request& request,tamp_msgs::registerdata::Response& response){

	ROS_INFO("Registration of knowledge data receieved");

	for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){

		if(it->name[0]==request.names[0]){
		   
		    dataBase_.erase(it);
		    ROS_INFO("Database %s found and removed",request.names[0].c_str());
		    break;
		}

		else
			{it++;}
	}

	world inst;
	for(const auto item:request.names){
		inst.name.push_back(item);
	}
	
	for(std::size_t i=0;i<request.pose.size();i++){
		inst.value.push_back(request.pose[i]);
	}

	dataBase_.push_back(inst);
	ROS_INFO("New Database Values");
	//updatesceneItself();
    //for(int i=0;i<dataBase_.size();i++)
		//dataBase_[i].print();

	response.update=true;
return true;
}

void tamp_knowledge::updatePlaces(){
	std::vector<float> place1,place2,place3;
	 for(size_t i=0;i<dataBase_.size();i++){

        if(dataBase_[i].name[0]=="place1"){

        	place1= dataBase_[i].value;
        }

        else if(dataBase_[i].name[0]=="place2"){

        	place2= dataBase_[i].value;

        }

        else if(dataBase_[i].name[0]=="place3"){

        	place3= dataBase_[i].value;

        }
    }

     for(size_t i=0;i<dataBase_.size();i++){

        if(dataBase_[i].name[0]=="place1-pregrasp" ||dataBase_[i].name[0]=="place1-postgrasp" ){

        		if(place1.size()==0 || place1.size()==1){
        		dataBase_[i].value[2] = 0.08;
        	}
        	else if(place1.size()==2){
        		dataBase_[i].value[2] = 0.08;

        	}
        	else if(place1.size()==3){
        		dataBase_[i].value[2] = 0.08;

        	}
        	else if(place1.size()==4){
        		dataBase_[i].value[2] = 0.08;

        	}
        }

        else if(dataBase_[i].name[0]=="place1-grasp"){

        	
        	if(place1.size()==0 || place1.size()==1){
        		dataBase_[i].value[2] = -0.09;
        	}
        	else if(place1.size()==2){
        		dataBase_[i].value[2] = -0.04;

        	}
        	else if(place1.size()==3){
        		dataBase_[i].value[2] = 0.0;

        	}
        	else if(place1.size()==4){
        		dataBase_[i].value[2] = 0.04;

        	}

        }

        else if(dataBase_[i].name[0]=="place2-pregrasp" ||dataBase_[i].name[0]=="place2-postgrasp"){

     
        		if(place2.size()==0 || place2.size()==1){
        		dataBase_[i].value[2] = 0.08;
        	}
        	else if(place2.size()==2){
        		dataBase_[i].value[2] = 0.08;

        	}
        	else if(place2.size()==3){
        		dataBase_[i].value[2] = 0.08;

        	}
        	else if(place2.size()==4){
        		dataBase_[i].value[2] = 0.08;

        	}

        }
        else if(dataBase_[i].name[0]=="place2-grasp"){

        		if(place2.size()==0 || place2.size()==1){
        		dataBase_[i].value[2] = -0.09;
        	}
        	else if(place2.size()==2){
        		dataBase_[i].value[2] = -0.04;

        	}
        	else if(place2.size()==3){
        		dataBase_[i].value[2] = 0.0;

        	}
        	else if(place2.size()==4){
        		dataBase_[i].value[2] = 0.04;

        	}

        }

        else if(dataBase_[i].name[0]=="place3-pregrasp" ||dataBase_[i].name[0]=="place3-postgrasp"){

        	  

        		if(place3.size()==0 || place3.size()==1){
        		dataBase_[i].value[2] = 0.08;
        	}
        	else if(place3.size()==2){
        		dataBase_[i].value[2] = 0.08;

        	}
        	else if(place3.size()==3){
        		dataBase_[i].value[2] = 0.08;

        	}
        	else if(place3.size()==4){
        		dataBase_[i].value[2] = 0.08;

        	}

        }
        else if(dataBase_[i].name[0]=="place3-grasp"){

        		if(place3.size()==0 || place3.size()==1){
        		dataBase_[i].value[2] = -0.09;
        	}
        	else if(place3.size()==2){
        		dataBase_[i].value[2] = -0.04;

        	}
        	else if(place3.size()==3){
        		dataBase_[i].value[2] = 0.0;

        	}
        	else if(place3.size()==4){
        		dataBase_[i].value[2] = 0.04;

        	}

        }

    }



}
bool tamp_knowledge::knowledgeQueryPanda(tamp_msgs::knowledge::Request& request, tamp_msgs::knowledge::Response& res){
	ROS_INFO("knowledge receieved Query for pandas");
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string type=request.reqType;
	string name=request.Name;
	string requestInfo=request.requestInfo;

	for(int j=0; j<dataBase_.size();j++)
	{
		if(dataBase_[j].name[0]==type){
			for(auto const item:dataBase_[j].name){
				res.names.push_back(item);
			}
			for(auto item:dataBase_[j].value){
				res.pose.push_back(item);
			}
			break;
			

		}
		
	}
	return true;

}

bool tamp_knowledge::knowledgeQuery(tamp_msgs::knowledge::Request& request, tamp_msgs::knowledge::Response& res){
    ROS_INFO("knowledge receieved Query");
    //updatePlaces();
    if(request.updatescene){
    	//tamp_msgs::knowledge::Response resp;
     //response =resp;
        //updatesimulationscene();
        //updateScene();
        updatesceneItself();
    }
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	string type=request.reqType;
	string name=request.Name;
	string requestInfo=request.requestInfo;

	cout<<"MSG:: type: "<<type<<", requestInfo: "<<requestInfo<<endl;
	vector<string> typeVec,requestInfoVec;
	boost::split(typeVec, type, boost::is_any_of("")); // Exmpl: Point-Point1, Point1, Cylinder-Cylinder1, Cylinder, Cylinder-Cylinder1-graspingPose1, Cylinder-Cylinder1-centerFrame
	boost::split(requestInfoVec, requestInfo, boost::is_any_of("-"));// Pose, Pose-Name, Center, Center-Name, boundingBox, boundingBall
	bool ret_name=false; // trturn name (if false: value)

	if(requestInfo.find("Name") != std::string::npos)
	{
		ret_name=true;
		cout<<"return name"<<endl;
	}
	else
	{
		cout<<"return values"<<endl;
	}

	for(int i=0;i<dataBase_.size();i++)
	{
		int Occurence=0;
		for(int j=0; j<dataBase_[i].name.size();j++)// cylinder cylinder1 graspingPose1, point4 Pose,
		{
			for(int k=0;k<typeVec.size();k++) // cylinder1 graspingPose1
			{
				//				cout<<"worldVec["<<i<<"].name["<<j<<"]: "<<worldVec[i].name[j]<<", typeVec["<<k<<"]: "<<typeVec[k]<<endl;
				if(dataBase_[i].name[j]==typeVec[k])
				{
					Occurence++;
					//					cout<<Occurence<<endl;
				}
			}
			if(Occurence==typeVec.size() && dataBase_[i].name.back().find(requestInfoVec[0]) != std::string::npos)
			{
				if(ret_name==true)
				{
					string KB_name;
					KB_name+=dataBase_[i].name[0];
					for(int l=1; l<dataBase_[i].name.size();l++)
						KB_name+="-"+dataBase_[i].name[l];

					cout<<"Res: "<<KB_name<<endl;
					res.names.push_back(KB_name);
				}
				else
				{   if(dataBase_[i].name.size()>1){
					for(size_t k=0;k<dataBase_[i].name.size();k++){

						res.names.push_back(dataBase_[i].name[k]);
					}
					
				}
					cout<<"Res: ";
					for(int m=0;m<dataBase_[i].value.size();m++)
					{
						res.pose.push_back(dataBase_[i].value[m]);

						cout<<dataBase_[i].value[m]<<" ";
					}
					
					
					cout<<endl;
					cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
					return true;// normally when ask for a vector value, it is just one vector value
				}
				break; // if a index of world vec happened, the same index should be passed and not happen again.
			}
		}
	}
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	return true;

	

}





void tamp_knowledge::updatesimulationscene(){

	tamp_msgs::objectssrv msg;
	msg.request.inscene = true;
	std::vector<string> names;
	std::vector<double> positions,sizes;
	if(simulationSceneClient.call(msg)){
		ROS_INFO("Requesting from simulation envrionmet for the objects");
		names=msg.response.names;
	 	positions=msg.response.position;
	 	sizes=msg.response.dimension;
	 	removeDataBaseInstant("obstacles",false);
	 	removeDataBaseInstant("cube",true);
	 	removeDataBaseInstant("cylinder",true);
        world obst;
        obst.name.push_back("obstacles");
	 	for(size_t i =0;i<names.size();i++){
			world inst;
			obst.name.push_back(names[i]);
			if(names[i]=="Table" || names[i]=="Cube"){
				
				inst.name.push_back("cube");
				inst.name.push_back(names[i]);
			}
			else{
				
				inst.name.push_back("cylinder");
				inst.name.push_back(names[i]);
			}
			
			for(size_t j =0;j<3;j++){
				inst.value.push_back(positions[i*3+j]);

			}
			for(size_t j =0;j<3;j++){
				inst.value.push_back(sizes[i*3+j]);

			}
			dataBase_.push_back(inst);

		}
		dataBase_.push_back(obst);

	}

updateSimulationObjectsMetric();
}

void tamp_knowledge::removeDataBaseInstant(string nameins,bool all){
	//ROS_INFO("removeDataBaseInstant");
	for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){

		if(it->name[0]==nameins){
		   
		    dataBase_.erase(it);
		    //ROS_INFO("Database %s found and removed",request.names[0].c_str());
		    if(all){
		    	//{it++;}
		    }
		    else{
		    	break;
		    }
		    
		}

		else
			{it++;}
	}
}


void tamp_knowledge::updateSimulationObjectsMetric(){
	ROS_INFO("updateSimulationObjectsMetric");
	std::vector<string> obsts;
	for(size_t i=0;i<dataBase_.size();i++){

		if(dataBase_[i].name[0]=="obstacles"){
			obsts= dataBase_[i].name;
			//obsts.erase(obsts.begin());
			//obsts.erase(obsts.begin());
			break;
		}
	}

	findClosests(obsts);
	findApprochingpoints();

}

void tamp_knowledge::findClosests(std::vector<string> &objects){
	ROS_INFO("findClosests");
     std::vector<std::vector<float>> allobjposes;
     std::vector<string> eligibleobjects;
	for(size_t i=0;i<dataBase_.size();i++){
		for(size_t j=0;j<objects.size();j++){
			//cout<<"checking for object : "<<objects[j]<<endl;
			if(dataBase_[i].name[0]=="cylinder"){
				if(dataBase_[i].name[1]==objects[j]){

					
					std::vector<float> obspos={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]};
					if(obspos[0]<1.0 && obspos[0]>0.5 && obspos[1]<0.5 && obspos[1]>-0.5){
						cout<<"Eligible object: "<<dataBase_[i].name[1]<<", x= "<<obspos[0]<<" ,y= "<<obspos[1]<<endl;
						eligibleobjects.push_back(dataBase_[i].name[1]);
						allobjposes.push_back(obspos);
						break;
					}
					
					

			}
		
		}

		}
		

	}



	std::vector<float> targetpos;
    std::vector<float> righeepos;
    std::vector<float> lefteepos;
    std::vector<float> basepos={0.0,0.0,0.0};
  


     tamp_msgs::ackquest msgr,msgl;
    geometry_msgs::PoseStamped posr,posl;
    msgr.request.arm ="right";
    if(robotStateClient.call(msgr))
    {
        posr =  msgr.response.eepos;
        righeepos = {posr.pose.position.x,posr.pose.position.y,posr.pose.position.z};   
    }

     msgl.request.arm ="left";
    if(robotStateClient.call(msgl))
    {
        posl =  msgl.response.eepos;
        lefteepos = {posl.pose.position.x,posl.pose.position.y,posl.pose.position.z}  ;    
    }


    tamp_msgs::targetsrv tms;
    tms.request.get = true;
    string targetobject;
    if(simulationTargetClient.call(tms)){
    	targetobject = tms.response.target;
    }
    for(size_t i=0;i<dataBase_.size();i++){
    	if(dataBase_[i].name.size()>1){
    		if(dataBase_[i].name[1]==targetobject){
			targetpos={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]};
			break;
			}
    	}
		

	}
	cout<<" Target pos is : "<<targetpos[0]<<" ," <<targetpos[1]<<" ," <<targetpos[2]<<endl;

	int clobtoree = minEucleadan(allobjposes,righeepos);
	int clobtolee = minEucleadan(allobjposes,lefteepos);
	int clobtotarget = minEucleadan(allobjposes,targetpos);
	int clobtobase = minEucleadan(allobjposes,basepos);

	world inst1,inst2,inst3,inst4,inst5;
	removeDataBaseInstant("clobree",false);
	removeDataBaseInstant("cloblee",false);
	removeDataBaseInstant("clobt",false);
	removeDataBaseInstant("clobb",false);
	removeDataBaseInstant("target",false);
	inst1.name.push_back("clobree");
	inst1.name.push_back(eligibleobjects[clobtoree]);
	dataBase_.push_back(inst1);
	inst2.name.push_back("cloblee");
	inst2.name.push_back(eligibleobjects[clobtolee]);
	dataBase_.push_back(inst2);
	inst3.name.push_back("clobt");
	inst3.name.push_back(eligibleobjects[clobtotarget]);
	dataBase_.push_back(inst3);
	inst4.name.push_back("clobb");
	inst4.name.push_back(eligibleobjects[clobtobase]);
	dataBase_.push_back(inst4);
	inst5.name.push_back("target");
	inst5.name.push_back(targetobject);
	dataBase_.push_back(inst5);




}

void tamp_knowledge::findApprochingpoints(){
	ROS_INFO("findApprochingpoints");
	string clobrees,cloblees,clobbs,targets,clobts;
	std::vector<float> clobreepos,clobleepos,clobbpos,clobtpos,targepos;
	for(size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name[0]=="clobree"){
			clobrees = dataBase_[i].name[1];
			cout<<"clobrees is: "<<clobrees<<endl;
		}
		else if(dataBase_[i].name[0]=="cloblee"){
			cloblees = dataBase_[i].name[1];
			cout<<"cloblees is: "<<cloblees<<endl;
		}
		else if(dataBase_[i].name[0]=="clobt"){
			clobts = dataBase_[i].name[1];
			cout<<"clobts is: "<<clobts<<endl;
		}
		else if(dataBase_[i].name[0]=="clobb"){
			clobbs = dataBase_[i].name[1];
			cout<<"clobbs is: "<<clobbs<<endl;
		}
		else if(dataBase_[i].name[0]=="target"){
			targets = dataBase_[i].name[1];
			cout<<"targets is: "<<targets<<endl;
		}
	}

	for(size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name.size()>1){ 

			if(dataBase_[i].name[1]==clobrees){
			clobreepos ={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]};
			cout<<"clobreepos: "<<clobreepos[0]<<" , "<<clobreepos[1]<<" , "<<clobreepos[2]<<" , "<<endl;
			break;
			}
		}
	}

	for(size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name.size()>1){ 

			if(dataBase_[i].name[1]==cloblees){
			clobleepos = {dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]};
				cout<<"clobleepos: "<<clobleepos[0]<<" , "<<clobleepos[1]<<" , "<<clobleepos[2]<<" , "<<endl;
				break;
			}
		}
	}
	for(size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name.size()>1){ 

			if(dataBase_[i].name[1]==clobbs){
				clobbpos = {dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]};
				cout<<"clobbpos: "<<clobbpos[0]<<" , "<<clobbpos[1]<<" , "<<clobbpos[2]<<" , "<<endl;
				break;
			}
		}
	}
	for(size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name.size()>1){ 

			if(dataBase_[i].name[1]==targets){
				targepos = {dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]};
				cout<<"targepos: "<<targepos[0]<<" , "<<targepos[1]<<" , "<<targepos[2]<<" , "<<endl;
				break;
			}
		}
	}

	for(size_t i=0;i<dataBase_.size();i++){
		if(dataBase_[i].name.size()>1){ 

			if(dataBase_[i].name[1]==clobts){
				clobtpos = {dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]};
				cout<<"clobtpos: "<<clobtpos[0]<<" , "<<clobtpos[1]<<" , "<<clobtpos[2]<<" , "<<endl;
				break;
			}
		}
	}



double preandgraspz = 0.04;	
double postgraspz = 0.3;
double offsetgrasp = 0.02;	
	
cout<<"ready for calculations hmmm"<<endl;
{
	//closest object to right ee
	double Yo = -0.7;
	double disfrob = 0.08;
	double X = clobreepos[0];
	double Y = clobreepos[1];
	
	double psi = atan((Y-Yo)/X);
	double yd = Y-disfrob*sin(psi);
	double xd = X - disfrob * cos(psi);
	double yo = Y+offsetgrasp*sin(psi);
	double xo = X + offsetgrasp * cos(psi);

	world inst,instgrasp,instpostgrasp;
	removeDataBaseInstant("clobree-pregrasp",false);
	inst.name.push_back("clobree-pregrasp");
	inst.value.push_back(xd);
	inst.value.push_back(yd);
	inst.value.push_back(preandgraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(1.57);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);
	removeDataBaseInstant("clobree-grasp",false);
	instgrasp.name.push_back("clobree-grasp");
	instgrasp.value.push_back(xo);
	instgrasp.value.push_back(yo);
	instgrasp.value.push_back(preandgraspz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(1.57);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);

	removeDataBaseInstant("clobree-postgrasp",false);
	instpostgrasp.name.push_back("clobree-postgrasp");
	instpostgrasp.value.push_back(xo);
	instpostgrasp.value.push_back(yo);
	instpostgrasp.value.push_back(postgraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(1.57);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);
}

{
	//closest objet to left ee
	double Yo = 0.7;
	double disfrob = 0.08;
	double X = clobleepos[0];
	double Y = clobleepos[1];
	
	double psi = atan((Y-Yo)/X);
	double yd = Y-disfrob*sin(psi);
	double xd = X - disfrob * cos(psi);
	double yo = Y+offsetgrasp*sin(psi);
	double xo = X + offsetgrasp * cos(psi);

	world inst,instgrasp,instpostgrasp;
	removeDataBaseInstant("cloblee-pregrasp",false);
	inst.name.push_back("cloblee-pregrasp");
	inst.value.push_back(xd);
	inst.value.push_back(yd);
	inst.value.push_back(preandgraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(1.57);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);
	removeDataBaseInstant("cloblee-grasp",false);
	instgrasp.name.push_back("cloblee-grasp");
	instgrasp.value.push_back(xo);
	instgrasp.value.push_back(yo);
	instgrasp.value.push_back(preandgraspz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(1.57);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);

	removeDataBaseInstant("cloblee-postgrasp",false);
	instpostgrasp.name.push_back("cloblee-postgrasp");
	instpostgrasp.value.push_back(xo);
	instpostgrasp.value.push_back(yo);
	instpostgrasp.value.push_back(postgraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(1.57);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);
}
	
{
	//closest object to base
	double Yo;
	if(clobbpos[1]>0){
		Yo = 0.7;
	}
	else{
		Yo = -0.7;
	}
	
	double disfrob = 0.08;
	double X = clobbpos[0];
	double Y = clobbpos[1];
	
	double psi = atan((Y-Yo)/X);
	double yd = Y-disfrob*sin(psi);
	double xd = X - disfrob * cos(psi);
	double yo = Y+offsetgrasp*sin(psi);
	double xo = X + offsetgrasp * cos(psi);
	world inst,instgrasp,instpostgrasp;
	removeDataBaseInstant("clobb-pregrasp",false);
	inst.name.push_back("clobb-pregrasp");
	inst.value.push_back(xd);
	inst.value.push_back(yd);
	inst.value.push_back(preandgraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(1.57);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);
	removeDataBaseInstant("clobb-grasp",false);
	instgrasp.name.push_back("clobb-grasp");
	instgrasp.value.push_back(xo);
	instgrasp.value.push_back(yo);
	instgrasp.value.push_back(preandgraspz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(1.57);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);
	removeDataBaseInstant("clobb-postgrasp",false);

	instpostgrasp.name.push_back("clobb-postgrasp");
	instpostgrasp.value.push_back(xo);
	instpostgrasp.value.push_back(yo);
	instpostgrasp.value.push_back(postgraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(1.57);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);
}

{
	//closest object to target
	double Yo;
	if(clobtpos[1]>0){
		Yo = 0.7;
	}
	else{
		Yo = -0.7;
	}
	
	double disfrob = 0.08;
	double X = clobtpos[0];
	double Y = clobtpos[1];
	
	double psi = atan((Y-Yo)/X);
	double yd = Y-disfrob*sin(psi);
	double xd = X - disfrob * cos(psi);
	double yo = Y+offsetgrasp*sin(psi);
	double xo = X + offsetgrasp * cos(psi);
	world inst,instgrasp,instpostgrasp;
	removeDataBaseInstant("clobt-pregrasp",false);
	inst.name.push_back("clobt-pregrasp");
	inst.value.push_back(xd);
	inst.value.push_back(yd);
	inst.value.push_back(preandgraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(1.57);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);
	removeDataBaseInstant("clobt-grasp",false);
	instgrasp.name.push_back("clobt-grasp");
	instgrasp.value.push_back(xo);
	instgrasp.value.push_back(yo);
	instgrasp.value.push_back(preandgraspz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(1.57);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);

	removeDataBaseInstant("clobt-postgrasp",false);
	instpostgrasp.name.push_back("clobt-postgrasp");
	instpostgrasp.value.push_back(xo);
	instpostgrasp.value.push_back(yo);
	instpostgrasp.value.push_back(postgraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(1.57);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);
}

{
	// target
	double Yo;
	if(targepos[1]>0){
		Yo = 0.7;
	}
	else{
		Yo = -0.7;
	}
	
	double disfrob = 0.08;
	double X = targepos[0];
	double Y = targepos[1];
	
	double psi = atan((Y-Yo)/X);
	double yd = Y-disfrob*sin(psi);
	double xd = X - disfrob * cos(psi);
	double yo = Y+offsetgrasp*sin(psi);
	double xo = X + offsetgrasp * cos(psi);

	world inst,instgrasp,instpostgrasp;
	removeDataBaseInstant("target-pregrasp",false);
	inst.name.push_back("target-pregrasp");
	inst.value.push_back(xd);
	inst.value.push_back(yd);
	inst.value.push_back(preandgraspz);
	inst.value.push_back(0.0);
	inst.value.push_back(1.57);
	inst.value.push_back(psi);
	inst.print();
	dataBase_.push_back(inst);
	removeDataBaseInstant("target-grasp",false);
	instgrasp.name.push_back("target-grasp");
	instgrasp.value.push_back(xo);
	instgrasp.value.push_back(yo);
	instgrasp.value.push_back(preandgraspz);
	instgrasp.value.push_back(0.0);
	instgrasp.value.push_back(1.57);
	instgrasp.value.push_back(psi);
	instgrasp.print();
	dataBase_.push_back(instgrasp);
	removeDataBaseInstant("target-postgrasp",false);

	instpostgrasp.name.push_back("target-postgrasp");
	instpostgrasp.value.push_back(xo);
	instpostgrasp.value.push_back(yo);
	instpostgrasp.value.push_back(postgraspz);
	instpostgrasp.value.push_back(0.0);
	instpostgrasp.value.push_back(1.57);
	instpostgrasp.value.push_back(psi);
	instpostgrasp.print();
	dataBase_.push_back(instpostgrasp);
}




}




int tamp_knowledge::minEucleadan(std::vector<std::vector<float>> objs,std::vector<float> des){
  //ROS_INFO("minEucleadan");
  std::vector<double> distances;
  for(std::size_t i=0;i<objs.size();i++){

  	double dx = objs[i][0] - des[0];
  	double dy = objs[i][1] - des[1];
  	//double dz = objs[i][2] - des[2];
  	//cout<<"dx : "<<dx<<"dy : "<<dy<<"dz : "<<dz<<endl;
    double dissq = dx*dx+dy*dy;
    double dis;
    //cout<<"dis square is : "<<dissq<<endl;
    if(dissq==0.0){
    	dis = 40;
    }
    else{
    	dis = sqrt(dissq);
    }
    
    distances.push_back(dis);
  }
   double min = distances[0];
   int index = 0;
    for(size_t i=0;i<distances.size();i++)
    {
        if(distances[i]<min){
        	min=distances[i];
    		index = i;
        }
       
    }

  return index;


}

int tamp_knowledge::minEucleadan(std::vector<std::vector<double>> objs,std::vector<double> des){
  //ROS_INFO("minEucleadan");
  std::vector<double> distances;
  for(std::size_t i=0;i<objs.size();i++){

  	double dx = objs[i][0] - des[0];
  	double dy = objs[i][1] - des[1];
  	//double dz = objs[i][2] - des[2];
  	//cout<<"dx : "<<dx<<"dy : "<<dy<<"dz : "<<dz<<endl;
    double dissq = dx*dx+dy*dy;
    double dis;
    //cout<<"dis square is : "<<dissq<<endl;
    if(dissq==0.0){
    	dis = 40;
    }
    else{
    	dis = sqrt(dissq);
    }
    
    distances.push_back(dis);
  }
   double min = distances[0];
   int index = 0;
    for(size_t i=0;i<distances.size();i++)
    {
        if(distances[i]<min){
        	min=distances[i];
    		index = i;
        }
       
    }

  return index;


}




void tamp_knowledge::updatesceneItself(){
	//ROS_INFO("Updating Scene ItSelf Database");
	//tamp_msgs::sceneobjects msg;
	//msg.request.update=true;
	std::vector<std_msgs::String> objtypes;
	std::vector<geometry_msgs::Transform> objposes;
    std::vector<float> boundboxvec;
    std::vector<string> objecttype;
    std::vector<string> grasptype;
    std::vector<string> objid;
    for(std::size_t i=0;i<dataBase_.size();i++){
       boost::split(objid, dataBase_[i].name[0], boost::is_any_of("-"));
       boost::split(objecttype, dataBase_[i].name[0], boost::is_any_of("_"));
       //ROS_INFO("Updating Scene ItSelf Database1");

       if(objecttype[0]=="cylinder" ||objecttype[0]=="cube"){
       //ROS_INFO("Updating Scene ItSelf Database2");
              boost::split(grasptype, objecttype[1], boost::is_any_of("-"));
              //ROS_INFO("Updating Scene ItSelf Database3");
              if(grasptype.size()>1){
	              if(grasptype[1]=="grasp"){
	              	//ROS_INFO("Updating Scene ItSelf Database4");

	                      std_msgs::String str;
			              str.data=objid[0];
			              objtypes.push_back(str);
			              geometry_msgs::Transform msg;
			              msg.translation.x=dataBase_[i].value[0];
			              msg.translation.y=dataBase_[i].value[1];
			              msg.translation.z=dataBase_[i].value[2];
			              msg.rotation.x = dataBase_[i].value[3];
			              msg.rotation.y = dataBase_[i].value[4];
			              msg.rotation.z = dataBase_[i].value[5];
			              objposes.push_back(msg);

              }
          }
          }
      }
            
              
    

   updateDataBase(objtypes,objposes);


}
       	
       	











void tamp_knowledge::updateScene(){
    ROS_INFO("Updating Scene Database");
	tamp_msgs::sceneobjects msg;
	msg.request.update=true;
	std::vector<std_msgs::String> objtypes;
	std::vector<geometry_msgs::Transform> objposes;
    std::vector<float> boundboxvec;
   if(sceneClient.call(msg)){

       objtypes=msg.response.types;
       objposes = msg.response.baseposes;
       boundboxvec = msg.response.boundbox;
       ROS_INFO("Updated Database");
   
   }

   else{

   	ROS_INFO("Couldn't Update Database");
   }

   updateDataBase(objtypes,objposes);

    


}

void tamp_knowledge::updateDataBase(std::vector<std_msgs::String> objtypes,std::vector<geometry_msgs::Transform> objposes){
    


    for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
    	 std::vector<string> requestInfoVec;
        boost::split(requestInfoVec, it->name[0], boost::is_any_of("_"));
    	if(it->name[0]=="num_of_objects"){
    		//it->value.clear();
          //  it->value.push_back(objtypes.size());
           
            dataBase_.erase(it);
    	}
    	else if(requestInfoVec[0]=="object"){
           
    		dataBase_.erase(it);
    	}
    	else if(it->name[0]=="boundbox")
    	{

    		dataBase_.erase(it);
    	}

		else
			{it++;}
	}

    world inst,instbound;
    inst.name.push_back("num_of_objects");
    inst.value.push_back(objtypes.size());
    nuofobj_=objtypes.size();
    //inst.name.resize(objtypes.size());
    for(std::size_t i=0;i<objtypes.size();i++){
    	inst.name.push_back(objtypes[i].data);

    }
    dataBase_.push_back(inst);
    
    //instbound.name.push_back("boundbox");
    //instbound.value = boundbouxing;
   // dataBase_.push_back(instbound);
    for(std::size_t i=0;i<objtypes.size();i++){
		world inst;
		//ROS_INFO("Inserting scene objects");
		//string objname = "object_"+to_string(i+1);
		//inst.name.push_back(objname);
		//inst.name.push_back("object");
		inst.name.push_back(objtypes[i].data);
		inst.value.push_back(objposes[i].translation.x);
		inst.value.push_back(objposes[i].translation.y);
		inst.value.push_back(objposes[i].translation.z);
		//inst.value.push_back(0.0);
		inst.value.push_back(objposes[i].rotation.x);
		inst.value.push_back(objposes[i].rotation.y);
		inst.value.push_back(objposes[i].rotation.z);
		dataBase_.push_back(inst);
	  

    }

     updatemetrics();

    //ROS_INFO("New Database Values");
   // for(int i=0;i<dataBase_.size();i++)
		//dataBase_[i].print();
	


}


void tamp_knowledge::updatemetrics(){
   
    tamp_msgs::ackquest msgr;
    geometry_msgs::PoseStamped posr;
    tamp_msgs::ackquest msgl;
    geometry_msgs::PoseStamped posl;
    msgr.request.arm ="right";
    msgl.request.arm ="left";
    ROS_INFO("msgl.request.arm");
    tamp_msgs::ackquest msg;
    //ros::spinOnce();
    if(robotStateClient.call(msgr))
    {
        posr =  msgr.response.eepos;
        cout<<"***********ree x:"<<posr.pose.position.x<<endl;
        cout<<"***********ree y:"<<posr.pose.position.y<<endl;
        cout<<"***********ree z:"<<posr.pose.position.z<<endl;
              
    }
    else{
         ROS_INFO("Couldn't Call end effector Service right");

    }
     if(robotStateClient.call(msgl))
    {
        posl =  msgl.response.eepos;
        
        
    }
    else{
         ROS_INFO("Couldn't Call end effector Service left");

    }

	for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
    	
    	
    	if(it->name[0]=="righteepose" || it->name[0]=="lefteepose"){

    		dataBase_.erase(it);
    	

    	}
    	

		else
			{it++;}
	}
	world instr,instl;
	instr.name.push_back("righteepose");
	instr.value.push_back(posr.pose.position.x);
	instr.value.push_back(posr.pose.position.y);
	instr.value.push_back(posr.pose.position.z);
	instr.value.push_back(posr.pose.orientation.x);
	instr.value.push_back(posr.pose.orientation.y);
	instr.value.push_back(posr.pose.orientation.z);
	instr.value.push_back(posr.pose.orientation.w);
    dataBase_.push_back(instr);

    instl.name.push_back("lefteepose");

	instl.value.clear();
    instl.value.push_back(posl.pose.position.x);
    instl.value.push_back(posl.pose.position.y);
    instl.value.push_back(posl.pose.position.z);
    instl.value.push_back(posl.pose.orientation.x);
    instl.value.push_back(posl.pose.orientation.y);
    instl.value.push_back(posl.pose.orientation.z);
    instl.value.push_back(posl.pose.orientation.w);
    dataBase_.push_back(instl);

   findClosestObjects();

    




}


void tamp_knowledge::findClosestObjects(){

     
    ROS_INFO("findClosestObjects");
    std::vector<std::vector<float>> posobjs;
    std::vector<float> targetpos;
    std::vector<float> righeepos;
    std::vector<float> lefteepos;
    std::vector<float> basepos={0.0,0.0,0.0};
    std::vector<string> objids;


    for(std::size_t i=0;i<dataBase_.size();i++){
        std::vector<string> requestInfoVec,grasptype;
        boost::split(requestInfoVec, dataBase_[i].name[0], boost::is_any_of("_"));
      if(requestInfoVec[0]=="cylinder" ||requestInfoVec[0]=="cube"){
      	
        boost::split(grasptype, dataBase_[i].name[0], boost::is_any_of("-"));

         if(grasptype.size()>1){

             if(grasptype[1]=="grasp"){
         
             		if(nuofobj_>1){
             			   if(grasptype[0]!="cylinder_target"){
					             std::vector<float> v{dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
					         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
					             posobjs.push_back(v);
					             objids.push_back(grasptype[0]);

      						}

             		}
             		else if(nuofobj_==1){
             			std::vector<float> v{dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
					    ,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
					    posobjs.push_back(v);
					    objids.push_back(grasptype[0]);


             		}
              }
      	
         }

      }
      if(dataBase_[i].name[0]=="righteepose"){

         righeepos ={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};

      }
     if(dataBase_[i].name[0]=="lefteepose"){

         lefteepos ={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
      }
      if(dataBase_[i].name[0]=="cylinder_target"){
          targetpos ={dataBase_[i].value[0],dataBase_[i].value[1],dataBase_[i].value[2]
         	,dataBase_[i].value[3],dataBase_[i].value[4],dataBase_[i].value[5]};
      }


    }


  //ROS_INFO("findClosestObjects2");

int mindistarget = minEucleadan(posobjs,targetpos);
int mindistrightee = minEucleadan(posobjs,righeepos);
int mindistleftee = minEucleadan(posobjs,lefteepos);
int mindistbase = minEucleadan(posobjs,basepos);
  //ROS_INFO("findClosestObjects3");
 for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
        std::vector<string> metricvec;
    	boost::split(metricvec,it->name[0], boost::is_any_of("-"));

    	if(metricvec[0]=="clobree"||metricvec[0]=="cloblee"||metricvec[0]=="clobb"
    		||metricvec[0]=="clobt"||metricvec[0]=="larob"){
            
             dataBase_.erase(it);

            }
    	

		else
			{it++;}
	}


world instlar;
instlar.name.push_back("larob");
instlar.name.push_back("cube_1");

dataBase_.push_back(instlar);




for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]=="cube_1-pregrasp"){
        instlar.name.clear();
        instlar.value.clear();
        instlar.name.push_back("larob-pregrasp");
		instlar.value=dataBase_[i].value;
		dataBase_.push_back(instlar);

	}
	if(dataBase_[i].name[0]=="cube_1-postgrasp"){

		instlar.name.clear();
        instlar.value.clear();
        instlar.name.push_back("larob-postgrasp");
		instlar.value=dataBase_[i].value;
		dataBase_.push_back(instlar);
	}
	if(dataBase_[i].name[0]=="cube_1-grasp"){

		instlar.name.clear();
        instlar.value.clear();
        instlar.name.push_back("larob-grasp");
		instlar.value=dataBase_[i].value;
		dataBase_.push_back(instlar);
	}
}





















world insteer,insteel,instb,instt;

insteer.name.push_back("clobree");
insteer.name.push_back(objids[mindistrightee]);
insteer.value.push_back(posobjs[mindistrightee][0]);
insteer.value.push_back(posobjs[mindistrightee][1]);
insteer.value.push_back(posobjs[mindistrightee][2]);
insteer.value.push_back(posobjs[mindistrightee][3]);
insteer.value.push_back(posobjs[mindistrightee][4]);
insteer.value.push_back(posobjs[mindistrightee][5]);
dataBase_.push_back(insteer);

 // ROS_INFO("findClosestObjects5");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobree-grasp");
insteer.value.push_back(posobjs[mindistrightee][0]);
insteer.value.push_back(posobjs[mindistrightee][1]);
insteer.value.push_back(posobjs[mindistrightee][2]);
insteer.value.push_back(posobjs[mindistrightee][3]);
insteer.value.push_back(posobjs[mindistrightee][4]);
insteer.value.push_back(posobjs[mindistrightee][5]);
dataBase_.push_back(insteer);
 // ROS_INFO("findClosestObjects6");
std::vector<float> valuespre,valuespost;
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistrightee]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistrightee]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}
 // ROS_INFO("findClosestObjects7");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobree-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
//  ROS_INFO("findClosestObjects8");
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobree-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);

 // ROS_INFO("findClosestObjects9");




insteel.name.push_back("cloblee");
insteel.name.push_back(objids[mindistleftee]);
insteel.value.push_back(posobjs[mindistleftee][0]);
insteel.value.push_back(posobjs[mindistleftee][1]);
insteel.value.push_back(posobjs[mindistleftee][2]);
insteel.value.push_back(posobjs[mindistleftee][3]);
insteel.value.push_back(posobjs[mindistleftee][4]);
insteel.value.push_back(posobjs[mindistleftee][5]);
dataBase_.push_back(insteel);
  //ROS_INFO("findClosestObjects10");

insteel.name.clear();
insteel.value.clear();
insteel.name.push_back("cloblee-grasp");
insteel.value.push_back(posobjs[mindistleftee][0]);
insteel.value.push_back(posobjs[mindistleftee][1]);
insteel.value.push_back(posobjs[mindistleftee][2]);
insteel.value.push_back(posobjs[mindistleftee][3]);
insteel.value.push_back(posobjs[mindistleftee][4]);
insteel.value.push_back(posobjs[mindistleftee][5]);
dataBase_.push_back(insteel);
 // ROS_INFO("findClosestObjects11");
valuespre.clear();
valuespost.clear();
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistleftee]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistleftee]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}
 // ROS_INFO("findClosestObjects12");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("cloblee-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("cloblee-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);





 // ROS_INFO("findClosestObjects13");
instb.name.push_back("clobb");
instb.name.push_back(objids[mindistbase]);
instb.value.push_back(posobjs[mindistbase][0]);
instb.value.push_back(posobjs[mindistbase][1]);
instb.value.push_back(posobjs[mindistbase][2]);
instb.value.push_back(posobjs[mindistbase][3]);
instb.value.push_back(posobjs[mindistbase][4]);
instb.value.push_back(posobjs[mindistbase][5]);
dataBase_.push_back(instb);


instb.name.clear();
instb.value.clear();
instb.name.push_back("clobb-grasp");
instb.value.push_back(posobjs[mindistbase][0]);
instb.value.push_back(posobjs[mindistbase][1]);
instb.value.push_back(posobjs[mindistbase][2]);
instb.value.push_back(posobjs[mindistbase][3]);
instb.value.push_back(posobjs[mindistbase][4]);
instb.value.push_back(posobjs[mindistbase][5]);
dataBase_.push_back(instb);




valuespre.clear();
valuespost.clear();
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistbase]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistbase]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}
  //ROS_INFO("findClosestObjects14");
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobb-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobb-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);




instt.name.push_back("clobt");
instt.name.push_back(objids[mindistarget]);
instt.value.push_back(posobjs[mindistarget][0]);
instt.value.push_back(posobjs[mindistarget][1]);
instt.value.push_back(posobjs[mindistarget][2]);
instt.value.push_back(posobjs[mindistarget][3]);
instt.value.push_back(posobjs[mindistarget][4]);
instt.value.push_back(posobjs[mindistarget][5]);
dataBase_.push_back(instt);


instt.name.clear();
instt.value.clear();
instt.name.push_back("clobt-grasp");
instt.value.push_back(posobjs[mindistarget][0]);
instt.value.push_back(posobjs[mindistarget][1]);
instt.value.push_back(posobjs[mindistarget][2]);
instt.value.push_back(posobjs[mindistarget][3]);
instt.value.push_back(posobjs[mindistarget][4]);
instt.value.push_back(posobjs[mindistarget][5]);
dataBase_.push_back(instt);






valuespre.clear();
valuespost.clear();
for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]==objids[mindistarget]+"-pregrasp"){

		valuespre=dataBase_[i].value;
	}
	if(dataBase_[i].name[0]==objids[mindistarget]+"-postgrasp"){

		valuespost=dataBase_[i].value;
	}
}

insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobt-pregrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespre[i]);

}
dataBase_.push_back(insteer);
insteer.name.clear();
insteer.value.clear();
insteer.name.push_back("clobt-postgrasp");
for(std::size_t i=0;i<6;i++){

   insteer.value.push_back(valuespost[i]);

}
dataBase_.push_back(insteer);








for(std::size_t i=0;i<dataBase_.size();i++){
	if(dataBase_[i].name[0]=="cylinder_target-grasp"){
       world insttarget;
       insttarget.name.push_back("target-grasp");
	   insttarget.value =dataBase_[i].value ;
	   dataBase_.push_back(insttarget);

	}
	else if(dataBase_[i].name[0]=="cylinder_target-pregrasp"){

		world insttarget;
       insttarget.name.push_back("target-pregrasp");
	   insttarget.value =dataBase_[i].value ;
	    dataBase_.push_back(insttarget);
	}
	else if(dataBase_[i].name[0]=="cylinder_target-postgrasp"){

		world insttarget;
       insttarget.name.push_back("target-postgrasp");
	   insttarget.value =dataBase_[i].value ;
	    dataBase_.push_back(insttarget);
	}
}
       world insttarget;
       insttarget.name.push_back("target");
       insttarget.name.push_back("cylinder_target");
	    dataBase_.push_back(insttarget);



}



void tamp_knowledge::eliminateCB(std_msgs::String::ConstPtr msg){
   ROS_INFO("Elimitating %s from scene",msg->data.c_str());
    string obj =msg->data;
    string itobj;

    for(size_t i=0;i<dataBase_.size();i++){

        if(dataBase_[i].name[0]==obj){

        	itobj= dataBase_[i].name[1];
        }
          


    }
   

  ROS_INFO("Elimitating %s from scene",itobj.c_str());
  for(vector<world>::iterator it=dataBase_.begin(); it!=dataBase_.end();){
    	
    	if(it->name[0]==itobj || it->name[0]==itobj+"-grasp" ||it->name[0]==itobj+"-pregrasp" ||it->name[0]==itobj+"-postgrasp"){
            
           
           dataBase_.erase(it);

    	}
    	else
			{it++;}

}
 
updatesceneItself();
}

void tamp_knowledge::printMap(std::map<int,string> themap){
	cout<<"$$$$$$$$$$$$$$$$CMAP IS $$$$$$$$$$$$$$$$$$$$$$$$$$$"<<endl;
	for (auto element: themap)
	{
		cout<<element.first<<", "<<element.second<<endl;
	}
	cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<endl;
}
tamp_knowledge::~tamp_knowledge(){


}
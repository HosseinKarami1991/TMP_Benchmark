#include "tamp_motion.hpp"


tamp_motion::tamp_motion(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
planning_scene_monitor_(planning_scene_monitor)
,optsright_(RIGHT_PLANNING_GROUP, ROBOT_DESCRIPTION)
,optsleft_(LEFT_PLANNING_GROUP, ROBOT_DESCRIPTION){

    
    cout<<"tamp_motion::tamp_motion"<<endl;



  trajservice = node_handle.advertiseService("tamp_motion_service", &tamp_motion::motionQuery,this);
  ackservice = node_handle.advertiseService("tamp_ack_service",&tamp_motion::ackQuery,this);
  tampKnowledgeClient= node_handle.serviceClient<tamp_msgs::knowledge>("tamp_knowledge_service");
  tampVREPMotionClient= node_handle.serviceClient<tamp_msgs::vrepmotion>("/tamp_vrep_motion_service");
  stop_motion = node_handle.subscribe("stop_motion",10,&tamp_motion::stopMotion,this);
  vrep_joint_state = node_handle.subscribe("/robot/joint_states",10,&tamp_motion::registerJointState,this);
  scene_plan_pub=node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
  vrep_rightarm_pub=node_handle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command",1);
  vrep_leftarm_pub=node_handle.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);


   if( !planning_scene_monitor_ )
  {
    loadPlanningSceneMonitor();
  }

  planning_scene_monitor_->startStateMonitor(JOINT_STATE_TOPIC);



  //moveit::planning_interface::MoveGroupInterface::Options optsright(RIGHT_PLANNING_GROUP, ROBOT_DESCRIPTION);
  //moveit::planning_interface::MoveGroupInterface::Options optsleft(LEFT_PLANNING_GROUP, ROBOT_DESCRIPTION);
  optsright_.robot_model_ = planning_scene_monitor_->getRobotModel();
  optsleft_.robot_model_ = planning_scene_monitor_->getRobotModel();
  move_group_left_.reset(new moveit::planning_interface::MoveGroupInterface(optsleft_));

 move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright_));

    cout<<"*****move_group.getEndEffectorLink();"<<move_group_right_->getEndEffectorLink()<<endl;
    
    cout<<move_group_right_->getEndEffectorLink()<<"this was ee link"<<endl;
    cout<<"*****move_group.getEndEffectorLink();"<<move_group_left_->getEndEffectorLink()<<endl;
    cout<<move_group_left_->getEndEffectorLink()<<"this was ee link"<<endl;
  ROS_DEBUG_STREAM_NAMED("baxter_move","Baxter Move Group Interface loaded");
  nuofplanning_=0;
  nuofexecution_=0;
}
bool tamp_motion::ackQuery(tamp_msgs::ackquest::Request& request, tamp_msgs::ackquest::Response& response){
     //  ros::AsyncSpinner spinner(1);
 //spinner.start();
     ROS_INFO("requested ee pos for %s arm",request.arm.c_str());
     if(!request.jointpose){
             geometry_msgs::PoseStamped temp;
     
             temp = getEECurrentpos(request.arm.c_str());
             std::vector<double> rpy = getEECurrentRPY(request.arm.c_str());
             temp.pose.orientation.x = rpy[0];
             temp.pose.orientation.y = rpy[1];
             temp.pose.orientation.z = rpy[2];
             response.eepos = temp;

     }

     else{

           
          response.jointposes = getJointPose(request.arm);

     }


     
     return true;




}
bool tamp_motion::motionQuery(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response){

  //ROS_INFO("Received motion planning request for %s arm with detail:",request.arm.c_str);
  //cout<<FBOLD()<<endl;
  cout<<"arm: "<<request.arm<<endl;
  cout<<"target position x y z: "<< request.targetpos.position.x <<" "<< request.targetpos.position.y <<" "<< request.targetpos.position.z <<endl;
  cout<<"target orientation Roll Pitch Yaw: "<< request.targetpos.orientation.x <<" "<< request.targetpos.orientation.y <<" "<< request.targetpos.orientation.z <<endl;
  cout <<"position and orientation tolerances: "<<request.position_tolerance.data<<" "<<request.orientation_tolerance.data<<endl;

 
 
 nuofplanning_++;

 double timenow = ros::Time::now().toSec();
 dominant_arm_ = request.arm;
 targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
 request.targetpos.orientation = tf2::toMsg(targetQuaternion_);
 targetPositionTolerance_ = request.position_tolerance.data;
 targetOrientationTolerance_ = request.orientation_tolerance.data;
 targetPos_ = request.targetpos;
 toexecute_ = request.execute;
 simulation_ = request.simulation;
 addcollision_ = request.withcollision;
 std::vector<double> targetjointpose = request.targetjointpos;
 if(targetjointpose.size()==7){
    sendtojointpose_=true;
 }
 else{
  sendtojointpose_=false;
 }

 ROS_INFO("planning in %s space",(sendtojointpose_)?" Joint ":" Cartesian ");




 if(addcollision_){
   if(simulation_){
      addSimulationCollision();
   }
   else{
     addColision();
   }
 
 }

 if(request.objecttoremve!=""){
    removeObject(request.objecttoremve);

 }
 //collisionObjects_ = request.collision_objects;
 //planning_scene_interface.addCollisionObjects(collisionObjects_);
 ros::AsyncSpinner spinner(1);
 spinner.start();
 bool simresult;

 if(dominant_arm_ == "right"){


   
    //move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright_));

    
    if(sendtojointpose_){
        move_group_right_->setMaxVelocityScalingFactor(0.5);

        move_group_right_->setMaxAccelerationScalingFactor(0.2);
        move_group_right_->setJointValueTarget(targetjointpose);
    
    }
    else{
        move_group_right_->setMaxVelocityScalingFactor(0.7);
        move_group_right_->clearPoseTargets();

        move_group_right_->setNumPlanningAttempts(20);
        move_group_right_->setPlanningTime(2);


        move_group_right_->setGoalPositionTolerance(targetPositionTolerance_);
        move_group_right_->setGoalOrientationTolerance(targetOrientationTolerance_);
        move_group_right_->setPoseTarget(targetPos_);
    }
   


    //moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;

    bool motion_result_ = (move_group_right_->plan(motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    response.success = motion_result_;
    if(motion_result_){
        solvedTraj_ = motion_plan_.trajectory_;
        response.soltraj = solvedTraj_;
        planningtime_ = motion_plan_.planning_time_;
        response.time = planningtime_;
         moveit::planning_interface::MoveItErrorCode code;
                  double timenow2 = ros::Time::now().toSec();
         double timepassed =timenow2-timenow;
         timespentforplanning_ +=timepassed;
         cout<<"overall motiom planning time is: "<<timespentforplanning_<<" Seconds"<<endl;
        if(toexecute_){
          nuofexecution_++;
            if(simulation_){
              ROS_INFO("Execution sending to VREP simulator");
                double timenow3 = ros::Time::now().toSec();
               //code = move_group_right_->asyncExecute(motion_plan_);
               //code = move_group_right_->execute(motion_plan_);
              
              simresult = sendToSimulation(solvedTraj_,dominant_arm_);
               double timenow4 = ros::Time::now().toSec();
             double timepassed2 =timenow4-timenow3;
             timespentforexecution_ +=timepassed2;
             cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
             cout<<"There has been : "<<nuofplanning_<<" plannings"<<endl;

            }

            else{
                double timenow3 = ros::Time::now().toSec();
           //code = move_group_right_->asyncExecute(motion_plan_);
                code = move_group_right_->execute(motion_plan_);
                 double timenow4 = ros::Time::now().toSec();
               double timepassed2 =timenow4-timenow3;
               timespentforexecution_ +=timepassed2;
               cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
               cout<<"There has been : "<<nuofplanning_<<" plannings"<<endl;
            }
          

        }
        if(code==moveit::planning_interface::MoveItErrorCode::SUCCESS || simresult){

          response.executedtrajectory = true;
        }
        else{
          response.executedtrajectory = false;
        }


    }
 
    //moveit::planning_interface::MoveItErrorCode code = move_group_right_->asyncMove();
    // response.success = motion_result_;
   //  if(motion_result_){
   //     response.soltraj = solvedTraj_;
   //     response.time = planningtime_;
    //    move_group_right_->execute(motion_plan_);
     //}
 
      // printCurrentJointValues();
    // getCurrentPose();
 }
 else if(dominant_arm_ == "left") {
     
        ROS_INFO("planning for left arm");
    //move_group_left_->setPoseTarget(targetPos_);

       if(sendtojointpose_){
        move_group_left_->setMaxVelocityScalingFactor(0.5);
        move_group_left_->setMaxAccelerationScalingFactor(0.2);
        move_group_left_->setJointValueTarget(targetjointpose);
    }
    else{
          
          move_group_left_->setMaxVelocityScalingFactor(0.7);
          move_group_left_->clearPoseTargets();
          move_group_left_->setNumPlanningAttempts(20);
          move_group_left_->setPlanningTime(2);
          move_group_left_->setGoalPositionTolerance(targetPositionTolerance_);
          move_group_left_->setGoalOrientationTolerance(targetOrientationTolerance_);
          move_group_left_->setPoseTarget(targetPos_);

    }
   



    bool motion_result_ =  (move_group_left_->plan(motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    response.success = motion_result_;
    if(motion_result_){
        solvedTraj_ = motion_plan_.trajectory_;
        response.soltraj = solvedTraj_;
        planningtime_ = motion_plan_.planning_time_;
        response.time = planningtime_;
        moveit::planning_interface::MoveItErrorCode code;
        double timenow2 = ros::Time::now().toSec();
         double timepassed =timenow2-timenow;
         timespentforplanning_ +=timepassed;
         cout<<"overall motiom planning time is: "<<timespentforplanning_<<" Seconds"<<endl;
        if(toexecute_){

          nuofexecution_++;
            if(simulation_){
              ROS_INFO("Execution sending to VREP simulator");
                double timenow3 = ros::Time::now().toSec();
               //code = move_group_right_->asyncExecute(motion_plan_);
               //code = move_group_right_->execute(motion_plan_);
              
              simresult = sendToSimulation(solvedTraj_,dominant_arm_);
               double timenow4 = ros::Time::now().toSec();
             double timepassed2 =timenow4-timenow3;
             timespentforexecution_ +=timepassed2;
             cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
             cout<<"There has been : "<<nuofplanning_<<" plannings"<<endl;

            }

            else{
                       double timenow3 = ros::Time::now().toSec();
           //code = move_group_left_->asyncExecute(motion_plan_);
                     code = move_group_left_->execute(motion_plan_);
                     double timenow4 = ros::Time::now().toSec();
                   double timepassed2 =timenow4-timenow3;
                   timespentforexecution_ +=timepassed2;
                   cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
                  cout<<"There has been : "<<nuofplanning_<<" plannings"<<endl;

            }



   


        }
          if(code==moveit::planning_interface::MoveItErrorCode::SUCCESS || simresult){

          response.executedtrajectory = true;
        }
        else{
          response.executedtrajectory = false;
        }

    }
    

    
    // printPosition(getEECurrentpos());
     //printRotation(getEECurrentRPY());
     
 }
 else{ROS_INFO("inserted arm %s is wrong!!!",dominant_arm_.c_str());}

 return true;
}

string tamp_motion::getEndEffectorLink(){
    if(dominant_arm_ == "right"){
        //move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright_));
        EELink = move_group_right_->getEndEffectorLink();
    }
    else if (dominant_arm_ == "left"){

       EELink = move_group_left_->getEndEffectorLink();
    }
   
return EELink;
}

geometry_msgs::PoseStamped tamp_motion::getEECurrentpos(string arm){

//	 if( !planning_scene_monitor_ )
  //{
 //   loadPlanningSceneMonitor();
//  }

//  planning_scene_monitor_->startStateMonitor(JOINT_STATE_TOPIC);
    //planning_scene_monitor_->updatesScene(planning_scene_monitor_->getPlanningScene());
     // move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright_));
    // move_group_left_.reset(new moveit::planning_interface::MoveGroupInterface(optsleft_));
     ros::AsyncSpinner spinner(1);
     spinner.start();

    if(arm == "right"){
        ROS_INFO("Getting right arm ee Pos");
        //
        EECurrentpos_ = move_group_right_->getCurrentPose(move_group_right_->getEndEffectorLink());
        printPosition(EECurrentpos_);
        printRotation(getEECurrentRPY("right"));
    }
    else if (arm == "left"){
        ROS_INFO("Getting left arm ee Pos");
        //
        EECurrentpos_ = move_group_left_->getCurrentPose(move_group_left_->getEndEffectorLink());
        printPosition(EECurrentpos_);
        printRotation(getEECurrentRPY("left"));
          
    }
return EECurrentpos_;
}


vector<double> tamp_motion::getEECurrentRPY(string arm){
    ros::AsyncSpinner spinner(1);
    spinner.start();
    if(arm == "right"){

        EECurrentRPY_ = move_group_right_->getCurrentRPY(move_group_right_->getEndEffectorLink());  
    }
    else if(arm == "left"){

         EECurrentRPY_ = move_group_left_->getCurrentRPY(move_group_left_->getEndEffectorLink());
    }


 return EECurrentRPY_;
}
void tamp_motion::setVisualTools(){

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("torso");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
  //  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //text_pose.translation().z() = 1.75;
   // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
  
}

void tamp_motion::printPosition(geometry_msgs::PoseStamped pos){

 std::cout<<"**********this is pos of end effector**********"<<std::endl;
 std::cout<<"x: "<<pos.pose.position.x<<std::endl;
 std::cout<<"y: "<<pos.pose.position.y<<std::endl;
 std::cout<<"z: "<<pos.pose.position.z<<std::endl;

}

void tamp_motion::printRotation(vector<double> v){

 std::cout<<"**********this is rotation of end effector**********"<<std::endl;
 std::cout<<"roll: "<<v.at(0)<<std::endl;
 std::cout<<"pitch: "<<v.at(1)<<std::endl;
 std::cout<<"yaw: "<<v.at(2)<<std::endl;

}


bool tamp_motion::loadPlanningSceneMonitor(){


  ROS_DEBUG_STREAM_NAMED("baxter_move","Loading planning scene monitor");

  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  ros::spinOnce();
  ros::Duration(0.5).sleep(); // todo: reduce this time?
  ros::spinOnce();

  if (!planning_scene_monitor_->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED("baxter_move","Planning scene not configured");
    return false;
  }

  return true;
}




bool tamp_motion::sendToPose(const geometry_msgs::PoseStamped& pose)
                                           
{
  if (pose.header.frame_id.empty())
  {
    ROS_ERROR_STREAM_NAMED("baxter_move","Frame ID was not set for requested pose");
    return false;
  }
         moveit::planning_interface::MoveGroupInterface::Options optsright(RIGHT_PLANNING_GROUP, ROBOT_DESCRIPTION);

    move_group_right_.reset(new moveit::planning_interface::MoveGroupInterface(optsright));

    move_group_right_->clearPoseTargets();

    move_group_right_->setPoseTarget(pose);
    move_group_right_->setNumPlanningAttempts(4);
    move_group_right_->setPlanningTime(20);
    move_group_right_->setGoalPositionTolerance(1e-3); // meters
    move_group_right_->setGoalOrientationTolerance(1e-2); // radians


   ros::AsyncSpinner spinner(1);
     spinner.start();
  
  motion_result_ =  (move_group_right_->plan(motion_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout << "motion_result_ " << motion_result_<< std::endl;

  //std::cout << "sending " << std::endl;
  //moveit::planning_interface::MoveItErrorCode code = move_group_right_->asyncMove();
  //std::cout << "sending finished" << std::endl;
  //ros::spinOnce();
  //ros::Duration(0.5).sleep();

  //return convertResult(code);
  return motion_result_;
}


bool tamp_motion::convertResult(moveit::planning_interface::MoveItErrorCode& code)
{
  switch (code.val)
  {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      ROS_INFO_STREAM_NAMED("baxter_move","Planning and execution succeeded");
      return true;

    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
      ROS_ERROR_STREAM_NAMED("baxter_move","Failed because of invalid motion plan");
      return false;

   
  }

  ROS_ERROR_STREAM_NAMED("baxter_move","Planning and execution failed with code " << code.val);
  return false;

}



 geometry_msgs::Pose tamp_motion::getCurrentPose()
                                                          
{
   
  robot_state::RobotState state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    const double *p =  state.getJointPositions("right_e1");
    std::cout << "position e1 is"<<*p<<std::endl;
  state.updateLinkTransforms();
  //Eigen::Affine3d pose = state.getGlobalLinkTransform("right_hand");
   //Eigen::Isometry3d  pose= state.getGlobalLinkTransform("right_hand");
  //geometry_msgs::Pose pose_msg = visual_tools->convertPose(pose);

  ROS_INFO_STREAM_NAMED("baxter_move","pose is:");
  std::cout << "geometry_msgs::PoseStamped pose_msg;\n";
  /*
  std::cout << "pose_msg.pose.position.x = " << pose_msg.position.x << ";\n";
  std::cout << "pose_msg.pose.position.y = " << pose_msg.position.y << ";\n";
  std::cout << "pose_msg.pose.position.z = " << pose_msg.position.z << ";\n";
  std::cout << "pose_msg.pose.orientation.x = " << pose_msg.orientation.x << ";\n";
  std::cout << "pose_msg.pose.orientation.y = " << pose_msg.orientation.y << ";\n";
  std::cout << "pose_msg.pose.orientation.z = " << pose_msg.orientation.z << ";\n";
  std::cout << "pose_msg.pose.orientation.w = " << pose_msg.orientation.w << ";\n";
*/
  // Feedback
  //std::cout << "pose_msg = " << pose << ";\n";
  //visual_tools->publishArrow(pose_msg, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  geometry_msgs::Pose pose_msg;
  return pose_msg;
}

std::vector<double> tamp_motion::getJointPose(string & arm){
      ros::AsyncSpinner spinner(1);
      spinner.start();

    std::vector<double> joint_values;
    if(arm =="right"){

        joint_values = move_group_right_->getCurrentJointValues();
      std::vector<std::string> joint_names = move_group_right_->getJointNames();
      std::cout<<"size of joint name is: "<<joint_names.size()<<std::endl;
      std::cout<<"size of joint value is: "<<joint_values.size()<<std::endl;
      std::cout << "<group_state name=\"\" group=\"" << "dominant_arm_" << "\">\n";
      for (std::size_t i = 0; i < joint_values.size(); ++i){
      std::cout << "  <joint name=\"" << joint_names[i] <<"\" value=\"" << joint_values[i] << "\" />\n";
      }
      std::cout << "</group_state>\n\n\n\n";
    }
    else if(arm =="left"){
          joint_values = move_group_left_->getCurrentJointValues();
          std::vector<std::string> joint_names = move_group_right_->getJointNames();
          std::cout<<"size of joint name is: "<<joint_names.size()<<std::endl;
          std::cout<<"size of joint value is: "<<joint_values.size()<<std::endl;
          std::cout << "<group_state name=\"\" group=\"" << "dominant_arm_" << "\">\n";
          for (std::size_t i = 0; i < joint_values.size(); ++i){
          std::cout << "  <joint name=\"" << joint_names[i] <<"\" value=\"" << joint_values[i] << "\" />\n";
          }
          std::cout << "</group_state>\n\n\n\n";



    }

    return joint_values;
    
}


moveit::planning_interface::MoveItErrorCode tamp_motion::sendToPose(const std::string &pose_name)
{
  ROS_INFO_STREAM_NAMED("baxter_move_group_interface","Sending to pose '" << pose_name << "'");

  move_group_right_->setNamedTarget(pose_name);
  move_group_right_->setPlanningTime(15);
  moveit::planning_interface::MoveItErrorCode result = move_group_right_->move();

  if( !result )
    ROS_ERROR_STREAM_NAMED("baxter_move_group_interface","Failed to send Baxter to pose '" << pose_name << "'");

  return result;
}


void tamp_motion::printCurrentJointValues(){
    

    ros::AsyncSpinner spinner(1);
    spinner.start();


   // planning_scene_monitor_->updatesScene(planning_scene_monitor_->getPlanningScene());

    std::vector<double> joint_values;
    joint_values = move_group_right_->getCurrentJointValues();
    std::vector<std::string> joint_names = move_group_right_->getJointNames();
    std::cout<<"size of joint name is: "<<joint_names.size()<<std::endl;
    std::cout<<"size of joint value is: "<<joint_values.size()<<std::endl;
    // Output XML
    std::cout << "<group_state name=\"\" group=\"" << "dominant_arm_" << "\">\n";
    for (std::size_t i = 0; i < joint_values.size(); ++i)
    {
    std::cout << "  <joint name=\"" << joint_names[i] <<"\" value=\"" << joint_values[i] << "\" />\n";
    }
    std::cout << "</group_state>\n\n\n\n";

}


std::vector<double> tamp_motion::getEEStateFromJoint(string &arm,std::vector<double> &jont_poses){

    //std::vector<double> joint_values;
  //initialize joint values
  robot_state::RobotState state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
  std::vector<double> eeposes;
  if(arm=="right"){

      state.setJointGroupPositions(RIGHT_PLANNING_GROUP, jont_poses);
      const Eigen::Affine3d & end_effector_state = state.getGlobalLinkTransform(move_group_right_->getEndEffectorLink());
      ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
      ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
      eeposes.push_back(end_effector_state.translation().x());
      eeposes.push_back(end_effector_state.translation().y());
      eeposes.push_back(end_effector_state.translation().z());

  }

  else if(arm=="left"){
      state.setJointGroupPositions(LEFT_PLANNING_GROUP, jont_poses);
      const Eigen::Affine3d & end_effector_state= state.getGlobalLinkTransform(move_group_right_->getEndEffectorLink());
      ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
      ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
      eeposes.push_back(end_effector_state.translation().x());
      eeposes.push_back(end_effector_state.translation().y());
      eeposes.push_back(end_effector_state.translation().z());

  }
  else{
    ROS_ERROR("The given arm doesn't exist");

  }

  return eeposes;
  
}















void tamp_motion::addColision(){

  ROS_INFO("Adding Colission Objects to Scene");

  if(!colisionobjectsids_.empty()){

    planning_scene_interface.removeCollisionObjects(colisionobjectsids_);
    colisionobjectsids_.clear();
    object_colors_.clear();
  }
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::CollisionObject collision_object,collision_object2,colisionstorage,colisionstorage2;
  ///////////////////////////////////////////////////////////////Table
  collision_object.header.frame_id = move_group_right_->getPlanningFrame();

  collision_object.id = "table";//the table
  collision_object.header.frame_id="world";
  shape_msgs::SolidPrimitive primitivetable;
  primitivetable.type = primitivetable.BOX;
  primitivetable.dimensions.resize(3);
  primitivetable.dimensions[0] = 1;
  primitivetable.dimensions[1] = 1.65;
  primitivetable.dimensions[2] = 1;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.7;
  box_pose.position.y = -0.1;
  box_pose.position.z =  -0.62;

  collision_object.primitives.push_back(primitivetable);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);


  moveit_msgs::ObjectColor bst1;
  bst1.id = "table";
  bst1.color.g=1.0f;
  bst1.color.b=0.0f;
  bst1.color.r=0.0f;
  bst1.color.a=1.0;
  object_colors_.push_back(bst1);

//////////////////////////////////////////////////////storage
  /*
  colisionstorage.header.frame_id = move_group_right_->getPlanningFrame();
  
  tamp_msgs::knowledge knmsg;
  knmsg.request.reqType="boxr-grasp";
  std::vector<double> boxposes;
  if(tampKnowledgeClient.call(knmsg)){

  boxposes = knmsg.response.pose;
  
  }
  colisionstorage.id = "boxr";
  colisionstorage.header.frame_id="world";
  shape_msgs::SolidPrimitive primitivestorage;
  primitivestorage.type = primitivestorage.BOX;
  primitivestorage.dimensions.resize(3);

  primitivestorage.dimensions[0] = 0.26;
  primitivestorage.dimensions[1] = 0.3;
  primitivestorage.dimensions[2] = 0.14;

  geometry_msgs::Pose storage_pose;
  storage_pose.orientation.w = 1.0;
  std::vector<double> xyzpos;
  if(boxposes.size()==6){
    storage_pose.position.x =  boxposes[0];
    storage_pose.position.y = boxposes[1];
     storage_pose.position.z =  -0.03;
  }
  
  else if(boxposes.size()==7){
    std::string side("right");
    xyzpos = getEEStateFromJoint(side,boxposes);
     storage_pose.position.x =  xyzpos[0];
    storage_pose.position.y = xyzpos[1];
     storage_pose.position.z =  -0.03;
  }
  

  colisionstorage.primitives.push_back(primitivestorage);
  colisionstorage.primitive_poses.push_back(storage_pose);
  colisionstorage.operation = colisionstorage.ADD;

  collision_objects.push_back(colisionstorage);


  moveit_msgs::ObjectColor b1;
  b1.color.g=1.0f;
  b1.color.b=0.0f;
  b1.color.r=1.0f;
  b1.color.a=1.0f;
  object_colors_.push_back(b1);

  colisionstorage2.header.frame_id = move_group_right_->getPlanningFrame();

  knmsg.request.reqType="boxl-grasp";
  std::vector<double> boxlposes;
  if(tampKnowledgeClient.call(knmsg)){

  boxlposes = knmsg.response.pose;
  colisionstorage2.id = "boxl";//the table
  colisionstorage2.header.frame_id="world";
  shape_msgs::SolidPrimitive primitivestorage;
  primitivestorage.type = primitivestorage.BOX;
  primitivestorage.dimensions.resize(3);

  primitivestorage.dimensions[0] = 0.26;
  primitivestorage.dimensions[1] = 0.3;
  primitivestorage.dimensions[2] = 0.15;

  geometry_msgs::Pose storage_pose_left;
  storage_pose_left.orientation.w = 1.0;
    std::vector<double> xyzposl;
  if(boxlposes.size()==6){
    storage_pose_left.position.x =  boxlposes[0];
    storage_pose_left.position.y = boxlposes[1];
     storage_pose_left.position.z =  -0.03;
  }
  
  else if(boxlposes.size()==7){
    std::string side("left");
    xyzposl = getEEStateFromJoint(side,boxlposes);
     storage_pose_left.position.x =  xyzposl[0];
    storage_pose_left.position.y = xyzposl[1];
     storage_pose_left.position.z =  -0.03;
  }



  colisionstorage2.primitives.push_back(primitivestorage);
  colisionstorage2.primitive_poses.push_back(storage_pose_left);
  colisionstorage2.operation = colisionstorage2.ADD;

  collision_objects.push_back(colisionstorage2);


  moveit_msgs::ObjectColor b2;
  b2.color.g=1.0f;
  b2.color.b=0.0f;
  b2.color.r=1.0f;
  b2.color.a=1.0f;
  object_colors_.push_back(b2);
  }
*/



////////////////////////////////////////////////////object on table
    tamp_msgs::knowledge knmsg;
    knmsg.request.reqType="num_of_objects";
    std::vector<string> objectsontable;
    if(tampKnowledgeClient.call(knmsg)){
      objectsontable = knmsg.response.names;
    }

    for(size_t i=0;i<objectsontable.size();i++){
        std::vector<string> cylname;
       boost::split(cylname, objectsontable[i], boost::is_any_of("_"));
       if(cylname[0]=="cylinder"){
          std::vector<double> cylpos;
          string cyname = objectsontable[i] + "-grasp";
          knmsg.request.reqType=cyname;
          if(tampKnowledgeClient.call(knmsg)){
                collision_object2.header.frame_id = move_group_right_->getPlanningFrame();
                cylpos = knmsg.response.pose;
                collision_object2.id = objectsontable[i];//the table
                colisionobjectsids_.push_back(collision_object2.id);
                collision_object2.header.frame_id="world";
                shape_msgs::SolidPrimitive primitiveobject;
                primitiveobject.type = primitiveobject.CYLINDER;
                primitiveobject.dimensions.resize(2);
                primitiveobject.dimensions[0] = 0.23;
                primitiveobject.dimensions[1] = 0.02;
                geometry_msgs::Pose obj_pose;
                obj_pose.orientation.w = 1.0;
                obj_pose.position.x =  cylpos[0];
                obj_pose.position.y = cylpos[1];
                obj_pose.position.z =  -0.03;
                collision_object2.primitives.push_back(primitiveobject);
                collision_object2.primitive_poses.push_back(obj_pose);
                collision_object2.operation = collision_object2.ADD;
                collision_objects.push_back(collision_object2);
                moveit_msgs::ObjectColor obst;
                if(cylname[1]=="target"){
                  obst.color.g=0.1f;
                  obst.color.b=0.1f;
                  obst.color.r=0.1f;
                  obst.color.a=1.0f;
                }
                else{
                  obst.color.g=0.0f;
                  obst.color.b=0.0f;
                  obst.color.r=1.0f;
                  obst.color.a=1.0f;

                }
                
                obst.id = objectsontable[i];
                object_colors_.push_back(obst);
                collision_objects.push_back(collision_object2);
          }



       }

    }

/*
    knmsg.request.reqType="object-grasp";
    std::vector<double> objposes;
 
   if(tampKnowledgeClient.call(knmsg)){
    objposes = knmsg.response.pose;
    collision_object2.id = "object";//the table
    collision_object2.header.frame_id="world";
    shape_msgs::SolidPrimitive primitiveobject;
    primitiveobject.type = primitiveobject.CYLINDER;
    primitiveobject.dimensions.resize(2);
    primitiveobject.dimensions[0] = 0.23;
    primitiveobject.dimensions[1] = 0.02;

    geometry_msgs::Pose obj_pose;
    obj_pose.orientation.w = 1.0;
      std::vector<double> xyzposobj;
  if(objposes.size()==6){
    obj_pose.position.x =  objposes[0];
    obj_pose.position.y = objposes[1];
     obj_pose.position.z =  -0.03;
  }
  
  else if(objposes.size()==7){
      std::string side("right");
    xyzposobj = getEEStateFromJoint(side,objposes);
     obj_pose.position.x =  xyzposobj[0];
    obj_pose.position.y = xyzposobj[1];
     obj_pose.position.z =  (primitiveobject.dimensions[0]/2)-0.14;
  }


    collision_object2.primitives.push_back(primitiveobject);
    collision_object2.primitive_poses.push_back(obj_pose);
    collision_object2.operation = collision_object2.ADD;

    collision_objects.push_back(collision_object2);


    moveit_msgs::ObjectColor obst;
    obst.color.g=0.0f;
    obst.color.b=0.0f;
    obst.color.r=1.0f;
    obst.color.a=1.0f;
    object_colors_.push_back(obst);
}
*/
////////////////////////////////////////////////
/*
std::vector<string> obstalsesname;
tamp_msgs::knowledge knmsg;
knmsg.request.reqType="num_of_objects";
int numofobjects;
if(tampKnowledgeClient.call(knmsg)){

   numofobjects = (int)knmsg.response.pose[0];
   obstalsesname = knmsg.response.names;
}
std::vector<std::vector<double>> obstaclespos;


for(std::size_t i=0;i<numofobjects;i++){
    tamp_msgs::knowledge knmsg;
    knmsg.request.reqType=obstalsesname[i]+"-grasp";
    ROS_INFO("Query for cylinders");
   if(tampKnowledgeClient.call(knmsg)){
   
     obstaclespos.push_back(knmsg.response.pose);
}


}


int j=0;
int k=0;
ROS_INFO("Query for cubes2");
for(std::size_t i=0;i<numofobjects;i++){
   std::vector<string> obstaclenametwo;
  boost::split(obstaclenametwo, obstalsesname[i], boost::is_any_of("_"));
   if(obstaclenametwo[0]=="cylinder"){
      j++;
      ROS_INFO("Query for cubes3");
      moveit_msgs::CollisionObject collision_cylinder;
      collision_cylinder.header.frame_id="world";
      std::vector<string> obstaclename;
      //boost::split(obstaclename, obstalsesname[i], boost::is_any_of("-"));
      collision_cylinder.id = obstalsesname[i];
      colisionobjectsids_.push_back(collision_cylinder.id);
      shape_msgs::SolidPrimitive primitivecyl;
      primitivecyl.type = primitivecyl.CYLINDER;
      primitivecyl.dimensions.resize(2);
      primitivecyl.dimensions[0] = 0.23;
      primitivecyl.dimensions[1] = 0.02;
      //primitive.dimensions[2] = 1;
        ROS_INFO("Query for cubes5");
      box_pose.orientation.w = 1.0;
      box_pose.orientation.x =0.0;
      box_pose.orientation.y =0.0;
      box_pose.orientation.z =0.0;
      box_pose.position.x =  obstaclespos[i][0];
      box_pose.position.y = obstaclespos[i][1];
      box_pose.position.z = (primitivecyl.dimensions[0]/2)-0.14 ;

      collision_cylinder.primitives.push_back(primitivecyl);
      collision_cylinder.primitive_poses.push_back(box_pose);
      collision_cylinder.operation = collision_cylinder.ADD;
      collision_objects.push_back(collision_cylinder);
      moveit_msgs::ObjectColor b3;
      if(obstalsesname[i]=="cylinder_target"){
            b3.color.r=1.0f;
           b3.color.g=0.0f;
           b3.color.b=0.0f;
           b3.color.a=1.0f;
      }

      else{
           b3.color.r=0.0f;
           b3.color.g=0.0f;
           b3.color.b=1.0f;
           b3.color.a=1.0f;

      }
    
     object_colors_.push_back(b3);
          ROS_INFO("Query for cubes6");
   }
    if(obstaclenametwo[0]=="cube"){

       ROS_INFO("Query for cubes4");
       k++;
       moveit_msgs::CollisionObject collision_box;
      collision_box.header.frame_id="world";

      collision_box.id = obstalsesname[i];
      colisionobjectsids_.push_back(collision_box.id);
      
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.12;
      primitive.dimensions[1] = 0.24;
      primitive.dimensions[2] = 0.3;

      geometry_msgs::Pose boxpose;
      boxpose.orientation.w = 1.0;
      boxpose.position.x =  0.64; ;
      boxpose.position.y = -0.05; ;
      boxpose.position.z = (primitive.dimensions[2]/2)-0.15 ;

      collision_box.primitives.push_back(primitive);
      collision_box.primitive_poses.push_back(boxpose);
      collision_box.operation = collision_box.ADD;
      collision_objects.push_back(collision_box);
      moveit_msgs::ObjectColor b2;
     b2.color.b=0.8f;
     b2.color.g=0.0;
     b2.color.r=0.8f;
     b2.color.a=0.9f;
     object_colors_.push_back(b2);





//////////////bounding box


    /* 
    geometry_msgs::Pose box_pose2;
    collision_object2.id = "cube_"+to_string(k+1);
      colisionobjectsids_.push_back(collision_object2.id);
      
      shape_msgs::SolidPrimitive primitive2;
      primitive2.type = primitive2.BOX;
      primitive2.dimensions.resize(3);
      primitive2.dimensions[0] = fabs(boundboxvec[7]);
      primitive2.dimensions[1] = fabs(boundboxvec[8]);
      primitive2.dimensions[2] = fabs(boundboxvec[9]);

      box_pose2.orientation.x= boundboxvec[3];
      box_pose2.orientation.y = boundboxvec[4];
      box_pose2.orientation.z = boundboxvec[5];
      box_pose2.orientation.w = boundboxvec[6];
      box_pose2.position.x =  boundboxvec[0];
      box_pose2.position.y = boundboxvec[1];
      box_pose2.position.z = boundboxvec[2] ;

      collision_object2.primitives.push_back(primitive2);
      collision_object2.primitive_poses.push_back(box_pose2);
      collision_object2.operation = collision_object2.ADD;
      collision_objects.push_back(collision_object2);
      moveit_msgs::ObjectColor b3;
     b3.color.b=float(0.3);
     b3.color.g=0.3;
     b3.color.r=0.9;
     b3.color.a=1.0f;
     object_colors_.push_back(b3);

*/




   //}
   




//}




//moveit_msgs::ObjectColor b1,b2;
//b1.color.g=255;
//b1.color.a=0.9;
//b2.color.r=255;
//b2.color.a=0.9;
//object_colors.push_back(b1);
//object_colors.push_back(b2);



 

  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects,object_colors_);
 // attachObjectToGripper();
}



void tamp_motion::attachObjectToGripper(){

  ROS_INFO("Attaching object to right gripper");

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "right_gripper";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "right_gripper";
  /* The id of the object */
  attached_object.object.id = "objectatt";

  /* A default pose */
  tf2::Quaternion ori;
  geometry_msgs::Quaternion quat_msg;
  ori.setRPY(0,3.14/2,0);
  quat_msg = tf2::toMsg(ori);
  geometry_msgs::Pose pose;
  pose.orientation=quat_msg;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.23;
  primitive.dimensions[1] = 0.02;


  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  attached_object.object.operation = attached_object.object.ADD;
  attached_object.touch_links = std::vector<std::string>{"r_gripper_l_finger_tip", "r_gripper_r_finger_tip","right_gripper_base","right_lower_forearm","r_gripper_l_finger","r_gripper_r_finger" };
  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  scene_plan_pub.publish(planning_scene);


}














void tamp_motion::removeObject(string objid){

  ROS_INFO("Removing object with id: %s",objid.c_str());
  std::vector<std::string> object_ids;
  object_ids.push_back(objid);
  planning_scene_interface.removeCollisionObjects(object_ids);



}



void tamp_motion::stopMotion(const std_msgs::Bool &msg){
  if(msg.data){
      if(dominant_arm_=="right"){
      move_group_right_->stop();
      //cout<<"Stopping Right Arm"<<endl;
      ROS_INFO("Stopping Right Arm");

       }
      else{
        
      move_group_left_->stop();
         // cout<<"Stopping Lelt Arm"<<endl;
          ROS_INFO("Stopping Left Arm");

           }

  }
  

}

bool tamp_motion::sendToSimulation(moveit_msgs::RobotTrajectory &traj, string &arm){
  ROS_INFO("tamp_motion::sendToSimulation");
  trajectory_msgs::JointTrajectory trajmsg = traj.joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jointpointsvector = trajmsg.points;
  std::vector<double> allposes;
  for(size_t i=0;i<jointpointsvector.size();i++){
    for(size_t j=0;j<7;j++){
      allposes.push_back(jointpointsvector[i].positions[j]);
    }

  }

  /*
  tamp_msgs::vrepmotion simsrv;
  simsrv.request.trajectory = allposes;
  simsrv.request.arm = arm;
  ROS_INFO("Sending a request to VREP simulator");
  if(tampVREPMotionClient.call(simsrv)){
    ROS_INFO("VREP simulator Service is awake");
    if(simsrv.response.result){
      return true;

    }
    else{
      return false;
    }

  }
  else{
    ROS_INFO("VREP simulator Service did not respond");
    return false;
  }
*/
  bool finalres;
  baxter_core_msgs::JointCommand cmdmsg;
  std::vector<double> poses;
  if(arm=="right"){

      for(size_t i=0;i<allposes.size()/7;i++){
          poses.clear();
          cmdmsg.command.clear();
          for (size_t j=0;j<7;j++)
          {
           cmdmsg.command.push_back(allposes[i*7+j]); 
           poses.push_back(allposes[i*7+j]);
          }
          vrep_rightarm_pub.publish(cmdmsg);
          double timenow1 = ros::Time::now().toSec();
          finalres = false;
          while (true){
            if(allJointsReached(poses,arm)){
              finalres = true;
              break;
            }
            if(ros::Time::now().toSec()-timenow1>2.0){
              finalres = false;
              break;
            }
          }

      }






  }
  else{

    for(size_t i=0;i<allposes.size()/7;i++){
          poses.clear();
          cmdmsg.command.clear();
          for (size_t j=0;j<7;j++)
          {
           cmdmsg.command.push_back(allposes[i*7+j]); 
           poses.push_back(allposes[i*7+j]);
          }
          vrep_leftarm_pub.publish(cmdmsg);
          double timenow1 = ros::Time::now().toSec();
          finalres = false;
          while (true){
            if(allJointsReached(poses,arm)){
              finalres = true;
              break;
            }
            if(ros::Time::now().toSec()-timenow1>2.0){
              finalres = false;
              break;
            }
          }

      }


  }

  return finalres;

}


bool tamp_motion::allJointsReached(std::vector<double> poses,string arm){
  //ROS_INFO("Checking to verigy whether joints reached in simulation");
    std::vector<double> errors;
    if(arm=="right"){
      for(size_t i =0;i<7;i++){
        double err = poses[i]-lastrightjoints_[i];
        errors.push_back(err);
      }
      
    }
    else{
       for(size_t i =0;i<7;i++){
        double err = poses[i]-lastleftjoints_[i];
        errors.push_back(err);
      }
      
    }

    double threshold = 0.05;
    bool condpass = true;
    for(size_t i =0;i<7;i++){
      if(errors[i]>threshold){
        condpass = false;
      }
    }

return condpass;

}


void tamp_motion::registerJointState(const sensor_msgs::JointState &msg){
  lastrightjoints_.clear();
  lastleftjoints_.clear();
  for(size_t i=0;i<7;i++){
     lastrightjoints_.push_back(msg.position[i]);
     lastleftjoints_.push_back(msg.position[i+7]);
  }
 

}

void tamp_motion::addSimulationCollision(){

  ROS_INFO("Adding simultion obstacles into MoveIt scene");
 if(!colisionobjectsids_.empty()){

    planning_scene_interface.removeCollisionObjects(colisionobjectsids_);
    colisionobjectsids_.clear();
    object_colors_.clear();
  }
  std::vector<moveit_msgs::CollisionObject> collision_objectsvector;
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  




  tamp_msgs::knowledge msg;
  msg.request.reqType = "obstacles";
  std::vector<string> nameofobjs;
    if(tampKnowledgeClient.call(msg)){

      nameofobjs = msg.response.names;

    }
    
    nameofobjs.erase(nameofobjs.begin());
    for(size_t i=0;i<nameofobjs.size();i++){

      std::vector<double> posesandsizes;
      std::vector<string> types;
      tamp_msgs::knowledge msg;
      msg.request.reqType = nameofobjs[i];
      if(tampKnowledgeClient.call(msg)){
        posesandsizes = msg.response.pose;
        types = msg.response.names;
        //types.erase(types.begin());
        if(types[0]=="cube"){
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = move_group_right_->getPlanningFrame();
            collision_object.id = types[1];//the table
            colisionobjectsids_.push_back(collision_object.id);
            collision_object.header.frame_id="world";
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = posesandsizes[3];
            primitive.dimensions[1] = posesandsizes[4];
            primitive.dimensions[2] = posesandsizes[5];

            geometry_msgs::Pose obj_pose;
            obj_pose.orientation.w = 1.0;
            obj_pose.position.x =  posesandsizes[0];
            obj_pose.position.y = posesandsizes[1];
            obj_pose.position.z =  posesandsizes[2];

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(obj_pose);
            collision_object.operation = collision_object.ADD;

            moveit_msgs::ObjectColor bst1;
            bst1.color.g=1.0f;
            bst1.color.b=0.0f;
            bst1.color.r=0.0f;
            bst1.color.a=1.0;
            object_colors_.push_back(bst1);
            collision_objectsvector.push_back(collision_object);
        }
        else if(types[0]=="cylinder"){

            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = move_group_right_->getPlanningFrame();
            collision_object.id = types[1];//the table
            colisionobjectsids_.push_back(collision_object.id);
            collision_object.header.frame_id="world";
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = posesandsizes[5];
            primitive.dimensions[1] = posesandsizes[4]/2;
           
            geometry_msgs::Pose obj_pose;
            obj_pose.orientation.w = 1.0;
            obj_pose.position.x =  posesandsizes[0];
            obj_pose.position.y = posesandsizes[1];
            obj_pose.position.z =  posesandsizes[2];

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(obj_pose);
            collision_object.operation = collision_object.ADD;

            moveit_msgs::ObjectColor bst1;
            bst1.color.g=0.0f;
            bst1.color.b=1.0f;
            bst1.color.r=0.0f;
            bst1.color.a=1.0;
            object_colors_.push_back(bst1);
            collision_objectsvector.push_back(collision_object);


        }

        else{
          ROS_INFO("the object type is not supported :(");
        }
      }

    }

    planning_scene_interface.addCollisionObjects(collision_objectsvector,object_colors_);

}









tamp_motion::~tamp_motion(){

    cout<<"tamp_motion::~tamp_motion()"<<endl;
   cout<<"overall motiom planning time is: "<<timespentforplanning_<<" Seconds"<<endl;

   cout<<"overall motiom execution time is: "<<timespentforexecution_<<" Seconds"<<endl;
   cout<<"There have been overally "<<nuofplanning_<<" number of plannings"<<endl;
   cout<<"There have been overally "<<nuofexecution_<<" number of executions"<<endl;
   

}
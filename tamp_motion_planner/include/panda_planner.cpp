#include "panda_planner.h"


panda_planner::panda_planner(/*planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor*/):optpanda(PLANNING_GROUP, ROBOT_DESCRIPTION)
//planning_scene_monitor_(planning_scene_monitor)
{

    cout<<"panda_planner::panda_planner"<<endl;

    panda1execution = node_handle.advertiseService("panda1_execute",&panda_planner::executePanda1,this);
     panda2execution = node_handle.advertiseService("panda2_execute",&panda_planner::executePanda2,this);
  vrep_ee_client =   node_handle.serviceClient<tamp_msgs::pandaee>("tamp_vrep_pandaconnector");
  knowledgeRegisterClient = node_handle.serviceClient<tamp_msgs::registerdata>("tamp_register_service");
  trajservice1 = node_handle.advertiseService(PANDA1_MOTION_SERVICE_TOPIC, &panda_planner::motionQuery1,this);
  trajservice2 = node_handle.advertiseService(PANDA2_MOTION_SERVICE_TOPIC,&panda_planner::motionQuery2,this);
  franka1jointsub = node_handle.subscribe(PANDA1_JOINT_STATE_TOPIC,5,&panda_planner::callBackfrankajoint1,this);
  franka2jointsub = node_handle.subscribe(PANDA2_JOINT_STATE_TOPIC,5,&panda_planner::callBackfrankajoint2,this);
  vrep_franka1_pub = node_handle.advertise<sensor_msgs::JointState>(PANDA1_JOINT_COMMAND_TOPIC,3);
  vrep_franka2_pub = node_handle.advertise<sensor_msgs::JointState>(PANDA2_JOINT_COMMAND_TOPIC,3);
  collision_object_panda1_client = node_handle.serviceClient<tamp_msgs::objectssrv>("/tamp_vrep_objects_wrtopanda1");
  collision_object_panda2_client = node_handle.serviceClient<tamp_msgs::objectssrv>("/tamp_vrep_objects_wrtopanda2");
  panda1FreePlan = node_handle.advertiseService(PANDA1_FREE_PLAN_TOPIC, &panda_planner::freePlan1,this);
  panda2FreePlan = node_handle.advertiseService(PANDA2_FREE_PLAN_TOPIC, &panda_planner::freePlan2,this);
  object_coloring_client = node_handle.serviceClient<tamp_msgs::objectcolorsrv>("/tamp_vrep_color_objects");

  planning_scene_monitor_ = planning_scene_monitor::PlanningSceneMonitorPtr();

   if( !planning_scene_monitor_ ){
      loadPlanningSceneMonitor();
    }

  planning_scene_monitor_->startStateMonitor(JONIT_STATE_TOPIC);
  optpanda.robot_model_ = planning_scene_monitor_->getRobotModel();
  move_group_panda_.reset(new moveit::planning_interface::MoveGroupInterface(optpanda));
  nuplanning1=0;
  nuplanning2=0;
  planning1time = 0.0;
  planning2time = 0.0;

}


bool panda_planner::executePanda1(tamp_msgs::executetraj::Request &req,tamp_msgs::executetraj::Response &res){
  ros::AsyncSpinner spinner(2);
    spinner.start();
  ROS_INFO("Received tajectory execution for panda1");
  if(sendFranka1ToSimulation(req.trajectory)){
      res.result = true;
  }
  else{
    res.result= false;
  }
}
bool panda_planner::executePanda2(tamp_msgs::executetraj::Request &req,tamp_msgs::executetraj::Response &res){
  ros::AsyncSpinner spinner(2);
    spinner.start();
      ROS_INFO("Received tajectory execution for panda2");
    if(sendFranka2ToSimulation(req.trajectory)){
        res.result = true;
    }
    else{
      res.result= false;
    }
}

bool panda_planner::loadPlanningSceneMonitor(){


  ROS_DEBUG_STREAM_NAMED("panda_move","Loading planning scene monitor");

  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  ros::spinOnce();
  ros::Duration(0.5).sleep(); // todo: reduce this time?
  ros::spinOnce();

  if (!planning_scene_monitor_->getPlanningScene()){
     ROS_ERROR_STREAM_NAMED("panda_move","Planning scene not configured");
     return false;
  }

  return true;
}








bool panda_planner::sendFranka1ToSimulation(moveit_msgs::RobotTrajectory &traj){
  
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
      //ROS_INFO("publishing %d series of joint positions",i+1);
      double timenow1 = ros::Time::now().toSec();
      finalres = false;
      while (true){
        if(allFranka1JointsReached(poses)){
          finalres = true;
          break;
        }
        if(ros::Time::now().toSec()-timenow1>2.0){
          finalres = false;
         // ROS_INFO("time passed more than 2 seconds");
          break;
        }
      }

  }

 
 

  return finalres;

}

bool panda_planner::sendFranka2ToSimulation(moveit_msgs::RobotTrajectory &traj){
  
  //ROS_INFO("panda_planner::sendFranka2ToSimulation");
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

  }

 
 

  return finalres;

}




bool panda_planner::allFranka1JointsReached(std::vector<double> &poses){
    //ROS_INFO("panda_planner::allFranka1JointsReached");
    if(!franka1jointerrors_.empty()){

      franka1jointerrors_.clear();
    }
    for(size_t i =0;i<7;i++){
      double err = poses[i]-franka1joints_[i];
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

bool panda_planner::allFranka2JointsReached(std::vector<double>& poses){
    //ROS_INFO("panda_planner::allFranka2JointsReached");
     if(!franka2jointerrors_.empty()){
      
      franka2jointerrors_.clear();
    }
    
    for(size_t i =0;i<7;i++){
      double err = poses[i]-franka2joints_[i];
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





void panda_planner::callBackfrankajoint1(const sensor_msgs::JointStateConstPtr& msg){
  if(!franka1joints_.empty()){

      franka1joints_.clear();
    }
  

  for(size_t i=0;i<7;i++){
     franka1joints_.push_back(msg->position[i]);
     
  }
 

}


void panda_planner::callBackfrankajoint2(const sensor_msgs::JointStateConstPtr& msg){
  if(!franka2joints_.empty()){

      franka2joints_.clear();
    }

  for(size_t i=0;i<7;i++){
     franka2joints_.push_back(msg->position[i]);
     
  }
 

}


bool panda_planner::motionQuery1(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response){
    
    nuplanning1++;
    double timenow1 = ros::Time::now().toSec();
    ROS_INFO("Received plannig for franka 1");
    cout<<"Requested POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
    cout<<"Requested ORIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
    ros::AsyncSpinner spinner(1);
    spinner.start();
   // move_group_panda_->setEndEffectorLink("panda_leftfinger");
   // ROS_INFO_NAMED("Hint", "Planning frame: %s", move_group_panda_->getPlanningFrame().c_str());
    //ROS_INFO_NAMED("Hint", "End effector link: %s", move_group_panda_->getEndEffectorLink().c_str());

    //tf::Vector3  vec(1,1,0);
    //tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
   // tf::Transform t1(q,vec);
   // tf::Vector3  vec2(2,1,0);
   // tf::Quaternion q2 = tf::createQuaternionFromRPY(0, 0, 0);
   // tf::Transform t2(q2,vec2);

    //tf::Transform ptf = t1*t2;

   // tf::Vector3 pvec = ptf.getOrigin();
    //tf::Quaternion pq = ptf.getRotation();
   // cout<<"product pvev is "<<pvec.x() <<"\t"<<pvec.y()<<"\t"<<pvec.z()<<endl;


    tf::Vector3 de0(request.targetpos.position.x,request.targetpos.position.y,request.targetpos.position.z);
    tf::Quaternion qe0 = tf::createQuaternionFromRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
    tf::Transform TE0(qe0,de0);

    tamp_msgs::pandaee eesrv;
    eesrv.request.robot = 3;
    geometry_msgs::Pose gpos,gposeo;
    if(vrep_ee_client.call(eesrv)){
      gpos = eesrv.response.pose;
    }
    else{
      ROS_INFO("could not connect to vrep to retrive T8E");
    }
     
     eesrv.request.robot = 5;
     if(vrep_ee_client.call(eesrv)){
      gposeo = eesrv.response.pose;
    }
    else{
      ROS_INFO("could not connect to vrep to retrive T8E");
    }

    tf::Vector3 d8e(gpos.position.x,gpos.position.y,gpos.position.z);
    tf::Quaternion q8e = tf::createQuaternionFromRPY(gpos.orientation.x, gpos.orientation.y, gpos.orientation.z);
    tf::Transform T8E(q8e,d8e);



   // tf::Vector3 deo(gposeo.position.x,gposeo.position.y,gposeo.position.z);
   // tf::Quaternion qeo = tf::createQuaternionFromRPY(gposeo.orientation.x, gposeo.orientation.y, gposeo.orientation.z);
    //tf::Transform T8E(q8e,d8e);
   // tf::Matrix3x3 REO(qeo);

    tf::Transform T80 = T8E* TE0;
    //T80 = T80.inverse();

    tf::Vector3 d80 = T80.getOrigin();
    tf::Quaternion r80 = T80.getRotation();
    cout<<"product position is "<<d80.x() <<"\t"<<d80.y()<<"\t"<<d80.z()<<endl;
    cout<<"product orientation is "<<r80.x() <<"\t"<<r80.y()<<"\t"<<r80.z()<<"\t"<<r80.w()<<endl;
    //tf::Vector3 p80 = qe
   // tf::Transform T80I = T80.inverse();
    
    //tf::Vector3 d80i = T80I.getOrigin();
    ///cout<<"product inverse  is "<<d80i.x() <<"\t"<<d80i.y()<<"\t"<<d80i.z()<<endl;

   tf::Matrix3x3 RE8(q8e);
    tf::Matrix3x3 R80(r80);
   //tf::Matrix3x3 qe8 = matrix.inverse();
  // tf::Vector3 d80a = -de0 - RE8 * d8e ; 
    tf::Vector3 dis = de0 + R80 *RE8 * d8e;

    cout<<"affine position is "<<dis.x() <<"\t"<<dis.y()<<"\t"<<dis.z()<<endl;
   
   //cout<<"affine relationship is "<<d80a.x() <<"\t"<<d80a.y()<<"\t"<<d80a.z()<<endl;

    
    if(request.withcollision){
        addCollisionObjects(1);
    }
    else{
      removeColission();
    }
    if(request.objecttoremve!=""){
      removeObjectCollision(request.objecttoremve);
    }
    joint_model_group_ = move_group_panda_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robot_state::RobotState robot_current_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    std::vector<double> joint_group_positions;

    if(request.currentrobot){
          joint_group_positions = franka1joints_;
    }
    else{
        joint_group_positions = request.initaljointvalues;
    }
    
    robot_current_state_.setJointGroupPositions(joint_model_group_, joint_group_positions);
    move_group_panda_->setStartState(robot_current_state_);
    std::vector<double> goal_position = request.targetjointpos;
   // move_group_panda_->setMaxAccelerationScalingFactor(0.4);
    // move_group_panda_->setStartState(*move_group_panda_->getCurrentState());

    if(goal_position.size()>1){
      ROS_INFO("Sending joint positions");
      move_group_panda_->setMaxVelocityScalingFactor(0.8);
      move_group_panda_->setJointValueTarget(goal_position);
    }
    else{
          //move_group_panda_->setPlannerId("RRTConnectkConfigDefault");
          ROS_INFO("Planner id is: %s",move_group_panda_->getPlannerId().c_str());
          ROS_INFO("Sending cartesian positions");
          double postol = request.position_tolerance.data;
          double oritol = request.orientation_tolerance.data;
         
          
          //geometry_msgs::Pose target_pose1;
          //target_pose1.orientation.w = 1.0;
         // target_pose1.position.x = 0.28;
          //target_pose1.position.y = -0.2;
         // target_pose1.position.z = 0.5;
         
         if(request.withendeffector){
          ROS_INFO("Compensating End effector");
            request.targetpos.position.x = dis.x();
            request.targetpos.position.y = dis.y();
            request.targetpos.position.z = dis.z();
            cout<<"POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
           
            request.targetpos.orientation.x = r80.x();
            request.targetpos.orientation.y = r80.y();
            request.targetpos.orientation.z = r80.z();
            request.targetpos.orientation.w = r80.w();
           
           
           // targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
            cout<<"RIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<<request.targetpos.orientation.w<<endl;
         }
         else{
            targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
            request.targetpos.orientation = tf2::toMsg(targetQuaternion_);
            cout<<"POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
            cout<<"RIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
         }
        
          targetPos_ = request.targetpos;
          move_group_panda_->setMaxVelocityScalingFactor(0.7);
          move_group_panda_->clearPoseTargets();

          move_group_panda_->setNumPlanningAttempts(3);
          move_group_panda_->setPlanningTime(1);


          move_group_panda_->setGoalPositionTolerance(postol);
          move_group_panda_->setGoalOrientationTolerance(oritol);
          move_group_panda_->setPoseTarget(targetPos_);

    }

   
    ROS_INFO("planning path");
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    
    bool motion_result = (move_group_panda_->plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    response.success=motion_result;
    if(motion_result){
      response.soltraj=motion_plan.trajectory_;
    }
    if(request.execute && motion_result){
      ROS_INFO("executing the planned path");
        
      response.executedtrajectory = sendFranka1ToSimulation(motion_plan.trajectory_);
    }

    double dt = ros::Time::now().toSec()- timenow1;
    planning1time+= dt;
   
return true;
}


bool panda_planner::motionQuery2(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response){
  nuplanning2++;
  double timenow1 = ros::Time::now().toSec();
  ROS_INFO("Received plannig for franka 2");
  cout<<"Requested POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
    cout<<"Requested ORIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //move_group_panda_->setEndEffectorLink("panda_leftfinger");
    ROS_INFO_NAMED("Hint", "Planning frame: %s", move_group_panda_->getPlanningFrame().c_str());
    ROS_INFO_NAMED("Hint", "End effector link: %s", move_group_panda_->getEndEffectorLink().c_str());



    tf::Vector3 de0(request.targetpos.position.x,request.targetpos.position.y,request.targetpos.position.z);
    tf::Quaternion qe0 = tf::createQuaternionFromRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
    tf::Transform TE0(qe0,de0);

    tamp_msgs::pandaee eesrv;
    eesrv.request.robot = 4;
    geometry_msgs::Pose gpos,gposeo;
    if(vrep_ee_client.call(eesrv)){
      gpos = eesrv.response.pose;
    }
    else{
      ROS_INFO("could not connect to vrep to retrive T8E");
    }
     
     eesrv.request.robot = 6;
     if(vrep_ee_client.call(eesrv)){
      gposeo = eesrv.response.pose;
    }
    else{
      ROS_INFO("could not connect to vrep to retrive T8E");
    }

    tf::Vector3 d8e(gpos.position.x,gpos.position.y,gpos.position.z);
    tf::Quaternion q8e = tf::createQuaternionFromRPY(gpos.orientation.x, gpos.orientation.y, gpos.orientation.z);
    tf::Transform T8E(q8e,d8e);



   // tf::Vector3 deo(gposeo.position.x,gposeo.position.y,gposeo.position.z);
   // tf::Quaternion qeo = tf::createQuaternionFromRPY(gposeo.orientation.x, gposeo.orientation.y, gposeo.orientation.z);
    //tf::Transform T8E(q8e,d8e);
   // tf::Matrix3x3 REO(qeo);

    tf::Transform T80 = T8E* TE0;
    //T80 = T80.inverse();

    tf::Vector3 d80 = T80.getOrigin();
    tf::Quaternion r80 = T80.getRotation();
    cout<<"product position is "<<d80.x() <<"\t"<<d80.y()<<"\t"<<d80.z()<<endl;
    cout<<"product orientation is "<<r80.x() <<"\t"<<r80.y()<<"\t"<<r80.z()<<"\t"<<r80.w()<<endl;
    //tf::Vector3 p80 = qe
   // tf::Transform T80I = T80.inverse();
    
    //tf::Vector3 d80i = T80I.getOrigin();
    ///cout<<"product inverse  is "<<d80i.x() <<"\t"<<d80i.y()<<"\t"<<d80i.z()<<endl;

   tf::Matrix3x3 RE8(q8e);
    tf::Matrix3x3 R80(r80);
   //tf::Matrix3x3 qe8 = matrix.inverse();
  // tf::Vector3 d80a = -de0 - RE8 * d8e ; 
    tf::Vector3 dis = de0 + R80 *RE8 * d8e;

    cout<<"affine position is "<<dis.x() <<"\t"<<dis.y()<<"\t"<<dis.z()<<endl;
   
   //cout<<"affine relationship is "<<d80a.x() <<"\t"<<d80a.y()<<"\t"<<d80a.z()<<endl;

    if(request.withcollision){
        addCollisionObjects(2);
    }
    else{
      removeColission();
    }

    if(request.objecttoremve!=""){
      removeObjectCollision(request.objecttoremve);
    }
    
    joint_model_group_ = move_group_panda_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robot_state::RobotState robot_current_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    std::vector<double> joint_group_positions;

    if(request.currentrobot){
          joint_group_positions = franka2joints_;
    }
    else{
        joint_group_positions = request.initaljointvalues;
    }
    
        robot_current_state_.setJointGroupPositions(joint_model_group_, joint_group_positions);
        move_group_panda_->setStartState(robot_current_state_);
        std::vector<double> goal_position = request.targetjointpos;
       // move_group_panda_->setMaxVelocityScalingFactor(0.8);
       // move_group_panda_->setMaxAccelerationScalingFactor(0.4);

    if(goal_position.size()>1){
      ROS_INFO("Sending joint positions");
      move_group_panda_->setMaxVelocityScalingFactor(0.8);
      move_group_panda_->setJointValueTarget(goal_position);
    }
    else{
              //move_group_panda_->setPlannerId("RRTConnectkConfigDefault");
          //ROS_INFO("Planner id is: %s",move_group_panda_->getPlannerId().c_str());
          ROS_INFO("Sending cartesian positions");
          double postol = request.position_tolerance.data;
          double oritol = request.orientation_tolerance.data;
         
          
          //geometry_msgs::Pose target_pose1;
          //target_pose1.orientation.w = 1.0;
         // target_pose1.position.x = 0.28;
          //target_pose1.position.y = -0.2;
         // target_pose1.position.z = 0.5;
         
         if(request.withendeffector){
          ROS_INFO("Compensating End effector");
            request.targetpos.position.x = dis.x();
            request.targetpos.position.y = dis.y();
            request.targetpos.position.z = dis.z();
            cout<<"POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
           
            request.targetpos.orientation.x = r80.x();
            request.targetpos.orientation.y = r80.y();
            request.targetpos.orientation.z = r80.z();
            request.targetpos.orientation.w = r80.w();
           
           
           // targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
            cout<<"RIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<<request.targetpos.orientation.w<<endl;
         }
         else{
            targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
            request.targetpos.orientation = tf2::toMsg(targetQuaternion_);
            cout<<"POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
            cout<<"RIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
         }
        
         targetPos_ = request.targetpos;
          move_group_panda_->setMaxVelocityScalingFactor(0.7);
           move_group_panda_->clearPoseTargets();

          move_group_panda_->setNumPlanningAttempts(3);
          move_group_panda_->setPlanningTime(1);


        move_group_panda_->setGoalPositionTolerance(postol);
        move_group_panda_->setGoalOrientationTolerance(oritol);
        move_group_panda_->setPoseTarget(targetPos_);
    }
    ROS_INFO("planning path");
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool motion_result = (move_group_panda_->plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    response.success=motion_result;
    if(motion_result){
      response.soltraj=motion_plan.trajectory_;
    }
   
    if(request.execute && motion_result){
       ROS_INFO("executing the planned path");
      response.executedtrajectory = sendFranka2ToSimulation(motion_plan.trajectory_);
    }
    double dt = ros::Time::now().toSec()- timenow1;
   planning2time+= dt;
return true;


  
}

void panda_planner::addCollisionObjects(const int& robot_number){

   ROS_INFO("panda_planner::addCollisionObjects for panda %d", robot_number);
 if(!colisionobjectsids_.empty()){

    planning_scene_interface_.removeCollisionObjects(colisionobjectsids_);
    colisionobjectsids_.clear();
    object_colors_.clear();
  }
  std::vector<moveit_msgs::CollisionObject> collision_objectsvector;


  tamp_msgs::objectssrv msg;
  
  std::vector<string> nameofobjs;
  std::vector<double> dimenofobj,poseofobjects;
  if(robot_number==1){
     if(collision_object_panda1_client.call(msg)){

      nameofobjs = msg.response.names;
      dimenofobj = msg.response.dimension;
      poseofobjects = msg.response.position;

    }
  }
  else{
    if(collision_object_panda2_client.call(msg)){

      nameofobjs = msg.response.names;
      dimenofobj = msg.response.dimension;
      poseofobjects = msg.response.position;

    }
  }
   
   // for(size_t i=0;i<nameofobjs.size();i++){
   //   cout<<"collision object name: "<<nameofobjs[i]<<endl;
   // }

    for(size_t i=0;i<nameofobjs.size();i++){

        if(nameofobjs[i]=="Cube" || nameofobjs[i]=="Cube0" || nameofobjs[i]=="Cube1"){
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = move_group_panda_->getPlanningFrame();
            collision_object.id = nameofobjs[i];//the table
            colisionobjectsids_.push_back(collision_object.id);
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = dimenofobj[i*3];
            primitive.dimensions[1] = dimenofobj[i*3+1];
            primitive.dimensions[2] = dimenofobj[i*3+2];

            geometry_msgs::Pose obj_pose;
            obj_pose.orientation.w = 1.0;
            obj_pose.position.x =  poseofobjects[i*3];
            obj_pose.position.y = poseofobjects[i*3+1];
            obj_pose.position.z =  poseofobjects[i*3+2];

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
        else {

            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = move_group_panda_->getPlanningFrame();
            collision_object.id = nameofobjs[i];
            colisionobjectsids_.push_back(collision_object.id);
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = dimenofobj[i*3+2];
            primitive.dimensions[1] =  dimenofobj[i*3]/2;
           
            geometry_msgs::Pose obj_pose;
            obj_pose.orientation.w = 1.0;
            //cout<<nameofobjs[i]<<" x: "<<poseofobjects[i*3]<<" y: "<<poseofobjects[i*3+1]<<" z: "<<poseofobjects[i*3+2]<<endl;
            obj_pose.position.x =  poseofobjects[i*3];
            obj_pose.position.y = poseofobjects[i*3+1];
            obj_pose.position.z =  poseofobjects[i*3+2]+dimenofobj[i*3]/2;

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

    }

    planning_scene_interface_.addCollisionObjects(collision_objectsvector,object_colors_);

}

void panda_planner::removeColission(){
   ROS_INFO("panda_planner::removeColission");
 if(!colisionobjectsids_.empty()){
    ROS_INFO("removing Objects from scene");
    planning_scene_interface_.removeCollisionObjects(colisionobjectsids_);
    colisionobjectsids_.clear();
    object_colors_.clear();
  }
}

void panda_planner::removeObjectCollision(string &object){
  
  std::vector<string> colobj;
  colobj.push_back(object);
  if(!colisionobjectsids_.empty()){
 
    planning_scene_interface_.removeCollisionObjects(colobj);
    ROS_INFO("Removing object %s from scene",object.c_str());
  }


}


bool panda_planner::freePlan1(tamp_msgs::pandafreeplan::Request &req, tamp_msgs::pandafreeplan::Response & res){
  ROS_INFO("Received a request for planning free for panda 1");
  double timenow1 = ros::Time::now().toSec();
  freeplan1time= 0.0;
  tamp_msgs::objectssrv msg;
  std::vector<string> nameofobjs;
  std::vector<double> dimenofobj,poseofobjects;
  if(collision_object_panda1_client.call(msg)){

      nameofobjs = msg.response.names;
      dimenofobj = msg.response.dimension;
      poseofobjects = msg.response.position;

    }
  else{
    ROS_INFO("Objects server is not present");
    return false;
  }
  int tarindex,tarindex2;
  for(size_t i=0;i<nameofobjs.size();i++){

        if(nameofobjs[i]==msg.response.target){
          tarindex=i;
          cout<<"target position, x: "<<poseofobjects[i*3]<< ",y: "<<poseofobjects[i*3+1]<<" ,z: "<<poseofobjects[i*3+2]<<endl;
        }
        else if(nameofobjs[i]==msg.response.target2){
          tarindex2=i;
          cout<<"target position, x: "<<poseofobjects[i*3]<< ",y: "<<poseofobjects[i*3+1]<<" ,z: "<<poseofobjects[i*3+2]<<endl;
        }

  }
  int susplan = 0;
  std::vector<double> psis;
  for(size_t i=0;i<11;i++){
    tamp_msgs::trajquest::Request request;
    tamp_msgs::trajquest::Response res;
    request.targetpos.position.x = poseofobjects[tarindex*3];
    request.targetpos.position.y = poseofobjects[tarindex*3+1];
    request.targetpos.position.z = poseofobjects[tarindex*3+2];

    request.targetpos.orientation.x=0.0;
    request.targetpos.orientation.y=1.57;
    request.targetpos.orientation.z=-1.5+i*0.3;

    request.position_tolerance.data=0.01;
    request.orientation_tolerance.data=0.05;
    request.execute=false;
    request.simulation=true;
    request.withcollision=false;
    request.currentrobot=true;
     motionQuery1(request,res);
     if(res.success){
        susplan++;
        psis.push_back(-1.5+i*0.3);
        //cout<<"There have been "<<susplan<<" successfull plannings"<<endl;
     }
  }

  cout<<"There have been "<<susplan<<" successfull plannings"<<endl;
  cout<<"feasible approaching psis: "<<"\t";
  for(auto item :psis){
    cout<<item<<" , "<<"\t";
  }
  cout<<endl;

 if(!psis.empty()){
    std::vector<string> triangled_objects = isInsidePolygon(fabs(psis[0]),
  fabs(psis.back()),nameofobjs,poseofobjects,msg.response.target,msg.response.target2);
   tamp_msgs::objectcolorsrv colorsrv;
   colorsrv.request.objects = triangled_objects;
   colorsrv.request.colors={0,1,0};
 
   cout<<(object_coloring_client.call(colorsrv))?"sent service request for coloring objects\n":"Couldnt communicatefor coloring objects\n";



    tamp_msgs::registerdata regsrv;
  std::vector<string> namess;
  namess.push_back("r1t1");
  namess.insert(std::end(namess), std::begin(triangled_objects), std::end(triangled_objects));
  regsrv.request.pose.push_back(triangled_objects.size());
  regsrv.request.names = namess;
  if(knowledgeRegisterClient.call(regsrv)){
    ROS_INFO("Data for r1t1 registered to knowledge base");
  }
  else{
      ROS_INFO("could not register r1t1 to knowledge base");
  }


 }
 else{
  cout<<"bo feasible psi found :(: "<<"\n";

  tamp_msgs::registerdata regsrv;
  std::vector<string> namess;
  namess.push_back("r1t1");
  namess.push_back("no");
  regsrv.request.pose.push_back(0);
  regsrv.request.names = namess;
  if(knowledgeRegisterClient.call(regsrv)){
    ROS_INFO("Data for r1t1 registered to knowledge base");
  }
  else{
      ROS_INFO("could not register r1t1 to knowledge base");
  }
 }

 




   int susplan2 = 0;
  std::vector<double> psis2;
  for(size_t i=0;i<11;i++){
    tamp_msgs::trajquest::Request request2;
    tamp_msgs::trajquest::Response res2;
    request2.targetpos.position.x = poseofobjects[tarindex2*3];
    request2.targetpos.position.y = poseofobjects[tarindex2*3+1];
    request2.targetpos.position.z = poseofobjects[tarindex2*3+2];

    request2.targetpos.orientation.x=0.0;
    request2.targetpos.orientation.y=1.57;
    request2.targetpos.orientation.z=-1.5+i*0.3;

    request2.position_tolerance.data=0.01;
    request2.orientation_tolerance.data=0.05;
    request2.execute=false;
    request2.simulation=true;
    request2.withcollision=false;
    request2.currentrobot=true;
     motionQuery1(request2,res2);
     if(res2.success){
        susplan2++;
        psis2.push_back(-1.5+i*0.3);
        //cout<<"There have been "<<susplan<<" successfull plannings"<<endl;
     }
  }

  cout<<"There have been "<<susplan2<<" successfull plannings"<<endl;
  cout<<"feasible approaching psis: "<<"\t";
  for(auto item :psis2){
    cout<<item<<" , "<<"\t";
  }
  cout<<endl;
 
 if(!psis2.empty()){
    std::vector<string> triangled_objects2 = isInsidePolygon(fabs(psis2[0]),
  fabs(psis2.back()),nameofobjs,poseofobjects,msg.response.target2,msg.response.target);
   tamp_msgs::objectcolorsrv colorsrv2;
   colorsrv2.request.objects = triangled_objects2;
   colorsrv2.request.colors={0,0.5,0};
   cout<<(object_coloring_client.call(colorsrv2))?"sent service request for coloring objects\n":"Couldnt communicatefor coloring objects\n";

     tamp_msgs::registerdata regsrv;
    std::vector<string> namess;
    namess.push_back("r1t2");
    namess.insert(std::end(namess), std::begin(triangled_objects2), std::end(triangled_objects2));
    regsrv.request.pose.push_back(triangled_objects2.size());
    regsrv.request.names = namess;
    if(knowledgeRegisterClient.call(regsrv)){
      ROS_INFO("Data for r1t2 registered to knowledge base");
    }
    else{
        ROS_INFO("could not register r1t2 to knowledge base");
    }
 }

 else{
  cout<<"bo feasible psi found :(: "<<"\n";


   tamp_msgs::registerdata regsrv;
    std::vector<string> namess;
    namess.push_back("r1t2");
     namess.push_back("no");
    regsrv.request.pose.push_back(0);
    regsrv.request.names = namess;
    if(knowledgeRegisterClient.call(regsrv)){
      ROS_INFO("Data for r1t2 registered to knowledge base");
    }
    else{
        ROS_INFO("could not register r1t2 to knowledge base");
    }
 }

 freeplan1time+=ros::Time::now().toSec()-timenow1;
  

}

bool panda_planner::freePlan2(tamp_msgs::pandafreeplan::Request &req, tamp_msgs::pandafreeplan::Response & res){
  ROS_INFO("Received a request for planning free for panda 2");
    double timenow1 = ros::Time::now().toSec();
  freeplan2time= 0.0;
    tamp_msgs::objectssrv msg;
  std::vector<string> nameofobjs;
  std::vector<double> dimenofobj,poseofobjects;
  if(collision_object_panda2_client.call(msg)){

      nameofobjs = msg.response.names;
      dimenofobj = msg.response.dimension;
      poseofobjects = msg.response.position;

    }
  else{
    ROS_INFO("Objects server is not present");
    return false;
  }
  int tarindex,tarindex2;
  for(size_t i=0;i<nameofobjs.size();i++){

        if(nameofobjs[i]==msg.response.target){
          tarindex=i;
          cout<<"target position, x: "<<poseofobjects[i*3]<< ",y: "<<poseofobjects[i*3+1]<<" ,z: "<<poseofobjects[i*3+2]<<endl;
        }
        else if(nameofobjs[i]==msg.response.target2){
          tarindex2=i;
          cout<<"target position, x: "<<poseofobjects[i*3]<< ",y: "<<poseofobjects[i*3+1]<<" ,z: "<<poseofobjects[i*3+2]<<endl;

        }

  }
  int susplan = 0;
  std::vector<double> psis;
  for(size_t i=0;i<11;i++){
    tamp_msgs::trajquest::Request request;
    tamp_msgs::trajquest::Response res;
    request.targetpos.position.x = poseofobjects[tarindex*3];
    request.targetpos.position.y = poseofobjects[tarindex*3+1];
    request.targetpos.position.z = poseofobjects[tarindex*3+2];

    request.targetpos.orientation.x=0.0;
    request.targetpos.orientation.y=1.57;
    //request.targetpos.orientation.z=-1.5+i*0.3;
    request.targetpos.orientation.z=-1.5+i*0.3;

    request.position_tolerance.data=0.01;
    request.orientation_tolerance.data=0.05;
    request.execute=false;
    request.simulation=true;
    request.withcollision=false;
    request.currentrobot=true;
     motionQuery2(request,res);
     if(res.success){
        susplan++;
         psis.push_back(-1.5+i*0.3);
        //cout<<"There have been "<<susplan<<" successfull plannings"<<endl;
     }
  }

  cout<<"There have been "<<susplan<<" successfull plannings"<<endl;
  cout<<"feasible approaching psis: "<<"\t";
  for(auto item :psis){
    cout<<item<<" , "<<"\t";
  }
  cout<<endl;
 if(!psis.empty()){
    std::vector<string> triangled_objects = isInsidePolygon(fabs(psis[0]),
  fabs(psis.back()),nameofobjs,poseofobjects,msg.response.target,msg.response.target2);
    tamp_msgs::objectcolorsrv colorsrv;
   colorsrv.request.objects = triangled_objects;
   colorsrv.request.colors={0,0,1.0};
 
   cout<<(object_coloring_client.call(colorsrv))?"sent service request for coloring objects\n":"Couldnt communicatefor coloring objects\n";

   tamp_msgs::registerdata regsrv;
  std::vector<string> namess;
  namess.push_back("r2t1");
  namess.insert(std::end(namess), std::begin(triangled_objects), std::end(triangled_objects));
  regsrv.request.pose.push_back(triangled_objects.size());
  regsrv.request.names = namess;
  if(knowledgeRegisterClient.call(regsrv)){
    ROS_INFO("Data for r2t1 registered to knowledge base");
  }
  else{
    ROS_INFO("could not register r2t1 to knowledge base");
  }


 }
 else{
  cout<<"bo feasible psi found :(: "<<"\n";
  tamp_msgs::registerdata regsrv;
  std::vector<string> namess;
  namess.push_back("r2t1");
  namess.push_back("no");
  regsrv.request.pose.push_back(0);
  regsrv.request.names = namess;
  if(knowledgeRegisterClient.call(regsrv)){
    ROS_INFO("Data for r2t1 registered to knowledge base");
  }
  else{
    ROS_INFO("could not register r2t1 to knowledge base");
  }

 }




   int susplan2 = 0;
  std::vector<double> psis2;
  for(size_t i=0;i<11;i++){
    tamp_msgs::trajquest::Request request2;
    tamp_msgs::trajquest::Response res2;
    request2.targetpos.position.x = poseofobjects[tarindex2*3];
    request2.targetpos.position.y = poseofobjects[tarindex2*3+1];
    request2.targetpos.position.z = poseofobjects[tarindex2*3+2];

    request2.targetpos.orientation.x=0.0;
    request2.targetpos.orientation.y=1.57;
    request2.targetpos.orientation.z=-1.5+i*0.3;

    request2.position_tolerance.data=0.01;
    request2.orientation_tolerance.data=0.05;
    request2.execute=false;
    request2.simulation=true;
    request2.withcollision=false;
    request2.currentrobot=true;
     motionQuery2(request2,res2);
     if(res2.success){
        susplan2++;
        psis2.push_back(-1.5+i*0.3);
        //cout<<"There have been "<<susplan<<" successfull plannings"<<endl;
     }
  }

  cout<<"There have been "<<susplan2<<" successfull plannings"<<endl;
  cout<<"feasible approaching psis: "<<"\t";
  for(auto item :psis2){
    cout<<item<<" , "<<"\t";
  }
  cout<<endl;
 
 if(!psis2.empty()){
    std::vector<string> triangled_objects2 = isInsidePolygon(fabs(psis2[0]),
  fabs(psis2.back()),nameofobjs,poseofobjects,msg.response.target2,msg.response.target);
   tamp_msgs::objectcolorsrv colorsrv2;
   colorsrv2.request.objects = triangled_objects2;
   colorsrv2.request.colors={0,0,0.7};
 
   cout<<(object_coloring_client.call(colorsrv2))?"sent service request for coloring objects\n":"Couldnt communicatefor coloring objects\n";

   tamp_msgs::registerdata regsrv;
  std::vector<string> namess;
  namess.push_back("r2t2");
  namess.insert(std::end(namess), std::begin(triangled_objects2), std::end(triangled_objects2));
  regsrv.request.pose.push_back(triangled_objects2.size());
  regsrv.request.names = namess;
  if(knowledgeRegisterClient.call(regsrv)){
    ROS_INFO("Data for r2t2 registered to knowledge base");
  }
  else{
    ROS_INFO("could not register r2t2 to knowledge base");
  }
 }
 else{
  cout<<"bo feasible psi found :(: "<<"\n";
   tamp_msgs::registerdata regsrv;
  std::vector<string> namess;
  namess.push_back("r2t2");
  namess.push_back("no");
  regsrv.request.pose.push_back(0);
  regsrv.request.names = namess;
  if(knowledgeRegisterClient.call(regsrv)){
    ROS_INFO("Data for r2t2 registered to knowledge base");
  }
  else{
    ROS_INFO("could not register r2t2 to knowledge base");
  }
 }

 freeplan2time+=ros::Time::now().toSec()-timenow1;




}

std::vector<string> panda_planner::findObjectsInsideTri(const double & alpha, const double & beta,
                                                        const std::vector<string> object_names,
                                                        const std::vector<double> object_poses,
                                                        const string &target,const string &other_target)
{

  point target_p;
  for(size_t i=0;i<object_names.size();i++){
      if(object_names[i]==target){
          target_p.x = object_poses[i*3];
          target_p.y = object_poses[i*3+1];
          break;
      }
  }
  
  const double distance = sqrt(target_p.x*target_p.x+target_p.y*target_p.y);
  const double gamma = PI + atan(target_p.y/target_p.x);
  const double y1 = distance * fabs(sin(alpha)/sin(PI-alpha-gamma));
  const double y2 = - distance * fabs(sin(beta)/sin(gamma-beta));

  point p1,p2;
  p1.x = 0.0;
  p1.y = y1;
  p2.x=0.0;
  p2.y = y2;
  std::vector<string> pesky_triangale;
   for (size_t i = 1; i < object_names.size(); ++i){
      if(object_names[i]!=target && object_names[i]!=other_target){
          point object_p;
          object_p.x=object_poses[i*3];
          object_p.y=object_poses[i*3+1];
          if(pointInTriangle(object_p,target_p,p1,p2)){ 
              pesky_triangale.push_back(object_names[i]);
              cout<<" object in triangle: "<<object_names[i]<<endl;
          }
      }
      
      
   }
  cout<<" Total number of objects in triangle: "<<pesky_triangale.size()<<endl;
 return pesky_triangale;

}

inline double panda_planner::sign(const point &p1, const point &p2, const point &p3){

  return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool panda_planner::pointInTriangle (point pt, point v1, point v2, point v3)
{
    double d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(pt, v1, v2);
    d2 = sign(pt, v2, v3);
    d3 = sign(pt, v3, v1);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

std::vector<string> panda_planner::isInsidePolygon(const double & alpha, const double & beta,
                                                        const std::vector<string> object_names,
                                                        const std::vector<double> object_poses,
                                                        const string &target,const string &other_target)

{

   const double d = 0.4;

  point target_p;
  for(size_t i=0;i<object_names.size();i++){
      if(object_names[i]==target){
          target_p.x = object_poses[i*3];
          target_p.y = object_poses[i*3+1];
          break;
      }
  }
  const double gamma = (fabs(alpha)+fabs(beta))/2;
  const double delta = fabs(beta-gamma);
  const double f = tan(delta) * fabs(target_p.y);
  const double yop = target_p.y - f;
  double vertx[4] = {target_p.x,target_p.x,0,0};
  double verty[4] = {target_p.y-d/2,target_p.y+d/2,yop+d/2,yop-d/2};


   std::vector<string> pesky_polygon;

   for (size_t i = 1; i < object_names.size(); ++i){
      if(object_names[i]!=target && object_names[i]!=other_target){

          if(pnpoly(4,vertx,verty,object_poses[i*3],object_poses[i*3+1])){ 
              pesky_polygon.push_back(object_names[i]);
              cout<<" object in triangle: "<<object_names[i]<<endl;
          }
      }
      
      
   }
  cout<<" Total number of objects in triangle: "<<pesky_polygon.size()<<endl;
 return pesky_polygon;

}

int panda_planner::pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

panda_planner::~panda_planner(){
  cout<<"freeplan1 time: "<< freeplan1time <<" freePlan2 time: "<<freeplan2time<<endl;
  cout<<"#planning1: "<< nuplanning1 <<" #planning1: "<<nuplanning2<<endl;
  cout<<"planning time age1: "<< planning1time <<" planning time age1 "<<planning2time<<endl;

}




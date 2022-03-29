#include "pr2_planner.h"


pr2_planner::pr2_planner(/*planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor*/)
:optpanda(PLANNING_GROUP, ROBOT_DESCRIPTION),optrightarm(PLANNING_GROUP_RIGHT,ROBOT_DESCRIPTION)
,optleftarm(PLANNING_GROUP_LEFT,ROBOT_DESCRIPTION),optwholeleft(PLANNING_GROUP_WL,ROBOT_DESCRIPTION),
optwholeright(PLANNING_GROUP_WR,ROBOT_DESCRIPTION)
//planning_scene_monitor_(planning_scene_monitor)
{

    cout<<"pr2_planner::pr2_planner"<<endl;
  vrep_ee_client =   node_handle.serviceClient<tamp_msgs::pandaee>("tamp_vrep_pr2_ee");

  pr2execution = node_handle.advertiseService("panda1_execute",&pr2_planner::executePr2,this);
  trajservice1 = node_handle.advertiseService(PANDA1_MOTION_SERVICE_TOPIC, &pr2_planner::motionQuery,this);
  armtrajservice = node_handle.advertiseService("tamp_motion_arm_service", &pr2_planner::motionQueryArm,this);
  wholebodytrajservice = node_handle.advertiseService("tamp_motion_whole_body_service", &pr2_planner::motionQueryWholeBody,this);
  pr2jointsub = node_handle.subscribe(PR2_JOINT_STATE_TOPIC,50,&pr2_planner::callBackPr2,this);
  vrep_pr2_pub = node_handle.advertise<sensor_msgs::JointState>(PR2_JOINT_COMMAND_TOPIC,1);
  pr2basevelocitysub = node_handle.subscribe(PR2_BASE_VELOCITY_STATE_TOPIC,50,&pr2_planner::callBackBaseVelPr2,this);
  pr2Wheelvelocitysub = node_handle.subscribe(PR2_WHEEL_VELOCITY_STATE_TOPIC,50,&pr2_planner::callBackWheelVelPr2,this);
  rightarmsub = node_handle.subscribe("/vrep/right_arm/joint_states",10,&pr2_planner::rightArmCb,this);
  leftarmsub = node_handle.subscribe("/vrep/left_arm/joint_states",10,&pr2_planner::leftArmCb,this);
  wholerightsub = node_handle.subscribe("/vrep/whole_right/joint_states",10,&pr2_planner::wholeRightCb,this);
  wholeleftsub = node_handle.subscribe("/vrep/whole_left/joint_states",10,&pr2_planner::wholeLeftCb,this);
  vrep_right_pub = node_handle.advertise<sensor_msgs::JointState>("/vrep/pr2/right_arm",3);
  vrep_left_pub = node_handle.advertise<sensor_msgs::JointState>("/vrep/pr2/left_arm",3);
  collision_object_client = node_handle.serviceClient<tamp_msgs::objectssrv>("tamp_vrep_objects_benchmarks");
  remove_object_scene_server = node_handle.advertiseService("remove_object_scene",&pr2_planner::removeObjectFromScene,this);
  planning_scene_monitor_ = planning_scene_monitor::PlanningSceneMonitorPtr();

   if( !planning_scene_monitor_ ){
      loadPlanningSceneMonitor();
    }

  planning_scene_monitor_->startStateMonitor(JONIT_STATE_TOPIC);
  optpanda.robot_model_ = planning_scene_monitor_->getRobotModel();
  optleftarm.robot_model_ = planning_scene_monitor_->getRobotModel();
  optrightarm.robot_model_ = planning_scene_monitor_->getRobotModel();
  move_group_pr2_.reset(new moveit::planning_interface::MoveGroupInterface(optpanda));
  move_group_right_arm_.reset(new moveit::planning_interface::MoveGroupInterface(optrightarm));
  move_group_left_arm_.reset(new moveit::planning_interface::MoveGroupInterface(optleftarm));
  move_group_whole_left_.reset(new moveit::planning_interface::MoveGroupInterface(optwholeleft));
  move_group_whole_right_.reset(new moveit::planning_interface::MoveGroupInterface(optwholeright));
  nuplanning1=0;
  nuplanning2=0;
  planning1time = 0.0;
  planning2time = 0.0;
  pr2_joints_={0.0, 0.0, 0.0};
 pthread_mutex_init(&pr2_base_controller_lock_, NULL);  
  node_handle.getParam("trajectory_controller", trajectory_controller);
    if (trajectory_controller.empty())
    {
      ROS_FATAL_STREAM_NAMED(
          "test_trajectory",
          "No joint trajectory controller parameter found on the parameter server");
     // exit(-1);
    }
    right_arm_action_client_.reset(new pr2_arm_control_client("pr2/right_position_trajectory_controller/follow_joint_trajectory/", true));
    left_arm_action_client_.reset(new pr2_arm_control_client("pr2/left_position_trajectory_controller/follow_joint_trajectory/", true));
    base_action_client_.reset(new pr2_arm_control_client("pr2/base_position_trajectory_controller/follow_joint_trajectory/", true));
    forbiddenobject_="";
    nuofplrightarm =0;
    nuofplleftarm = 0;
    nuofplbase = 0;
     nuofexeright =0;
    nuofexeleft = 0;
    nuofexebase = 0;
    pltrightarm = 0.0;
    pltimeleftarm = 0.0 ;
    pltbase= 0.0 ;
    exetrightarm = 0.0;
    exetleftarm=0.0;
    exetbase=0.0;
}
pr2_planner::~pr2_planner(){
  cout<<"pr2_planner::~pr2_planner"<<endl;
  cout<<"# planning for right arm: "<<nuofplrightarm<<endl;
  cout<<"# planning for left arm: "<<nuofplleftarm<<endl;
  cout<<"# planning for Base: "<<nuofplbase<<endl;
  cout<<"planning Time for right arm: "<<pltrightarm<<endl;
  cout<<"planning Time for left arm: "<<pltimeleftarm<<endl;
  cout<<"planning Time for Base: "<<pltbase<<endl;
  cout<<"# execution for right arm: "<<nuofexeright<<endl;
  cout<<"# execution for left arm: "<<nuofexeleft<<endl;
  cout<<"# execution for Base: "<<nuofexebase<<endl;
  cout<<"execution Time for right arm: "<<exetrightarm<<endl;
  cout<<"execution Time for left arm: "<<exetleftarm<<endl;
  cout<<"execution Time for Base: "<<exetbase<<endl;
}


void pr2_planner::rightArmCb(const sensor_msgs::JointStatePtr & msg){

  rightarm_joints_.clear();
  rightarm_joints_ = msg->position;




}

void pr2_planner::leftArmCb(const sensor_msgs::JointStatePtr & msg){

  leftarm_joints_.clear();
  leftarm_joints_ = msg->position;
}

void pr2_planner::wholeRightCb(const sensor_msgs::JointStatePtr & msg){

  whole_right_joints_.clear();

  whole_right_joints_ = msg->position;




}

void pr2_planner::wholeLeftCb(const sensor_msgs::JointStatePtr & msg){

  whole_left_joints_.clear();
  whole_left_joints_ = msg->position;
}

bool pr2_planner::loadPlanningSceneMonitor(){

   ROS_DEBUG_STREAM_NAMED("pr2_move","Loading planning scene monitor for pr2 robot");

  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  ros::spinOnce();
  ros::Duration(0.5).sleep(); // todo: reduce this time?
  ros::spinOnce();

  if (!planning_scene_monitor_->getPlanningScene()){
     ROS_ERROR_STREAM_NAMED("pr2_move","Planning scene not configured");
     return false;
  }

  return true;

}
void pr2_planner::callBackPr2(const geometry_msgs::Pose& joint_state){
  pthread_mutex_lock(&pr2_base_controller_lock_);
   if(!pr2_joints_.empty()){

      pr2_joints_.clear();
    }
  

  //cout<<"x: "<<joint_state.position.x<<"y: "<<joint_state.position.y<<"psi: "<<joint_state.orientation.z<<endl;
  pr2_joints_.push_back(joint_state.position.x);
  pr2_joints_.push_back(joint_state.position.y);
  pr2_joints_.push_back(joint_state.orientation.z);
  pthread_mutex_unlock(&pr2_base_controller_lock_);
     
  
}

void pr2_planner::callBackBaseVelPr2(const geometry_msgs::AccelPtr & base_vel){
  pthread_mutex_lock(&pr2_base_controller_lock_);
  if(!pr2_base_velocities_.empty()){
      pr2_base_velocities_.clear();
  }
  pr2_base_velocities_.push_back(base_vel->linear.x);
  pr2_base_velocities_.push_back(base_vel->linear.y);
  pr2_base_velocities_.push_back(base_vel->angular.z);
  pthread_mutex_unlock(&pr2_base_controller_lock_);
}

void pr2_planner::callBackWheelVelPr2(const sensor_msgs::JointStatePtr & wheel_vel){
  pthread_mutex_lock(&pr2_base_controller_lock_);
  if(!pr2_wheel_velocities_.empty()){
      pr2_wheel_velocities_.clear();
  }
   if(!pr2_caster_positions_.empty()){
      pr2_caster_positions_.clear();
  }
  pr2_caster_positions_.push_back(wheel_vel->position[0]);
  pr2_caster_positions_.push_back(wheel_vel->position[1]);
  pr2_caster_positions_.push_back(wheel_vel->position[2]);
  pr2_caster_positions_.push_back(wheel_vel->position[3]);

  pr2_wheel_velocities_.push_back(wheel_vel->velocity[0]);
  pr2_wheel_velocities_.push_back(wheel_vel->velocity[1]);
  pr2_wheel_velocities_.push_back(wheel_vel->velocity[2]);
  pr2_wheel_velocities_.push_back(wheel_vel->velocity[3]);

  pthread_mutex_unlock(&pr2_base_controller_lock_);
}




bool pr2_planner::executePr2(tamp_msgs::executetraj::Request &req,tamp_msgs::executetraj::Response &res){

}

bool pr2_planner::motionQuery(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response){
    cout<<"Base group base frame is : "<<move_group_pr2_->getPlanningFrame()<<endl;
    nuofplbase++;
    double timenow1 = ros::Time::now().toSec();
    ROS_INFO("Received plannig for pr2 robot");
    cout<<"Requested POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
    cout<<"Requested ORIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
    ros::AsyncSpinner spinner(1);
    spinner.start();
   


    
    if(request.withcollision){
       addCollisionObjects();
    }
    else{
       removeColission();
    }
    if(request.objecttoremve!=""){
      //removeObjectCollision(request.objecttoremve);
    }
    
    joint_model_group_whole_right = move_group_whole_right_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_WR);
   joint_model_group_left = move_group_left_arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT);
    robot_state::RobotState robot_current_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    std::vector<double> joint_group_positions_whole_right,joint_group_positions_left;

    if(request.currentrobot){
      ROS_INFO("copying robot base joints to joint group");
          
          
          //joint_group_positions_base = pr2_joints_;
          joint_group_positions_left = leftarm_joints_;
         joint_group_positions_whole_right = whole_right_joints_;
          
          
    }
    else{
        //joint_group_positions_base = request.initaljointvalues;
    }
    
    //ROS_INFO("setting current state to one from vrep for base");
    //robot_current_state_.setJointGroupPositions(joint_model_group_base, joint_group_positions_base);
    ROS_INFO("setting current state to one from vrep for whole right ");
    robot_current_state_.setJointGroupPositions(joint_model_group_whole_right, joint_group_positions_whole_right);
    ROS_INFO("setting current state to one from vrep for left arm");
    robot_current_state_.setJointGroupPositions(joint_model_group_left, joint_group_positions_left);
    ROS_INFO("setting current state to one from vrep");
   // move_group_pr2_->setStartState(robot_current_state_);
      move_group_whole_right_->setStartState(robot_current_state_);
      move_group_left_arm_->setStartState(robot_current_state_);
    std::vector<double> goal_position = request.targetjointpos;


    if(goal_position.size()>1){
      ROS_INFO("Sending joint positions");
      move_group_pr2_->setMaxVelocityScalingFactor(0.8);
      move_group_pr2_->setJointValueTarget(goal_position);
    }
    else{
          ROS_INFO("Planner id is: %s",move_group_pr2_->getPlannerId().c_str());
          ROS_INFO("Sending cartesian positions");
          double postol = request.position_tolerance.data;
          double oritol = request.orientation_tolerance.data;
          targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
          request.targetpos.orientation = tf2::toMsg(targetQuaternion_);
          cout<<"POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
          cout<<"RIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
          targetPos_ = request.targetpos;
          move_group_pr2_->setMaxVelocityScalingFactor(0.7);
          move_group_pr2_->clearPoseTargets();

          move_group_pr2_->setNumPlanningAttempts(5);
          move_group_pr2_->setPlanningTime(1);


          move_group_pr2_->setGoalPositionTolerance(postol);
          move_group_pr2_->setGoalOrientationTolerance(oritol);
          move_group_pr2_->setPoseTarget(targetPos_);

    }

   
    ROS_INFO("planning path");
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    
    bool motion_result = (move_group_pr2_->plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    response.success=motion_result;
    double dt = ros::Time::now().toSec()- timenow1;
    pltbase += dt;
    if(motion_result){
      response.soltraj=motion_plan.trajectory_;
    }
    if(request.execute && motion_result){
      ROS_INFO("executing the planned path");
      nuofexebase++;
     // move_group_pr2_->execute(motion_plan);
      //response.executedtrajectory = sendPr2ToSimulation(motion_plan.trajectory_);
      double timenow2 = ros::Time::now().toSec();
      response.executedtrajectory = sendPr2BaseToSimulationAction(motion_plan.trajectory_);
      double dt = ros::Time::now().toSec()- timenow2;
      exetbase+= dt;
    }

   
   
return true;
}

bool pr2_planner::motionQueryArm(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response){
    double timenow1;
    ROS_INFO("Received plannig for pr2 arm %s",request.arm.c_str());
    if(request.arm=="right"){
           nuofplrightarm++;
           timenow1 = ros::Time::now().toSec();

        }
        else{
          nuofplleftarm++;
          timenow1 = ros::Time::now().toSec();
    }
   // move_group_right_arm_->setEndEffectorLink("r_gripper_motor_screw_link");
    //cout<<"right arm base frame: "<<move_group_right_arm_->getPlanningFrame()<<", right endeffector link: "<<move_group_right_arm_->getEndEffectorLink()<<endl;
    //cout<<"left arm base frame: "<<move_group_left_arm_->getPlanningFrame()<<", left endeffector link: "<<move_group_left_arm_->getEndEffectorLink()<<endl;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    cout<<"distacne E/0 is: "<<request.targetpos.position.x <<"\t"<<request.targetpos.position.y<<"\t"<<request.targetpos.position.z<<endl;
        cout<<"orientation E/0 is RPY: "<<request.targetpos.orientation.x <<"\t"<<request.targetpos.orientation.y<<"\t"<<request.targetpos.orientation.z<<endl;
     double postol = request.position_tolerance.data;
     double oritol = request.orientation_tolerance.data;

    if(request.withcollision){
       addCollisionObjects();
    }
    else{
      //removeColission();
    }
    if(request.objecttoremve!=""){
      //removeObjectCollision(request.objecttoremve);
    }
    
      ROS_INFO("setting joint model group");
      joint_model_group_whole_right = move_group_whole_right_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_WR);
   joint_model_group_left = move_group_left_arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_LEFT);
    robot_state::RobotState robot_current_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    std::vector<double> joint_group_positions_whole_right,joint_group_positions_left;


    if(request.currentrobot){
         
            ROS_INFO("copying joint pos to model group");
            joint_group_positions_whole_right = whole_right_joints_;
          
            joint_group_positions_left = leftarm_joints_;
            
          
          
    }
    else{
       // joint_group_positions_right = request.initaljointvalues;
    }

    std::vector<double> goal_position = request.targetjointpos;
    
    ROS_INFO("setting robot current state");
    //robot_current_state_.setJointGroupPositions(joint_model_group_base, joint_group_positions_base);
    robot_current_state_.setJointGroupPositions(joint_model_group_whole_right, joint_group_positions_whole_right);
    robot_current_state_.setJointGroupPositions(joint_model_group_left, joint_group_positions_left);
    //if(request.arm=="right"){
      ROS_INFO("setting start state of right move group");
      move_group_whole_right_->setStartState(robot_current_state_);
      move_group_left_arm_->setStartState(robot_current_state_);
    //}
   // else{
      
   // }
       cout<<"pos tol: "<<postol<<endl;
    cout<<"orie tol: "<<oritol<<endl;
    cout<<"Requested POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
    cout<<"Requested ORIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
    if(request.withendeffector){
        ROS_INFO("Including end effector in the IK group");
        tf::Vector3 de0(request.targetpos.position.x,request.targetpos.position.y,request.targetpos.position.z);
        tf::Quaternion qe0 = tf::createQuaternionFromRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
        tf::Transform TE0(qe0,de0);
        tamp_msgs::pandaee eesrv;
        if(request.arm=="right"){
           eesrv.request.robot = 1;
        }
        else{
          eesrv.request.robot = 2;
        }
        std::vector<double> matrix;
        geometry_msgs::Pose gpos;
        if(vrep_ee_client.call(eesrv)){
          gpos = eesrv.response.pose;
          matrix = eesrv.response.matrix;
        }
        else{
          ROS_INFO("could not connect to vrep to retrive T8E");
        }
        tf::Vector3 d8e(gpos.position.x,gpos.position.y,gpos.position.z);
       tf::Quaternion q8e = tf::createQuaternionFromRPY(gpos.orientation.x, gpos.orientation.y, gpos.orientation.z);
      

        tf::Vector3 d8ef(matrix[3],matrix[7],matrix[11]);
        tf::Matrix3x3 q8ef(matrix[0],matrix[4],matrix[8],matrix[1],matrix[5],matrix[9],matrix[2],matrix[6],matrix[10]);
         tf::Matrix3x3 R8ef(q8ef);
         tf::Transform T8EF(q8ef,d8ef);
        tf::Transform T8E(q8e,d8e);
        tf::Transform T80 =  T8E * TE0  ;
        tf::Transform T80f =  T8EF.inverse() * TE0  ;
        tf::Vector3 d80ff = T80f.getOrigin();
        tf::Quaternion rr80ff = T80f.getRotation();
        tf::Vector3 d80 = T80.getOrigin();
        tf::Quaternion rr80 = T80.getRotation();
        cout<<"product position is "<<d80.x() <<"\t"<<d80.y()<<"\t"<<d80.z()<<endl;
        cout<<"product orientation is "<<rr80.x() <<"\t"<<rr80.y()<<"\t"<<rr80.z()<<"\t"<<rr80.w()<<endl;
        tf::Matrix3x3 RE8(q8e);
        tf::Matrix3x3 R80(rr80);
        tf::Vector3 dis = de0 + R80 *RE8 * d8e;
        cout<<"affine position is "<<dis.x() <<"\t"<<dis.y()<<"\t"<<dis.z()<<endl;
       // ROS_INFO("Compensating End effector");
        tf::Matrix3x3 RE0(qe0);

       tf::Matrix3x3 Rot80 =  R8ef.inverse() * RE0;
       double ya,pi,ro;
       Rot80.getEulerYPR(ya,pi,ro);
       tf::Quaternion qot80;
       Rot80.getRotation(qot80);
       cout<<"product RPY is  is "<<ro <<"\t"<<pi<<"\t"<<ya<<endl;
         cout<<"product orientation is "<<rr80.x() <<"\t"<<rr80.y()<<"\t"<<rr80.z()<<"\t"<<rr80.w()<<endl;


          cout<<"fff position is "<<d80ff.x() <<"\t"<<d80ff.y()<<"\t"<<d80ff.z()<<endl;
        cout<<"fff orientation is "<<rr80ff.x() <<"\t"<<rr80ff.y()<<"\t"<<rr80ff.z()<<"\t"<<rr80ff.w()<<endl;
        request.targetpos.position.x = dis.x();
        request.targetpos.position.y = dis.y();
        request.targetpos.position.z = dis.z();
        //cout<<"POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
       
        request.targetpos.orientation.x = qot80.x();
        request.targetpos.orientation.y = qot80.y();
        request.targetpos.orientation.z = qot80.z();
        request.targetpos.orientation.w = qot80.w();





        tf::Vector3 de7(gpos.position.x,gpos.position.y,gpos.position.z);

        tf::Quaternion qe7 = tf::createQuaternionFromRPY(gpos.orientation.x, gpos.orientation.y, gpos.orientation.z);
        tf::Transform TE7(qe7,de7);
        tf::Transform T7E = TE7.inverse();

        tf::Transform T70 = TE0 * T7E;
        tf::Vector3 dis70 = T70.getOrigin();
        tf::Quaternion rot70 = T70.getRotation();
        tf::Matrix3x3 m(rot70);
         double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);


       // cout<<"distacne 7/0 is: "<<dis70.x() <<"\t"<<dis70.y()<<"\t"<<dis70.z()<<endl;
        //cout<<"orientation 7/0 is RPY: "<<roll <<"\t"<<pitch<<"\t"<<yaw<<endl;






    }
    else{
        targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
        request.targetpos.orientation = tf2::toMsg(targetQuaternion_);
    }

    
   
    if(request.arm=="right"){
        std::vector<string> v = move_group_right_arm_->getJoints();
        for(auto joint:v){
          cout<<joint<<"\t";
        }


        if(goal_position.size()>1){
            ROS_INFO("Sending joint positions");
            move_group_right_arm_->setMaxVelocityScalingFactor(0.8);
            move_group_right_arm_->setJointValueTarget(goal_position);
        }
        else{
            move_group_right_arm_->setMaxVelocityScalingFactor(0.7);
            move_group_right_arm_->clearPoseTargets();

            move_group_right_arm_->setNumPlanningAttempts(3);
            move_group_right_arm_->setPlanningTime(1);


            move_group_right_arm_->setGoalPositionTolerance(postol);
            move_group_right_arm_->setGoalOrientationTolerance(oritol);
            move_group_right_arm_->setPoseTarget(request.targetpos);
            ROS_INFO("planning path in task space for right arm");
        }
        

        
        moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    
        bool motion_result = (move_group_right_arm_->plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        response.success=motion_result;
        
        double dt = ros::Time::now().toSec()- timenow1;
        pltrightarm+= dt;
       
            
        
        if(motion_result){
          response.soltraj=motion_plan.trajectory_;
        }
        if(request.execute && motion_result){
          ROS_INFO("executing the planned path for the righ arm");
          double timenow2 = ros::Time::now().toSec();

          //moveit::planning_interface::MoveItErrorCode code = move_group_right_arm_->execute(motion_plan);
         // response.executedtrajectory = sendRightArmToSimulation(motion_plan.trajectory_);
          response.executedtrajectory = sendRightArmToSimulationAction(motion_plan.trajectory_);
          nuofexeright++;
          double dt = ros::Time::now().toSec()- timenow2;
          exetrightarm+= dt;
        }
    }
    else{

         if(goal_position.size()>1){
            ROS_INFO("Sending joint positions");
            move_group_left_arm_->setMaxVelocityScalingFactor(0.8);
            move_group_left_arm_->setJointValueTarget(goal_position);
        }
        else{
            move_group_left_arm_->setMaxVelocityScalingFactor(0.7);
            move_group_left_arm_->clearPoseTargets();

            move_group_left_arm_->setNumPlanningAttempts(3);
            move_group_left_arm_->setPlanningTime(1);


            move_group_left_arm_->setGoalPositionTolerance(postol);
            move_group_left_arm_->setGoalOrientationTolerance(oritol);
            move_group_left_arm_->setPoseTarget(request.targetpos);

        ROS_INFO("planning path in task space for left arm");

        }
       
        moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

        bool motion_result = (move_group_left_arm_->plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        response.success=motion_result;
        double dt = ros::Time::now().toSec()- timenow1;
        pltimeleftarm+= dt;
        if(motion_result){
            response.soltraj=motion_plan.trajectory_;
        }
        if(request.execute && motion_result){
            ROS_INFO("executing the planned path for the left arm");

          //response.executedtrajectory = sendLefArmToSimulation(motion_plan.trajectory_);
          double timenow2 = ros::Time::now().toSec();
          response.executedtrajectory = sendLeftArmToSimulationAction(motion_plan.trajectory_);
          nuofexeleft++;
          double dt = ros::Time::now().toSec()- timenow2;
          exetleftarm+= dt;
        }
    }
    
return true;
}

bool pr2_planner::sendRightArmToSimulationAction(moveit_msgs::RobotTrajectory &traj){
    right_arm_action_client_->waitForServer();  // will wait for infinite time

    ROS_INFO_NAMED("test_trajetory", "Action server started, sending goal.");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj.joint_trajectory;
    //std::cout << "Trajectry:\n" << goal.trajectory << std::endl;
    right_arm_action_client_->sendGoal(goal);

    // Wait for the action to return
    double wait_extra_padding = 2;  // time to wait longer than trajectory itself
    bool finished_before_timeout = right_arm_action_client_->waitForResult(
        ros::Duration(goal.trajectory.points.back().time_from_start.toSec() + wait_extra_padding));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = right_arm_action_client_->getState();
      ROS_INFO_NAMED("test_trajetory", "Action finished: %s", state.toString().c_str());
    }
    else
      ROS_INFO_NAMED("test_trajetory", "Action did not finish before the time out.");

   // ROS_INFO_STREAM_NAMED("test_trajectory", "TestTrajectory Finished");
  return finished_before_timeout;
}

bool pr2_planner::sendLeftArmToSimulationAction(moveit_msgs::RobotTrajectory &traj){
    left_arm_action_client_->waitForServer();  // will wait for infinite time

    ROS_INFO_NAMED("test_trajetory", "Action server started, sending goal to left arm.");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj.joint_trajectory;
    //std::cout << "Trajectry:\n" << goal.trajectory << std::endl;
    left_arm_action_client_->sendGoal(goal);

    // Wait for the action to return
    double wait_extra_padding = 2;  // time to wait longer than trajectory itself
    bool finished_before_timeout = left_arm_action_client_->waitForResult(
        ros::Duration(goal.trajectory.points.back().time_from_start.toSec() + wait_extra_padding));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = left_arm_action_client_->getState();
      ROS_INFO_NAMED("test_trajetory", "Action finished for left arm: %s", state.toString().c_str());
    }
    else
      ROS_INFO_NAMED("test_trajetory", "Action did not finish before the time out for left arm.");

   // ROS_INFO_STREAM_NAMED("test_trajectory", "TestTrajectory Finished for left arm");
  return finished_before_timeout;
}

bool pr2_planner::motionQueryWholeBody(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response){

    ROS_INFO("Received plannig for whole pr2  %s",request.arm.c_str());
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

     double postol = request.position_tolerance.data;
          double oritol = request.orientation_tolerance.data;

    if(request.withcollision){
       addCollisionObjects();
    }
    else{
      removeColission();
    }
    if(request.objecttoremve!=""){
      //removeObjectCollision(request.objecttoremve);
    }
    if(request.arm=="right"){
      ROS_INFO("setting joint model group");
      joint_model_group_ = move_group_whole_right_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_WR);
    }
    else{
      joint_model_group_ = move_group_whole_left_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_WL);

    }
    
    robot_state::RobotState robot_current_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    std::vector<double> joint_group_positions;

    if(request.currentrobot){
          if(request.arm=="right"){
            ROS_INFO("copying joint pos to model group");
            joint_group_positions = whole_right_joints_;
            robot_current_state_.setJointGroupPositions(joint_model_group_, joint_group_positions);
            move_group_whole_right_->setStartState(robot_current_state_);
            
             
          }
          else{
            joint_group_positions = whole_left_joints_;
            robot_current_state_.setJointGroupPositions(joint_model_group_, joint_group_positions);
            move_group_whole_left_->setStartState(robot_current_state_);
             
          }
          
    }
    else{
      if(request.arm=="right"){
          move_group_whole_right_->setStartState(robot_current_state_);
      }
      else{
          move_group_whole_left_->setStartState(robot_current_state_);
      }

    }
   
    
    
   
    

    targetQuaternion_.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
    request.targetpos.orientation = tf2::toMsg(targetQuaternion_);
    cout<<"pos tol: "<<postol<<endl;
    cout<<"orie tol: "<<oritol<<endl;
    cout<<"Requested POSE: "<<request.targetpos.position.x <<": "<<request.targetpos.position.y <<": "<<request.targetpos.position.z<<": "<<endl;
    cout<<"Requested ORIEN: "<<request.targetpos.orientation.x <<": "<<request.targetpos.orientation.y <<": "<<request.targetpos.orientation.z<<": "<< request.targetpos.orientation.w<<endl;
    if(request.arm=="right"){
        std::vector<string> v = move_group_whole_right_->getJoints();
        for(auto joint:v){
          cout<<joint<<"\t";
        }
        move_group_whole_right_->setMaxVelocityScalingFactor(0.7);
        move_group_whole_right_->clearPoseTargets();

        move_group_whole_right_->setNumPlanningAttempts(3);
        move_group_whole_right_->setPlanningTime(1);


        move_group_whole_right_->setGoalPositionTolerance(postol);
        move_group_whole_right_->setGoalOrientationTolerance(oritol);
        move_group_whole_right_->setPoseTarget(request.targetpos);

        ROS_INFO("planning path");
        moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    
        bool motion_result = (move_group_whole_right_->plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        response.success=motion_result;
        if(motion_result){
          response.soltraj=motion_plan.trajectory_;
        }
        if(request.execute && motion_result){
          ROS_INFO("executing the planned path");
          //moveit::planning_interface::MoveItErrorCode code = move_group_right_arm_->execute(motion_plan);
          //response.executedtrajectory = sendRightArmToSimulation(motion_plan.trajectory_);
         // response.executedtrajectory = sendWholeRightToAction(motion_plan.trajectory_);
          response.executedtrajectory = sendRightArmToSimulationAction(motion_plan.trajectory_);
        }
    }
    else{
        move_group_whole_left_->setMaxVelocityScalingFactor(0.7);
        move_group_whole_left_->clearPoseTargets();

        move_group_whole_left_->setNumPlanningAttempts(3);
        move_group_whole_left_->setPlanningTime(1);


        move_group_whole_left_->setGoalPositionTolerance(postol);
        move_group_whole_left_->setGoalOrientationTolerance(oritol);
        move_group_whole_left_->setPoseTarget(request.targetpos);

        ROS_INFO("planning path");
        moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

        bool motion_result = (move_group_whole_left_->plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        response.success=motion_result;
        if(motion_result){
            response.soltraj=motion_plan.trajectory_;
        }
        if(request.execute && motion_result){
            ROS_INFO("executing the planned path");

          response.executedtrajectory = sendLefArmToSimulation(motion_plan.trajectory_);
        }
    }
    

}

bool pr2_planner::sendWholeRightToAction(moveit_msgs::RobotTrajectory &traj){
    moveit_msgs::RobotTrajectory basetraj,armtrajectory;
    basetraj = traj;
    armtrajectory = traj;
    //modify base joints
    ros::AsyncSpinner spinner(2);
    spinner.start();
    basetraj.joint_trajectory.joint_names.clear();
    for (int i = 0; i < 3; ++i)
    {
      basetraj.joint_trajectory.joint_names.push_back(traj.joint_trajectory.joint_names[i]);
    }
    
    for (int i = 0; i < traj.joint_trajectory.points.size(); ++i)
    {   
        basetraj.joint_trajectory.points[i].positions.clear();
        basetraj.joint_trajectory.points[i].velocities.clear();
        basetraj.joint_trajectory.points[i].accelerations.clear();
        for (int j = 0; j < 3; ++j){
          basetraj.joint_trajectory.points[i].positions.push_back(traj.joint_trajectory.points[i].positions[j]);
          basetraj.joint_trajectory.points[i].velocities.push_back(traj.joint_trajectory.points[i].velocities[j]);
          basetraj.joint_trajectory.points[i].accelerations.push_back(traj.joint_trajectory.points[i].accelerations[j]);
        }
    }

    ////modify arm joints
    armtrajectory.joint_trajectory.joint_names.clear();
    for (int i = 3; i < 11; ++i)
    {
      armtrajectory.joint_trajectory.joint_names.push_back(traj.joint_trajectory.joint_names[i]);
    }
    
    for (int i = 0; i < traj.joint_trajectory.points.size(); ++i)
    {   
        armtrajectory.joint_trajectory.points[i].positions.clear();
        armtrajectory.joint_trajectory.points[i].velocities.clear();
        armtrajectory.joint_trajectory.points[i].accelerations.clear();
        for (int j = 3; j < 11; ++j){
          armtrajectory.joint_trajectory.points[i].positions.push_back(traj.joint_trajectory.points[i].positions[j]);
          armtrajectory.joint_trajectory.points[i].velocities.push_back(traj.joint_trajectory.points[i].velocities[j]);
          armtrajectory.joint_trajectory.points[i].accelerations.push_back(traj.joint_trajectory.points[i].accelerations[j]);
        }
    }

    

return  sendPr2BaseToSimulationAction(basetraj) && sendRightArmToSimulationAction(armtrajectory);
}

bool pr2_planner::sendRightArmToSimulation(moveit_msgs::RobotTrajectory &traj){

  ROS_INFO("pr2_planner::sendRightArmToSimulation");
  trajectory_msgs::JointTrajectory trajmsg = traj.joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jointpointsvector = trajmsg.points;
  std::vector<double> allposes;
  for(size_t i=0;i<jointpointsvector.size();i++){
    for(size_t j=0;j<8;j++){
      allposes.push_back(jointpointsvector[i].positions[j]);
    }

  }
 
  bool finalres;
  
  std::vector<double> poses;
      

  for(size_t i=0;i<allposes.size()/8;i++){
      poses.clear();
      cmdmsgright_.position.clear();
      for (size_t j=0;j<8;j++)
      {
       cmdmsgright_.position.push_back(allposes[i*8+j]); 
       poses.push_back(allposes[i*8+j]);
      }

      vrep_right_pub.publish(cmdmsgright_);
      //ROS_INFO("publishing %d series of joint positions",i+1);
      double timenow1 = ros::Time::now().toSec();
      finalres = false;
      while (true){
        if(allRightArmJointsReached(poses)){
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
bool pr2_planner::allRightArmJointsReached(std::vector<double>& poses){
   // ROS_INFO("pr2_planner::allRightArmJointsReached");
     if(!rightarm_errors_.empty()){
      
      rightarm_errors_.clear();
    }
    
    for(size_t i =0;i<8;i++){
      double err = poses[i]-rightarm_joints_[i];
      rightarm_errors_.push_back(err);
    }
      
   

    double threshold = 0.1;
    double thr2 = 0.01;
    bool condpass = true;
    for(size_t i =1;i<8;i++){
      if(fabs(rightarm_errors_[i])>threshold){
        condpass = false;
      }
    }
    if(fabs(rightarm_errors_[0])>thr2){
        condpass = false;
      }

return condpass;

}

bool pr2_planner::sendLefArmToSimulation(moveit_msgs::RobotTrajectory &traj){
   ROS_INFO("pr2_planner::sendLefArmToSimulation");
  trajectory_msgs::JointTrajectory trajmsg = traj.joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jointpointsvector = trajmsg.points;
  std::vector<double> allposes;
  for(size_t i=0;i<jointpointsvector.size();i++){
    for(size_t j=0;j<8;j++){
      allposes.push_back(jointpointsvector[i].positions[j]);
    }

  }
 
  bool finalres;
  
  std::vector<double> poses;
      

  for(size_t i=0;i<allposes.size()/8;i++){
      poses.clear();
      cmdmsgleft_.position.clear();
      for (size_t j=0;j<8;j++)
      {
       cmdmsgleft_.position.push_back(allposes[i*8+j]); 
       poses.push_back(allposes[i*8+j]);
      }

      vrep_left_pub.publish(cmdmsgleft_);
      //ROS_INFO("publishing %d series of joint positions",i+1);
      double timenow1 = ros::Time::now().toSec();
      finalres = false;
      while (true){
        if(allLeftArmJointsReached(poses)){
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

bool pr2_planner::allLeftArmJointsReached(std::vector<double>& poses){
   // ROS_INFO("pr2_planner::allRightArmJointsReached");
     if(!leftarm_errors_.empty()){
      
      leftarm_errors_.clear();
    }
    
    for(size_t i =0;i<8;i++){
      double err = poses[i]-leftarm_joints_[i];
      leftarm_errors_.push_back(err);
    }
      
   

    double threshold = 0.1;
    double thr2 =0.01;
    bool condpass = true;
    for(size_t i =1;i<8;i++){
      if(fabs(leftarm_errors_[i])>threshold){
        condpass = false;
      }
    }

     if(fabs(leftarm_errors_[0])>thr2){
        condpass = false;
      }
return condpass;

}
bool pr2_planner::sendPr2BaseToSimulationAction(moveit_msgs::RobotTrajectory &traj){
      base_action_client_->waitForServer();  // will wait for infinite time

    ROS_INFO_NAMED("test_trajetory", "Action server started, sending goal to Base.");

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj.joint_trajectory;
    //std::cout << "Trajectry:\n" << goal.trajectory << std::endl;
    base_action_client_->sendGoal(goal);

    // Wait for the action to return
    double wait_extra_padding = 2;  // time to wait longer than trajectory itself
    bool finished_before_timeout = base_action_client_->waitForResult(
        ros::Duration(goal.trajectory.points.back().time_from_start.toSec() + wait_extra_padding));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = base_action_client_->getState();
      ROS_INFO_NAMED("test_trajetory", "Action finished for the base: %s", state.toString().c_str());
    }
    else
      ROS_INFO_NAMED("test_trajetory", "Action did not finish before the time out for the base.");

  return finished_before_timeout;
}
bool pr2_planner::sendPr2ToSimulation(moveit_msgs::RobotTrajectory &traj){
  ROS_INFO("pr2_planner::sendPr2ToSimulation");
  trajectory_msgs::JointTrajectory trajmsg = traj.joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> jointpointsvector = trajmsg.points;
  std::vector<double> allposes,allvel;
  std::vector<ros::Duration> durations;
  for(size_t i=0;i<jointpointsvector.size();i++){
    for(int j=0;j<3;j++){
      allposes.push_back(jointpointsvector[i].positions[j]);
      allvel.push_back(jointpointsvector[i].velocities[j]);
      
    }
    durations.push_back(jointpointsvector[i].time_from_start);
  }

  double  d = 0.224;
  std::vector<double> xo{d,d,-d,-d};
  std::vector<double> yo{d,-d,d,-d};



 double k1=2.0;
 double k2 = 2.0;
 double k3 = 2.0;

std::vector<double> theta_command;
std::vector<double> wheel_command;

 for(int j=2;j<jointpointsvector.size()+1;j++){
    double error = 0.2;
    
    cout<<"************************** "<<j<<endl;
    std::vector<double> v_desired,p_desired,v_command,v_pub;
      v_desired = {allvel[3*j-3],allvel[3*j-2],allvel[3*j-1]};
      p_desired = {allposes[3*j-3],allposes[3*j-2],allposes[3*j-1]};
      //p_desired[0] = p_desired[0] * cos(pr2_joints_[2]) + p_desired[1] * sin(pr2_joints_[2]);
     // p_desired[1] = -p_desired[0] * sin(pr2_joints_[2]) + p_desired[1] * cos(pr2_joints_[2]);

     // cout<<"p_desired"<<p_desired[0]<<" ,"<<p_desired[1]<<" ,"<<p_desired[2]<<endl;
    sensor_msgs::JointState jointmsg;
     PID pid_vx(0.05,10.3,-10.3,5.3,0.0,0.0);
     PID pid_vy(0.05,10.3,-10.3,5.3,0.0,0.0);
     PID pid_w(0.05,10.3,-10.3,5.3,0.0,0.0);
    theta_command.clear();
    v_pub.resize(3);
     
    
     while(error>0.03){

          std::this_thread::sleep_for(50ms);
          theta_command.clear();
          wheel_command.clear();
          double time1,time2,dt;
          v_command.clear();
          v_pub.clear();

         // p_desired[0] = (p_desired[0]) * cos(pr2_joints_[2]) + (p_desired[1]) * sin(pr2_joints_[2]);
          //p_desired[1] = -(p_desired[0] ) * sin(pr2_joints_[2]) + (p_desired[1] ) * cos(pr2_joints_[2]);
          //pthread_mutex_lock(&pr2_base_controller_lock_);
         //pr2_joints_[0] = (pr2_joints_[0]) * cos(pr2_joints_[2]) + (pr2_joints_[1]) * sin(pr2_joints_[2]);
        // pr2_joints_[1] = -(pr2_joints_[0] ) * sin(pr2_joints_[2]) + (pr2_joints_[1] ) * cos(pr2_joints_[2]);
         //pthread_mutex_unlock(&pr2_base_controller_lock_);
         // cout<<"dx: "<<p_desired[0] -pr2_joints_[0] <<" dy: "<<p_desired[1] -pr2_joints_[1] <<" dth: " <<p_desired[2]- pr2_joints_[2]<<endl;
         // v_command.push_back(pid_vx.calculate(p_desired[0],pr2_joints_[0]));
          //v_command.push_back(pid_vy.calculate(p_desired[1],pr2_joints_[1]));
          //v_command.push_back(pid_w.calculate(p_desired[2],pr2_joints_[2]));
          v_command.push_back(5 * (p_desired[0]-pr2_joints_[0]));
          v_command.push_back(5 * (p_desired[1]-pr2_joints_[1]));
          v_command.push_back(5 * (p_desired[2]-pr2_joints_[2]));
          //cout<<"V_PIDDDD"<<v_command[0]<<" ,"<<v_command[1]<<" ,"<<v_command[2]<<endl;
          double vax = v_command[0] * cos(pr2_joints_[2]) + v_command[1] * sin(pr2_joints_[2]);
          double vay = -1.0 *  v_command[0] * sin(pr2_joints_[2]) + v_command[1] * cos(pr2_joints_[2]);
          double vaz = v_command[2];
          v_pub.push_back(vax);
          v_pub.push_back(vay);
          v_pub.push_back(vaz);

          //cout<<"p_desired"<<p_desired[0]<<" ,"<<p_desired[1]<<" ,"<<p_desired[2]<<endl;
         // cout<<"p_actuall"<<pr2_joints_[0]<<" ,"<<pr2_joints_[1]<<" ,"<<pr2_joints_[2]<<endl;
          //cout<<"V_command"<<v_command[0]<<" ,"<<v_command[1]<<" ,"<<v_command[2]<<endl;
          //cout<<"V_PUBBBBB"<<v_pub[0]<<" ,"<<v_pub[1]<<" ,"<<v_pub[2]<<endl;
        //  for (int i = 0; i < 3; ++i)
         // {
         ///     if(fabs(v_command[i])<0.01){
          //       v_command[i]= 0.0;
         ///     }
               
         // }


          
        



       

            jointmsg.velocity.clear();
            jointmsg.velocity=v_pub;
           
            vrep_pr2_pub.publish(jointmsg);
             
             
            double ex = p_desired[0]-pr2_joints_[0] ;
            double ey = p_desired[1]-pr2_joints_[1];
            double epsi = p_desired[2]-pr2_joints_[2];
            error = sqrt(ex * ex + ey * ey + epsi * epsi);
   
     
      }
      


  }
  cout<<"publishing zero velocity"<<endl;
  sensor_msgs::JointState jointmsg;
   for (int i = 0; i < 3; ++i){

      jointmsg.velocity.push_back(0.0);
      
    }
    vrep_pr2_pub.publish(jointmsg);
  
 



}


void pr2_planner::publishBaseStates(const sensor_msgs::JointState &jointmsg){

  sensor_msgs::JointState castermsg;
  std::vector<double> v;
  //jointmsg.velocity.clear();
  // for (int i = 0; i < 4; ++i){
//
  //    castermsg.velocity.push_back(0.0);
   //   castermsg.velocity.push_back(jointmsg.velocity[i+1]);
  //    v.push_back(jointmsg.velocity[i+1]);
  //  }
//vrep_pr2_pub.publish(castermsg);
  // / 
    /*
    while(true){

      if(checkIfCastersReached(v)){
        break;
      }

    }
    */
    //std::this_thread::sleep_for(500ms);
   // ros::Duration(0.4).sleep();
    vrep_pr2_pub.publish(jointmsg);



}


bool pr2_planner::checkIfCastersReached(const std::vector<double> v){

  double e1,e2,e3,e4;

  e1 = fabs(v[0] - pr2_caster_positions_[0]);
  e2 = fabs(v[1] - pr2_caster_positions_[1]);
  e3 = fabs(v[2] - pr2_caster_positions_[2]);
  e4 = fabs(v[3] - pr2_caster_positions_[3]);

  double error = 0.05;
  if(e1<=error && e1<=error && e1<=error && e1<=error){
    return true;
  }

  return false;

}

void pr2_planner::addCollisionObjects(){

  cout<<"pr2_planner::addCollisionObjects"<<endl;
  /*
    std::vector<moveit_msgs::CollisionObject> collision_objectsvector;
    std::vector<moveit_msgs::ObjectColor> object_colors;

    moveit_msgs::CollisionObject collision_object,collision_object2;
    collision_object.id = "Cube1";//the table
    collision_object.header.frame_id="base_footprint";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 0.5;

    geometry_msgs::Pose obj_pose;
    obj_pose.orientation.w = 1.0;
    obj_pose.position.x =  2.0;
    obj_pose.position.y = 0.0;
    obj_pose.position.z =  0.37;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(obj_pose);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::ObjectColor bst1;
    bst1.color.g=1.0f;
    bst1.color.b=0.0f;
    bst1.color.r=0.0f;
    bst1.color.a=1.0;
    object_colors.push_back(bst1);
    collision_objectsvector.push_back(collision_object);


    collision_object2.id = "Cube2";//the table
    collision_object2.header.frame_id="base_footprint";
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 1.0;
    primitive2.dimensions[1] = 1.0;
    primitive2.dimensions[2] = 0.5;

    geometry_msgs::Pose obj_pose2;
    obj_pose2.orientation.w = 1.0;
    obj_pose2.position.x =  1.0;
    obj_pose2.position.y = 1.1;
    obj_pose2.position.z =  0.37;

    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(obj_pose2);
    collision_object2.operation = collision_object2.ADD;

    moveit_msgs::ObjectColor bst2;
    bst2.color.g=1.0f;
    bst2.color.b=1.0f;
    bst2.color.r=0.0f;
    bst2.color.a=1.0;
    object_colors.push_back(bst2);
    collision_objectsvector.push_back(collision_object2);

    planning_scene_interface.addCollisionObjects(collision_objectsvector,object_colors);

*/




    tamp_msgs::objectssrv objsrv;
    collision_object_client.call(objsrv);
    std::vector<moveit_msgs::ObjectColor> object_colors;
     std::vector<moveit_msgs::CollisionObject> collision_objectsvector;
    for (int i = 0; i < objsrv.response.names.size(); ++i)
    {
      /*
        if(objsrv.response.types[i]=="cylinder"){

        }

        else if(objsrv.response.types[i]=="cylinder2"){

        }
        else if(objsrv.response.types[i]=="cube"){
          
        }

        else if(objsrv.response.types[i]=="cube2"){

        }
        */
       if(objsrv.response.names[i]!=forbiddenobject_){

            moveit_msgs::CollisionObject collision_object;
          collision_object.id = objsrv.response.names[i];//the table
          collision_object.header.frame_id="base_footprint";
          shape_msgs::SolidPrimitive primitive;
          geometry_msgs::Pose obj_pose;
          if(objsrv.response.types[i]=="cube"){
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = objsrv.response.dimension[3*i];
            primitive.dimensions[1] = objsrv.response.dimension[3*i+1];
            primitive.dimensions[2] = objsrv.response.dimension[3*i+2];

              obj_pose.orientation.x = objsrv.response.orientation[4*i];
          obj_pose.orientation.y = objsrv.response.orientation[4*i+1];
          obj_pose.orientation.z = objsrv.response.orientation[4*i+2];
          obj_pose.orientation.w = objsrv.response.orientation[4*i+3];
            obj_pose.position.x =  objsrv.response.position[3*i];
            obj_pose.position.y = objsrv.response.position[3*i+1];
            obj_pose.position.z =  objsrv.response.position[3*i+2];
          }
          else if(objsrv.response.types[i]=="cylinder"){
              primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = objsrv.response.dimension[3*i+2];
            primitive.dimensions[1] = objsrv.response.dimension[3*i]/2;

            obj_pose.orientation.w = 1;
            obj_pose.position.x =  objsrv.response.position[3*i];
            obj_pose.position.y = objsrv.response.position[3*i+1];
            obj_pose.position.z =  objsrv.response.position[3*i+2];//+objsrv.response.dimension[3*i+2]/2;
          }
          else if(objsrv.response.types[i]=="cylinder2"){
              primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = objsrv.response.dimension[3*i+2];
            primitive.dimensions[1] = objsrv.response.dimension[3*i]/2;


            obj_pose.orientation.w = 1;
            obj_pose.position.x =  objsrv.response.position[3*i];
            obj_pose.position.y = objsrv.response.position[3*i+1];
            obj_pose.position.z =  objsrv.response.position[3*i+2];//+objsrv.response.dimension[3*i+2]/2;


          }
          else{
             primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = objsrv.response.dimension[3*i];
            primitive.dimensions[1] = objsrv.response.dimension[3*i+1];
            primitive.dimensions[2] = objsrv.response.dimension[3*i+2];
          }


          

          
          //obj_pose.orientation.x = objsrv.response.orientation[4*i];
          //obj_pose.orientation.y = objsrv.response.orientation[4*i+1];
          //obj_pose.orientation.z = objsrv.response.orientation[4*i+2];
         // obj_pose.orientation.w = objsrv.response.orientation[4*i+3];
         

          collision_object.primitives.push_back(primitive);
          collision_object.primitive_poses.push_back(obj_pose);
          collision_object.operation = collision_object.ADD;

          moveit_msgs::ObjectColor bst1;
          bst1.color.g=objsrv.response.colors[3*i+1];
          bst1.color.b=objsrv.response.colors[3*i+2];
          bst1.color.r=objsrv.response.colors[3*i];
          bst1.color.a=1.0;
          object_colors.push_back(bst1);
          collision_objectsvector.push_back(collision_object);

       }
        
    }

    //here we add arena to the obstacles vector,we decompose arena geometry to 4 cubes for simlicity
    for (int i = 0; i < 4; ++i)
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = "arena"+to_string(i);
        collision_object.header.frame_id="base_footprint";
        shape_msgs::SolidPrimitive primitive;
        geometry_msgs::Pose obj_pose;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 5.5;
        primitive.dimensions[1] = 0.03;
        primitive.dimensions[2] = 0.5;
         tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0, i * 1.57);
        obj_pose.orientation.x =  q.x();
        obj_pose.orientation.y =  q.y();
        obj_pose.orientation.z =  q.z();
        obj_pose.orientation.w =  q.w();
       
        obj_pose.position.x = 2.75 * sin(i*1.57) ;
        obj_pose.position.y = -2.75 * sin((i+1)*1.57);
        obj_pose.position.z =  0.25;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(obj_pose);
        collision_object.operation = collision_object.ADD;

        moveit_msgs::ObjectColor bst1;
        bst1.color.g=0.6;
        bst1.color.b=0.6;
        bst1.color.r=0.98;
        bst1.color.a=1.0;
        object_colors.push_back(bst1);
        collision_objectsvector.push_back(collision_object);
    }
    
    planning_scene_interface.addCollisionObjects(collision_objectsvector,object_colors);
}

void pr2_planner::removeColission(){
  std::vector<std::string> object_ids;
  object_ids.push_back("Cube1");
  object_ids.push_back("Cube2");
  planning_scene_interface.removeCollisionObjects(object_ids);
}

bool pr2_planner::removeObjectFromScene(tamp_msgs::removeobject::Request &req,tamp_msgs::removeobject::Response &res){
  cout<<"pr2_planner::removeObjectFromScene"<<endl;
 
  string objid = req.object;
  cout<<"Removing " <<objid<<" from scene"<<endl;
  std::vector<std::string> object_ids;
  
  forbiddenobject_ = objid;
  object_ids.push_back(forbiddenobject_);
  planning_scene_interface.removeCollisionObjects(object_ids);

  res.result = true;

  return true;

}
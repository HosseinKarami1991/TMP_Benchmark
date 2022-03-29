/*********************************************************************
 Author: Hossein Karami
 this script is been inspired by moveit_tutorial package
 *********************************************************************/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
 #include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include<tamp_msgs/trajquest.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/utils.h>
using namespace std;
//void openGripper(trajectory_msgs::JointTrajectory& posture);
//void closedGripper(trajectory_msgs::JointTrajectory& posture);
geometry_msgs::Pose targetrec;
double postol;
double oritol;
//obj_msgs::trajquest::Request& targetrec;
bool newpos;
bool sustraj;
bool computationdone_=false;
moveit_msgs::RobotTrajectory  mytraj;
void printpos(geometry_msgs::PoseStamped pos){

 std::cout<<"**********this is pos of end effector**********"<<std::endl;
 std::cout<<"x: "<<pos.pose.position.x<<std::endl;
 std::cout<<"y: "<<pos.pose.position.y<<std::endl;
 std::cout<<"z: "<<pos.pose.position.z<<std::endl;

}

void printrot(std::vector<double> v){

 std::cout<<"**********this is rotation of end effector**********"<<std::endl;
 std::cout<<"roll: "<<v.at(0)<<std::endl;
 std::cout<<"pitch: "<<v.at(1)<<std::endl;
 std::cout<<"yaw: "<<v.at(2)<<std::endl;

}

void robstcb(sensor_msgs::JointState js){





}




bool trsrcb(tamp_msgs::trajquest::Request& request, tamp_msgs::trajquest::Response& response)
 {
   //targetrec.orientation.w=request.targetpos.orientation.w;
   //targetrec.orientation.x=request.targetpos.orientation.x;
   //targetrec.orientation.y=request.targetpos.orientation.y;
   //targetrec.orientation.z=request.orientation.z;
   //targetrec.position.x=request.position.x; 
   //targetrec.position.y=request.position.y;
   //targetrec.position.z=request.position.z;
  newpos = true;
  

  tf2::Quaternion q;
  q.setRPY(request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z);
  request.targetpos.orientation = tf2::toMsg(q);
  postol =  request.position_tolerance.data;
  oritol = request.orientation_tolerance.data;

  //tf2::Quaternion myQuaternion;
 // myQuaternion.setRPY( request.targetpos.orientation.x, request.targetpos.orientation.y, request.targetpos.orientation.z );
  //ROS_INFO_STREAM(*myQuaternion);
 // ROS_INFO_STREAM(*(myQuaternion+1));
  //ROS_INFO_STREAM(*(myQuaternion+2));
 // ROS_INFO_STREAM(*(myQuaternion+3));
  //computationdone_ = false;

   targetrec=request.targetpos;
  std::cout<<"traj_service_vrep is called *** y is  "<<targetrec.position.y<<std::endl;
  //ros::Duration(2).sleep();
   
  
  // while(computationdone_==false){
//    cout<<"computationdone_ : "<<computationdone_<<endl;

 //  }

//********************************


      













//***************************88
  response.soltraj = mytraj;
  
  return true;

    
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
   //ros::Subscriber = node_handle.Subscribe("/sim_ros_interface/vrep/baxter/joint_states",10,robstcb);
  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  ros::ServiceServer trajservice = node_handle.advertiseService("traj_service_vrep", trsrcb);
  ros::Publisher trajpub = node_handle.advertise<moveit_msgs::RobotTrajectory>("trajectoryofrobot", 100);
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "right_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
   std::cout<<move_group.getEndEffectorLink()<<"this was ee link"<<std::endl;
  // Visualization
  // ^^^^^^^^^^^^^
  geometry_msgs::PoseStamped dd = move_group.getCurrentPose(move_group.getEndEffectorLink());
  std::vector<double> vb = move_group.getCurrentRPY(move_group.getEndEffectorLink());
  printpos(dd);
  printrot(vb);
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("torso");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
 // text_pose.translation().z() = 1.75;
  //visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());











/*

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  move_group.setGoalPositionTolerance(0.5);
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1;
  target_pose1.position.x = 0.7;
  target_pose1.position.y = 0.07;
  target_pose1.position.z = 0.47;
  move_group.setPoseTarget(targetrec);
ROS_INFO("***********x is [%f]", targetrec.position.y);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  moveit_msgs::RobotTrajectory  mytraj  = my_plan.trajectory_;
  trajpub.publish(mytraj);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    dd = move_group.getCurrentPose(move_group.getEndEffectorLink());
   vb = move_group.getCurrentRPY(move_group.getEndEffectorLink());
  printpos(dd);
  printrot(vb);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();


  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  moveit_msgs::RobotTrajectory  mytraj2  = my_plan.trajectory_;
  trajpub.publish(mytraj2);
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
 dd = move_group.getCurrentPose(move_group.getEndEffectorLink());
   vb = move_group.getCurrentRPY(move_group.getEndEffectorLink());
  printpos(dd);
  printrot(vb);

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "right_hand";
  ocm.header.frame_id = "torso";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);


  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  move_group.setPoseTarget(target_pose1);


  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");
  dd = move_group.getCurrentPose(move_group.getEndEffectorLink());
   vb = move_group.getCurrentRPY(move_group.getEndEffectorLink());
  printpos(dd);
  printrot(vb);
  move_group.clearPathConstraints();

  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose3);

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
 dd = move_group.getCurrentPose(move_group.getEndEffectorLink());
   vb = move_group.getCurrentRPY(move_group.getEndEffectorLink());
  printpos(dd);
  printrot(vb);

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;




  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object2.id = "CYLINDER";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive2.CYLINDER;
  primitive2.dimensions.resize(2);
  primitive2.dimensions[0] = 1.5;
  primitive2.dimensions[1] = 0.1;
 

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose cyl_pose;
  cyl_pose.orientation.w = 1.0;
  cyl_pose.position.x = 0.8;
  cyl_pose.position.y = -0.15;
  cyl_pose.position.z = 0.1;

  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(cyl_pose);
  collision_object2.operation = collision_object2.ADD;





















  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object2);
  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = 0;
  another_pose.position.z = 0.2;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                      "robot");

  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
                      "robot");

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
  trajectory_msgs::JointTrajectory poss;
  openGripper(poss);
  closedGripper(poss);
  // END_TUTORIAL

  ros::shutdown();


*/











while(ros::ok()){
  if(newpos){
        move_group.setGoalPositionTolerance(postol);
        move_group.setGoalOrientationTolerance(oritol);
        move_group.setPoseTarget(targetrec);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::cout<<"tolerance is  "<<move_group.getGoalPositionTolerance()<<std::endl;
        sustraj = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        cout<<"*********THe success*********"<<sustraj<<endl;
        
        move_group.execute(my_plan);
        mytraj  = my_plan.trajectory_;
        computationdone_ = true;
        trajpub.publish(mytraj);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
        //visual_tools.publishAxisLabeled(targetrec, "pose1");
       // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        //visual_tools.trigger();
       // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
        dd = move_group.getCurrentPose(move_group.getEndEffectorLink());
        vb = move_group.getCurrentRPY(move_group.getEndEffectorLink());
        printpos(dd);
        printrot(vb);
        
        dd = move_group.getCurrentPose("base");
        vb = move_group.getCurrentRPY("base");
        printpos(dd);
        printrot(vb);

       // move_group.move(my_plan)
  }
sustraj = false;
newpos=false;
ros::spinOnce();

}








  return 0;
}


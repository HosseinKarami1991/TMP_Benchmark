cmake_minimum_required(VERSION 2.8.3)
project(tamp_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
   geometry_msgs
   moveit_msgs
)


 add_message_files(
 
   DIRECTORY msg
FILES
   tampSimulationReq.msg
   tampSimulationRes.msg
   tampJoints.msg
   baxterControlCommand.msg
   change_planner.msg
 )


add_service_files(
   DIRECTORY srv
   FILES
       obj.srv
       trajquest.srv
       sceneobjects.srv
       knowledge.srv
       ackquest.srv
       knowledgetamp.srv
	registerdata.srv
       lastgraph.srv	
	resetsrv.srv
	objectstatus.srv
	registerplace.srv
	vrepmotion.srv
	objectssrv.srv
	vrepgripper.srv
	targetsrv.srv
	removeobject.srv
	pandafreeplan.srv
	objectcolorsrv.srv
	changeplansrv.srv
	updatescene.srv
	pandaee.srv
	executetraj.srv
	hanoipp.srv

 )


 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   moveit_msgs
 )


catkin_package(

  CATKIN_DEPENDS roscpp std_msgs 
  message_generation
  message_runtime
 geometry_msgs
   moveit_msgs

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



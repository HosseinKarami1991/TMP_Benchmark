cmake_minimum_required(VERSION 2.8.3)
project(tamp_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  moveit_commander
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rosmsg
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  controller_manager
  tamp_msgs

   
  moveit_ros_planning 
   
  
  #moveit_grasps
  baxter_core_msgs
  std_msgs
  geometry_msgs
  #baxter_control
  
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   tf2_geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tamp_motion_class
  CATKIN_DEPENDS geometric_shapes moveit_commander moveit_core moveit_ros_perception moveit_ros_planning_interface moveit_visual_tools roscpp rosmsg tf2_geometry_msgs tf2_ros sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  INCLUDE_DIRS
  include/
CATKIN_DEPENDS 
  LIBRARIES 
    baxter_move_group_interface
  
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(tamp_motion_class include/tamp_motion.cpp)
add_dependencies(tamp_motion_class moveit ${tamp_msgs_EXPORTED_TARGETS})

add_library(pid_class include/my_pid.cpp)


add_library(panda_planner_class include/panda_planner.cpp)
add_dependencies(panda_planner_class moveit ${tamp_msgs_EXPORTED_TARGETS})

add_library(pr2_planner_class include/pr2_planner.cpp)
add_dependencies(pr2_planner_class moveit ${tamp_msgs_EXPORTED_TARGETS})
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/tamp_motion_planner_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############
add_executable(tamp_motion_baxter src/tamp_motion_baxter.cpp)
target_link_libraries(tamp_motion_baxter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS tamp_motion_baxter DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(tamp_motion_class ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(tamp_motion_main src/tamp_motion_main.cpp)
target_link_libraries(tamp_motion_main 
		${catkin_LIBRARIES} 
		${Boost_LIBRARIES} 
		tamp_motion_class
               )
install(TARGETS tamp_motion_main DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})





target_link_libraries(panda_planner_class ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(panda_motion_main src/tamp_motion_panda.cpp)
target_link_libraries(panda_motion_main 
		${catkin_LIBRARIES} 
		${Boost_LIBRARIES} 
		panda_planner_class
               )
install(TARGETS panda_motion_main DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




target_link_libraries(pr2_planner_class ${catkin_LIBRARIES} ${Boost_LIBRARIES} pid_class)
add_executable(pr2_motion_main src/tamp_motion_pr2.cpp)
target_link_libraries(pr2_motion_main 
		${catkin_LIBRARIES} 
		${Boost_LIBRARIES} 
		pr2_planner_class
               )
install(TARGETS pr2_motion_main DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_executable(subpub src/subpub.cpp)
target_link_libraries(subpub ${catkin_LIBRARIES} ${Boost_LIBRARIES})
## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
add_library(baxter_move_group_interface src/tools/baxter_move_group_interface.cpp)
target_link_libraries(baxter_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Demos ------------------------------
#add_executable(send_random src/send_random.cpp)
#target_link_libraries(send_random baxter_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Tools ------------------------------
add_executable(get_joint_values src/tools/get_joint_values.cpp)
target_link_libraries(get_joint_values ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(end_effector_position src/tools/end_effector_position.cpp)
target_link_libraries(end_effector_position ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 



add_executable(send_ready src/send_ready.cpp)
target_link_libraries(send_ready ${catkin_LIBRARIES} ${Boost_LIBRARIES} baxter_move_group_interface)

#add_library(pr2_hw include/pr_hw_interface.cpp)
#target_link_libraries(pr2_hw ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_executable(pr2_hw_main src/pr2_hw_main.cpp)
#target_link_libraries(pr2_hw_main ${catkin_LIBRARIES} ${Boost_LIBRARIES} pr2_hw)

#add_executable(panda1 src/tamp_motion_panda1.cpp)
#add_dependencies(panda1 moveit ${tamp_msgs_EXPORTED_TARGETS})
#target_link_libraries(panda1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Test -------------------------------
#add_executable(gripper_open_close src/tests/gripper_open_close.cpp)
#target_link_libraries(gripper_open_close ${catkin_LIBRARIES} ${Boost_LIBRARIES})
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tamp_motion_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

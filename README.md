# TMP_Benchmark
# Tamp_Benchmark

## Introduction

This repository is composed of all the essential components required for implementing task and motion planning benchmark problems. The essential components are as follows:

1. andor_graph : represents task representatin using AND/OR graphs.
2. andor_planner: solves task planning problem and communicates with tamp_interface component.
3. tamp_interface: is responsible to translate high level task planning commmands to low-level motion planning problem.
4. tamp_knowledge: All the neccessary data is stored and modified during runtime in this component and is responsible for providing data to all the queries.
5. tamp_motin_planner: plans and executes motions for robot arms and base. It deponds on Moveit.
6. tamp_msgs: All the ros messages and srvs used to communicate between all the components.

Beside essential components, some sub components as listed below:(attention: these sub-components are mainly developed by other developers and we only modified them according to our application).

1. ros_control_boilerpale: modified to control pr2 robot trajectory execution.
2. pr2_common : modified the base mobile part.
3. pr2tamp_moveit_config: a moveit package for our pr2.

All the benchmarks are implemented in simuation environment, Coppeliasim (ex V-Rep) robot simulator with version 4.0.0 , and all the components are ros based(tested with kinetic version of ROS).
## Usage
To run the benchmarks you need to clone whole this repository into your catkin workspace and make them by catkin_make command. Once you secsessfully compiled them, open coppelliasim and open scenes from there. 

1. Run Coppeliasim scene. e.g,. hanoi.ttt 
2. Run moveit package made for pr2:
    In Terminal 1:
   * `roslaunch pr2tamp_moveit_config demo.launch`
3. Run joint positin and trajectory controllers:
    In Terminal 2:
   * `roslaunch ros_control_boilerplate pr2_hardware.launch`
4. Run tamp motion planner:
    In Terminal 3:
   * `rosrun tamp_motion_planner pr2_motion_main`
5. Run corresponding launch file of the tamp knowledge problem,e.g, tamp_knowledge_hanoi
     In Terminal 4:
  * `roslaunch tamp_knowledge tamp_knowledge_hanoi.launch`
6. Run corresponding launch file of the tamp interface problem,e.g, tamp_interface_hanoi
     In Terminal 5:
  * `roslaunch tamp_interface tamp_interface_hanoi.launch`
7. Run andor_graph
     In Terminal 6:
  * `rosrun andor_graph andor`
 8. Run andor_planner 
     In Terminal 7:
  * `rosrun andor_planner seq_planner`


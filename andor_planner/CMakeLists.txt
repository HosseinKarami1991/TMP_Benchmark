cmake_minimum_required(VERSION 2.8.3)
project(andor_planner)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  baxter_core_msgs
  message_generation
  std_msgs
  andor_msgs
	robot_interface_msgs

)
include_directories(
	INCLUDE_DIRS
	include/
)


catkin_package(  CATKIN_DEPENDS baxter_core_msgs message_generation roscpp rospy std_msgs geometry_msgs  andor_msgs robot_interface_msgs)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
ADD_LIBRARY(seq_planner_class_youbot include/seq_planner_class_youbot.cpp)
ADD_LIBRARY(seq_planner_class include/seq_planner_class.cpp)
ADD_LIBRARY(basic_classes include/basic_classes.cpp)


add_executable(seq_planner src/seq_planner_main.cpp )

add_dependencies(seq_planner baxter_core_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp andor_msgs_generate_messages_cpp robot_interface_msgs_generate_messages_cpp)

add_dependencies(seq_planner_class baxter_core_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp andor_msgs_generate_messages_cpp robot_interface_msgs_generate_messages_cpp)

target_link_libraries(seq_planner
   ${catkin_LIBRARIES}
seq_planner_class
basic_classes

 )

#add_executable(seq_planner_youbot src/seq_planner_youbot.cpp )

#add_dependencies(seq_planner_youbot baxter_core_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp andor_msgs_generate_messages_cpp robot_interface_msgs_generate_messages_cpp)

add_dependencies(seq_planner_class_youbot baxter_core_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp andor_msgs_generate_messages_cpp robot_interface_msgs_generate_messages_cpp)


#target_link_libraries(seq_planner_youbot
#   ${catkin_LIBRARIES}
#seq_planner_class_youbot
#basic_classes

 #)

add_executable(multi_agent_planner src/multi_planner.cpp )
target_link_libraries(multi_agent_planner seq_planner_class basic_classes ${catkin_LIBRARIES})


add_executable(panda1_planner src/panda1_planner.cpp)
target_link_libraries(panda1_planner seq_planner_class basic_classes ${catkin_LIBRARIES})

add_executable(panda2_planner src/panda2_planner.cpp)
target_link_libraries(panda2_planner seq_planner_class basic_classes ${catkin_LIBRARIES})


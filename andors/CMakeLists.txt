cmake_minimum_required(VERSION 2.8.3)
project(andor_graph)
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
  #baxter_core_msgs
  message_generation
  std_msgs
  andor_msgs

)
include_directories(
	INCLUDE_DIRS
	include/
)



catkin_package(  CATKIN_DEPENDS message_generation roscpp rospy std_msgs geometry_msgs  andor_msgs)#baxter_core_msgs 


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_library(andor_aonode include/andor_aonode.cpp)
add_library(andor_aograph include/andor_aograph.cpp)



add_executable(andor src/andor_main.cpp )
add_executable(andor_youbot src/andor_youbot.cpp )
add_dependencies(andor_aograph andor_msgs_generate_messages_cpp)
add_dependencies(andor_youbot geometry_msgs_generate_messages_cpp andor_msgs_generate_messages_cpp)
add_dependencies(andor geometry_msgs_generate_messages_cpp andor_msgs_generate_messages_cpp) #baxter_core_msgs_generate_messages_cpp  

target_link_libraries(andor
   ${catkin_LIBRARIES}
andor_aonode
andor_aograph
 )
target_link_libraries(andor_youbot
   ${catkin_LIBRARIES}
andor_aonode
andor_aograph
 )
target_link_libraries(andor_aograph andor_aonode)


add_executable(multi_agent src/multiple_andor.cpp)
target_link_libraries(multi_agent  ${catkin_LIBRARIES} andor_aonode andor_aograph)



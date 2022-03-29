cmake_minimum_required(VERSION 2.8.3)
project(tamp_knowledge)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  roscpp
  std_msgs
  tamp_msgs
)
catkin_package(
  INCLUDE_DIRS include
 LIBRARIES tamp_knowledge
  CATKIN_DEPENDS geometry_msgs moveit_msgs roscpp std_msgs tamp_msgs

)

include_directories(

  ${catkin_INCLUDE_DIRS}
  include/
)
add_library(tamp_knowledge include/tamp_knowledge.cpp)
add_dependencies(tamp_knowledge  ${catkin_LIBRARIES})
add_executable(tamp_knowledge_main src/tamp_knowledge_main.cpp)
target_link_libraries(tamp_knowledge_main ${catkin_LIBRARIES} ${Boost_LIBRARIES} tamp_knowledge)


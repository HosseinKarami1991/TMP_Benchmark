cmake_minimum_required(VERSION 2.8.3)
project(andor_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs )

add_message_files(
  DIRECTORY msg
  FILES
  Node.msg
  Hyperarc.msg)

add_service_files(
  DIRECTORY srv
  FILES
  andorSRV.srv)

generate_messages(DEPENDENCIES geometry_msgs std_msgs )

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs  )

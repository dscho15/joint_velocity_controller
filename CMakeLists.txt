cmake_minimum_required(VERSION 3.7.2)

project(joint_velocity_controller)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Catkin-related packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp urdf 
    kdl_parser 
    std_msgs
)

# KDL
find_package(orocos_kdl REQUIRED)

catkin_package(
  DEPENDS 
     orocos_kdl
  CATKIN_DEPENDS 
    urdf 
    kdl_parser
    std_msgs
)

###########
## Build ##
###########
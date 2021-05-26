cmake_minimum_required(VERSION 3.7.2)

project(joint_velocity_controller)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Catkin-related packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    urdf 
    kdl_parser
    dynamics_wrapper
)

catkin_package(
  CATKIN_DEPENDS 
    urdf 
    kdl_parser
    std_msgs
    dynamics_wrapper
)

###########
## Build ##
###########
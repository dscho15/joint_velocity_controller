cmake_minimum_required(VERSION 3.7.2)

project(joint_velocity_controller)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp urdf kdl_parser std_msgs)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS 
     orocos_kdl
  CATKIN_DEPENDS 
    urdf 
    kdl_parser
    std_msgs
  INCLUDE_DIRS 
    include
  LIBRARIES
)

add_executable(${PROJECT_NAME}_node src/joint_velocity_controller.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

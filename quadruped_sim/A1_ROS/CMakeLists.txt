cmake_minimum_required(VERSION 3.0.2)
project(quadruped_ctrl)


set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  genmsg
  joint_state_controller
  robot_state_publisher
  gazebo_ros
  std_msgs
  tf
  geometry_msgs
  sensor_msgs
  a1_description
  unitree_legged_msgs
  unitree_controller
  unitree_legged_real
)
find_package(gazebo REQUIRED)

add_message_files(
  FILES
  locomotion_status.msg
)

add_service_files(
  FILES
  QuadrupedCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs sensor_msgs unitree_legged_msgs
  #DEPENDS system_lib
)

message(STATUS ${PROJECT_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}
  src
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

include_directories("/usr/local/include/lcm/")
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/robot/include)
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/robot/include/rt)
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/common/include/)
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/common/FootstepPlanner)
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/lcm-types/cpp)
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/user/MIT_Controller)
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/sim/include)
include_directories($ENV{CHEETAH_SOFTWARE_PATH}/third-party/qpOASES/include)
link_directories($ENV{CHEETAH_SOFTWARE_PATH}/build/robot/librobot.so)
link_directories($ENV{CHEETAH_SOFTWARE_PATH}/build/common/libbiomimetics.so)
link_directories($ENV{CHEETAH_SOFTWARE_PATH}/build/user/MIT_Controller/libuser_controller.so)
include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include
        $ENV{ALIENGO_SDK_PATH}/include)
link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
link_directories($ENV{ALIENGO_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} libaliengo_comm.so)


link_directories(${GAZEBO_LIBRARY_DIRS})

#Set already exists at the top of this file. Currently set to "Release"
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/body.cpp
  src/Gazebo_interface.cpp
  src/convert.h
)


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        "$ENV{CHEETAH_SOFTWARE_PATH}/build/user/MIT_Controller/libuser_controller.so"
        "$ENV{CHEETAH_SOFTWARE_PATH}/build/robot/librobot.so"
        "$ENV{CHEETAH_SOFTWARE_PATH}/build/common/libbiomimetics.so"
        lcm

    )


add_executable(a1_servo src/main.cpp)
target_link_libraries(a1_servo ${PROJECT_NAME} ${catkin_LIBRARIES}
)
add_dependencies(a1_servo ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(locomotion_status_publisher src/locomotion_status_publisher.cpp)
target_link_libraries(locomotion_status_publisher ${PROJECT_NAME} ${catkin_LIBRARIES}
)
add_dependencies(locomotion_status_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

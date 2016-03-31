cmake_minimum_required(VERSION 2.8.3)
project(vrep_plugin_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  camera_info_manager
  sensor_msgs
  message_generation
  shape_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

# Service messages
add_service_files(
  FILES
  AddShape.srv
  SetShapeColour.srv
  AddForce.srv
  AddForceTorque.srv
  IsSceneStatic.srv
  ResetScene.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  shape_msgs
  std_msgs
)


# Use c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


catkin_package(
               CATKIN_DEPENDS roscpp rosconsole std_msgs roslib geometry_msgs shape_msgs std_msgs vrep_common
               DEPENDS roscpp rosconsole std_msgs roslib image_transport sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/hydro/lib")

add_library(v_repExtROSradServer src/vrep_plugin_loop.cpp
                                 src/v_repLib.cpp
                                 src/ros_server.cpp
)

add_dependencies(v_repExtROSradServer
          ${catkin_EXPORTED_TARGETS}
          ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(v_repExtROSradServer
    pthread
    dl
    roslib
    rosconsole
    rostime
    rospack
    roscpp_serialization
    roscpp
    boost_system
   ${catkin_LIBRARIES}
)

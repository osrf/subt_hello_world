cmake_minimum_required(VERSION 2.8.3)
project(subt_solution_launch)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  geometry_msgs
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  subt_communication_broker
  subt_ign
  subt_msgs
  tf2_ros
)

find_package(Boost REQUIRED)

set(IGN_MATH_VER 6)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)
set(IGN_MSGS_VER 4)
find_package(ignition-msgs${IGN_MSGS_VER} REQUIRED)

catkin_package()

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(artifact_origin_finder
  src/find_artifact_origin.cpp
)

add_executable(artifact_reporter
  src/artifact_reporter.cpp
)

target_link_libraries(artifact_origin_finder
  ${catkin_LIBRARIES}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
)

target_link_libraries(artifact_reporter
  ${catkin_LIBRARIES}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)

install(TARGETS artifact_origin_finder artifact_reporter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS scripts/simple_solution.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

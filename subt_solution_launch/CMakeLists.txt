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
  tf2_geometry_msgs
  tf2_sensor_msgs
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

add_executable(base_link_flattener
  src/base_link_costmap_projector.cpp
)

add_executable(point_cloud_filter
  src/pc_filter.cpp
)

add_executable(artifact_reporter_comms
  src/artifact_reporter_comms.cpp
)

add_executable(report_relay
  src/report_relay.cpp
)

target_link_libraries(artifact_origin_finder
  ${catkin_LIBRARIES}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
)

target_link_libraries(artifact_reporter
  ${catkin_LIBRARIES}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)

target_link_libraries(base_link_flattener
  ${catkin_LIBRARIES}
)

target_link_libraries(point_cloud_filter
  ${catkin_LIBRARIES}
)

target_link_libraries(artifact_reporter_comms
  ${catkin_LIBRARIES}
  # ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)

target_link_libraries(report_relay
  ${catkin_LIBRARIES}
  # ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)

install(TARGETS
  artifact_origin_finder
  artifact_reporter
  artifact_reporter_comms
  base_link_flattener
  point_cloud_filter
  report_relay
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

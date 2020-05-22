cmake_minimum_required(VERSION 2.8.3)
project(subt_solution_launch)
find_package(catkin REQUIRED)
catkin_package()
include_directories()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS scripts/simple_solution.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

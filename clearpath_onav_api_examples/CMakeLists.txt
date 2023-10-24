cmake_minimum_required(VERSION 3.0.2)
project(clearpath_onav_api_examples)

find_package(catkin REQUIRED COMPONENTS
)


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/simple_mission_1
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


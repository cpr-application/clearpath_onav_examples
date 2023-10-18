cmake_minimum_required(VERSION 3.0.2)
project(cpr_onav_api_examples_lib)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup(
)

catkin_package(
)

#catkin_install_python(PROGRAMS
#  base_node.py
#  goal.py
#  point_lat_lon.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES
#  base_node.py
#  goal.py
#  point_lat_lon.py
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

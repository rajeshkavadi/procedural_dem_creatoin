cmake_minimum_required(VERSION 3.0.2)
project(dem_viz)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/heatmap
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

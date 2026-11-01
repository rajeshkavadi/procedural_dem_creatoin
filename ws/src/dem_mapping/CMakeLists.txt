cmake_minimum_required(VERSION 3.0.2)
project(dem_mapping)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dem_node
  scripts/gnss_align
  scripts/georef_export
  scripts/gnss_warp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

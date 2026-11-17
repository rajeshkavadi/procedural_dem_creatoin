cmake_minimum_required(VERSION 3.0.2)
project(dem_replay)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/replay
  scripts/imu_relay
  scripts/stream_monitor
  scripts/inspect_bag
  scripts/record_odom
  scripts/pose_play
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

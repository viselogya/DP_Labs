cmake_minimum_required(VERSION 3.0.2)
project(lab1_4_arm_control)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/cycle_motion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

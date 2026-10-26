cmake_minimum_required(VERSION 3.0.2)
project(lab1_2_manipulator)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/move_to_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

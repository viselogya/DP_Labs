cmake_minimum_required(VERSION 3.0.2)
project(lab1_3_xacro_platform)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch robots rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

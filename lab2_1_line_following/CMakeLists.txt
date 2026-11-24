cmake_minimum_required(VERSION 3.0.2)
project(lab2_1_line_following)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/line_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch models urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

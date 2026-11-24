cmake_minimum_required(VERSION 3.0.2)
project(lab2_2_cycle)
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/odometry_publisher.py
  scripts/localization_publisher.py
  scripts/cycle_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch maps worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md cycle.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

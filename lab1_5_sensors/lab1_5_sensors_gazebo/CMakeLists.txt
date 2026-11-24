cmake_minimum_required(VERSION 3.0.2)
project(lab1_5_sensors_gazebo)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

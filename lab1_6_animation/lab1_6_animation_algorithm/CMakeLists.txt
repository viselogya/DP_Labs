cmake_minimum_required(VERSION 3.8)
project(lab1_6_animation_algorithm)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp sensor_msgs tf2)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(vacuum_sweep src/vacuum_sweep.cpp)
target_compile_features(vacuum_sweep PRIVATE cxx_std_11)
add_dependencies(vacuum_sweep ${catkin_EXPORTED_TARGETS})
target_link_libraries(vacuum_sweep ${catkin_LIBRARIES})

install(TARGETS vacuum_sweep RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

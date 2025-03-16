cmake_minimum_required(VERSION 3.0.2)
project(path_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

aux_source_directory(./src SRC_LIST)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path_tracker ${SRC_LIST})

target_link_libraries(path_tracker ${catkin_LIBRARIES})
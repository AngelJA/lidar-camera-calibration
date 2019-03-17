cmake_minimum_required(VERSION 2.8.3)
project(calib_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(plane_fit_node src/plane_fit_node.cpp)
target_link_libraries(plane_fit_node ${catkin_LIBRARIES})
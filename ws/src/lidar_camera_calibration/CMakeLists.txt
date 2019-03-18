cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera_calibration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs)

find_package(OpenCV 3.2 REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(lidar_camera_calibration_node src/lidar_camera_calibration.cpp)
target_link_libraries(lidar_camera_calibration_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
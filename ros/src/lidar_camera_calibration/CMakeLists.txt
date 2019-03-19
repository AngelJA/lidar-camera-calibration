cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera_calibration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    cv_bridge
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs)

find_package(OpenCV 3.2 REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(run_calibration src/lidar_camera_calibration.cpp)
target_link_libraries(run_calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
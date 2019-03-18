#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <unordered_map>

// pcl related
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// messages
#include <geometry_msgs/PointStamped.h>
using geometry_msgs::PointStampedConstPtr;

#include <sensor_msgs/Image.h>
using sensor_msgs::ImageConstPtr;

#include <sensor_msgs/PointCloud2.h>
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

#include <visualization_msgs/Marker.h>
using visualization_msgs::Marker;

class LidarCameraCalibration
{
    LidarCameraCalibration();

    // callbacks
    void ClickedPointCallback(PointStampedConstPtr msg);
    void ImageCallback(ImageConstPtr msg);
    void PointCloudCallback(PointCloud2ConstPtr msg);

    // data
    PointCloud2ConstPtr point_cloud;
    ImageConstPtr image;


    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscribers;
    std::unordered_map<std::string, ros::Publisher> publishers;
};
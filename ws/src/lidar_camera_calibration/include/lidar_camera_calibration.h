#pragma once

#include <ros/ros.h>

// opencv related
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

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
using geometry_msgs::Point;
using geometry_msgs::PointStampedConstPtr;

#include <sensor_msgs/Image.h>
using sensor_msgs::Image;
using sensor_msgs::ImageConstPtr;

#include <sensor_msgs/PointCloud2.h>
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

#include <visualization_msgs/Marker.h>
using visualization_msgs::Marker;

class LidarCameraCalibration
{
public:
    LidarCameraCalibration();

    // callbacks
    void ClickedPointCallback(PointStampedConstPtr msg);
    void ImageCallback(ImageConstPtr msg);
    void PointCloudCallback(PointCloud2ConstPtr msg);

private:
    // lidar processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPointCloud(Point point);
    void PerformPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    void PublishPlaneMarker(Point point);
    pcl::ModelCoefficients::Ptr plane_model_coefficients;

    // camera processing
    int checkerboard_rows;
    int checkerboard_columns;
    double checkerboard_square_size;
    cv::Mat_<double> checkerboard_points;

    // data
    PointCloud2ConstPtr last_point_cloud;
    ImageConstPtr image;

    std::vector<cv::Vec<double, 3>> theta_lidar;
    std::vector<double> alpha_lidar;
    std::vector<cv::Vec<double, 3>> theta_camera;
    std::vector<double> alpha_camera;

    // ros stuff
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscribers;
    ros::Publisher marker_publisher;
    ros::Publisher processed_image_publisher;
    ros::Publisher pc_test_pub;
};
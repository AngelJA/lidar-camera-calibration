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
#include <std_msgs/Empty.h>
using std_msgs::EmptyConstPtr;

#include <geometry_msgs/PointStamped.h>
using geometry_msgs::Point;
using geometry_msgs::PointStampedConstPtr;

#include <sensor_msgs/Image.h>
using sensor_msgs::Image;
using sensor_msgs::ImageConstPtr;

#include <sensor_msgs/PointCloud2.h>
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

#include <sensor_msgs/CameraInfo.h>
using sensor_msgs::CameraInfoConstPtr;

#include <visualization_msgs/Marker.h>
using visualization_msgs::Marker;

class LidarCameraCalibration
{
public:
    LidarCameraCalibration();

    // callbacks
    void ClickedPointCallback(PointStampedConstPtr msg);
    void ImageCallback(ImageConstPtr msg);
    void CameraInfoCallback(CameraInfoConstPtr msg);
    void PointCloudCallback(PointCloud2ConstPtr msg);
    void CalibrateCallback(EmptyConstPtr msg);

private:
    // lidar processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilterLastPointCloud(Point point);
    cv::Vec<double, 4> PerformPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    void PublishPlaneMarker(Point point);

    PointCloud2ConstPtr last_point_cloud;
    pcl::ModelCoefficients::Ptr plane_model_coefficients;
    std::vector<cv::Vec<double, 4>> lidar_plane_models;

    // camera processing
    boost::shared_ptr<std::vector<cv::Point2f>> DetectCorners(ImageConstPtr msg);
    cv::Vec<double, 4> SolveCameraPlaneModel();
    void PublishProcessedImage(ImageConstPtr msg);

    cv::Mat_<double> checkerboard_points;
    cv::Mat_<double> camera_matrix;
    std::vector<cv::Vec<double, 4>> camera_plane_models;
    boost::shared_ptr<std::vector<cv::Point2f>> last_camera_corners;

    // checkerboard params
    int checkerboard_rows;
    int checkerboard_columns;
    double checkerboard_square_size;

    // ros stuff
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subscribers;
    boost::shared_ptr<ros::Subscriber> camera_info_sub;
    ros::Publisher marker_publisher;
    ros::Publisher processed_image_publisher;
    ros::Publisher pc_test_pub;
};
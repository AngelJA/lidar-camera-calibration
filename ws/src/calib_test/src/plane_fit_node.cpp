#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <geometry_msgs/PointStamped.h>

using geometry_msgs::PointStampedConstPtr;
using sensor_msgs::PointCloud2ConstPtr;
using sensor_msgs::PointCloud2;

PointCloud2ConstPtr point_cloud;
ros::Publisher segment_pub, point_cloud_pub, inliers_pub;

void PointCloudCallback(PointCloud2ConstPtr msg)
{
    point_cloud = msg;
}

void ClickedPointCallback(PointStampedConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*point_cloud, *cloud);
    
    std::cout << cloud->size() << std::endl;

    // segment out just the area around the clicked point
    double d = 0.25;
    pcl::PointIndices::Ptr remove_indices = boost::make_shared<pcl::PointIndices>();
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (cloud->points[i].x < msg->point.x - d ||
            cloud->points[i].x > msg->point.x + d ||
            cloud->points[i].y < msg->point.y - d ||
            cloud->points[i].y > msg->point.y + d ||
            cloud->points[i].z < msg->point.z - d ||
            cloud->points[i].z > msg->point.z + d)
            {
                remove_indices->indices.push_back(i);
            }
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(remove_indices);
    extract.setNegative(true);
    extract.filter(*cloud);

    PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    segment_pub.publish(msg_out);

    // find plane fit
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);

    std::cout << inliers->indices.size() << " inliers out of " << cloud->size() << std::endl;

    extract.setNegative(false);
    extract.setIndices(inliers);
    extract.filter(*cloud);
    std::cout << cloud->size() << std::endl;

    PointCloud2 inliers_out;
    pcl::toROSMsg(*cloud, inliers_out);
    inliers_pub.publish(inliers_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_fit_node");
    ros::NodeHandle nh;
    ros::Subscriber clicked_point_sub = nh.subscribe("/clicked_point", 1, ClickedPointCallback);
    ros::Subscriber point_cloud_sub = nh.subscribe("/velodyne_points", 1, PointCloudCallback);
    segment_pub = nh.advertise<PointCloud2>("/segment_point_cloud", 1);
    point_cloud_pub = nh.advertise<PointCloud2>("/plane_point_cloud", 1);
    inliers_pub = nh.advertise<PointCloud2>("/inliers", 1);
    ros::spin();
    return 0;
}
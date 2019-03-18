#include <lidar_camera_calibration.h>


#include <std_msgs/Float32MultiArray.h>

using std_msgs::Float32MultiArray;
using std_msgs::MultiArrayDimension;
using geometry_msgs::Point;

ros::Publisher segment_pub, point_cloud_pub, inliers_pub, model_plane_pub, arrow_pub;

// void ClickedPointCallback(PointStampedConstPtr msg)
// {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
//     pcl::fromROSMsg(*point_cloud, *cloud);

//     // segment out just the area around the clicked point
//     double d = 0.25;
//     pcl::PointIndices::Ptr remove_indices = boost::make_shared<pcl::PointIndices>();
//     for (size_t i = 0; i < cloud->size(); ++i)
//     {
//         if (cloud->points[i].x < msg->point.x - d ||
//             cloud->points[i].x > msg->point.x + d ||
//             cloud->points[i].y < msg->point.y - d ||
//             cloud->points[i].y > msg->point.y + d ||
//             cloud->points[i].z < msg->point.z - d ||
//             cloud->points[i].z > msg->point.z + d)
//             {
//                 remove_indices->indices.push_back(i);
//             }
//     }

//     pcl::ExtractIndices<pcl::PointXYZI> extract;
//     extract.setInputCloud(cloud);
//     extract.setIndices(remove_indices);
//     extract.setNegative(true);
//     extract.filter(*cloud);

//     PointCloud2 msg_out;
//     pcl::toROSMsg(*cloud, msg_out);
//     segment_pub.publish(msg_out);

//     // find plane fit
//     pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//     pcl::SACSegmentation<pcl::PointXYZI> seg;
//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setDistanceThreshold(0.03);
//     seg.setInputCloud(cloud);
//     seg.segment(*inliers, *coeff);

//     // make sure plane normal points in negative x
//     if (coeff->values[0] > 0) {
//         coeff->values[0] *= -1;
//         coeff->values[1] *= -1;
//         coeff->values[2] *= -1;
//     }

//     extract.setNegative(false);
//     extract.setIndices(inliers);
//     extract.filter(*cloud);

//     PointCloud2 inliers_out;
//     pcl::toROSMsg(*cloud, inliers_out);
//     inliers_pub.publish(inliers_out);

//     // publish model coefficients
//     Float32MultiArray coeff_msg;
//     coeff_msg.layout.dim.push_back(MultiArrayDimension());
//     coeff_msg.layout.dim[0].size = 4;
//     coeff_msg.layout.dim[0].stride = 1;
//     coeff_msg.layout.dim[0].label = "coefficients";
//     for (float val : coeff->values)
//     {
//         coeff_msg.data.push_back(val);
//     }
//     model_plane_pub.publish(coeff_msg);

//     // draw arrow at clicked point representing plane normal
//     //double length = std::sqrt(std::pow(coeff->values[0], 2) + std::pow(coeff->values[1], 2) + std::pow(coeff->values[2], 2));
//     Marker arrow;
//     arrow.header.frame_id = "lidar_link";
//     arrow.header.stamp = ros::Time();
//     arrow.type = visualization_msgs::Marker::ARROW;
//     arrow.action = visualization_msgs::Marker::ADD;
//     Point p;
//     p.x = msg->point.x;
//     p.y = msg->point.y;
//     p.z = msg->point.z;
//     arrow.points.push_back(p);
//     p.x += coeff->values[0] / 3;
//     p.y += coeff->values[1] / 3;
//     p.z += coeff->values[2] / 3;
//     arrow.points.push_back(p);
//     arrow.scale.x = 0.03;
//     arrow.scale.y = 0.05;
//     arrow.scale.z = 0.05;
//     arrow.color.a = 1;
//     arrow.color.r = 1;
//     arrow_pub.publish(arrow);
// }

LidarCameraCalibration::LidarCameraCalibration()
{
    this->subscribers.push_back(this->nh.subscribe("/clicked_point", 1, &LidarCameraCalibration::ClickedPointCallback, this));
    this->subscribers.push_back(this->nh.subscribe("lidar_points", 1, &LidarCameraCalibration::PointCloudCallback, this));
    
    this->publishers["inliers"] = this->nh.advertise<PointCloud2>("inliers", 1);
    this->publishers["markers"] = this->nh.advertise<Marker>("markers", 1);

    // segment_pub = nh.advertise<PointCloud2>("segment_point_cloud", 1);
    // point_cloud_pub = nh.advertise<PointCloud2>("plane_point_cloud", 1);
    // model_plane_pub = nh.advertise<Float32MultiArray>("model_plane_coeffs", 1);
    ros::spin();
}

void LidarCameraCalibration::ClickedPointCallback(PointStampedConstPtr msg)
{

}

void LidarCameraCalibration::ImageCallback(ImageConstPtr msg)
{
    this->image = msg;
}

void LidarCameraCalibration::PointCloudCallback(PointCloud2ConstPtr msg)
{
    this->point_cloud = msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_camera_calibration_node");
    LidarCameraCalibration node();
    return 0;
}
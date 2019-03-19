#include <lidar_camera_calibration.h>

#include <iostream>

LidarCameraCalibration::LidarCameraCalibration() : nh("~")
{
    this->subscribers.push_back(this->nh.subscribe("/clicked_point", 1, &LidarCameraCalibration::ClickedPointCallback, this));
    this->subscribers.push_back(this->nh.subscribe("image", 1, &LidarCameraCalibration::ImageCallback, this));
    this->subscribers.push_back(this->nh.subscribe("lidar_points", 1, &LidarCameraCalibration::PointCloudCallback, this));
    this->subscribers.push_back(this->nh.subscribe("calibrate", 1, &LidarCameraCalibration::CalibrateCallback, this));
    
    this->marker_publisher = this->nh.advertise<Marker>("markers", 1);
    this->processed_image_publisher = this->nh.advertise<Image>("processed_image", 1);
    this->pc_test_pub = this->nh.advertise<PointCloud2>("test_pc", 1);

    assert(this->nh.getParam("checkerboard/rows", this->checkerboard_rows));
    assert(this->nh.getParam("checkerboard/columns", this->checkerboard_columns));
    assert(this->nh.getParam("checkerboard/square_size", this->checkerboard_square_size));
    this->checkerboard_points = cv::Mat_<double>(this->checkerboard_rows * this->checkerboard_columns, 3, CV_64F);
    for (int i = 0; i < this->checkerboard_rows * this->checkerboard_columns; ++i)
    {
        this->checkerboard_points(i, 0) = (i / this->checkerboard_rows) * this->checkerboard_square_size;
        this->checkerboard_points(i, 1) = (i % this->checkerboard_rows) * this->checkerboard_square_size;
        this->checkerboard_points(i, 2) = 0.0;
    }

    // segment_pub = nh.advertise<PointCloud2>("segment_point_cloud", 1);
    // point_cloud_pub = nh.advertise<PointCloud2>("plane_point_cloud", 1);
    // model_plane_pub = nh.advertise<Float32MultiArray>("model_plane_coeffs", 1);
    ros::spin();
}

void LidarCameraCalibration::ClickedPointCallback(PointStampedConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud = this->FilterPointCloud(msg->point);
    
    PointCloud2 test;
    pcl::toROSMsg(*filtered_point_cloud, test);
    this->pc_test_pub.publish(test);

    this->PerformPlaneFit(filtered_point_cloud);

}

void LidarCameraCalibration::ImageCallback(ImageConstPtr msg)
{
    this->image = msg;

    cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvShare(msg);
    cv_bridge::CvImageConstPtr gray_image_ptr = cv_bridge::toCvShare(msg, "mono8");
    std::vector<cv::Point2f> corners;
    cv::Size pattern_size(this->checkerboard_rows, this->checkerboard_columns);
    bool pattern_found = cv::findChessboardCorners(
        gray_image_ptr->image,
        pattern_size,
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if (pattern_found)
    {
        // refine detected corners
        // Use a radius of half the minimum distance between corners. This should be large enough to snap to the
        // correct corner, but not so large as to include a wrong corner in the search window.
        auto dist = [](cv::Point2f& a, cv::Point2f& b){ return cv::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)); };
        double min_distance = dist(corners[0], corners[1]);
        for (int i = 1; i < corners.size(); ++i)
        {
            if ((i + 1) % this->checkerboard_columns)
            {
                min_distance = std::min(min_distance, dist(corners[i], corners[i + 1]));
            }
            if (i + this->checkerboard_columns < corners.size())
            {
                min_distance = std::min(min_distance, dist(corners[i], corners[i + this->checkerboard_columns]));
            }
        }

        int radius = std::ceil(min_distance * 0.5);
        cv::cornerSubPix(
            gray_image_ptr->image,
            corners,
            cv::Size(radius, radius),
            cv::Size(-1, -1),
            cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        // // still needs to be ported from python
        // if corners[0, 0, 1] > corners[-1, 0, 1]:
        //     numpy.flipud(corners)

        // ret, r, t = cv2.solvePnP(self._object_points, corners, self._camera_matrix, (0, 0, 0, 0))
        // R, _ = cv2.Rodrigues(r)

        // self._last_theta_c = numpy.array([R[:, 2]]).T
        // self._last_alpha_c = t.T.dot(self._last_theta_c)

        cv::drawChessboardCorners(image_ptr->image, pattern_size, corners, pattern_found);
    }


    this->processed_image_publisher.publish(image_ptr->toImageMsg());
}

void LidarCameraCalibration::PointCloudCallback(PointCloud2ConstPtr msg)
{
    this->last_point_cloud = msg;
}

void LidarCameraCalibration::CalibrateCallback(EmptyConstPtr msg)
{

}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarCameraCalibration::FilterPointCloud(Point point)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*this->last_point_cloud, *cloud);

    double d = 0.25;
    pcl::PointIndices::Ptr remove_indices(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (cloud->points[i].x < point.x - d ||
            cloud->points[i].x > point.x + d ||
            cloud->points[i].y < point.y - d ||
            cloud->points[i].y > point.y + d ||
            cloud->points[i].z < point.z - d ||
            cloud->points[i].z > point.z + d)
            {
                remove_indices->indices.push_back(i);
            }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(remove_indices);
    extract.setNegative(true);
    extract.filter(*cloud);

    return cloud;
}

void LidarCameraCalibration::PerformPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(point_cloud);
    seg.segment(*inliers, *coefficients);

    // make sure plane normal points in negative x
    if (coefficients->values[0] > 0) {
        coefficients->values[0] *= -1;
        coefficients->values[1] *= -1;
        coefficients->values[2] *= -1;
    }

    this->theta_lidar.push_back(cv::Vec<double, 3>(
        coefficients->values[0],
        coefficients->values[1],
        coefficients->values[2]));
    this->alpha_lidar.push_back(coefficients->values[3]);

    cv::Mat_<double> m(3, this->theta_lidar.size(), CV_64F);
    for (int i = 0; i < this->theta_lidar.size(); ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            m(j, i) = this->theta_lidar[i][j];
        }
    }
    std::cout << "matrix = " << m << std::endl;
}

void PublishPlaneMarker(Point point)
{
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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_camera_calibration_node");
    LidarCameraCalibration node;
    return 0;
}
#include <lidar_camera_calibration.h>

#include <iostream>

LidarCameraCalibration::LidarCameraCalibration() : nh("~")
{
    this->subscribers.push_back(this->nh.subscribe("/clicked_point", 1, &LidarCameraCalibration::ClickedPointCallback, this));
    this->subscribers.push_back(this->nh.subscribe("image", 1, &LidarCameraCalibration::ImageCallback, this));
    this->subscribers.push_back(this->nh.subscribe("lidar_points", 1, &LidarCameraCalibration::PointCloudCallback, this));
    this->subscribers.push_back(this->nh.subscribe("calibrate", 1, &LidarCameraCalibration::CalibrateCallback, this));
    this->camera_info_sub = boost::make_shared<ros::Subscriber>(this->nh.subscribe("camera_info", 1, &LidarCameraCalibration::CameraInfoCallback, this));

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
    this->camera_matrix = cv::Mat_<double>(3, 3, CV_64F);

    ros::spin();
}

void LidarCameraCalibration::ClickedPointCallback(PointStampedConstPtr msg)
{
    if (this->last_camera_corners->size())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud = this->FilterLastPointCloud(msg->point);
        PlaneFitResults result = this->PerformPlaneFit(filtered_point_cloud);

        this->StoreLidarInliers(filtered_point_cloud, result.inliers);
        this->lidar_plane_models.push_back(std::array<double, 4>{{
            result.coefficients->values[0],
            result.coefficients->values[1],
            result.coefficients->values[2],
            result.coefficients->values[3]
        }});
        this->camera_plane_models.push_back(this->SolveCameraPlaneModel());

        std::vector<std::vector<std::array<double, 4>>> models;
        models.push_back(this->lidar_plane_models);
        models.push_back(this->camera_plane_models);

        for (auto model : models)
        {
            cv::Mat_<double> m(4, model.size(), CV_64F);
            for (int i = 0; i < model.size(); ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    m(j, i) = model[i][j];
                }
            }
            std::cout << "matrix = " << m << std::endl;
        }
    }
    else
    {
        ROS_INFO("Target not detected in last camera frame.");
    }
}

void LidarCameraCalibration::ImageCallback(ImageConstPtr msg)
{
    this->last_camera_corners = this->DetectCorners(msg);
    this->PublishProcessedImage(msg);
}

void LidarCameraCalibration::CameraInfoCallback(CameraInfoConstPtr msg)
{
    this->camera_matrix(0, 0) = msg->K[0];
    this->camera_matrix(0, 1) = msg->K[1];
    this->camera_matrix(0, 2) = msg->K[2];
    this->camera_matrix(1, 0) = msg->K[3];
    this->camera_matrix(1, 1) = msg->K[4];
    this->camera_matrix(1, 2) = msg->K[5];
    this->camera_matrix(2, 0) = msg->K[6];
    this->camera_matrix(2, 1) = msg->K[7];
    this->camera_matrix(2, 2) = msg->K[8];

    this->camera_info_sub = nullptr;
}

void LidarCameraCalibration::PointCloudCallback(PointCloud2ConstPtr msg)
{
    this->last_point_cloud = msg;
}

double LidarCameraCalibration::FunctionToMinimize(const arma::vec& vals, arma::vec* grad_out, void* opt_data)
{
    arma::vec t({vals[0], vals[1], vals[2]});
    arma::mat R = this->q2R(arma::vec({vals[3], vals[4], vals[5], vals[6]}));

    return RMSDistanceLidarPointsToCameraPlanes(R, t);
};

void LidarCameraCalibration::CalibrateCallback(EmptyConstPtr msg)
{
    // v = [tx, ty, tz, qw, qx, qy, qz]
    arma::vec v({0, 0, 0, 1, 0, 0, 0});
    optim::algo_settings_t settings;
    settings.verbose_print_level = 2;
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    optim::nm(v, std::bind(&LidarCameraCalibration::FunctionToMinimize, this, _1, _2, _3), 0, settings);
}

arma::vec LidarCameraCalibration::R2q(arma::mat &R)
{
    arma::vec q(4, arma::fill::zeros);
    double t = 1 + arma::trace(R);

    if (t <= 0)
    {
        ROS_INFO("Trace is <= 0");
    }
    else
    {
        double s = 0.5 / cv::sqrt(t);
        q(0) = 0.5 * cv::sqrt(t);
        q(1) = (R(2, 1) - R(1, 2)) * s;
        q(2) = (R(0, 2) - R(2, 0)) * s;
        q(3) = (R(1, 0) - R(0, 1)) * s;
    }

    return q;
}

arma::mat LidarCameraCalibration::q2R(arma::vec q)
{
    arma::mat R(3, 3, arma::fill::zeros);
    arma::vec q_norm = q / arma::norm(q);
    
    double qw = q_norm(0);
    double qx = q_norm(1);
    double qy = q_norm(2);
    double qz = q_norm(3);
    double qw2 = qw * qw;
    double qx2 = qx * qx;
    double qy2 = qy * qy;
    double qz2 = qz * qz;

    R(0, 0) = qx2 - qy2 - qz2 + qw2;
    R(0, 1) = 2 * (qx * qy - qz * qw);
    R(0, 2) = 2 * (qx * qz + qy * qw);
    R(1, 0) = 2 * (qx * qy + qz * qw);
    R(1, 1) = -qx2 + qy2 - qz2 + qw2;
    R(1, 2) = 2 * (qy * qz - qx * qw);
    R(2, 0) = 2 * (qx * qz - qy * qw);
    R(2, 1) = 2 * (qy * qz + qx * qw);
    R(2, 2) = -qx2 - qy2 + qz2 + qw2;
    
    return R;
}

boost::shared_ptr<std::vector<cv::Point2f>> LidarCameraCalibration::DetectCorners(ImageConstPtr msg)
{
    cv_bridge::CvImageConstPtr gray_image_ptr = cv_bridge::toCvShare(msg, "mono8");
    boost::shared_ptr<std::vector<cv::Point2f>> corners(new std::vector<cv::Point2f>);
    cv::Size pattern_size(this->checkerboard_rows, this->checkerboard_columns);

    cv::findChessboardCorners(
        gray_image_ptr->image,
        pattern_size,
        *corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if (corners->size())
    {
        // refine detected corners
        // Use a radius of half the minimum distance between corners. This should be large enough to snap to the
        // correct corner, but not so large as to include a wrong corner in the search window.
        auto dist = [](cv::Point2f& a, cv::Point2f& b){ return cv::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)); };
        double min_distance = dist(corners->at(0), corners->at(1));
        for (int i = 1; i < corners->size(); ++i)
        {
            if ((i + 1) % this->checkerboard_columns)
            {
                min_distance = std::min(min_distance, dist(corners->at(i), corners->at(i + 1)));
            }
            if (i + this->checkerboard_columns < corners->size())
            {
                min_distance = std::min(min_distance, dist(corners->at(i), corners->at(i + this->checkerboard_columns)));
            }
        }

        int radius = std::ceil(min_distance * 0.5);
        cv::cornerSubPix(
            gray_image_ptr->image,
            *corners,
            cv::Size(radius, radius),
            cv::Size(-1, -1),
            cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        // // need to test whether this is necessary
        // if corners[0, 0, 1] > corners[-1, 0, 1]:
        //     numpy.flipud(corners)
    }
    return corners;
}

std::array<double, 4> LidarCameraCalibration::SolveCameraPlaneModel()
{
    std::vector<double> r;
    cv::Vec<double, 3> t;
    cv::solvePnP(
        this->checkerboard_points,
        *this->last_camera_corners,
        this->camera_matrix,
        std::vector<double>(4, 0),
        r,
        t);
    
    cv::Mat_<double> R;
    cv::Rodrigues(r, R);

    return std::array<double, 4>{{
        R(0, 2),
        R(1, 2),
        R(2, 2),
        t.dot(R.col(2))}};
}

void LidarCameraCalibration::PublishProcessedImage(ImageConstPtr msg)
{
    cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvShare(msg);
    if (this->last_camera_corners->size())
    {
        cv::Size pattern_size(this->checkerboard_rows, this->checkerboard_columns);
        cv::drawChessboardCorners(image_ptr->image, pattern_size, *this->last_camera_corners, true);
    }
    this->processed_image_publisher.publish(image_ptr->toImageMsg());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarCameraCalibration::FilterLastPointCloud(Point point)
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

void LidarCameraCalibration::StoreLidarInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, pcl::PointIndices::Ptr indices)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(point_cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*point_cloud);
    
    this->lidar_inliers.push_back(arma::mat(4, indices->indices.size(), arma::fill::ones));
    arma::mat& points = this->lidar_inliers.back();

    for (std::vector<pcl::PointXYZ>::size_type i = 0; i < point_cloud->points.size(); ++i)
    {
        points(0, i) = point_cloud->points[i].x;
        points(1, i) = point_cloud->points[i].y;
        points(2, i) = point_cloud->points[i].z;
    }
}

double LidarCameraCalibration::RMSDistanceLidarPointsToCameraPlanes(arma::mat &R, arma::vec &t)
{
    return 0.0;
}

LidarCameraCalibration::PlaneFitResults LidarCameraCalibration::PerformPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
    PlaneFitResults result;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03); // TODO: make this a param
    seg.setInputCloud(point_cloud);
    seg.segment(*result.inliers, *result.coefficients);

    // make sure plane normal points in negative x
    if (result.coefficients->values[0] > 0) {
        result.coefficients->values[0] *= -1;
        result.coefficients->values[1] *= -1;
        result.coefficients->values[2] *= -1;
    }

    return result;
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
#include "../include/vio_node/VioNode.hpp"
#include <rclcpp/rclcpp.hpp>

VioNode::VioNode() : Node("VIO_estimator")
{
    // Image Synchronizer
    sub_left_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, IMAGE0_TOPIC);
    sub_right_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, IMAGE1_TOPIC);
    sub_imu_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, IMU_TOPIC);
    using syncpolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Imu>;
    sync = std::make_shared<message_filters::Synchronizer<syncpolicy>>(
        syncpolicy(10), *sub_left_, *sub_right_, *sub_imu_);
    sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02));
    sync->registerCallback(std::bind(&VioNode::sensor_cb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    path_msg_.header.frame_id = "map";
    setPubSub();
    setParams();
}
VioNode::~VioNode(){}

void VioNode::setPubSub() {
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("vio/pose", 10);

    blue_dots_pub_ = this->create_publisher<std_msgs::msg::Int32>("vio/stats/blue_dots", 10);
    green_dots_pub_ = this->create_publisher<std_msgs::msg::Int32>("vio/stats/green_dots", 10);
    red_dots_pub_ = this->create_publisher<std_msgs::msg::Int32>("vio/stats/red_dots", 10);

    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 50);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vio/point_cloud", 10);
    return;
}

void VioNode::setParams() {

    std::vector<double> K0_vec_default = {458.654, 457.296, 367.215, 248.375};
    std::vector<double> T_BS_0_default = {
        0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
        0.999557249008 ,  0.0149672133247, 0.025715529948 , -0.064676986768,
       -0.0257744366974,  0.00375618835797, 0.999660727178 , 0.00981073058949,
        0.0, 0.0, 0.0, 1.0
    };
    std::vector<double> K1_vec_default = {457.587, 456.134, 379.999, 255.238};
    std::vector<double> T_BS_1_default = {
        0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
        0.999598781151 ,  0.0130119051815, 0.0251588363115,  0.0453689425024,
       -0.0253898008918,  0.0179005838253, 0.999517347078 ,  0.00786212447038,
        0.0, 0.0, 0.0, 1.0
    };
    this->declare_parameter<std::vector<double>>("cam0.intrinsics", K0_vec_default);
    this->declare_parameter<std::vector<double>>("cam0.T_BS.data", T_BS_0_default);
    this->declare_parameter<std::vector<double>>("cam1.intrinsics", K1_vec_default);
    this->declare_parameter<std::vector<double>>("cam1.T_BS.data", T_BS_1_default);

    std::vector<double> K0_vec = this->get_parameter("cam0.intrinsics").as_double_array();
    std::vector<double> T_BS_0 = this->get_parameter("cam0.T_BS.data").as_double_array();
    std::vector<double> K1_vec = this->get_parameter("cam1.intrinsics").as_double_array();
    std::vector<double> T_BS_1 = this->get_parameter("cam1.T_BS.data").as_double_array();

    // construct Cam0, Cam1
    //cam0
    cv::Mat K0 = (cv::Mat_<double>(3,3) << K0_vec[0], 0, K0_vec[2], // fu, 0, cu
                                           0, K0_vec[1], K0_vec[3], // 0, fv, cv
                                           0, 0, 1);
    //cam1
    cv::Mat K1 = (cv::Mat_<double>(3,3) << K1_vec[0], 0, K1_vec[2], // fu, 0, cu
                                           0, K1_vec[1], K1_vec[3], // 0, fv, cv
                                           0, 0, 1);
    //T_BS_0
    cv::Mat T0_4x4 = cv::Mat(4, 4, CV_64F, T_BS_0.data()).clone();
    cv::Mat T0_3x4 = T0_4x4(cv::Rect(0, 0, 4, 3));
    cv::Mat P0 = K0 * T0_3x4;
    
    cv::Mat T1_4x4 = cv::Mat(4, 4, CV_64F, T_BS_1.data()).clone();
    cv::Mat T1_3x4 = T1_4x4(cv::Rect(0, 0, 4, 3));
    cv::Mat P1 = K1 * T1_3x4;
    RCLCPP_INFO(this->get_logger(), "Camera projection matrices set.");
    RCLCPP_INFO(this->get_logger(), "P0: \n[%f, %f, %f, %f;\n %f, %f, %f, %f;\n %f, %f, %f, %f]", 
        P0.at<double>(0,0), P0.at<double>(0,1), P0.at<double>(0,2), P0.at<double>(0,3),
        P0.at<double>(1,0), P0.at<double>(1,1), P0.at<double>(1,2), P0.at<double>(1,3),
        P0.at<double>(2,0), P0.at<double>(2,1), P0.at<double>(2,2), P0.at<double>(2,3));
    RCLCPP_INFO(this->get_logger(), "P1: \n[%f, %f, %f, %f;\n %f, %f, %f, %f;\n %f, %f, %f, %f]", 
        P1.at<double>(0,0), P1.at<double>(0,1), P1.at<double>(0,2), P1.at<double>(0,3),
        P1.at<double>(1,0), P1.at<double>(1,1), P1.at<double>(1,2), P1.at<double>(1,3),
        P1.at<double>(2,0), P1.at<double>(2,1), P1.at<double>(2,2), P1.at<double>(2,3));                

    estimator.setCalibration(P0, P1);

    return;
}

void VioNode::sensor_cb(const sensor_msgs::msg::Image::ConstSharedPtr& cam0_msg, 
    const sensor_msgs::msg::Image::ConstSharedPtr& cam1_msg, 
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{      
    cv_bridge::CvImagePtr cam0_ptr = cv_bridge::toCvCopy(cam0_msg, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImagePtr cam1_ptr = cv_bridge::toCvCopy(cam1_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img0, img1;
    img0 = cam0_ptr->image;
    img1 = cam1_ptr->image;
    time = cam0_msg->header.stamp.sec + cam0_msg->header.stamp.nanosec * (1e-9);
    cv::Mat visualized_image;
    stopwatch.start_time();       
    visualized_image = estimator.inputImage(time, img0, img1);
    elasped_time = stopwatch.end_time();
    double processing_time_s = elasped_time / 1000.0;
    double processing_hz = 1.0 / processing_time_s;
    RCLCPP_INFO(this->get_logger(), 
                "VIO Image Processing Time: %.2f ms (%.2f Hz)", 
                elasped_time,
                processing_hz);
    cv_bridge::CvImage out_msg;
    out_msg.header = cam0_msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; //MONO8
    out_msg.image = visualized_image;
    
    // img_pub_->publish(*cam1_msg);
    img_pub_->publish(*out_msg.toImageMsg());
    // publish for RQT PLOT
    auto stats = estimator.getLastFeatureStats();
    auto blue_msg = std_msgs::msg::Int32();
    auto green_msg = std_msgs::msg::Int32();
    auto red_msg = std_msgs::msg::Int32();

    blue_msg.data = stats.blue_count;
    green_msg.data = stats.green_count;
    red_msg.data = stats.red_count;

    blue_dots_pub_->publish(blue_msg);
    green_dots_pub_->publish(green_msg);
    red_dots_pub_->publish(red_msg);
    // Publish point cloud
    const auto& world_points = this->estimator.getWorldPoints();
    RCLCPP_INFO(this->get_logger(), "Publishing %zu 3D points.", world_points.size());
    if (!world_points.empty())
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = cam0_msg->header.stamp;
        cloud_msg.header.frame_id = "map"; 
        
        cloud_msg.height = 1;
        cloud_msg.width = world_points.size();
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        // Define the fields (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        modifier.resize(world_points.size());

        // Iterate through the data and fill the cloud
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& pt : world_points)
        {
            *iter_x = pt.x;
            *iter_y = pt.y;
            *iter_z = pt.z;
            
            ++iter_x; ++iter_y; ++iter_z;
        }
        
        pub_pointcloud_->publish(cloud_msg);
    }
    this->imu0_callback(imu_msg);
    this->publish_odometry();
}

void VioNode::imu0_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * (1e-9);
    double dx = msg->linear_acceleration.x;
    double dy = msg->linear_acceleration.y;
    double dz = msg->linear_acceleration.z;

    double ax = msg->angular_velocity.x;
    double ay = msg->angular_velocity.y;
    double az = msg->angular_velocity.z;

    Eigen::Vector3d acc(dx, dy, dz);
    Eigen::Vector3d gyr(ax, ay, az);
    RCLCPP_INFO(this->get_logger(), "Received IMU: t=%.6f, acc=(%.4f, %.4f, %.4f), gyr=(%.4f, %.4f, %.4f)", t, dx, dy, dz, ax, ay, az);
    estimator.integrateIMU(t,acc,gyr);
    
    return;
}
void VioNode::publish_odometry()
{
    Eigen::Vector3d pos = estimator.getPosition();
    Eigen::Quaterniond quat = estimator.getOrientation();
    Eigen::Vector3d vel = estimator.getVelocity();

    rclcpp::Time now = this->get_clock()->now();

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = now;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";

    odometry.pose.pose.position.x = pos.x();
    odometry.pose.pose.position.y = pos.y();
    odometry.pose.pose.position.z = pos.z();
    odometry.pose.pose.orientation.x = quat.x();
    odometry.pose.pose.orientation.y = quat.y();
    odometry.pose.pose.orientation.z = quat.z();
    odometry.pose.pose.orientation.w = quat.w();
    odometry.twist.twist.linear.x = vel.x();
    odometry.twist.twist.linear.y = vel.y();
    odometry.twist.twist.linear.z = vel.z();

    pub_odometry_->publish(odometry);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = odometry.pose.pose;
    
    path_msg_.header.stamp = now;
    path_msg_.poses.push_back(pose_stamped);

    if (path_msg_.poses.size() > 1000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }
    pub_path_->publish(path_msg_);
    // geometry_msgs::msg::TransformStamped t;
    // t.header.stamp = now;
    // t.header.frame_id = "map";
    // t.child_frame_id = "base_link";

    // t.transform.translation.x = pos.x();
    // t.transform.translation.y = pos.y();
    // t.transform.translation.z = pos.z();
    // t.transform.rotation.x = quat.x();
    // t.transform.rotation.y = quat.y();
    // t.transform.rotation.z = quat.z();
    // t.transform.rotation.w = quat.w();

    // tf_broadcaster_->sendTransform(t);
}



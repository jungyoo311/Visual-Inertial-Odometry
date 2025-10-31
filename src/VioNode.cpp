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
    sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.00001));
    sync->registerCallback(std::bind(&VioNode::sensor_cb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    return;
}

void VioNode::setParams() {
    return;
}

void VioNode::sensor_cb(const sensor_msgs::msg::Image::ConstSharedPtr& cam0_msg, 
    const sensor_msgs::msg::Image::ConstSharedPtr& cam1_msg, 
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{      
    rclcpp::Time left_time = cam0_msg->header.stamp;
    rclcpp::Time right_time = cam1_msg->header.stamp;
    rclcpp::Time imu_time = imu_msg->header.stamp;
    double time_diff = std::abs(left_time.nanoseconds() - imu_time.nanoseconds());

    RCLCPP_INFO(this->get_logger(), 
      "Received synchronized pair! Time diff: %.10f s, left: %.10f s, right: %.10f s, imu: %.10f s", time_diff / 1e9, 
      left_time.nanoseconds() / 1e9, right_time.nanoseconds() / 1e9, imu_time.nanoseconds() / 1e9);

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
}

VioEstimator estimator; //TODO: move this to class member
void VioNode::imu0_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

    odometry.pose.pose.position.x = pos.x()/1000;
    odometry.pose.pose.position.y = pos.y()/1000;
    odometry.pose.pose.position.z = pos.z()/1000;
    odometry.pose.pose.orientation.x = quat.x();
    odometry.pose.pose.orientation.y = quat.y();
    odometry.pose.pose.orientation.z = quat.z();
    odometry.pose.pose.orientation.w = quat.w();
    odometry.twist.twist.linear.x = vel.x()/1000;
    odometry.twist.twist.linear.y = vel.y()/1000;
    odometry.twist.twist.linear.z = vel.z()/1000;

    pub_odometry_->publish(odometry);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = pos.x();
    t.transform.translation.y = pos.y();
    t.transform.translation.z = pos.z();
    t.transform.rotation.x = quat.x();
    t.transform.rotation.y = quat.y();
    t.transform.rotation.z = quat.z();
    t.transform.rotation.w = quat.w();

    tf_broadcaster_->sendTransform(t);
}



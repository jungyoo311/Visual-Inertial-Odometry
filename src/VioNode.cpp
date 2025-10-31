#include <map>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#define IMAGE0_TOPIC "/cam0/image_raw"
#define IMAGE1_TOPIC "/cam1/image_raw"
#define IMU_TOPIC "/imu0"
#include "../include/vio_node/VioNode.hpp"

VioNode::VioNode() : Node("VIO_estimator")
{
    // imu queue depth 50 seems enough?
    rclcpp::QoS imu_qos(rclcpp::KeepLast(50));
    imu_qos.best_effort();
    sub_imu0 = this -> create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, imu_qos, 
                                                                    std::bind(&VioNode::imu0_callback, this, std::placeholders::_1));
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(100));
    sub_cam0.subscribe(this, IMAGE0_TOPIC, qos.get_rmw_qos_profile());
    sub_cam1.subscribe(this, IMAGE1_TOPIC, qos.get_rmw_qos_profile());
    uint32_t q_size = 10;
    //initialize synchronizer - Policy: Approximate time 3ms
    sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(q_size), sub_cam0, sub_cam1);
    sync->setAgePenalty(0.003);
    sync->registerCallback(std::bind(&VioNode::img_cb, this, std::placeholders::_1, std::placeholders::_2));
    
    
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("vio/pose", 10);

    blue_dots_pub = this->create_publisher<std_msgs::msg::Int32>("vio/stats/blue_dots", 10);
    green_dots_pub = this->create_publisher<std_msgs::msg::Int32>("vio/stats/green_dots", 10);
    red_dots_pub = this->create_publisher<std_msgs::msg::Int32>("vio/stats/red_dots", 10);

    pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 50);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create a timer to publish odometry at 30Hz (approx 33ms)
    pub_timer = this->create_wall_timer(std::chrono::milliseconds(33), 
                                         std::bind(&VioNode::publish_odometry, this));
}
VioNode::~VioNode(){};

VioEstimator estimator;
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

    pub_odometry->publish(odometry);

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

    tf_broadcaster->sendTransform(t);
}
void VioNode::img_cb(const sensor_msgs::msg::Image::ConstSharedPtr& cam0_msg, const sensor_msgs::msg::Image::ConstSharedPtr& cam1_msg)
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
        
        // img_pub->publish(*cam1_msg);
        img_pub->publish(*out_msg.toImageMsg());
        // publish for RQT PLOT
        auto stats = estimator.getLastFeatureStats();
        auto blue_msg = std_msgs::msg::Int32();
        auto green_msg = std_msgs::msg::Int32();
        auto red_msg = std_msgs::msg::Int32();

        blue_msg.data = stats.blue_count;
        green_msg.data = stats.green_count;
        red_msg.data = stats.red_count;

        blue_dots_pub->publish(blue_msg);
        green_dots_pub->publish(green_msg);
        red_dots_pub->publish(red_msg);
}


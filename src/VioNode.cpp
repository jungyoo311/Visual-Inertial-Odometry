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
//0923: 
//0922: image_syncer() done but needs to be fixed. manual threading->message_filters,
//sync_thread and timer_ could try to access the buffers at the exact same time, leading to unpredictable behavior.
// USE message_filters lib for synchronization. 
// CMakeLists.txt looks ok? my code works but it has unpredictable delays,, probably threading issues....
//0921: finished setup. now subscribe from ros2 node and publish on the rviz for input-output check

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

    // call vio estiamtor class instance using imu input member function here
    estimator.inputIMU(t, acc, gyr);
}

/**
 * @brief 
 * @details 
 * 
 */
void VioNode::image_syncer(const sensor_msgs::msg::Image::ConstSharedPtr& cam0_msg, const sensor_msgs::msg::Image::ConstSharedPtr& cam1_msg)
{
    try {
        
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

    } catch(const std::exception& e){
        RCLCPP_INFO(this->get_logger(), "syncing error %s", e.what());
    }
}

// constructor
VioNode::VioNode() : Node("VIO_estimator")
{
    // pre-integration means integrate imu0 first?
    sub_imu0 = this -> create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)), 
                                                                    std::bind(&VioNode::imu0_callback, this, std::placeholders::_1));
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(100));
    sub_cam0.subscribe(this, IMAGE0_TOPIC, qos.get_rmw_qos_profile());
    sub_cam1.subscribe(this, IMAGE1_TOPIC, qos.get_rmw_qos_profile());
    uint32_t q_size = 10;
    //initialize synchronizer - Policy: Approximate time 3ms
    sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(q_size), sub_cam0, sub_cam1);
    sync->setAgePenalty(0.003);
    sync->registerCallback(std::bind(&VioNode::image_syncer, this, std::placeholders::_1, std::placeholders::_2));
    
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("vio/pose", 10);
}

// deallocate
VioNode::~VioNode(){};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
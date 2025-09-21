#include <map>
#include <thread>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#define IMAGE0_TOPIC "/cam0/image_raw"
#define IMAGE1_TOPIC "/cam1/image_raw"
#define IMU_TOPIC "/imu0"
#include "../include/vio_node/VioNode.hpp"

//0921: finished setup. now subscribe from ros2 node and publish on the rviz for input-output check

// a buffer to hold incoming sensor msgs from ROS2 subscribers
queue<sensor_msgs::msg::Image:ConstPtr> img0_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img1_buf;
queue<sensor_msgs::msg::Imu::ConstPtr> imu0_buf;
std::mutex m_buf;
void VioNode::imu_callback(const sensor_msgs::msgs::Imu::SharedPtr msg)
{
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * (1e-9);
    double dx = msg->linear_acceleration.x;
    double dy = msg->linear_acceleration.y;
    double dz = msg->linear_acceleration.z;

    double ax = msg->angular_velocity.x;
    double ay = msg->angular_velocity.x;
    double az = msg->angular_velocity.x;

    Vector3d acc(dx, dy, dz);
    Vector3d gyr(ax, ay, az);

    // call vio estiamtor class instance using imu input member function here
}
void VioNode::image0_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    m_buf.lock();
    // ensures only one thread can access the buffers at a time
    img0_buf.push(msg);
    m_buf_unlock();
}
void VioNode::image1_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    m_buf.lock();
    img1_buf.push(msg);
    m_buf.unlock();
}
void VioNode::image_syncer()
{

}
// constructor
VioNode::VioNode() : Node("VIO_estimator")
{
    // pre-integration means integrate imu0 first?
    sub_imu0 = node -> create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)), imu_callback);
    //create subscription here - cam0, cam1, imu0
    sub_cam0 = node->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), image0_callback);
    sub_cam1 = node->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), image1_callback);
}
// deallocate
~VioNode::VioNode(){};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        audo node = rclcpp::Node::make_shared();
        rclcpp::spin(node);
    } catch(const std::exception& e){
        cout<<"error: %s" << e.what() << endl;
        return 1;
    }    
    
    rclcpp::shutdown();
    return 0;
}
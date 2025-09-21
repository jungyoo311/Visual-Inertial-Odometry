#ifndef VIO_NODE
#define VIO_NODE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

// shared ptrs to msgs input
sensor_msgs::msg::Image ImageMsg;
sensor_msgs::msg::Imu ImuMsg;

class Vio_Node : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new VIO Ros node
        */
        Vio_Node();
    private:
        /**
         * @brief callback for incoming img msgs
        */
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        /**
         * @brief callback for incoming imu msgs
        */
        void imu_callback(const sensor_msgs::msgs::Imu::SharedPtr msg);
        /**
         * @brief Process a synchronized set of IMU and image measurements.
        */
        //ROS subscribers, publishers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        

        // vio
        // data sync class
}
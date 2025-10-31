#ifndef VIO_NODE
#define VIO_NODE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <queue>
#include <eigen3/Eigen/Dense>
#include "../include/core/VioEstimator.hpp"
#include "../utils/measure_time.hpp"
#include "std_msgs/msg/int32.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
using namespace std;

class VioNode : public rclcpp::Node
{
    public:

        VioNode();
        ~VioNode();
        /**
         * @brief Process a synchronized set of IMU and image measurements
        */
        // extract two images with the same t from two different topics
        void img_cb(const sensor_msgs::msg::Image::ConstSharedPtr& cam0_msg, const sensor_msgs::msg::Image::ConstSharedPtr& cam1_msg);
    private:
        //ROS subscribers, publishers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu0;
        message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam0;
        message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam1;

        // vio
        // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr blue_dots_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr green_dots_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr red_dots_pub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;

        rclcpp::TimerBase::SharedPtr pub_timer;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        // stereo img synchronizer
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync;

        VioEstimator estimator;
        cv::Mat disparityMap;
        cv::Mat depthMap;

        StopWatch stopwatch;
        double elasped_time;
        double time;
        // a buffer to hold incoming sensor msgs from ROS2 subscribers
        queue<sensor_msgs::msg::Image::SharedPtr> img0_buf;
        queue<sensor_msgs::msg::Image::SharedPtr> img1_buf;
        queue<sensor_msgs::msg::Imu::SharedPtr> imu0_buf;
        std::mutex m_buf;
        queue<pair<double, Eigen::Vector3d>> accBuf;
        queue<pair<double, Eigen::Vector3d>> gyrBuf;
        // rclcpp::TimerBase::SharedPtr timer_;
        void publish_odometry();
        /**
         * @brief Put IMU info and push them into a queue(buffer)
        */
        void imu0_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif
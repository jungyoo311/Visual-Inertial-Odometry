#ifndef VIO_NODE
#define VIO_NODE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <nav_msgs/msg/odometry.hpp>
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

using namespace std;

// shared ptrs to msgs input
sensor_msgs::msg::Image ImageMsg;
sensor_msgs::msg::Imu ImuMsg;

class VioNode : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new VIO Ros node
        */
        VioNode();
        ~VioNode();
        /**
         * @brief Process a synchronized set of IMU and image measurements
        */
        // extract two images with the same t from two different topics
        void image_syncer(const sensor_msgs::msg::Image::ConstSharedPtr& cam0_msg, const sensor_msgs::msg::Image::ConstSharedPtr& cam1_msg);
    private:
        //ROS subscribers, publishers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu0;
        message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam0;
        message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam1;

        // vio
        // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
        
        // stereo img synchronizer
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync;

        VioEstimator estimator;
        cv::Mat disparityMap;
        cv::Mat depthMap;

        double time;
        // a buffer to hold incoming sensor msgs from ROS2 subscribers
        queue<sensor_msgs::msg::Image::SharedPtr> img0_buf;
        queue<sensor_msgs::msg::Image::SharedPtr> img1_buf;
        queue<sensor_msgs::msg::Imu::SharedPtr> imu0_buf;
        std::mutex m_buf;
        queue<pair<double, Eigen::Vector3d>> accBuf;
        queue<pair<double, Eigen::Vector3d>> gyrBuf;
        StopWatch stopwatch;
        double elasped_time;

        rclcpp::TimerBase::SharedPtr timer_;

        /**
         * @brief callback for incoming imu msgs
        */
        void imu0_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif
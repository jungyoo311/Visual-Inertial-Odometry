#ifndef VIO_NODE
#define VIO_NODE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <eigen3/Eigen/Dense>
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
    private:
        //ROS subscribers, publishers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam0;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam1;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu0;

        // vio
        rclcpp::Publisher<sensor_msgs::Image>::SharedPtr depth_map_pub;
        
        // stereo img synchronizer
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync;

        cv::Mat disparityMap;
        cv::Mat depthMap;
        /**
         * @brief callback for incoming img msgs
        */
        void image0_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void image1_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        /**
         * @brief callback for incoming imu msgs
        */
        void imu_callback(const sensor_msgs::msgs::Imu::SharedPtr msg);
        /**
         * @brief Process a synchronized set of IMU and image measurements
        */
        // extract two images with the same t from two different topics
        void image_syncer();
};

#endif
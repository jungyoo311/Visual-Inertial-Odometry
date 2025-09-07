#ifndef VIO_NODE_HPP
#define VIO_NODE_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>
#include <functional>
using namespace std::chrono_literals;

class VioNode : public rclcpp::Node
{
    public:
        VioNode();
        ~VioNode();
    private:
        double fx_;
        double fy_;
        double cx_;
        double cy_;
        double baseline;
        bool calibration_loaded;

        int window_size;
        int min_disp;
        int num_disp;
        
        cv::Ptr<cv::StereoSGBM> stereoSGBM;
        //subscriber
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
        // synced subscriber
        message_filters::Subscriber<sensor_msgs::msg::Image> left_gray_sub;
        message_filters::Subscriber<sensor_msgs::msg::Image> right_gray_sub;
        //publisher - vio
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vio_pub;
        //stereo img synchornizer
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync;

        void processCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void processVio(const cv::Mat& left_gray, const cv::Mat& right_gray);
        void processSync(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg, const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);
        
};

#endif
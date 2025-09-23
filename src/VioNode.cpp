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
}
void VioNode::image0_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    // ensures only one thread can access the buffers at a time. it automatically unlocks when function ends.
    std::lock_guard<std::mutex> lock(m_buf);
    img0_buf.push(msg);
}
void VioNode::image1_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_buf);
    img1_buf.push(msg);
}
/**
 * @brief input: ConstPtr?SharedPtr of img_msg
 * in case of using multiple datasets, check the encoding type-> convert to mono8(ROS2 preference)
 *
 */
cv::Mat VioNode::getImageFromMsg(sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    cv_bridge::CvImagePtr ImagePtr;
    
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.encoding = "mono8";
        img.data = img_msg->data;
        ImagePtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ImagePtr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img = ImagePtr->image.clone();
    return img;
}
/**
 * @brief supports only stereo img input. 0.003s(3ms) sync tolerance
 * @details calls getImageFromMsg() member function
 * 
 */
void VioNode::image_syncer()
{
    while(1)
    {
        cv::Mat img0, img1;
        double time = 0.0;
        std_msgs::msg::Header header;
        std::lock_guard<std::mutex> lock(m_buf);
        if (!img0_buf.empty() && !img1_buf.empty())
        {
            // compare timestamps with allowable 3ms tolerance
            double time0 = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
            double time1 = img1_buf.front()->header.stamp.sec + img1_buf.front()->header.stamp.nanosec * (1e-9);
            if (time0 < time1 - 0.003)
            {
                // drop img0
                img0_buf.pop();
            }
            else if(time0 > time1 + 0.003)
            {
                // drop img1
                img1_buf.pop();
            }
            else
            {
                time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                header = img0_buf.front()->header;
                img0 = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
                img1 = getImageFromMsg(img1_buf.front());
                img1_buf.pop();
            }
        }
        
        //TODO: if !image0.empty() -> call estimator here
        estimator.inputImage(time, img0, img1);

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
// Timer callback to publish messages
void VioNode::publisher_timer_callback()
{
    // Check if the buffers have messages to process
    if (img0_buf.empty() || img1_buf.empty()) {
        RCLCPP_INFO(this->get_logger(), "Buffers are empty, not publishing.");
        return;
    }
    
    // Create and publish a dummy image message
    auto image_msg_to_publish = img1_buf.front();
    img1_buf.pop();
    img_pub->publish(*image_msg_to_publish);
}
// constructor
VioNode::VioNode() : Node("VIO_estimator")
{
    // pre-integration means integrate imu0 first?
    sub_imu0 = this -> create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)), 
                                                                    std::bind(&VioNode::imu0_callback, this, std::placeholders::_1));
    //create subscription here - cam0, cam1, imu0
    sub_cam0 = this->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), 
                                                                    std::bind(&VioNode::image0_callback, this, std::placeholders::_1));
    sub_cam1 = this->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), 
                                                                    std::bind(&VioNode::image1_callback, this, std::placeholders::_1));
    // Create a publisher for estimated pose messages
    // pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("vio/pose", 10);
    img_pub = this->create_publisher<sensor_msgs::msg::Image>("vio/pose", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&VioNode::publisher_timer_callback, this)
    );
}
// deallocate
VioNode::~VioNode(){};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VioNode>();
    std::thread sync_thread(&VioNode::image_syncer, node.get());
    rclcpp::spin(node);
       
    
    rclcpp::shutdown();
    return 0;
}
#ifndef VIOESTIMATOR_HPP
#define VIOESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/odometry.hpp>
class VioEstimator
{
    public:
        VioEstimator();
        ~VioEstimator();
        cv::Mat inputImage(double t, const cv::Mat &img0, const cv::Mat &img1);
        void setMask(const cv::Mat& image);
        void detectAndAdd(const cv::Mat& image, cv::Mat& BGR_img);
        /**
         * @brief 
         * @return a structured list of all features currently being tracked
         */
        cv::Mat trackImage(const double time, const cv::Mat& new_image_left, const cv::Mat& new_image_right);
        struct FeatureStats
        {
            int blue_count = 0;
            int green_count = 0;
            int red_count = 0;
        };

        const std::vector<FeatureStats>& getStatsHistory() const { return stats_history;} //getter
        FeatureStats getLastFeatureStats() const;
        // push it to the queue
        void inputIMU(double t, const Eigen::Vector3d &linearAccel, const Eigen::Vector3d &angularVel);
        // Euler equation: calculate orientation, velocity, position
        void integrateIMU(double t, const Eigen::Vector3d &linearAccel, const Eigen::Vector3d &angularVel);
        //getters
        Eigen::Vector3d getPosition() const { return pos; }
        Eigen::Quaterniond getOrientation() const { return quat; }
        Eigen::Vector3d getVelocity() const { return vel; }
        void setCalibration(const cv::Mat& P0, const cv::Mat& P1);
        const std::vector<cv::Point3f>& getWorldPoints() const { return m_world_pts; }
    private:
        int inputImageCnt;
        double prev_time;
        double curr_time;
        struct FeatureObject // entire struct is the data-type
        {
            std::vector<cv::Point2f> prev_pts, curr_pts;
            std::vector<int> track_cnt; // quality level
            std::vector<int> ids;
        };
        FeatureObject obj;
        int max_cnt = 100;
        double minDistance = 20; //min possible Euclidean distance between the returned corners
        bool hasPrediction;
        cv::Mat m_mask;
        cv::Mat prev_img;
        int n_id = 0;
        bool is_first_frame = true;
        std::vector<FeatureStats> stats_history;

        //triangulation parameters
        cv::Mat cam0_proj_mat;
        cv::Mat cam1_proj_mat;
        std::vector<cv::Point3f> m_world_pts; //3D point clouds
        // usage for imu integration stage 2: imu preintegration
        // std::queue<Eigen::Vector3d> accBuf;
        // std::queue<Eigen::Vector3d> gyrBuf;
        
        // imu stage 1 integration values
        Eigen::Vector3d pos;
        Eigen::Quaterniond quat;
        Eigen::Vector3d vel;
        double latest_t;
        double delta_t;
        
        
        Eigen::Matrix3d R_t;
        Eigen::Vector3d a_t, b_a, g{0, 0, 9.81};
        
        Eigen::Vector3d w_t, b_g;
        bool is_imu_initialized = false;
        
};

#endif
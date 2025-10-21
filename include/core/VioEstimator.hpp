#ifndef VIOESTIMATOR_HPP
#define VIOESTIMATOR_HPP

#include <thread>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

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
};

#endif
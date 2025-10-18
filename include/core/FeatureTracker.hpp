#ifndef FEATURE_TRACKER
#define FEATURE_TRACKER

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <map>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <queue>
#include <vector>
#include "../utils/measure_time.hpp"
class FeatureTracker
{
    public:
    /**
     * @brief maintain a consistent set of tracked feature points 
     *          across consecutive stereo image frames.
     */
        FeatureTracker();
        ~FeatureTracker();
        
        /**
         * @brief Maintain Feature Quality and Distribution. Prioritizes keeping features that have been tracked for a long time.
         *  
         */
        void setMask(const cv::Mat& image);
        void detectAndAdd(const cv::Mat& image, cv::Mat& BGR_img);
        double distance(cv::Point2f &pt1, cv::Point2f &pt2);
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

        const std::vector<FeatureStats>& getStatsHistory() const { return stats_history;}
    private:
        struct FeatureObject // entire struct is the data-type
        {
            std::vector<cv::Point2f> prev_pts, curr_pts;
            std::vector<int> track_cnt; // quality level
            std::vector<int> ids;
            // std::vector<uchar> status;
            // std::vector<float> err;
        };
        FeatureObject obj;
        // std::map<int, FeatureObject> m_tracked_features; // holds feature currently being tracked.
        // int m_next_feature_id;
        int max_cnt = 150;
        double minDistance = 30; //min possible Euclidean distance between the returned corners
        double prev_time;
        double curr_time;
        bool hasPrediction;
        cv::Mat m_mask;
        cv::Mat prev_img;
        int n_id = 0;
        bool is_first_frame = true;
        std::vector<FeatureStats> stats_history;
};
#endif

// Each methods should be: len(function) < 100
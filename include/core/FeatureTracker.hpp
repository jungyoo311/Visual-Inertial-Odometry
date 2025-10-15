#ifndef FEATURE_TRACKER
#define FEATURE_TRACKER

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <map>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <queue>

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
        cv::Mat detectAndAdd(const cv::Mat& image);
        double distance(cv::Point2f &pt1, cv::Point2f &pt2);
        /**
         * @brief 
         * @return a structured list of all features currently being tracked
         */
        cv::Mat trackImage(const double time, const cv::Mat& new_image_left, const cv::Mat& new_image_right);

    private:
        struct FeatureObject // entire struct is the data-type
        {
            std::vector<cv::Point2f> prev_pts, curr_pts;
            std::vector<int> track_cnt; // quality level
            std::vector<int> ids;
            bool prev_consistent;
            bool curr_consistent;
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
        int n_id = 0;
};
#endif

// Each methods should be: len(function) < 100
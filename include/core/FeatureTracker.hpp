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
        struct FeatureObject
        {
            cv::Point2f pt_left;
            cv::Point2f pt_right;
        };
        using TrackData = std::map<int, FeatureObject>;
        void setMask(const cv::Mat& image);
        cv::Mat detectAndAdd(const cv::Mat& image);
        double distance(cv::Point2f &pt1, cv::Point2f &pt2);
        /**
         * @brief 
         * @return a structured list of all features currently being tracked
         */
        cv::Mat trackImage(const double time, const cv::Mat& new_image_left, const cv::Mat& new_image_right);
        /**
         * @brief Find and Add new FeaturePoints in areas where it weren't tracking yet.
         *          Assign new unique ID and add them to the tracker's list of managed pts.
         * @return add new features to the m_tracked_features, increment the m_next_feature_id counter 
         *          for each new feature added.
         */
        

    private:
        std::vector<cv::Point2f> prev_pts, curr_pts;
        std::map<int, FeatureObject> m_tracked_features; // holds feature currently being tracked.
        std::map<int, FeatureObject> new_feature_tracked;
        int m_next_feature_id;
        int MAX_FEATURES; // the max number of features want to track in total.
        double prev_time;
        double curr_time;
        bool hasPrediction;
        cv::Mat m_mask;
        
        
        //Testing command
};
#endif

// tracker 의 가장 기본 적인 구성요소 + 그걸 짜기 위해선 뭐가 필요? + member function이 뭐가 필요? + 내가 짜보자.
// 하나의 메서드 len(function) < 100
#ifndef VIOESTIMATOR_HPP
#define VIOESTIMATOR_HPP

#include <thread>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "FeatureTracker.hpp"

class VioEstimator
{
    public:
        VioEstimator();
        ~VioEstimator();
        cv::Mat inputImage(double t, const cv::Mat &img0, const cv::Mat &img1);
        void inputIMU(double t, const Eigen::Vector3d &linearAcc, const Eigen::Vector3d &angularVel);
        // void slideWindow();
        FeatureTracker::FeatureStats getLastFeatureStats() const;

    private:
        std::mutex mBuf;
        std::queue<Eigen::Vector3d> accBuf;
        std::queue<Eigen::Vector3d> gyrBuf;
        int inputImageCnt;
        // Testing cmd
        double curr_time;
        FeatureTracker tracker;
};

#endif
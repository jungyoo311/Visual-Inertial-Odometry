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
        void inputImage(double t, const cv::Mat &img0, const cv::Mat &img1);

    private:
        std::mutex mBuf;
        std::queue<Eigen::Vector3d> accBuf;
        std::queue<Eigen::Vector3d> gyrBuf;
        int inputImageCnt;
};

#endif
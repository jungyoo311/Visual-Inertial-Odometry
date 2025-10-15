#include "../include/core/VioEstimator.hpp"

VioEstimator::VioEstimator()
{
    // printf("entered estimator");
}
VioEstimator::~VioEstimator(){}
/**
 * @brief input stereo imgs and calls featureTracker
 */
cv::Mat VioEstimator::inputImage(double t, const cv::Mat &img0, const cv::Mat &img1)
{
    inputImageCnt++;
    // call FeatureTracker
    
    cv::Mat visualized_image;
    visualized_image = tracker.trackImage(t, img0, img1);
    
    return visualized_image;

    // What method i will use for extracting the features from the left/right imgs?: 
    // printf("inside of VioEstimator::inputImage");
}

void VioEstimator::inputIMU(double t, const Eigen::Vector3d &linearAcc, const Eigen::Vector3d &angularVel)
{
    // accBuf.push();
    // gyrBuf.push();
}
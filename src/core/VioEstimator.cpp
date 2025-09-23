#include "../include/core/VioEstimator.hpp"

VioEstimator::VioEstimator()
{
    // printf("entered estimator");
}
VioEstimator::~VioEstimator(){}
/**
 * @brief input stereo imgs and calls featureTracker
 */
void VioEstimator::inputImage(double t, const cv::Mat &img0, const cv::Mat &img1)
{
    inputImageCnt++;
    // call FeatureTracker
    // printf("inside of VioEstimator::inputImage");
    // RCLCPP_INFO(this->get_logger(), "inside of VioEstimator::inputImage")
    //  printf("inside of VioEstimator::inputImage");
}
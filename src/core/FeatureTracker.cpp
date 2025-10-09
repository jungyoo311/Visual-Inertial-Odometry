#include "../../include/core/FeatureTracker.hpp"


//constructor
FeatureTracker::FeatureTracker()
{
    
}
//deallocate
FeatureTracker::~FeatureTracker(){}
// void FeatureTracker::addPoints()
// {
//     for (auto &p : n_pts)
//     {
//         curr_pts.push_back(p);
//         ids.push_back(n_id++);
//         track_cnt.push_back(1);
//     }
// }

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

cv::Mat FeatureTracker::trackImage(double curr_time, const cv::Mat& img0, const cv::Mat& img1)
{
    cv::Mat curr_img;
    int row, col;
    curr_img = img0;
    row = curr_img.rows;
    col = curr_img.cols;
    cv::Mat rightImg = img1;
    curr_pts.clear();
    // if (prev_pts.size() > 0)
    // { 
    //     //track existing features
    //     // use LK to track pts from pre_left -> new_left
    //     cv::calcOpticalFlowPyrLK(
    //         prevImg, // not specified
    //         nextImg, // not specified
    //         prevPts, // not specified
    //         nextPts, // not specified
    //         status,
    //         err,
    //         winSize=(21,21),
    //         maxLevel=3, // max pyr level num
    //     )
    // }
    setMask(img0);
    // if no existing feature pts
    
    
    return detectAndAdd(img0);
}
/**
 * @brief region of interest(ROI), input CV_8UC1
 * @return Array of ROI
 */
void FeatureTracker::setMask(const cv::Mat& image)
{
    // m_mask is same size as image, initialized as zeros
    m_mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    cv::Point top_left_corner(image.cols / 4, image.rows / 4);
    cv::Point bottom_right_corner(image.cols * 3 / 4, image.rows * 3 / 4);
    cv::Rect ROI(top_left_corner, bottom_right_corner);
    cv::rectangle(m_mask, ROI, cv::Scalar(255), cv::FILLED);
    /**
     1. initialize clean state in all white.
     2. bundle feature data for sorting
     3. prioritize by sorting
     4. prepare to rebuild the feature list
     5. iterate, select, and mask out
     */
}
cv::Mat FeatureTracker::detectAndAdd(const cv::Mat& image)
{
    std::vector<cv::Point2f> corners;
    int maxCorners = 100;
    double qualityLevel = 0.01; // should be fixed. not hardcoded
    double minDistance = 10; //min possible Euclidean distance between the returned corners

    cv::goodFeaturesToTrack(
        image, //inputArray image
        corners, // outputArray corners
        maxCorners,
        qualityLevel, // corners less than X eigenValues are rejected.
        minDistance,
        m_mask, // make sure mask it NOT empty 
        3, // blockSize=
        false, //useHarrisDetector=
        0.04 // k
    );
    for(int i = 0; i < corners.size(); ++i)
    {
        FeatureObject new_obj;
        new_obj.pt_left = corners[i];
        new_obj.pt_right = cv::Point2f(-1.0);
        m_tracked_features[m_next_feature_id] = new_obj;
        m_next_feature_id++;
        cv::circle(image, new_obj.pt_left, 5, cv::Scalar(0,0,255), -1);
    }

    hasPrediction = true;
    return image;
}
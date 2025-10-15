#include "../../include/core/FeatureTracker.hpp"

FeatureTracker::FeatureTracker(){}
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

    setMask(img0);
    
    return detectAndAdd(img0);
}
/**
 * @brief region of interest(ROI), input CV_8UC1
 * track_cnt: from the most stable feature to the least stable. - Quality Level
*      
* 1. starts with a blank white mask
* 2. for each feature it checks its location on the mask is still black
* 3. if the spot is blank, that feature is a "keeper". survives the filter. To claim its territory, 
*      it marks a circuclar area around itself on the mask as "taken" - colors black
* 4. If the spot is already taken, it means a more stable feature was too close and claimed the area first.
*      it's a "dropper" and is discarded.
 */
void FeatureTracker::setMask(const cv::Mat& image)
{
    // step 1: create a vector of features from the map for sorting
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
    for (int i = 0; i < cnt_pts_id.size(); i++)
        cnt_pts_id.push_back(std::make_pair(obj.track_cnt[i], std::make_pair(obj.curr_pts[i], obj.ids[i])));
    // step 2: sort the features by track_cnt in descending order
    // lambda function: return true if the track_count of element a is greater than the track count of element b.
    // this tells the sort algorithm to place the elements with higher track counts earlier in the vector, 
    // resulting in a sort in descending order based on track_cnt
    std::sort(cnt_pts_id.begin(), cnt_pts_id.end(),[](const std::pair<int, std::pair<cv::Point2f, int>> &a, const std::pair<int, std::pair<cv::Point2f, int>> &b) {
        return a.first > b.first; // return true if the track_cnt of elem a > b
    });
    // step 3: create a blank mask and a new map for "keeper" features
    m_mask = cv::Mat(image.rows, image.cols, CV_8UC1, cv::Scalar(255));
    obj.curr_pts.clear();
    obj.ids.clear();
    obj.track_cnt.clear();
    // step 4: Run the competition for space
    for(auto const&iter : cnt_pts_id)
    {
        if (m_mask.at<uchar>(iter.second.first) == 255)
        {
            // this feature is keeper -> update member variables
            obj.curr_pts.push_back(iter.second.first);
            obj.ids.push_back(iter.second.second);
            obj.track_cnt.push_back(iter.first);
            cv::circle(m_mask, iter.second.first, minDistance, 0, -1);
        }
    }
    // stpe 5: Replace the old feature map with the curated "keeper" map

}
cv::Mat FeatureTracker::detectAndAdd(const cv::Mat& image)
{
    std::vector<cv::Point2f> corners;
    double qualityLevel = 0.01;
    cv::goodFeaturesToTrack(
        image, //inputArray image
        corners, // outputArray corners
        max_cnt,
        qualityLevel, // corners less than X eigenValues are rejected.
        minDistance,
        m_mask, // make sure mask it NOT empty 
        3, // blockSize=
        false, //useHarrisDetector=
        0.04 // k
    );
    cv::Mat BGR_img;
    cv::cvtColor(image, BGR_img, cv::COLOR_GRAY2BGR);
    for(const auto& p: corners)
    {
        obj.curr_pts.push_back(p);
        obj.ids.push_back(n_id++); // assign new unique id
        obj.track_cnt.push_back(1); // new features have a track count of 1
        cv::circle(BGR_img, p, 5, cv::Scalar(0,255,0), -1); // dot projection drawing
    }

    hasPrediction = true;
    return BGR_img;
}
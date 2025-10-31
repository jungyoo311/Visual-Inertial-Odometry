#include "../include/core/VioEstimator.hpp"

VioEstimator::VioEstimator() :
    latest_t(0),
    delta_t(0),
    pos(Eigen::Vector3d::Zero()),
    quat(Eigen::Quaterniond::Identity()),
    vel(Eigen::Vector3d::Zero()),
    R_t(Eigen::Matrix3d::Identity()),
    b_a(Eigen::Vector3d::Zero()),
    b_g(Eigen::Vector3d::Zero())
{}
VioEstimator::~VioEstimator(){}
/**
 * @brief input stereo imgs and calls featureTracker
 */
cv::Mat VioEstimator::inputImage(double t, const cv::Mat &img0, const cv::Mat &img1)
{
    inputImageCnt++;
    cv::Mat visualized_image;
    visualized_image = trackImage(t, img0, img1);
    
    return visualized_image;
}

cv::Mat VioEstimator::trackImage(double curr_time, const cv::Mat& img0, const cv::Mat& img1)
{
    cv::Mat BGR_img;
    cv::cvtColor(img0, BGR_img, cv::COLOR_GRAY2BGR);
    cv::Mat right_img = img1;
    FeatureStats current_stats;

    if (is_first_frame)
    {
        //first frame only detect features
        is_first_frame = false;
        prev_img = img0.clone();
        setMask(img0);
        detectAndAdd(img0, BGR_img);
        return BGR_img;
    }
    
    std::vector<cv::Point2f> next_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    std::vector<cv::Point2f> prev_pts_for_drawing = obj.curr_pts; // save old points for drawing
    size_t points_before_add = 0;

    //calculate optical flow
    cv::calcOpticalFlowPyrLK(
        prev_img,
        img0,
        obj.curr_pts,
        next_pts,
        status,
        err
    );

    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i]) {
            // Prev(True) Curr(True): Green-dots
            cv::circle(BGR_img, next_pts[i], 5, cv::Scalar(0, 255, 0), -1);
            current_stats.green_count++; // count GREEN
        } else {
            // Prev(True) Curr(False): Red-dots
            cv::circle(BGR_img, prev_pts_for_drawing[i], 5, cv::Scalar(0, 0, 255), -1);
            current_stats.red_count++; // count RED
        }
    }

    //clean up points that were lost
    std::vector<cv::Point2f> surviving_pts;
    std::vector<int> surviving_ids;
    std::vector<int> surviving_track_cnt;

    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i]) {
            surviving_pts.push_back(next_pts[i]);
            surviving_ids.push_back(obj.ids[i]);
            surviving_track_cnt.push_back(obj.track_cnt[i] + 1);
        }
    }
    obj.curr_pts = surviving_pts;
    obj.ids = surviving_ids;
    obj.track_cnt = surviving_track_cnt;

    // 4. Filter survivors and then add new features (which will be drawn in blue)
    setMask(img0);
    points_before_add = obj.curr_pts.size();
    detectAndAdd(img0, BGR_img);
    current_stats.blue_count = obj.curr_pts.size() - points_before_add; // count BLUE

    // 5. Update state for the next cycle
    prev_img = img0.clone();
    stats_history.push_back(current_stats);
    
    return BGR_img;
}
/**
 * @brief region of interest(ROI), input CV_8UC1
 * track_cnt: from the most stable feature to the least stable. - Quality Level
 *      
 */
void VioEstimator::setMask(const cv::Mat& image)
{
    // step 1: create a vector of features from the map for sorting
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
    for (int i = 0; i < obj.curr_pts.size(); i++)
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
}
void VioEstimator::detectAndAdd(const cv::Mat& image, cv::Mat& BGR_img)
{
    if (max_cnt > obj.curr_pts.size())
    {
        std::vector<cv::Point2f> corners;
        double qualityLevel = 0.01;
        //corner detection
        cv::goodFeaturesToTrack(
            image, //inputArray image
            corners, // outputArray corners
            max_cnt - obj.curr_pts.size(),
            qualityLevel, // corners less than X eigenValues are rejected.
            minDistance,
            m_mask, // make sure mask it NOT empty 
            3, // blockSize=
            false, //useHarrisDetector=
            0.04 // k
        );
        
        for(const auto& p: corners)
        {
            obj.curr_pts.push_back(p);
            obj.ids.push_back(n_id++); // assign new unique id
            obj.track_cnt.push_back(1); // new features have a track count of 1
            cv::circle(BGR_img, p, 5, cv::Scalar(255,0,0), -1); // BLUE dot projection drawing
        }
    }
}

VioEstimator::FeatureStats VioEstimator::getLastFeatureStats() const
{
    const auto& history = getStatsHistory();
    if(history.empty())
    {
        return VioEstimator::FeatureStats();
    }
    return history.back();
}

void VioEstimator::integrateIMU(double t, const Eigen::Vector3d &linearAccel, const Eigen::Vector3d &angularVel)
{
    if (!is_imu_initialized)
    {
        // first frame processing
        latest_t = t;
        is_imu_initialized = true;
        return;
    }
    delta_t = t - latest_t;
    latest_t = t;
    
    Eigen::Vector3d unbiased_gyro = angularVel - b_g;
    Eigen::Vector3d unbiased_accel = linearAccel - b_a;
    double angle = unbiased_gyro.norm() * delta_t;
    Eigen::Vector3d axis = (angle > 1e-9) ? (unbiased_gyro * delta_t / angle) : Eigen::Vector3d(0, 0, 1);
    Eigen::AngleAxisd angle_axis(angle, axis);
    Eigen::Quaterniond delta_q(angle_axis);
    //calculate orientation
    // quat = quat @ Eigen::Quaterniond::exp(0.5 * (w_t - b_g) * delta_t);
    quat  = (quat * delta_q).normalized();
    R_t = quat.toRotationMatrix();

    Eigen::Vector3d a_world = R_t * unbiased_accel - g;

    //calculate position
    pos = pos + vel*delta_t + 0.5 * a_world * (delta_t) * (delta_t);

    //calculate velocity
    vel = vel + a_world * delta_t;
    
}

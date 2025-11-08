#include "../include/core/VioEstimator.hpp"

VioEstimator::VioEstimator() :
    latest_t(0),
    delta_t(0),
    pos(Eigen::Vector3d::Zero()),
    quat(Eigen::Quaterniond::Identity()),
    vel(Eigen::Vector3d::Zero()),
    R_t(Eigen::Matrix3d::Identity()),
    b_a(Eigen::Vector3d::Zero()),
    b_g(Eigen::Vector3d::Zero()),
    cam0_proj_mat(cv::Mat::zeros(3,4,CV_64F)),
    cam1_proj_mat(cv::Mat::zeros(3,4,CV_64F))
{}
VioEstimator::~VioEstimator(){}

void VioEstimator::setCalibration(const cv::Mat& P0, const cv::Mat& P1)
{
    P0.copyTo(cam0_proj_mat);
    P1.copyTo(cam1_proj_mat);
}

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

    //clear the prev 3d pts
    m_world_pts.clear();

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

    std::vector<cv::Point2f> surviving_pts_left;
    std::vector<int> surviving_ids;
    std::vector<int> surviving_track_cnt;

    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i]) {
            surviving_pts_left.push_back(next_pts[i]);
            surviving_ids.push_back(obj.ids[i]);
            surviving_track_cnt.push_back(obj.track_cnt[i] + 1);
        }
    }
    std::vector<cv::Point2f> surviving_pts_right;
    std::vector<uchar> stereo_status;

    // stereo optical flow: left img to right img
    cv::calcOpticalFlowPyrLK(
        img0, // prev_img? img0?
        right_img,
        surviving_pts_left,
        surviving_pts_right,
        stereo_status,
        err
    );

    std::vector<cv::Point2f> final_survivor_pts;
    std::vector<int> final_survivor_ids;
    std::vector<int> final_survivor_track_cnt;
    
    std::vector<cv::Point2f> triangulated_pts_left;
    std::vector<cv::Point2f> triangulated_pts_right;

    std::vector<cv::Point3f> points_3d; // store 3d pts from triangulation


    for (size_t i = 0; i < stereo_status.size(); i++)
    {
        if(stereo_status[i])
        {
            // this pts exist in img0, img1
            final_survivor_pts.push_back(surviving_pts_left[i]);
            final_survivor_ids.push_back(surviving_ids[i]);
            final_survivor_track_cnt.push_back(surviving_track_cnt[i]);

            //triangulation here or outside? option 1
            triangulated_pts_left.push_back(surviving_pts_left[i]);
            triangulated_pts_right.push_back(surviving_pts_right[i]);
        }
    }
    // triangulation here? option 2
    if (!triangulated_pts_left.empty())
    {
        //output 4d matrix
        cv::Mat pts_4d;
        cv::triangulatePoints(
            cam0_proj_mat,
            cam1_proj_mat, // 3x4 projection matrices
            triangulated_pts_left,
            triangulated_pts_right,
            pts_4d // 4x4 output matrix
        );
        //convert 4D -> 3D by deviding by w
        for (int i = 0; i < pts_4d.cols; i++)
        {
            cv::Mat pt_4d = pts_4d.col(i);
            float w = pt_4d.at<float>(3,0); // get w
            if(std::abs(w) > 1e-6)
            {
                cv::Point3f pt_3d(
                    pt_4d.at<float>(0,0)/w,
                    pt_4d.at<float>(1,0)/w,
                    pt_4d.at<float>(2,0)/w
                );
                m_world_pts.push_back(pt_3d);
            }
        }
    }

    obj.curr_pts = final_survivor_pts;
    obj.ids = final_survivor_ids;
    obj.track_cnt = final_survivor_track_cnt;

    // 4. Filter survivors and then add new features (which will be drawn in blue)
    setMask(img0);
    points_before_add = obj.curr_pts.size();
    detectAndAdd(img0, BGR_img);
    current_stats.blue_count = obj.curr_pts.size() - points_before_add; // count BLUE
    //surviving pts are compared with right detection results.

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
        Eigen::Vector3d measured_accel_norm = linearAccel.normalized();
        Eigen::Vector3d world_gravity_norm = g.normalized();
        quat = Eigen::Quaterniond::FromTwoVectors(measured_accel_norm, world_gravity_norm);
        R_t = quat.toRotationMatrix();
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
    quat  = (quat * delta_q).normalized();
    R_t = quat.toRotationMatrix();
    Eigen::Vector3d a_world = R_t * unbiased_accel - g;
    //calculate position
    pos = pos + vel*delta_t + 0.5 * a_world * (delta_t) * (delta_t);
    //calculate velocity
    vel = vel + a_world * delta_t;
}

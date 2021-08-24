#include "frontend.h"
#include "config.h"

Frontend::Frontend(const std::string& configFilePath)
{
    Config::setParameterFile(configFilePath);
    gftt_ = cv::GFTTDetector::create(Config::getParameter<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::getParameter<int>("num_features_init");
    num_features_ = Config::getParameter<int>("num_features");
    // camera_left_ = Camera::Ptr(new Camera(configFilePath, 0));

    frame_curr_ = Frame::Ptr(new Frame());
    frame_last_ = Frame::Ptr(new Frame());

    // viewer_ = Viewer::Ptr(new Viewer("testMyslam"));
}

void Frontend::addFrameCurr(Frame::Ptr newFrame)
{
    frame_curr_ = newFrame;

    switch (status_)
    {
        case FrontendStatus::INITING:
            stereoInit();
            break;
        case FrontendStatus::TRACKING_1:
        case FrontendStatus::TRACKING_0:
            LKTrack();
            break;
        case FrontendStatus::TRACKING_X:
            break;
        default:
            break;
    }

    frame_last_ = frame_curr_;
}

bool Frontend::stereoInit()
{
    int num_features_left = detectFeatures();
    if (num_features_left < num_features_init_)
        return false;

    // int num_features_righ = findFeaturesInRight();
    // if (num_features_righ < num_features_init_)
    //     return false;
    status_ = TRACKING_1;
    return true;
}

void Frontend::LKTrack()
{
    /**
     *  1. detect features of current frame
     *  2. compute PyreLK flow
     *  3. send result to viewer model
     */
    std::vector<cv::Point2f> kps_last;
    std::vector<cv::Point2f> kps_curr;
    for (Feature::Ptr& kp : frame_last_->featuresLeft_)
        kps_last.push_back(kp->framePose_.pt);
    
    // test ----------------------------------------------------------------------------
    // detectFeatures();
    // LOG(INFO) << "\n"
    //             << "frame_curr_: " << frame_curr_->featuresLeft_.size() << std::endl
    //             << "frame_last_: " << frame_last_->featuresLeft_.size() << std::endl;
    // return;
    // test ----------------------------------------------------------------------------

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        frame_last_->imageLeft_, frame_curr_->imageLeft_, kps_last,
        kps_curr, status, error);
    
    int num_good_points = 0;
    for (size_t i = 0; i < kps_curr.size(); ++i)
    {
        if (status[i])
        {
            cv::KeyPoint newkp(kps_curr[i], 7);
            Feature::Ptr newfe(new Feature(frame_curr_, newkp));
            newfe->mappoint_.reset();
            frame_curr_->featuresLeft_.push_back(newfe);
            num_good_points++;
        }
    }

    num_features_tracking_ = num_good_points;
    if (num_features_tracking_ < num_features_inlier_)
    {
        detectFeatures();
        LOG(INFO) << "IN Frontend::LKTrack():\n" << " new keyframe ID: " << frame_curr_->id_ << std::endl;
    }
    
    LOG(INFO) << "IN Frontend::LKTrack():\n" << num_good_points << " features found in last frame\n";

    // draw result and send to viewer
    cv::Mat result = frame_curr_->imageLeft_.clone();
    int size_kps_curr = kps_curr.size();
    for (int i  = 0; i < size_kps_curr; ++i)
    {
        if (status[i])
        {
            cv::circle(result, kps_curr[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(result, kps_last[i], kps_curr[i], cv::Scalar(0, 250, 0));
        }
    }
    if (viewer_)
        viewer_->addFrameCurr(result);
}

int Frontend::detectFeatures()
{
    cv::Mat mask(frame_curr_->imageLeft_.size(), CV_8UC1, 255);
    for (Feature::Ptr& kp : frame_curr_->featuresLeft_)
    {
        // mark known area
        cv::rectangle(
            mask, 
            kp->framePose_.pt - cv::Point2f(10.0f, 10.0f),
            kp->framePose_.pt + cv::Point2f(10.0f, 10.0f),
            0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(frame_curr_->imageLeft_, keypoints, mask);
    int cnt_detected = 0;
    for (auto& kp : keypoints)
    {
        frame_curr_->featuresLeft_.push_back(
            Feature::Ptr(new Feature(frame_curr_, kp))
        );
        cnt_detected++;
    }

    LOG(INFO) << "IN Frontend::detectFeatures():\n" << cnt_detected << " new features detected\n";
    return cnt_detected;
}

int Frontend::findFeaturesInRight()
{
    // not implemented yet
    // require class_map
    return 0;
}

void Frontend::closeViewer()
{
    if (viewer_ != nullptr)
        viewer_->viewerClose();
}

// settings
bool Frontend::setViewer(Viewer::Ptr viewer)
{
    if (viewer == nullptr)
        return false;
    viewer_ = viewer;
    return true;
}
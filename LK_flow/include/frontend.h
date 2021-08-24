#ifndef FRONTEND_H
#define FRONTEND_H

#include "common.h"
#include "camera.h"
#include "frame.h"
#include "viewer.h"

class Frontend
{
public:
    typedef std::shared_ptr<Frontend> Ptr;
private:
    // status
    enum FrontendStatus {INITING, TRACKING_1, TRACKING_0, TRACKING_X};
    FrontendStatus status_ = INITING;
    // camera
    // Camera::Ptr camera_left_ = nullptr;
    // viewer
    Viewer::Ptr viewer_ = nullptr;

    // OpenCV utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv

    // other parameters
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_inlier_ = 20;
    int num_featuers_needed_for_keyframe_ = 80;

public:
    /**
     *  @brief 
     *  frames are set to public for the sake of
     *  the convenience in acquiring the frames,
     *  by viewer thread.
     */
    Frame::Ptr frame_curr_ = nullptr;
    Frame::Ptr frame_last_ = nullptr;

public:
    Frontend() {}
    Frontend(const std::string& configFilePath);

    bool setFrameCurr(const Frame::Ptr& frame);
    void addFrameCurr(Frame::Ptr newFrame);
    bool stereoInit();

    void LKTrack();
    int detectFeatures();
    int findFeaturesInRight();

    void closeViewer();


    bool setCamera(Camera::Ptr camLeft, Camera::Ptr camRigh);
    bool setViewer(Viewer::Ptr viewer);
};

#endif
#ifndef FRONTEND_H
#define FRONTEND_H

#include "common.h"
#include "camera.h"
#include "frame.h"

class Frontend
{
public:
    typedef std::shared_ptr<Frontend> Ptr;
private:
    // status
    enum FrontendStatus {INITING, TRACKING_1, TRACKING_0, TRACKING_X};
    FrontendStatus runStatus_ = INITING;
    // camera
    Camera::Ptr camera_left_ = nullptr;

    // OpenCV utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
    cv::Mat error_;
    std::vector<uchar> lkStatus_;
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
    Frontend();
    Frontend(const std::string& configFilePath);

    /**
     * @brief new frame->current frame
     *        last frame->current frame
     * @param frame 
     * @param flag 0: original img 1: gray scale img
     */
    bool initFrameCurr();
    bool setFrameCurr(const Frame::Ptr& frame);
    void addFrameCurr();
    /**
     * @brief use cv::GFTTDetector to detect LK
     *        features
     */
    void LKTrack();
    void detectFeatures(Frame::Ptr& frame);
    /**
     * @brief Set the Camera object
     * 
     * @param camLeft 
     * @param camRigh 
     * @return true 
     * @return false 
     */
    bool setCamera(Camera::Ptr camLeft, Camera::Ptr camRigh);
};

#endif
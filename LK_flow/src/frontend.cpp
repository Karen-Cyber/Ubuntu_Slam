#include "frontend.h"
#include "config.h"

Frontend::Frontend()
{
    gftt_ = cv::GFTTDetector::create(Config::getParameter<int>("num_features"), 0.01, 20);
}

Frontend::Frontend(const std::string& configFilePath)
{
    gftt_ = cv::GFTTDetector::create(Config::getParameter<int>("num_features"), 0.01, 20);
    camera_left_ = Camera::Ptr(new Camera(configFilePath, 0));
}

bool Frontend::initFrameCurr()
{
    // init new frame
    frame_curr_ = Frame::Ptr(new Frame);
    // assign image to frame_curr_
    camera_left_->newFrame(frame_curr_);

    return true;
}

void Frontend::addFrameCurr()
{
    camera_left_->newFrame(frame_curr_);

    switch (runStatus_)
    {
        case INITING:
            frame_last_ = frame_curr_;
            break;
        case TRACKING_1:
        case TRACKING_0:
            LKTrack();
            break;
        case TRACKING_X:
            break;
    }

    frame_last_ = frame_curr_;
}

void Frontend::LKTrack()
{
    /**
     *  1. detect features of current frame
     *  2. compute PyreLK flow
     *  3. send result to viewer model
     */

}

void Frontend::detectFeatures(Frame::Ptr& frame)
{

}
#ifndef FRAME_H
#define FRAME_H

#include "common.h"

class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    cv::Mat imageLeft_;
    cv::Mat imageRigh_;
    std::vector<cv::KeyPoint> featuresLeft_;
    std::vector<cv::KeyPoint> featuresRight_;

    Frame() {}
    ~Frame() {}
};

#endif
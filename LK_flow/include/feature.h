#ifndef FEATURE_H
#define FEATURE_H

#include "common.h"
struct Frame;
struct MapPoint;

struct Feature
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;
    std::weak_ptr<MapPoint> mappoint_;
    cv::KeyPoint framePose_;

    bool outlier = false;
    bool onImageLeft = true;

    Feature() {}
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& kp)
    : frame_(frame), framePose_(kp) {}
};

#endif
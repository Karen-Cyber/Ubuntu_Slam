#ifndef FRAME_H
#define FRAME_H

#include "common.h"
#include "feature.h"

struct Frame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;
    bool isKeyFrame_ = false;
    cv::Mat imageLeft_;
    cv::Mat imageRigh_;
    std::vector<Feature::Ptr> featuresLeft_;
    std::vector<Feature::Ptr> featuresRigh_;

    Frame() {}
    Frame(unsigned long id);
    ~Frame() {}

    /// 工厂构建模式，分配id 
    static std::shared_ptr<Frame> createFrame();
};

#endif
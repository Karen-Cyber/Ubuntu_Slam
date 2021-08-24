#ifndef VISUAL_ODEMETRY_H
#define VISUAL_ODEMETRY_H

#include "common.h"
#include "frontend.h"
#include "config.h"

class VisualOdometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;
private:
    std::string configFilePath_;

    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_righ_ = nullptr;
    Frontend::Ptr frontend_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

public:
    VisualOdometry(const std::string& configFilePath);
    bool init();
    bool step();
    void run();
};

#endif
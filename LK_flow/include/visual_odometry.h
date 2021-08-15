#ifndef VISUAL_ODEMETRY_H
#define VISUAL_ODEMETRY_H

#include "common.h"
#include "frontend.h"

class VisualOdometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
    bool inited_ = false;
    std::string configFilePath_;

    Frontend::Ptr frontend = nullptr;


};

#endif
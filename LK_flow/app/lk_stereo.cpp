#include <gtest/gtest.h>

#include "common.h"
#include "visual_odometry.h"

const std::string configFilePath = "/home/karenfu/VisualSlam_Study/Projects_nogit/LK_flow/config/default.yaml";

int main(int argc, char** argv)
{
    VisualOdometry::Ptr testVisual = VisualOdometry::Ptr(new VisualOdometry(configFilePath));
    if (!testVisual->init())
        LOG(INFO) << "VisualSlam initialized failed\n";
    
    testVisual->run();

    return 0;
}
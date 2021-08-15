#include <gtest/gtest.h>

#include "common.h"
#include "config.h"
#include "camera.h"
#include "frontend.h"

const std::string configFilePath = "/home/karenfu/VisualSlam_Study/Projects_nogit/LK_flow/config/default.yaml";

TEST(MyslamTest, class_Config)
{
    EXPECT_TRUE(Config::setParameterFile(configFilePath));
    EXPECT_NEAR(Config::getParameter<float>("camera_fx"), 507.9, 1.0);
}

TEST(MyslamTest, class_Camera)
{
    Camera::Ptr testCamera = Camera::Ptr(new Camera(configFilePath, 0));
    cv::Mat testFrame;
    testCamera->newFrameLorR(testFrame, 0);

    EXPECT_GT(testFrame.rows, 0);
    EXPECT_GT(testFrame.cols, 0);
    EXPECT_NEAR(testFrame.rows, 480, 1.0);
    EXPECT_NEAR(testFrame.cols, 640, 1.0);
}

TEST(MyslamTest, class_Frontend)
{
    Frontend::Ptr testFrontend = Frontend::Ptr(new Frontend(configFilePath));
    testFrontend->initFrameCurr();

    EXPECT_GT(testFrontend->frame_curr_->image_.rows, 0);
    EXPECT_GT(testFrontend->frame_curr_->image_.cols, 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
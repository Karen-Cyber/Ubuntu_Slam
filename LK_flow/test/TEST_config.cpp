#include <gtest/gtest.h>

#include "common.h"
#include "config.h"
#include "camera.h"
#include "frontend.h"
#include "viewer.h"

const std::string configFilePath = "/home/karenfu/VisualSlam_Study/Projects_nogit/LK_flow/config/default.yaml";

// TEST(MyslamTest, class_Config)
// {
//     EXPECT_TRUE(Config::setParameterFile(configFilePath));
//     EXPECT_NEAR(Config::getParameter<float>("camera_fx"), 507.9, 1.0);
// }

// TEST(MyslamTest, class_Camera)
// {
//     Camera::Ptr testCamera = Camera::Ptr(new Camera(configFilePath, 0));
//     Frame::Ptr testFrame = Frame::Ptr(new Frame());
//     testCamera->newFrame(testFrame);

//     EXPECT_GT(testFrame->imageLeft_.rows, 0);
//     EXPECT_GT(testFrame->imageLeft_.cols, 0);
//     EXPECT_GT(testFrame->imageRigh_.rows, 0);
//     EXPECT_GT(testFrame->imageRigh_.cols, 0);
//     EXPECT_NEAR(testFrame->imageLeft_.rows, 480, 1.0);
//     EXPECT_NEAR(testFrame->imageLeft_.cols, 640, 1.0);
//     EXPECT_NEAR(testFrame->imageRigh_.rows, 480, 1.0);
//     EXPECT_NEAR(testFrame->imageRigh_.cols, 640, 1.0);
// }

// TEST(MyslamTest, class_Viewer)
// {
//     Viewer::Ptr testViewer = Viewer::Ptr(new Viewer("test"));
//     // cv::waitKey(0);
//     testViewer->viewerClose();
// }

// TEST(MyslamTest, class_Frontend)
// {
//     Frontend::Ptr testFrontend = Frontend::Ptr(new Frontend(configFilePath));
//     testFrontend->addFrameCurr();
//     for (int i = 0; i < 10; ++i)
//     {
//         // requires stereo camera connected
//         testFrontend->addFrameCurr();
//     }
//     testFrontend->closeViewer();

//     EXPECT_GT(testFrontend->frame_curr_->imageLeft_.rows, 0);
//     EXPECT_GT(testFrontend->frame_curr_->imageRigh_.cols, 0);
// }

int main(int argc, char** argv)
{
    {
        Frontend::Ptr testFrontend = Frontend::Ptr(new Frontend(configFilePath));
        /**
         *  @brief 
         *  if you want to get a real camera, then,
         *  remember to set the parameter video = 1
         */
        Camera::Ptr testCamera = Camera::Ptr(new Camera(configFilePath, 0, 1));
        Viewer::Ptr testViewer = Viewer::Ptr(new Viewer("testMyslam"));
        testFrontend->setViewer(testViewer);
        while (true)
        {
            Frame::Ptr testFrame = Frame::createFrame();
            testCamera->newFrame(testFrame);
            testFrontend->addFrameCurr(testFrame);
        }
        testFrontend->closeViewer();
    }
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
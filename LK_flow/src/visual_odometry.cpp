#include "visual_odometry.h"

VisualOdometry::VisualOdometry(const std::string& configFilePath)
: configFilePath_(configFilePath)
{

}

bool VisualOdometry::init()
{
    if (!Config::setParameterFile(configFilePath_))
        return false;
    frontend_ = Frontend::Ptr(new Frontend(configFilePath_));
    camera_left_ = Camera::Ptr(new Camera(configFilePath_, 0, 1));
    // no need for camera right yet
    viewer_ = Viewer::Ptr(new Viewer("testMyslam"));
    if (!frontend_->setViewer(viewer_))
        return false;

    return true;
}

bool VisualOdometry::step()
{
    auto t1 = std::chrono::steady_clock::now();
    Frame::Ptr newFrame = Frame::createFrame();
    camera_left_->newFrame(newFrame);
    frontend_->addFrameCurr(newFrame);
    auto t2 = std::chrono::steady_clock::now();
    auto td = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "IN VisualOdometry::step():\n" << "time used: " << td.count() << "s\n";

    return true;
}

void VisualOdometry::run()
{
    while (true)
    {
        if (!step())
            break;
    }

    frontend_->closeViewer();
}
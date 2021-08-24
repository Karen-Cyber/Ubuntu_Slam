#include "viewer.h"

Viewer::Viewer() : winName_("EMPTY")
{
    viewer_running_ = true;
    frame_curr_ = cv::Mat(cv::Size(640, 480), CV_8UC1, 255);
    viewer_thread_ = std::thread(std::bind(&Viewer::viewerLoop, this));
}

Viewer::Viewer(const std::string winName) : winName_(winName)
{
    viewer_running_ = true;
    frame_curr_ = cv::Mat(cv::Size(640, 480), CV_8UC1, 255);
    viewer_thread_ = std::thread(std::bind(&Viewer::viewerLoop, this));
}


void Viewer::addFrameCurr(cv::Mat frame)
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    frame_curr_ = frame.clone();
}

void Viewer::showTrackCurr() const
{

}

void Viewer::viewerLoop()
{
    while (viewer_running_)
    {
        // what's wrong with this statement?
        // uncomment this to find the problem...
        // std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (frame_curr_.data)
        {
            cv::imshow(winName_, frame_curr_);
            cv::waitKey(1);
        }
        // 5000 us = 0.005s
        usleep(5000);
    }

    LOG(INFO) << "Viewer thread exit\n";
}

void Viewer::viewerClose()
{
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    viewer_running_ = false;
    viewer_thread_.join();
}
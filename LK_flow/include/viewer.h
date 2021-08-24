#ifndef VIEWER_H
#define VIEWER_H

#include "common.h"

class Viewer
{
private:
    // todo
    std::thread viewer_thread_;
    bool viewer_running_ = false;

    cv::Mat frame_curr_;

    std::mutex viewer_data_mutex_;

public:
    typedef std::shared_ptr<Viewer> Ptr;
    std::string winName_;

    Viewer();
    Viewer(const std::string winName);
    ~Viewer() {}

    void addFrameCurr(cv::Mat frame);
    void showTrackCurr() const;
    void viewerLoop();
    void viewerClose();
};

#endif
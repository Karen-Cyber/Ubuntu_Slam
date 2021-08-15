#ifndef VIEWER_H
#define VIEWER_H

#include "common.h"

class Viewer
{
private:
    // todo

public:
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer() {}
    ~Viewer() {}

    void showFrame(const cv::Mat& frame, const std::string winName);
};

#endif
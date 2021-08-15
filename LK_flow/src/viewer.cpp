#include "viewer.h"

void Viewer::showFrame(const cv::Mat& frame, const std::string winName)
{
    cv::namedWindow(winName);
    cv::imshow(winName, frame);
}
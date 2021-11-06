#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
using namespace cv;

#include <iostream>
#include <cstring>
#include <fstream>
#include <fmt/core.h>
#include <fmt/format.h>
using namespace std;

int main(int argc, char** argv)
{
    cv::Mat image_test = cv::imread("../images/left0.jpg");
    cv::imshow("test", image_test);
    cv::imwrite("test.jpg", image_test);
    cv::waitKey(0);
}
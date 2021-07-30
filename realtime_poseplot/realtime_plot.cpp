#include <iostream>
#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pangolin/pangolin.h>

/**
 * @brief You will need a stereo camera to run this
 * 
 * @param frame the origin frame from stereo camera
 * @param img_l img_l will hold the seperate image of left eye
 * @param img_r img_r will hold the seperate image of right eye
 * @param img_d depth of pixels will be compute
 * @param sgbm  OpenCV tool to compute the depth
 */
void comput_disparity(
    const cv::Mat& frame,
    cv::Mat& img_l,
    cv::Mat& img_r,
    cv::Mat& img_d, 
    const cv::Ptr<cv::StereoSGBM>& sgbm
);

void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
);


int main(int argc, char** argv)
{
    // Camera initialization
    if (argc != 2)
    {
        cout << "usage: realtime_plot <device_num>\n";
        return -1;
    }
    int device_num;
    while (*argv[1] != '\0')
    {
        device_num = int(*argv[1] - 48);
        argv[1]++;
    }
    cv::VideoCapture cap(device_num);
    if (!cap.isOpened())
    {
        cout << "Error opening camera\n";
        return -2;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    // Some OpenCV preparations
    cv::Mat img_f;
    cv::Mat img_l;
    cv::Mat img_r;
    cv::Mat img_d;
    cv::Mat img_1;
    cv::Mat img_2;
    // I have no idea how to decide on these parameters
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32
    );
    vector<cv::KeyPoint> keypoints_1;
    vector<cv::KeyPoint> keypoints_2;
    vector<cv::DMatch>   matches;
    while (true)
    {
        cap >> img_f;
        comput_disparity(img_f, img_l, img_r, img_d, sgbm);
        cv::imshow("disparity", img_d);
        if (cv::waitKey(30) == 27) break;
    }

    return 0;
}

void comput_disparity(
    const cv::Mat& frame,
    cv::Mat& img_l,
    cv::Mat& img_r,
    cv::Mat& img_d,
    const cv::Ptr<cv::StereoSGBM>& sgbm
)
{
    img_l = frame(cv::Rect(0, 0, frame.size().width / 2, frame.size().height));
    img_r = frame(cv::Rect(frame.size().width / 2, 0, frame.size().width / 2, frame.size().height));
    sgbm->compute(img_l, img_r, img_d);
    img_d.convertTo(img_d, CV_32F, 1.0f / 16.0f);
}

void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
)
{

}
#include <iostream>
#include <vector>
#include <string>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

void pose_estimation_2d2d(
    const vector<cv::KeyPoint> keypoints_1,
    const vector<cv::KeyPoint> keypoints_2,
    vector<cv::DMatch>& matches,
    cv::Mat& R, cv::Mat& t
);

void feature_detect_match(
    vector<cv::KeyPoint>& keypoints_1,
    vector<cv::KeyPoint>& keypoints_2,
    vector<cv::DMatch>& matches
);

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "usage: mono_pose path_to_datasets\n";
        return -1;
    }
    string filePath = argv[1];

    // OpenCV prepare
    vector<cv::KeyPoint> keypoints_1;
    vector<cv::KeyPoint> keypoints_1;
    cv::Mat descriptions_1;
    cv::Mat descriptions_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    cv::Mat img1;
    cv::Mat img2;

    for (int i = 1; i < 7; ++i)
    {
        img1 = cv::imread(filePath + to_string(i) + ".jpg", CV_LOAD_IMAGE_COLOR);
        img2 = cv::imread(filePath + to_string(i + 1) + ".jpg", CV_LOAD_IMAGE_COLOR);
        
    }

    return 0;
}

void pose_estimation_2d2d(
    const vector<cv::KeyPoint> keypoints_1,
    const vector<cv::KeyPoint> keypoints_2,
    vector<cv::DMatch>& matches,
    cv::Mat& R, cv::Mat& t
)
{
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1250.7, 0.0, 330.2, 0.0, 1250.7, 312.0, 0.0, 0.0, 1.0);

    vector<cv::Point2f> points_1;
    vector<cv::Point2f> points_2;

}

void feature_detect_match(
    vector<cv::KeyPoint>& keypoints_1,
    vector<cv::KeyPoint>& keypoints_2,
    vector<cv::DMatch>& matches
)
{
    
}
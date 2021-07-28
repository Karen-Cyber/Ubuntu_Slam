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
        cv::imshow("test", img2);
        cv::waitKey(0);
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
    for (int i = 0; i < int(matches.size()); ++i)
    {
        points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points_2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    cv::Mat essential = cv::findEssentialMat(
        points_1,
        points_2,
        1250.7,
        cv::Point2d(330.2, 312.0));
    cv::recoverPose(essential, points_1, points_2, R, t, 1250.7, cv::Point2d(330.2, 312.0));

    cout << "R is:\n" << R << endl;
    cout << "t is:\n" << t << endl;
}

void feature_detect_match(
    const cv::Mat& img1,
    const cv::Mat& img2,
    vector<cv::KeyPoint>& keypoints_1,
    vector<cv::KeyPoint>& keypoints_2,
    vector<cv::DMatch>& matches
)
{
    cv::Mat descriptions_1;
    cv::Mat descriptions_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img1, keypoints_1);
    detector->detect(img2, keypoints_2);
    extractor->compute(img1, keypoints_1, descriptions_1);
    extractor->compute(img2, keypoints_2, descriptions_2);
    vector<cv::DMatch> pre_matches;
    matcher->match(descriptions_1, descriptions_2, pre_matches);

    // Find the minimum distance
    double min_dist = 10000.0;
    for (int i = 0; i < descriptions_1.rows; ++i)
        if (pre_matches[i].distance < min_dist)
            min_dist = pre_matches[i].distance;
    // Filter bad matches
    for (int i = 0; i < descriptions_1.rows; ++i)
        if (pre_matches[i].distance <= max(1.5 * min_dist, 25.0))
            matches.push_back(pre_matches[i]);
}
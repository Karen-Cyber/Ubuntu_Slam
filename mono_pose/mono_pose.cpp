#include <iostream>
#include <fstream>
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
    const cv::Mat& img1,
    const cv::Mat& img2,
    vector<cv::KeyPoint>& keypoints_1,
    vector<cv::KeyPoint>& keypoints_2,
    vector<cv::DMatch>& matches
);

void cvR_eiR(const cv::Mat& cvR, Eigen::Matrix3d& eiR);
void cvt_eit(const cv::Mat& cvt, Eigen::Vector3d& eit);

void transform_record(const cv::Mat& cvR, const cv::Mat& cvt, ofstream& fout);

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "usage: mono_pose path_to_datasets\n";
        return -1;
    }
    string filePath = argv[1];
    ofstream write_trajectory("trajectory.txt");
    ofstream write_transformt("transformt.txt");

    // OpenCV prepare
    vector<cv::KeyPoint> keypoints_1;
    vector<cv::KeyPoint> keypoints_2;
    vector<cv::DMatch> matches;
    cv::Mat descriptions_1;
    cv::Mat descriptions_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    cv::Mat img1;
    cv::Mat img2;
    cv::Mat R;
    cv::Mat t;

    // Eigen Prepare
    Eigen::Matrix3d eiR;
    Eigen::Vector3d eit;
    Eigen::Vector3d eiv(0, 0, 0);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    for (int i = 1; i < 8; ++i)
    {
        img1 = cv::imread(filePath + "/" + to_string(i)     + ".jpg", CV_LOAD_IMAGE_COLOR);
        img2 = cv::imread(filePath + "/" + to_string(i + 1) + ".jpg", CV_LOAD_IMAGE_COLOR);
        feature_detect_match(img1, img2, keypoints_1, keypoints_2, matches);
        pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
        cvR_eiR(R, eiR);
        cvt_eit(t, eit);
        T.rotate(eiR);
        T.pretranslate(eit);
        eiv = T * eiv;
        cout << "Transformation:\n" << T.matrix() << endl;
        cout << "Current position:\n" << eiv << endl;
        write_trajectory << eiv.transpose() << endl;
        write_transformt << T.matrix() << endl;
    }

    write_trajectory.close();
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

void cvR_eiR(const cv::Mat& cvR, Eigen::Matrix3d& eiR)
{
    eiR <<  cvR.at<double>(0, 0), cvR.at<double>(0, 1), cvR.at<double>(0, 2),
            cvR.at<double>(1, 0), cvR.at<double>(1, 1), cvR.at<double>(1, 2),
            cvR.at<double>(2, 0), cvR.at<double>(2, 1), cvR.at<double>(2, 2);
}
void cvt_eit(const cv::Mat& cvt, Eigen::Vector3d& eit)
{
    eit <<  cvt.at<double>(0, 0), cvt.at<double>(0, 1), cvt.at<double>(0, 2);
}

void transform_record(const cv::Mat& cvR, const cv::Mat& cvt, ofstream& fout)
{
    Eigen::Matrix3d eiR;
    cvR_eiR(cvR, eiR);
    Eigen::Quaterniond eiQ(eiR);
    fout << eiQ.coeffs().transpose() << ' ';
    fout << cvt.at<double>(0, 0) << ' ' << cvt.at<double>(0, 1) << ' ' << cvt.at<double>(0, 2) << endl;
}
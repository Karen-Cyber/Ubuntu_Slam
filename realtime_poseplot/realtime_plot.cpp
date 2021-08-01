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

// Function related variables
double min_dist = 30.0;
cv::Mat descriptors_1;
cv::Mat descriptors_2;
// Used in OpenCV3
cv::Ptr<cv::FeatureDetector>     detector   = cv::ORB::create();
cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
cv::Ptr<cv::DescriptorMatcher>   matcher    = cv::DescriptorMatcher::create("BruteForce-Hamming");
void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
);

void pose_estimation_3d3d(
    const vector<cv::Point3f>& points_1,
    const vector<cv::Point3f>& points_2,
    Eigen::Matrix3d& eiR,
    Eigen::Vector3d& eit
);

void pose_estimation_2d2d(
    const vector<cv::KeyPoint>& keypoints_1,
    const vector<cv::KeyPoint>& keypoints_2,
    const vector<cv::DMatch>& matches,
    cv::Mat& cvR,
    cv::Mat& cvt
);

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& cvK);

// -----------------------------------------------------------------

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
    // Camera intrinsics
    Eigen::Matrix3d eiK;
    cv::Mat cvK;
    eiK <<  5.0791239611052424e+02, 0.0,                    3.3133318255234428e+02, 
            0.0,                    4.8299256896714132e+02, 2.4492723215900779e+02, 
            0.0,                    0.0,                    1.0;
    cvK = (
        cv::Mat_<double>(3, 3) << 
            5.0791239611052424e+02, 0.0,                    3.3133318255234428e+02, 
            0.0,                    4.8299256896714132e+02, 2.4492723215900779e+02, 
            0.0,                    0.0,                    1.0); 

    // Some OpenCV preparations
    cv::Mat img_f;
    cv::Mat img_l;
    cv::Mat img_r;
    cv::Mat img_d;
    // I have no idea how to decide on these parameters
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32
    );
    vector<cv::Point3f>  points3d_1;
    vector<cv::Point3f>  points3d_2;
    vector<cv::KeyPoint> keypoints_1;
    vector<cv::KeyPoint> keypoints_2;
    vector<cv::DMatch>   matches;

    // Transformation matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d eiR;
    Eigen::Vector3d eit;
    cv::Mat cvR;
    cv::Mat cvt;

    // Start real time plot
    // Reference preparations
    cv::Mat ref_l;
    cv::Mat ref_d;
    cap >> img_f;
    comput_disparity(img_f, img_l, img_r, img_d, sgbm);
    ref_l = img_l.clone();
    ref_d = img_d;
    cout << img_d << endl;
    cout << "Reference image\n";
    cv::imshow("test", img_d / float(96.0 * 16.0));
    cv::waitKey(0);

    while (true)
    {
        cap >> img_f;
        comput_disparity(img_f, img_l, img_r, img_d, sgbm);
        // cv::imshow("disparity", img_d / float(96.0 * 16.0));
        cv::imshow("ref_l", ref_l);
        cv::imshow("img_l", img_l);

        // Start matching 3D-pairs
        find_feature_matches(img_l, ref_l, keypoints_1, keypoints_2, matches);

        // for (cv::DMatch& m : matches)
        // {
        //     unsigned short d1 = img_d.ptr<unsigned short>
        //         (int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        //     unsigned short d2 = ref_d.ptr<unsigned short>
        //         (int(keypoints_2[m.queryIdx].pt.y))[int(keypoints_2[m.queryIdx].pt.x)];
        //     if (d1 <= 0 || d2 <= 0) continue;
        //     cv::Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, cvK);
        //     cv::Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, cvK);
        //     points3d_1.push_back(cv::Point3f(p1.x * d1, p1.y * d1, d1));
        //     points3d_2.push_back(cv::Point3f(p2.x * d2, p2.y * d2, d2));
        // }
        // cout << "3d-3d pairs: " << points3d_1.size() << endl;
        // // pose_estimation_3d3d
        // pose_estimation_3d3d(points3d_2, points3d_1, eiR, eit);

        // pose_estimation_2d2d
        pose_estimation_2d2d(keypoints_1, keypoints_2, matches, cvR, cvt);
        cout << "------------\n";
        cout << "R:\n";
        cout << cvR << endl;
        cout << "t:\n";
        cout << cvt << endl;

        // Refresh
        matches.clear();
        keypoints_1.clear();
        keypoints_2.clear();
        // points3d_1.clear();
        // points3d_2.clear();
        // Move forward
        
        // ref_d = img_d;

        if (cv::waitKey(10) == 97) break;
        ref_l = img_l.clone();
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
    img_d.convertTo(img_d, CV_32F, 1.0f);
    // img_d /= 96.0;
}

void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
)
{
    // 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<cv::DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);
    
    for (int i = 0; i < descriptors_1.rows; ++i)
    {
        if (match[i].distance <= min_dist)
            matches.push_back(match[i]);
    }
}

// Bad output ???
void pose_estimation_3d3d(
    const vector<cv::Point3f> &points_1,
    const vector<cv::Point3f> &points_2,
    Eigen::Matrix3d& eiR,
    Eigen::Vector3d& eit
)
{
    cv::Point3f p1, p2;     // center of mass
    int N = points_1.size();
    for (int i = 0; i < N; i++) 
    {
        p1 += points_1[i];
        p2 += points_2[i];
    }
    p1 = cv::Point3f(cv::Vec3f(p1) / N);
    p2 = cv::Point3f(cv::Vec3f(p2) / N);
    vector<cv::Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) 
    {
        q1[i] = points_1[i] - p1;
        q2[i] = points_2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) 
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    // cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // cout << "U=" << U << endl;
    // cout << "V=" << V << endl;

    eiR = U * (V.transpose());
    if (eiR.determinant() < 0) 
    {
        eiR = -eiR;
    }
    eit = Eigen::Vector3d(p1.x, p1.y, p1.z) - eiR * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    // R = (cv::Mat_<double>(3, 3) <<
    //     R_(0, 0), R_(0, 1), R_(0, 2),
    //     R_(1, 0), R_(1, 1), R_(1, 2),
    //     R_(2, 0), R_(2, 1), R_(2, 2)
    // );
    // t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

void pose_estimation_2d2d(
    const vector<cv::KeyPoint>& keypoints_1,
    const vector<cv::KeyPoint>& keypoints_2,
    const vector<cv::DMatch>& matches,
    cv::Mat& cvR,
    cv::Mat& cvt
)
{
    vector<cv::Point2f> points_1;
    vector<cv::Point2f> points_2;

    for (int i = 0; i < matches.size(); ++i)
    {
        points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points_2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    cv::Mat cvE = cv::findEssentialMat(points_1, points_2, 507.912f, cv::Point2d(331.3, 244.9));
    cv::recoverPose(cvE, points_1, points_2, cvR, cvt, 507.9, cv::Point2d(331.3, 244.9));
}

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& cvK)
{
    return cv::Point2d(
        (p.x - cvK.at<double>(0, 2)) / cvK.at<double>(0, 0),
        (p.y - cvK.at<double>(1, 2)) / cvK.at<double>(1, 1)
    );
}
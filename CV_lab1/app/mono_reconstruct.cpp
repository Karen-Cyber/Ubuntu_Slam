#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
using namespace std;

#include <dirent.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/visualization/cloud_viewer.h>

// global constant
cv::Mat intrinsics(cv::Matx33d(
    3.0556046899380012e+03, 0.0, 1.9106976138108419e+03,
    0.0, 3.0556046899380012e+03, 1.5624986258910974e+03,
    0.0, 0.0, 1.0
    )); 

cv::Mat distortion = (
    cv::Mat_<double>(5, 1) << 
    4.0882896544766414e-02,
    -3.6673376986422540e-01,
    1.5228838795122823e-02,
    1.1493987917927448e-02,
    1.3983214034297489e+00
    );

cv::FileStorage parameter_file;
void set_parameter_file(const string& file_name)
{
    parameter_file = cv::FileStorage(file_name.c_str(), cv::FileStorage::READ);
    if (!parameter_file.isOpened())
    {
        cerr << "parameter file not found.\n";
        exit(1);
    }
}

template<typename T>
T get_parameter(const string& key)
{
    return T(parameter_file[key]);
}


/**
 * @brief Get the name list object
 * 
 * @param dir_path the absolute path of directory containing images.
 * @param name_list the output array that stores the path or file names.
 * @param mode absolute path or only name of images, 1 for absolute and 0 for only file name.
 */
void get_name_list(string dir_path, vector<string>& name_list, int mode);

void feature_matching(
    const cv::Mat& img1,
    const cv::Mat& img2,
    vector<cv::KeyPoint>& keypoints1,
    vector<cv::KeyPoint>& keypoints2,
    vector<cv::DMatch>& matches
);

void solve_Essential_Fundamental(
    const vector<cv::KeyPoint>& keypoints1,
    const vector<cv::KeyPoint>& keypoints2,
    const vector<cv::DMatch>& matches,
    cv::Mat& E,
    cv::Mat& F,
    cv::Mat& R,
    cv::Mat& t
);

bool check_rotation_coherent(const cv::Mat& R);

cv::Point2f pixel2cam(const cv::Point2f &p, const cv::Mat& K);

void triangulation_impl1(
    const vector<cv::KeyPoint> keypoints1,
    const vector<cv::KeyPoint> keypoints2,
    const vector<cv::DMatch> matches,
    const cv::Mat& R,
    const cv::Mat& t,
    vector<cv::Point3d>& world_points 
);

void triangulation_impl2(
    const vector<cv::KeyPoint> keypoints1,
    const vector<cv::KeyPoint> keypoints2,
    const vector<cv::DMatch> matches,
    const cv::Mat& R,
    const cv::Mat& t,
    vector<cv::Point3d>& world_points 
);

int main(int argc, char** argv)
{
    // parse arguments
    if (argc != 2)
    {
        cerr << "usage: mono_reconstruct <parameter file path>\n";
        exit(1);
    }
    set_parameter_file(string(argv[1]));
    string base_dir = get_parameter<string>("dataset_dir");
    // read in images
    vector<string> image_paths;
    get_name_list(base_dir, image_paths, 1);
    vector<cv::Mat> images;
    double scale_factor = get_parameter<double>("scale_factor");
    for (const string& image_path : image_paths)
    {
        cv::Mat original = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        cv::Mat halfscal;
        cv::resize(original, halfscal, cv::Size(), scale_factor, scale_factor);
        images.push_back(halfscal);
        cv::imshow("test_image", halfscal);
        cv::waitKey(50);
    }

    // matching feature points
    vector<cv::KeyPoint> keypoints1;
    vector<cv::KeyPoint> keypoints2;
    vector<cv::DMatch> matches;
    feature_matching(
        images[get_parameter<int>("image1")], 
        images[get_parameter<int>("image2")], 
        keypoints1, keypoints2, matches
    );

    // recovering position
    cv::Mat E;
    cv::Mat F;
    cv::Mat R;
    cv::Mat t;
    solve_Essential_Fundamental(keypoints1, keypoints2, matches, E, F, R, t);
    check_rotation_coherent(R);
    // triangulate points
    vector<cv::Point3d> world_points;
    triangulation_impl1(keypoints1, keypoints2, matches, R, t, world_points);

    // visualization using PCL
    cout << "point cloud size: " << world_points.size() << endl;
    // for (const cv::Point3d point : world_points)
    //     cout << point.x << ' ' << point.y << ' ' << point.z << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    double metric = get_parameter<double>("metric");
    for (int i = 0; i < world_points.size(); ++i)
    {
        pcl::PointXYZRGB p;
        p.x = world_points[i].x * metric;
        p.y = world_points[i].y * metric;
        p.z = world_points[i].z * metric;
        p.r = int(p.x);
        p.g = int(p.y);
        p.b = int(p.z);
        pointcloud->points.push_back(p);
    }

    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(pointcloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
        get_parameter<int>("point_size"),
        "sample cloud"
    );
    viewer.spin();

    return 0;
}

void get_name_list(string dir_path, vector<string>& name_list, int mode)
{
    DIR* dir;
    if ((dir = opendir(dir_path.c_str())) == NULL)
    {
        cerr << "error opening directory...\n";
        exit(2);
    }

    dirent* entptr;
    while ((entptr = readdir(dir)) != NULL)
    {
        string absolute_path = dir_path;
        // current directory and upper directory 
        if (!strcmp(entptr->d_name, ".") || !strcmp(entptr->d_name, ".."))
            continue;
        // common file or link file
        if (entptr->d_type == 8 || 10)
        {
            absolute_path.append("/").append(string(entptr->d_name));
            if (mode)
                name_list.push_back(absolute_path);
            else
                name_list.push_back(string(entptr->d_name));
        }
        // sub-directory
        if (entptr->d_type == 4)
        {
            get_name_list(dir_path.append("/").append(string(entptr->d_name)), name_list, mode);
        }
    }

    // sorted by idx or date if they have
    sort(name_list.begin(), name_list.end());
}

void feature_matching(
    const cv::Mat& img1,
    const cv::Mat& img2,
    vector<cv::KeyPoint>& keypoints1,
    vector<cv::KeyPoint>& keypoints2,
    vector<cv::DMatch>& matches
)
{
    // used for counting time
    chrono::steady_clock::time_point STA;
    chrono::steady_clock::time_point END;

    cv::Mat descriptors1;
    cv::Mat descriptors2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    STA = chrono::steady_clock::now();

    // step1: detect the position of feature points
    detector->detect(img1, keypoints1);
    detector->detect(img2, keypoints2);
    // step2: compute description of feature points
    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);
    // step3: match feature points from two images
    vector<cv::DMatch> original_matches;
    matcher->match(descriptors1, descriptors2, original_matches);
    // step4: filter matched points by suppressing non-extremum[非极大值抑制]
    auto min_max = minmax_element(
        original_matches.begin(), 
        original_matches.end(), 
        [](const cv::DMatch& m1, const cv::DMatch& m2) { return m1.distance < m2.distance; }
    );
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    for (int i = 0; i < descriptors1.rows; ++i)
    {
        // good match
        if (original_matches[i].distance <= max(2.0 * min_dist, 10.0))
            matches.push_back(original_matches[i]);
    }

    END = chrono::steady_clock::now();
    cout << "matching stage cost " << chrono::duration_cast<chrono::duration<double>>(END - STA).count() << " second(s).\n";

    // step5: draw the matched results
    cv::Mat output;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, output);
    cv::imshow("matched points", output);
    cv::waitKey(0);
}

void solve_Essential_Fundamental(
    const vector<cv::KeyPoint>& keypoints1,
    const vector<cv::KeyPoint>& keypoints2,
    const vector<cv::DMatch>& matches,
    cv::Mat& E,
    cv::Mat& F,
    cv::Mat& R,
    cv::Mat& t
)
{
    vector<cv::Point2f> points_for_solve1;
    vector<cv::Point2f> points_for_solve2;
    for (int i = 0; i < int(matches.size()); ++i)
    {
        points_for_solve1.push_back(keypoints1[matches[i].queryIdx].pt);
        points_for_solve2.push_back(keypoints2[matches[i].trainIdx].pt);
    }

    // compute the fundamental matrix
    F = cv::findFundamentalMat(points_for_solve1, points_for_solve2, CV_FM_8POINT);
    cout << "[fundamental matrix]:\n" << F << endl;
    // using F to compute E
    cv::Point2d principal_point(1.9106976138108419e+03, 1.5624986258910974e+03);
    double focal_length = 3.0556046899380012e+03;
    E = cv::Mat(intrinsics.t()) * F * cv::Mat(intrinsics);
    // E = cv::findEssentialMat(points_for_solve1, points_for_solve2, focal_length, principal_point);
    cout << "[Essential   matrix]:\n" << E << endl;
    // recover rotation and translation
    cv::SVD svd(E);
    cv::Matx33d W(
        0.0, -1.0,  0.0,
        1.0,  0.0,  0.0,
        0.0,  0.0,  1.0
    );
    R = cv::Mat(cv::Mat_<double>(svd.u * cv::Mat(W) * svd.vt));
    t = svd.u.col(2);
    cout << "[Rotation    matrix]:\n" << R << endl;
    cout << "[Translation matrix]:\n" << t << endl;
    // epipolar constraint
    cv::Mat t_x = (
        cv::Mat_<double>(3, 3) << 
             0,                  -t.at<double>(2, 0),  t.at<double>(1, 0),
             t.at<double>(2, 0),  0,                  -t.at<double>(0, 0),
            -t.at<double>(1, 0),  t.at<double>(0, 0),  0
    );

    for (int i = 0; i < matches.size(); ++i)
    {
        cv::Point2f pt1 = pixel2cam(keypoints1[matches[i].queryIdx].pt, intrinsics);
        cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1.0);
        cv::Point2f pt2 = pixel2cam(keypoints2[matches[i].trainIdx].pt, intrinsics);
        cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1.0);
        cv::Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint of " << i + 1 << "matched pair: " << d << endl;
    }
}

bool check_rotation_coherent(const cv::Mat& R)
{
    if (fabsf(cv::determinant(R) - 1.0) > 1e-7)
    {
        cerr << "det(R) != +-1.0, not a rotation matrix.\n";
        return false;
    }
    return true;
}

void triangulation_impl1(
    const vector<cv::KeyPoint> keypoints1,
    const vector<cv::KeyPoint> keypoints2,
    const vector<cv::DMatch> matches,
    const cv::Mat& R,
    const cv::Mat& t,
    vector<cv::Point3d>& world_points 
)
{
    cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0
    );
    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );

    vector<cv::Point2f> points_for_solve1;
    vector<cv::Point2f> points_for_solve2;
    for (int i = 0; i < matches.size(); ++i)
    {
        points_for_solve1.push_back(pixel2cam(keypoints1[matches[i].queryIdx].pt, intrinsics));
        points_for_solve2.push_back(pixel2cam(keypoints2[matches[i].trainIdx].pt, intrinsics));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, points_for_solve1, points_for_solve2, pts_4d);

    for (int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        world_points.push_back(p);
    }
}

void triangulation_impl2(
    const vector<cv::KeyPoint> keypoints1,
    const vector<cv::KeyPoint> keypoints2,
    const vector<cv::DMatch> matches,
    const cv::Mat& R,
    const cv::Mat& t,
    vector<cv::Point3d>& world_points 
)
{
    cv::Matx34d T1(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0
    );
    cv::Matx34d T2(
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );

    cv::Mat K_inverse;
    cv::invert(intrinsics, K_inverse, cv::DECOMP_LU);
    for (int i = 0; i < matches.size(); ++i)
    {
        cv::Point3d kp1(
            keypoints1[matches[i].queryIdx].pt.x, 
            keypoints1[matches[i].queryIdx].pt.y,
            1.0
        );
        cv::Point3d kp2(
            keypoints2[matches[i].trainIdx].pt.x, 
            keypoints2[matches[i].trainIdx].pt.y,
            1.0
        );

        cv::Point3d tmp;
        tmp = cv::Point3d(cv::Mat_<double>(K_inverse * cv::Mat_<double>(kp1)));
        kp1 = tmp;
        tmp = cv::Point3d(cv::Mat_<double>(K_inverse * cv::Mat_<double>(kp2)));
        kp2 = tmp;

        // solve for world coordinate X
        cv::Matx34d A(
            kp1.x * T1(2, 0) - T1(0, 0), kp1.x * T1(2, 1) - T1(0, 1), kp1.x * T1(2, 2) - T1(0, 2),
            kp1.y * T1(2, 0) - T1(1, 0), kp1.y * T1(2, 1) - T1(1, 1), kp1.y * T1(2, 2) - T1(1, 2),
            kp2.x * T2(2, 0) - T2(0, 0), kp2.x * T2(2, 1) - T2(0, 1), kp2.x * T2(2, 2) - T2(0, 2),
            kp2.y * T2(2, 0) - T2(1, 0), kp2.y * T2(2, 1) - T2(1, 1), kp2.y * T2(2, 2) - T2(1, 2)
        );
        cv::Matx41d B(
            -(kp1.x * T1(2, 3) - T1(0, 3)),
            -(kp1.y * T1(2, 3) - T1(1, 3)),
            -(kp2.x * T2(2, 3) - T2(0, 3)),
            -(kp2.y * T2(2, 3) - T2(1, 3))
        );

        // this means A * X = B, and try to find vector X
        cv::Mat_<double> X;
        cv::solve(A, B, X, cv::DECOMP_SVD);

        world_points.push_back(cv::Point3d(X(0), X(1), X(2)));
    }

}

cv::Point2f pixel2cam(const cv::Point2f &p, const cv::Mat& K)
{
    return cv::Point2f
    (
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

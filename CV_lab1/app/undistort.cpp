#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
using namespace std;

#include <dirent.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

/**
 * @brief Get the name list object
 * 
 * @param dir_path the absolute path of directory containing images.
 * @param name_list the output array that stores the path or file names.
 * @param mode absolute path or only name of images, 1 for absolute and 0 for only file name.
 */
void get_name_list(string dir_path, vector<string>& name_list, int mode);

int main(int argc, char** argv)
{
    cv::Mat intrinsics(3, 3, CV_64F);
    intrinsics.at<double>(0, 0) = 3.0556046899380012e+03; 
    intrinsics.at<double>(0, 1) = 0.0; 
    intrinsics.at<double>(0, 2) = 1.9106976138108419e+03;
    intrinsics.at<double>(1, 0) = 0.0; 
    intrinsics.at<double>(1, 1) = 3.0556046899380012e+03; 
    intrinsics.at<double>(1, 2) = 1.5624986258910974e+03;
    intrinsics.at<double>(2, 0) = 0.0; 
    intrinsics.at<double>(2, 1) = 0.0; 
    intrinsics.at<double>(2, 2) = 1.0;

    // or use the following initialization
    // cv::Matx33d intrinsics(
    //     3.0556046899380012e+03, 0.0, 1.9106976138108419e+03,
    //     0.0, 3.0556046899380012e+03, 1.5624986258910974e+03,
    //     0.0, 0.0, 1.0
    // );

    cv::Mat distortion(5, 1, CV_64F);
    distortion.at<double>(0, 0) =  4.0882896544766414e-02;
    distortion.at<double>(1, 0) = -3.6673376986422540e-01;
    distortion.at<double>(2, 0) =  1.5228838795122823e-02;
    distortion.at<double>(3, 0) =  1.1493987917927448e-02;
    distortion.at<double>(4, 0) =  1.3983214034297489e+00;


    vector<string> image_paths;
    get_name_list(string(argv[1]), image_paths, 1);

    for (const string& image_path : image_paths)
    {
        cv::Mat original = cv::imread(image_path);
        cv::Mat undistor;

        cv::undistort(original, undistor, intrinsics, distortion);
        cv::resize(undistor, original, cv::Size(), 0.2, 0.2);
        cv::imshow("undistorted image", original);
        cv::waitKey(0);
    }

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
            cout << absolute_path << endl;
        }
        // sub-directory
        if (entptr->d_type == 4)
        {
            get_name_list(dir_path.append("/").append(string(entptr->d_name)), name_list, mode);
        }
    }
}
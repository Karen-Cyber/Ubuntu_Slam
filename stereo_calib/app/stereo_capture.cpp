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
    VideoCapture cap(stoi(argv[1]));
    if (!cap.isOpened())
    {
        cout << "ERROR::OPENNING::CAMERA\n";
        return -1;
    }
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    Mat img_double;
    int n = 0;
    bool recording = false;
    int interleave = 15; //ms

    while (true)
    {
        Mat frame;
		cap >> frame;
		Mat leftImage, rightImage;
        Mat leftGray, rightGray;
        //split left image
		leftImage = frame(Rect(0, 0, frame.size().width / 2, frame.size().height));
        //split right image
		rightImage = frame(Rect(frame.size().width / 2, 0, frame.size().width / 2, frame.size().height));
        cv::cvtColor(leftImage, leftGray, CV_BGR2GRAY);
        cv::cvtColor(rightImage, rightGray, CV_BGR2GRAY);

		if (waitKey(interleave) == 97)
            recording = true;
        if (waitKey(interleave) == 98)
            recording = false;
        if (waitKey(interleave) == 99)
        {
            if (leftImage.data && rightImage.data)
            {
                imwrite("../images/left"  + fmt::format("{0:0>6}", n) + ".png", leftGray);
                imwrite("../images/right" + fmt::format("{0:0>6}", n) + ".png", rightGray);
            }
            n++;
        }
        if (recording)
        {
            if (leftImage.data && rightImage.data)
            {
                imwrite("../myDatasets/image_0/" + fmt::format("{0:0>6}", n) + ".png", leftGray);
                imwrite("../myDatasets/image_1/" + fmt::format("{0:0>6}", n) + ".png", rightGray);
            }
            
            cv::putText(leftImage, 
                        "recording" + to_string(n++),
                        cv::Point(10, 20),
                        cv::FONT_HERSHEY_TRIPLEX, 
                        0.8, cv::Scalar(50, 255, 50), 2, CV_AA);
        }
        else
        {
            cv::putText(leftImage, 
                        "ready", 
                        cv::Point(10, 20), 
                        cv::FONT_HERSHEY_TRIPLEX, 
                        0.8, cv::Scalar(50, 255, 50), 2, CV_AA);
        }
        if (waitKey(interleave) == 27)
            break;
        
        imshow("leftImage", leftImage);//left image
		imshow("rightImage", rightImage);//right image
    }

    // generate a XML file to locate images
    ofstream outFile("../config/stereo_calib.xml");
    ofstream outLeft("../config/leftImageList.xml");
    ofstream outRigh("../config/rightImageList.xml");
    if (!outLeft.is_open() || !outFile.is_open() || !outRigh.is_open())
    {
        cout << "ERROR::WRITTING::CONFIGFILE\n";
        return -1;
    }
    outFile << "<?xml version=\"1.0\"?>" << endl
            << "<opencv_storage>" << endl
            << "<imagelist>" << endl;
    outLeft << "<?xml version=\"1.0\"?>" << endl
            << "<opencv_storage>" << endl
            << "<imagelist>" << endl;
    outRigh << "<?xml version=\"1.0\"?>" << endl
            << "<opencv_storage>" << endl
            << "<imagelist>" << endl;
    for (int i = 0; i < n; ++i)
    {
        outFile << "\"/home/karen/myWorkSpace/C++Projects/stereo_calib/images/left" + to_string(i) << ".jpg\"" << endl;
        outLeft << "\"/home/karen/myWorkSpace/C++Projects/stereo_calib/images/left" + to_string(i) << ".jpg\"" << endl;
        outFile << "\"/home/karen/myWorkSpace/C++Projects/stereo_calib/images/right" + to_string(i)<< ".jpg\"" << endl;
        outRigh << "\"/home/karen/myWorkSpace/C++Projects/stereo_calib/images/right" + to_string(i)<< ".jpg\"" << endl;
    }
    outFile << "</imagelist>" << endl
            << "</opencv_storage>" << endl;
    outLeft << "</imagelist>" << endl
            << "</opencv_storage>" << endl;
    outRigh << "</imagelist>" << endl
            << "</opencv_storage>" << endl;
    outFile.close();
    outLeft.close();
    outRigh.close();

    return 0;
}

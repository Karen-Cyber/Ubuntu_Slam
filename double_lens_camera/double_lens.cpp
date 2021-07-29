#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
using namespace cv;

#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
    int device = 2;
    VideoCapture cap(device);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    Mat img_double;
    while (true)
    {
        Mat frame;
		cap >> frame;
		// resize(frame, frame, Size(1280, 480));//set size of image
		Mat leftImage, rightImage;
		leftImage = frame(Rect(0, 0, frame.size().width / 2, frame.size().height));//split left image
		rightImage = frame(Rect(frame.size().width / 2, 0, frame.size().width / 2, frame.size().height));//split right image
		imshow("leftImage", leftImage);//left image
		imshow("rightImage", rightImage);//right image
		if (waitKey(15) >= 0) break;
    }

    return 0;
}
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
using namespace cv;

#include <iostream>
#include <cstring>
using namespace std;

int main(int argc, char** argv)
{
    int device = 0;
    VideoCapture cap(device);
    // cap.open(device);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    Mat img_double;
    int n = 0;
    while (true)
    {
        Mat frame;
		cap >> frame;
		Mat leftImage, rightImage;
		leftImage = frame(Rect(0, 0, frame.size().width / 2, frame.size().height));//split left image
		rightImage = frame(Rect(frame.size().width / 2, 0, frame.size().width / 2, frame.size().height));//split right image
		imshow("leftImage", leftImage);//left image
		imshow("rightImage", rightImage);//right image
		if (waitKey(5) == 97)
        {
            imwrite("left"  + to_string(n) + ".png", leftImage);
            imwrite("right" + to_string(n) + ".png", rightImage);
            n++;
        }
        else if (waitKey(5) == 27)
            break;
    }

    return 0;
}

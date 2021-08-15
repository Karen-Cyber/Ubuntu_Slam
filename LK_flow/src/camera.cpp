#include "camera.h"
#include "config.h"
/**
 *  @brief 
 *  p_w: world coordinate of point
 *  T_c_w: according to the property of matrix multiplication,
 *         the transformation applied from right to left, thus
 *         T_c_w means the transformation from world to camera
 */

Camera::Camera(const std::string& configFilePath, int LorR)
{
    try
    {
        if (!Config::setParameterFile(configFilePath))
            throw("Error openning default.yml\n");
    }
    catch(const char* e)
    {
        std::cout << e;
    }

    double fx = Config::getParameter<double>("camera_fx");
    double fy = Config::getParameter<double>("camera_fy");
    double cx = Config::getParameter<double>("camera_cx");
    double cy = Config::getParameter<double>("camera_cy");
    double ba = Config::getParameter<double>("base_line");
    int dn = Config::getParameter<int>("device_num");
    int FW = Config::getParameter<int>("frame_width");
    int FH = Config::getParameter<int>("frame_height");

    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    baseLine_ = ba;
    frameWid_ = FW;
    frameHei_ = FH;
    deviceNum_ = dn;
    LorR_ = LorR;

    /**
     *  according to the LorR flag to indicate 
     *  whether this is a left cam or right cam
     *  0: left, 1: right
     */
    if (!LorR) baseLine_ *= -1.0;
    SE3 poseTemp(Eigen::AngleAxisd(0, Vec3(1, 0, 0)).toRotationMatrix(), Vec3(baseLine_ / 2.0, 0, 0));
    pose_ = poseTemp;
    poseInv_ = pose_.inverse();

    // camera initialization
    try
    {
        cap_.open(deviceNum_);
        if (!cap_.isOpened())
            throw("Error openning camera\n");
    }
    catch (const char* e)
    {
        std::cout << e;
    }
    cap_.set(CV_CAP_PROP_FRAME_WIDTH,  frameWid_);
    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, frameHei_);
}

void Camera::newFrame(Frame::Ptr& frame)
{
    cv::Mat temp;
    cap_ >> temp;
    frame->imageLeft_ = temp(cv::Rect(0, 0, frameWid_ / 2, frameHei_)).clone();
    frame->imageRigh_ = temp(cv::Rect(frameWid_ / 2, 0, frameWid_ / 2, frameHei_)).clone();
}

Camera::Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w) 
{
    return pose_ * T_c_w * p_w;
}

Camera::Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w) 
{
    return T_c_w.inverse() * poseInv_ * p_c;
}

Camera::Vec2 Camera::camera2pixel(const Vec3 &p_c) 
{
    return Vec2(
            fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
            fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

Camera::Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth) 
{
    return Vec3(
            (p_p(0, 0) - cx_) * depth / fx_,
            (p_p(1, 0) - cy_) * depth / fy_,
            depth
    );
}

Camera::Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w) 
{
    return camera2pixel(world2camera(p_w, T_c_w));
}

Camera::Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth) 
{
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

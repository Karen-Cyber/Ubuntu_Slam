#ifndef CAMERA_H
#define CAMERA_H

#include "common.h"
#include "frame.h"

class Camera
{
private:
    // typedef
    typedef Eigen::Matrix<double, 3, 1> Vec3;
    typedef Eigen::Matrix<double, 2, 1> Vec2;
    typedef Sophus::SE3d SE3;

    // Intrinsics
    double fx_, fy_, cx_, cy_, baseLine_;
    /**
     *  attention:
     *  baseline_ indicates the distance from one camera to 
     *  the center of the stereo camera [extrinsics]
     */
    int deviceNum_;
    cv::VideoCapture cap_;
    // inorder to be compatible
    int frameWid_;
    int frameHei_;
    int LorR_;

    // stereo_singleton transform
    SE3 pose_;      // transform from stereo to singleton
    SE3 poseInv_;   // transform from singleton to stereo

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    Camera(const std::string& configFilePath, int LorR = 0);

    ~Camera() {}

    SE3 getPose() const { return pose_; }

    void newFrame(Frame::Ptr& frame);

    // Transformation functions
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world (const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel (const Vec3 &p_w, const SE3 &T_c_w);
};

#endif
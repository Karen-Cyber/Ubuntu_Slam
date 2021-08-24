#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common.h"
#include "feature.h"

struct MapPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;
    bool outlier_ = false;
    Eigen::Vector3d worldPose_ = Eigen::Vector3d::Zero();
    // lock
    std::mutex data_mutex_;

    int observedTimes_ = 0;
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() {}
    MapPoint(unsigned long id, Eigen::Vector3d pose);

    Eigen::Vector3d getPose()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return worldPose_;
    }

    void setPose(Eigen::Vector3d pose)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        worldPose_ = pose;
    }

    void addObservations(Feature::Ptr feature)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observedTimes_++;
    }

    void decObservations(Feature::Ptr feature)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        for (auto iter = observations_.begin(); iter != observations_.end(); ++iter)
        {
            if (iter->lock() == feature)
            {
                observations_.erase(iter);
                // todo
                // feature->mappoint_.reset
                observedTimes_--;
                break;
            }
        }
    }

    static MapPoint::Ptr createNewMappoint();
};

#endif
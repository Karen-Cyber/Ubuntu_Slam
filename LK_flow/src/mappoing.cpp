#include "mappoint.h"
#include "feature.h"

MapPoint::MapPoint(unsigned long id, Eigen::Vector3d pose) : id_(id), worldPose_(pose) {}

MapPoint::Ptr MapPoint::createNewMappoint()
{
    static unsigned long factory_id = 0;
    MapPoint::Ptr newMapPoint(new MapPoint());
    newMapPoint->id_ = factory_id++;
    return newMapPoint;
}
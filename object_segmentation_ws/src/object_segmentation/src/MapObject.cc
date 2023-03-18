#include "MapObject.h"
// #include <Parameters.h>



MapObject::MapObject(/* args */)
{ 
    hsv = cv::Mat::zeros(1, 49, CV_64FC1);
}

MapObject::~MapObject()
{ }

void MapObject::InitializeMapObject(std::vector<MapObject*>& mspMapObjects)
{
    MapObject* new_obj = new MapObject();
    new_obj->mnClass = this->mnClass;
    new_obj->mnId = mspMapObjects.size();
    new_obj->bbox = this->bbox;
    new_obj->loc = this->loc;
    new_obj->rpy = this->rpy;
    new_obj->dim = this->dim;
    new_obj->hsv = this->hsv;
    new_obj->bbox_projected = this->bbox_projected;
    new_obj->points = this->points;
    new_obj->points_list.push_back(this->points);
    mspMapObjects.push_back(new_obj);
}

void MapObject::MergeWithFrameObject(MapObject* FrameObj)
{
    // should update some parameters
    // loc, rpy, dim should be updated by point cloud
    this->bbox = FrameObj->bbox; // update to newest bbox 
    this->hsv = FrameObj->hsv;  // update to newest hsv
    this->points = FrameObj->points + this->points;
    // this->points_list.push_back(FrameObj->points);
    if(this->points_list.size()>10)
        this->points_list.erase(this->points_list.begin());
    this->points.empty();
    for (size_t i = 0; i < this->points_list.size(); i++)
    {
        this->points = this->points + this->points_list[i];
    }
    

    // update
    // check association again: project obj points on image, calculate bbox, calculate iou
    // update points: historical points should remove if not in new bbox
    // update param, id, ,,,
    // other outlier rejection algorithm
    // map update
    // if an object are only seen less than 5 frame, should remove
}


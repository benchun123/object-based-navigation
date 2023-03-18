#ifndef MAPOBJECT_H
#define MAPOBJECT_H

// std c
#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

// #include "MapPoint.h"
// #include "Box2D.h"
// // #include "Map.h"
// #include "Converter.h"
// #include "Frame.h"
// #include "KeyFrame.h"
// #include "MatrixUtils.h"

// using namespace std;

class MapObject
{
private:

public:
    int mnId;           // object id in map
    int mnClass;
    Eigen::Vector4f bbox;
    Eigen::Vector3f loc;  
    Eigen::Vector3f rpy;      
    Eigen::Vector3f dim;   

    cv::Mat hsv;
    bool find_asso;   
    int asso_id;

	pcl::PointCloud<pcl::PointXYZRGBL> points; // convert from depth image

    // int mnConfidence;
    // // int mnAddedID;      
    // int mnLastAddID;        // frame id when add to map
    // int mnLastLastAddID;
    // cv::Rect mLastRect;
    // cv::Rect mLastLastRect;
    // cv::Rect mRectProject;
    // cv::Mat mSumPointsPos;
    // cv::Mat mCenter3D;     
    // bool bBadErase = false;

    // std::vector<Box2D*> mObjectFrame;
    // std::vector<int> mObjectFrameId;
    // std::vector<MapPoint*> mvpMapObjectMappoints;
    // std::vector<MapPoint*> mvpMapCurrentNewMappoints;
    // std::vector<MapPoint*> mvpMapObjectMappointsFilter;

    // bool DataAssociateUpdate(Box2D* ObjectFrame, 
    //                             Frame &mCurrentFrame, 
    //                             int &img_width, int& img_height,
    //                             int Flag);
    // void ComputeProjectRectFrame(int &img_width, int& img_height, Frame &mCurrentFrame);
    // bool WhetherOverlap(MapObject *CompareObj);
    // void UpdateObjPose();      // update object pose.
    // void CalculateObjectPose(); // calculate object pose by corners

    // // void IsolationForestDeleteOutliers();
    // void BigToSmall(MapObject *SmallObj, float overlap_x, float overlap_y, float overlap_z);
    // void DivideEquallyTwoObjs(MapObject *AnotherObj, float overlap_x, float overlap_y, float overlap_z);
    // void DealTwoOverlapObjs(MapObject *OverlapObj, float overlap_x, float overlap_y, float overlap_z);
    // void MergeTwoMapObjs(MapObject *RepeatObj);
    // void FilterMapPoints(vector<MapPoint *>& points_after_filter);
    // std::map<int, int> mmAppearSametime;// object id and times simultaneous appearances .


    MapObject(/* args */);
    ~MapObject();
    void InitializeMapObject(std::vector<MapObject*>& mspMapObjects);
    void MergeWithFrameObject(MapObject* FrameObj);
    
protected:
    std::mutex mMutexMapPoints;
    std::mutex mMutex;
};


#endif
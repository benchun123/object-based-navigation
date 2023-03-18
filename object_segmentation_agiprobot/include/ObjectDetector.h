#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h> // include pcl::OrganizedMultiPlaneSegmentation
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>  // include pcl::EuclideanClusterExtraction
#include <pcl/features/integral_image_normal.h>  // include pcl::IntegralImageNormalEstimation
#include <pcl/visualization/cloud_viewer.h>//include pcl::visualization::CloudViewer
#include <boost/make_shared.hpp>

#include <pcl/filters/statistical_outlier_removal.h> // include StatisticalOutlierRemoval
#include <pcl/filters/project_inliers.h>


using namespace std;
typedef pcl::PointXYZRGBL PointT;
typedef pcl::PointCloud <PointT> PointCloud;
class MapObject;

class ObjectDetector
{
public:
	// float fx, fy, cx, cy, mDepthMapFactor;  // Camera intrinsic parameters.
	cv::Mat imRGB, imDepth;
	// int imWidth, imHeight;	// image size
	// Eigen::Matrix3d Kalib;
	Eigen::Matrix4d Twc;
    // bool vis_enable;
    pcl::visualization::PCLVisualizer *viewer;
	pcl::PointCloud<pcl::PointXYZRGBL> rawCloudPoints; // convert from depth image

    // float ground_min, ground_max, wall_min, wall_max;

	std::vector<int> bbox_name_list;
    std::vector<Eigen::Vector4f> bbox_xywh_list;
	std::vector<pcl::PointCloud <pcl::PointXYZRGBL> >  bbox_points_raw; // point cloud in bbox
	std::vector<pcl::PointCloud <pcl::PointXYZRGBL> >  bbox_points_filter; // point cloud in bbox
	std::vector<Eigen::Matrix<float, 9, 1> > object_param_list;
	std::vector<cv::Mat> object_hsv;

	// int MultiPlane_SizeMin; // param for MultiPlaneSegmentation
	// float MultiPlane_AngleThre;
	// float MultiPlane_DistThre;
	// std::vector<cv::Mat> mvPlaneCoefficients; // plane normal 
	// std::vector<pcl::PointIndices > mvPlaneIndices; // plane points 
	std::vector<pcl::PointCloud <pcl::PointXYZRGBL> > mvPlanePoints; // plane points 
	pcl::PointIndices ground_wall_indices; // indices of ground and wall in point cloud
    std::vector<MapObject*> mspFrameObjects;    // objects in the map.

public:
	ObjectDetector() 
	{
		// Kalib.setIdentity();
		// Twc.setIdentity();
	};

	~ObjectDetector()
    {
        imRGB.release();
        imDepth.release();
    };

	bool ReadColorImage(string& filename);
	bool ReadDepthImage(string& filename);
	bool ReadOdomFile( Eigen::VectorXd& odom);

	bool ComputePointCloud();
	bool ReadBBox2DCloud(string& filename);
	bool ShowBBox2D(string& filename);

	bool FilterOutliers();
	void ClusterObjects();
	void GetObjectHSV();
	Eigen::VectorXf ComputeCuboidParam(pcl::PointCloud <pcl::PointXYZRGBL>& obj_pcl);
	bool ExtractObjectFromOrganizedPlanes(pcl::PointCloud<pcl::PointXYZRGBL>& bbox_pcl,
										pcl::PointCloud<pcl::PointXYZRGBL>& obj_pcl);

	void SaveCuboidParam(std::string& filename);
	void SaveFrameObjects();
	
	bool AssociateWithMapObject(std::vector<MapObject*>& mspMapObjects);
	void UpdateMapObject(std::vector<MapObject*>& mspMapObjects);

	bool ComputeLabelCloud(string& filename);
	int checkBbox(const int& x, const int& y, const Eigen::MatrixXd& bboxes);

};

#endif
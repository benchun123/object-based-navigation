#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"
#include "ObjectDetector.h"
#include "matrix_utils.h"
#include "MapObject.h"
#include <Parameters.h>

using namespace std;
using namespace Eigen;

bool ObjectDetector::ReadColorImage(string& filename)
{
	std::cout << "start reading rgb image "  << std::endl;
	imRGB = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
	if (imRGB.empty() || imRGB.depth() != CV_8U)
	{
		cout << "ERROR: cannot read color image. No such a file, or the image format is not 8UC3" << endl;
		return false;
	}
	return true;
}

bool ObjectDetector::ReadDepthImage(string& filename)
{
	std::cout << "start reading depth image "  << std::endl;
    // cv::Mat imDepth = cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
    imDepth = cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);
	if (imDepth.empty() || imDepth.depth() != CV_16U)
	{
		cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << endl;
		return false;
	}
	// change depth value, do not know why, maybe change value to [0,10]
	if (mDepthMapFactor == 0)
		cout << "WARNING: should read intrinsic param firstly" << endl;
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
}

bool ObjectDetector::ReadOdomFile(Eigen::VectorXd& odom)
{
	// Eigen::Matrix4d Tob;	// odom to base_link
	// Tob.setIdentity();
	// Tob.block(0,0,3,3) = Eigen::Quaterniond(odom(6),odom(3),odom(4),odom(5)).toRotationMatrix();
	// Tob.col(3).head(3) = Eigen::Vector3d(odom(0), odom(1), odom(2));
	// Twc = Tob * Tbc;

	// laser odom, laser movement, output is similar to wheel odom but accurate
	// tranform from odom to base_link, odom is fixed, base_link is moving
	Eigen::Matrix4d Tob;	// laser movement, origin: base link
	Tob.setIdentity();
	Tob.block(0,0,3,3) = Eigen::Quaterniond(odom(6),odom(3),odom(4),odom(5)).toRotationMatrix();
	Tob.col(3).head(3) = Eigen::Vector3d(odom(0), odom(1), odom(2));
	Twc = Tob * Tbc;

	return true;
}

bool ObjectDetector::ShowBBox2D(string& filename)
{
	std::cout << "start reading 2d bounding box "  << std::endl;
	Eigen::MatrixXd truth_data(2,6);
	if (!read_all_number_txt(filename, truth_data))
		return -1;
	if(truth_data.rows()==0) // no object
		return false;
	// truth_data.col(3) = truth_data.col(3) - truth_data.col(1);
	// truth_data.col(4) = truth_data.col(4) - truth_data.col(2);
	std::cout << "truth_data " << truth_data << std::endl;

	filterBBox2D(truth_data);
	std::cout << "truth_data " << truth_data << std::endl;

	cv::Mat plot_2d_img = imRGB.clone();
	Eigen::MatrixXd bboxes = truth_data.block(0,1,truth_data.rows(),4);
	plot_2d_bbox_with_xywh(plot_2d_img, bboxes);
	cv::imshow("2d bounding box", plot_2d_img);
	cv::waitKey(0);
	// for (size_t i = 0; i < bboxes.rows(); i++)
	// {
	// 	cv::Mat croppedImage = imRGB(cv::Rect(bboxes(i,0), bboxes(i,1), bboxes(i,2), bboxes(i,3)));
	// 	cv::imshow("croppedImage", croppedImage);
	// 	cv::waitKey(0);
	// }
    return true;
}

bool ObjectDetector::ReadBBox2DCloud(string& filename)
{
	std::cout << "start reading point cloud in 2d bounding box "  << std::endl;
	Eigen::MatrixXd truth_data(2,6);
	if (!read_all_number_txt(filename, truth_data))
		return -1;
	if(truth_data.rows()==0) // no object
		return false;

	// // convert xyxy to xywh
	// truth_data.col(3) = truth_data.col(3) - truth_data.col(1);
	// truth_data.col(4) = truth_data.col(4) - truth_data.col(2);
	std::cout << "truth_data " << truth_data << std::endl;

	// filter bbox?
	filterBBox2D(truth_data);

    for (size_t i = 0; i < truth_data.rows(); i++)
    {
		if(truth_data(i,0)!= 1)
			continue;
		if(truth_data(i,5) < 0.5)
			continue;
        Eigen::Vector4d obj_bbox = truth_data.row(i).segment<4>(1);
        int x_min = int(obj_bbox(0));
        int y_min = int(obj_bbox(1));
        int x_max = min(int(obj_bbox(0)+obj_bbox(2)), imDepth.cols);
        int y_max = min(int(obj_bbox(1)+obj_bbox(3)), imDepth.rows);
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points_cam;
		int cloudDis = 1; // use 1 to exact more planes
		for ( int m=y_min; m<y_max; m+=cloudDis )
		{
			for ( int n=x_min; n<x_max; n+=cloudDis )
			{
				float d = imDepth.ptr<float>(m)[n];
				// if(d==0)
				// {
				// 	continue;
				// }
				PointT p;
				p.z = d;
				p.x = ( n - cx) * p.z / fx;
				p.y = ( m - cy) * p.z / fy;
				// p.r = 177;
				// p.g = 177;
				// p.b = 177;
				p.b = imRGB.ptr<uchar>(m)[n*3];
				p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
				p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
				p.label = truth_data(i,0);
				obj_points_cam.points.push_back(p);
			}
		}
		obj_points_cam.height = ceil((y_max-y_min)/float(cloudDis));
		obj_points_cam.width = ceil((x_max-x_min)/float(cloudDis));

        if(vis_enable)
        {
            viewer = new pcl::visualization::PCLVisualizer("object_raw_cloud_viewer");
            viewer->addCoordinateSystem(1);
            viewer->setBackgroundColor(1.0, 1.0, 1.0);
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = obj_points_cam.makeShared();
            pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
            viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show cloud");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud");
            while (!viewer->wasStopped ())
            {
                viewer->spinOnce (100);
            }
            viewer->close();
        }

		// estimate plane before transfrom
		// filter outlier by planes (from exprience, it works very good)
		// reason, from camera capture principle, points of outlier are further than objects
		// 由于相机采集数据原理，平台相比于物体散射严重，点云质量不行，无法提取平面，相反通过提取平面反而能提出所有物体点云
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points_new;
		bool obj_exact = ExtractObjectFromOrganizedPlanes(obj_points_cam, obj_points_new);
		if(obj_exact == false) // no object point are detected
			continue;
		std::cout << "obj points raw: " << obj_points_cam.points.size() << " after filter: " << obj_points_new.points.size() << std::endl;

		// save info
		pcl::PointCloud<pcl::PointXYZRGBL>  obj_points_world;
		pcl::transformPointCloud(obj_points_new, obj_points_world, Twc);
		bbox_points_raw.push_back(obj_points_world);
        bbox_name_list.push_back(truth_data(i,0));
		bbox_xywh_list.push_back(obj_bbox.cast<float>());

        if(vis_enable)
        {
            viewer = new pcl::visualization::PCLVisualizer("object_raw_cloud_viewer");
            viewer->addCoordinateSystem(1);
            viewer->setBackgroundColor(1.0, 1.0, 1.0);
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = obj_points_world.makeShared();
            pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
            viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show cloud");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud");
            while (!viewer->wasStopped ())
            {
                viewer->spinOnce (100);
            }
            viewer->close();
        }
    }
    return true;
}

bool ObjectDetector::FilterOutliers()
{
	std::cout << "start FilterOutliers "  << std::endl;
	std::cout << "ground value: " << ground_min << " " << ground_max << std::endl;
	std::cout << "wall value: " << wall_min << " " << wall_max << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_without_ground(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_without_wall(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_euclidean(new pcl::PointCloud<pcl::PointXYZRGBL>);
	for (size_t i = 0; i < bbox_points_raw.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points = bbox_points_raw[i];

		// // filter voxel
		pcl::VoxelGrid<pcl::PointXYZRGBL> vgFilter;
		vgFilter.setInputCloud(obj_points.makeShared());
		vgFilter.setLeafSize(0.02f, 0.02f, 0.02f);
		vgFilter.setSaveLeafLayout(true);
		vgFilter.filter(obj_points);

		// filter ground
	  	pcl::PassThrough<pcl::PointXYZRGBL> pass;
		pass.setInputCloud(obj_points.makeShared());
		pass.setFilterFieldName ("z");
		pass.setFilterLimits(ground_min, ground_max); //-1.0, -0.8
		pass.setFilterLimitsNegative (true);// when false, filter from value 1 to value 2
		pass.filter (*cloud_without_ground);

		// // filter wall
		// pass.setInputCloud(cloud_without_ground);
		// pass.setFilterFieldName ("y");
		// pass.setFilterLimits(wall_min, wall_max); // (2.2, 2.4);
		// pass.setFilterLimitsNegative (true);// when false, filter from value 1 to value 2
		// pass.filter(*cloud_without_wall);
		*cloud_without_wall = *cloud_without_ground;


		// // // filter StatisticalOutlierRemoval
		// // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
		// // sor.setInputCloud (obj_cloud);
		// // sor.setMeanK(100);
		// // sor.setStddevMulThresh (1);
		// // sor.filter(*obj_cloud);
		// // filter RadiusOutlierRemoval
		// // pcl::RadiusOutlierRemoval<pcl::PointXYZRGBL> radius;
		// // radius.setInputCloud(obj_cloud);
		// // radius.setRadiusSearch(1.0);
		// // radius.setMinNeighborsInRadius(50);
		// // radius.setKeepOrganized(true);
		// // radius.filter(*obj_cloud);

		*cloud_euclidean = *cloud_without_wall;
		// // filter EuclideanClusterExtraction
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBL>);
		tree->setInputCloud (cloud_euclidean);
		std::vector<pcl::PointIndices> cluster_indices;
		//Do eucledean clustering
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBL> ec;
		ec.setClusterTolerance(0.03);
		ec.setMinClusterSize (double(1));
		ec.setMaxClusterSize (cloud_euclidean->points.size());
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_euclidean);
		ec.extract(cluster_indices);
		if (cluster_indices.size()== 0)
		{
			std::cout << "No points in cluster indicies" << std::endl;
		}
		int max_indice_id = 0;
		int max_indice_num = 0;
		for(int i=0;i<cluster_indices.size();i++)
		{
			if(cluster_indices[i].indices.size() > max_indice_num)
			{
				max_indice_num = cluster_indices[i].indices.size();
				max_indice_id = i;
			}
		}
		// for(int i=0;i<cluster_indices.size();i++)
		// {
		// 	pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
		// 	temp_indices->indices = cluster_indices[i].indices;
		// 	point_cloud_cluster.push_back(*pointCloudFromInd(temp_indices,cloud));
		// }
		pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
		temp_indices->indices = cluster_indices[max_indice_id].indices;
		pcl::ExtractIndices<pcl::PointXYZRGBL> extractIndices;
		extractIndices.setInputCloud(cloud_euclidean);
		extractIndices.setIndices(temp_indices);
		extractIndices.setNegative (false);
		extractIndices.filter(*cloud_euclidean);

		// if(vis_enable)
		// {
		// 	viewer = new pcl::visualization::PCLVisualizer("tttttttt");
		// 	viewer->addCoordinateSystem(1);
		// 	viewer->setBackgroundColor(0.0, 0.0, 0.0);
		// 	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = obj_cloud;
		//     pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
		//     viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show cloud");
		//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud");
		//     while (!viewer->wasStopped ())
		//     {
		//         viewer->spinOnce (100);
		//     }
		//     viewer->close();
		// }


        bbox_points_filter.push_back(*cloud_euclidean);

        // if(vis_enable)
        // {
        //     viewer = new pcl::visualization::PCLVisualizer("object_filter_cloud_viewer");
        //     viewer->addCoordinateSystem(1);
        //     viewer->setBackgroundColor(0.0, 0.0, 0.0);
        //     pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = cloud_without_wall;
        //     pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        //     viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show cloud");
        //     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud");
        //     while (!viewer->wasStopped ())
        //     {
        //         viewer->spinOnce (100);
        //     }
        //     // viewer->removePointCloud("show cloud");
        //     viewer->close();
        // }
	}
    return true;
}

void ObjectDetector::ClusterObjects()
{
	std::cout << "start ClusterObjects "  << std::endl;
	for (size_t i = 0; i < bbox_points_filter.size(); i++)
	{
		Eigen::VectorXf obj_param = ComputeCuboidParam(bbox_points_filter[i]);
		object_param_list.push_back(obj_param);

		Eigen::AngleAxisf obj_roll(obj_param(3), Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf obj_pitch(obj_param(4), Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf obj_yaw(obj_param(5), Eigen::Vector3f::UnitZ());
		Eigen::Quaternion<float> bbox_quaternion = obj_roll * obj_pitch * obj_yaw;
		Eigen::Matrix3f bbox_rot = bbox_quaternion.matrix().transpose();
		
		Eigen::Matrix4d obj_global;	
		obj_global.setIdentity();
		obj_global.block(0,0,3,3) = bbox_rot.cast<double>();
		obj_global.col(3).head(3) = Eigen::Vector3d(obj_param(0), obj_param(1), obj_param(2));
		Eigen::Matrix4d obj_local = Twc.inverse()*obj_global;	
		// std::cout << "obj_global: \n" << obj_global << std::endl;
		// std::cout << "Twc: \n" << Twc << std::endl;
		// std::cout << "obj_local: \n" << obj_local << std::endl;

		Eigen::Vector3d trans_tmp = obj_local.col(3).head(3);
		// Eigen::Vector3d dim_tmp = obj_param.tail(3).cast<double>();
		// Eigen::Vector3d dim_tmp = Eigen::Vector3d(obj_param(7)/2.0, obj_param(8)/2.0, obj_param(6)/2.0);
		Eigen::Vector3d dim_tmp = Eigen::Vector3d(obj_param(6)/2.0, obj_param(7)/2.0, obj_param(8)/2.0);
		Eigen::Matrix3d rot_tmp = obj_local.block(0,0,3,3);
		cv::Mat plot_3d_img = imRGB.clone();
		plot_3d_box_with_loc_dim_camera(plot_3d_img, Kalib, trans_tmp, dim_tmp, rot_tmp);
		cv::imshow("3d bounding box", plot_3d_img);
		cv::waitKey(0);
	}

	// // visualization
    if(vis_enable)
    {
        viewer = new pcl::visualization::PCLVisualizer("object_3dbbox_viewer");
        // viewer->addCoordinateSystem(1);
		viewer->setBackgroundColor(1.0, 1.0, 1.0);

		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_temp = rawCloudPoints.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgbtemp(cloud_temp);
        viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_temp, rgbtemp, "show cloud temp");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud temp");

        for (size_t obj_id = 0; obj_id < object_param_list.size(); obj_id++)
        {
            // add object point cloud
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = bbox_points_filter[obj_id].makeShared();
            pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
            viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show_cloud"+to_string(obj_id));
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show_cloud"+to_string(obj_id));
            // add cuboid
            Eigen::Vector3f bbox_trans = object_param_list[obj_id].head(3);
            Eigen::Vector3f bbox_whd = object_param_list[obj_id].tail(3);
            Eigen::AngleAxisf obj_roll(object_param_list[obj_id](3), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf obj_pitch(object_param_list[obj_id](4), Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf obj_yaw(object_param_list[obj_id](5), Eigen::Vector3f::UnitZ());
            Eigen::Quaternion<float> bbox_quaternion = obj_roll * obj_pitch * obj_yaw;
            Eigen::Matrix3f bbox_rot = bbox_quaternion.matrix().transpose();
            viewer->addCube(bbox_trans, bbox_quaternion, bbox_whd(0), bbox_whd(1), bbox_whd(2), "bbox"+ to_string(obj_id));
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox"+ to_string(obj_id));
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox"+ to_string(obj_id));
            // add Arrow with rotation
            float ratio = (bbox_whd(0)+bbox_whd(1)+bbox_whd(2))/3.0;
            pcl::PointXYZRGBL coordCentroid;
            pcl::PointXYZRGBL coordAxisX;
            pcl::PointXYZRGBL coordAxisY;
            pcl::PointXYZRGBL coordAxisZ;
            coordCentroid.x = bbox_trans(0);
            coordCentroid.y = bbox_trans(1);
            coordCentroid.z = bbox_trans(2);
            coordAxisX.x = ratio * bbox_rot(0, 0) + coordCentroid.x;
            coordAxisX.y = ratio * bbox_rot(0, 1) + coordCentroid.y;
            coordAxisX.z = ratio * bbox_rot(0, 2) + coordCentroid.z;
            coordAxisY.x = ratio * bbox_rot(1, 0) + coordCentroid.x;
            coordAxisY.y = ratio * bbox_rot(1, 1) + coordCentroid.y;
            coordAxisY.z = ratio * bbox_rot(1, 2) + coordCentroid.z;
            coordAxisZ.x = ratio * bbox_rot(2, 0) + coordCentroid.x;
            coordAxisZ.y = ratio * bbox_rot(2, 1) + coordCentroid.y;
            coordAxisZ.z = ratio * bbox_rot(2, 2) + coordCentroid.z;
            viewer->addArrow(coordAxisX, coordCentroid, 1.0, 0.0, 0.0, false, "arrow_x"+ to_string(obj_id));
            viewer->addArrow(coordAxisY, coordCentroid, 0.0, 1.0, 0.0, false, "arrow_y"+ to_string(obj_id));
            viewer->addArrow(coordAxisZ, coordCentroid, 0.0, 0.0, 1.0, false, "arrow_z"+ to_string(obj_id));
        }
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }
        viewer->close();
    }
}

void ObjectDetector::GetObjectHSV()
{
	for (size_t obj_id = 0; obj_id < bbox_xywh_list.size(); obj_id++)
	{
		Eigen::Vector4f cuboid_xywh = bbox_xywh_list[obj_id];
		std::cout << "cuboid_xywh " << cuboid_xywh.transpose() << std::endl;
		float u = cuboid_xywh(0);
		float v = cuboid_xywh(1);
		float width = cuboid_xywh(2);
		float height = cuboid_xywh(3);
		cv::Mat rgbImage = imRGB(cv::Rect(u, v, width, height));
		cv::Mat hsvImage;
		cv::cvtColor(rgbImage, hsvImage, CV_BGR2HSV);
		// cv::imshow("cccc", rgbImage);
		// cv::waitKey(0);

		//初始化计算直方图需要的实参
		//hue通道使用30个bin, saturation通道使用32个bin
		int h_bins = 30, s_bins = 32;
		int histSize[] = {h_bins, s_bins};
		//hue的取值范围从0到180， saturation的取值范围从0到256
		float h_ranges[] = {0, 180};
		float s_ranges[] = {0, 256};
		const float* ranges[] = {h_ranges, s_ranges};
		//使用第0和第1通道，即hue和saturation的通道
		int channels[] = {0, 1};
		//存储直方图
		cv::Mat hist;
		cv::calcHist(&hsvImage, 1, channels, cv::Mat(), hist, 2, histSize, ranges, true, false);
		cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
		//输出结果为二维直方图，行数为hue的bin值，列数为saturation的bin值
		object_hsv.push_back(hist);
	}
}

bool ObjectDetector::ComputePointCloud()
{
	std::cout << "\n----- ComputePointCloud -----" << std::endl;
	// translate to point cloud
	pcl::PointCloud<pcl::PointXYZRGBL> inputCloud;
	int cloudDis = 1; // use 1 to exact more planes
	for ( int m=0; m<imDepth.rows; m+=cloudDis )
	{
		for ( int n=0; n<imDepth.cols; n+=cloudDis )
		{
			float d = imDepth.ptr<float>(m)[n];
			PointT p;
			p.z = d;
			p.x = ( n - cx) * p.z / fx;
			p.y = ( m - cy) * p.z / fy;
			// p.r = 177;
			// p.g = 177;
			// p.b = 177;
            p.b = imRGB.ptr<uchar>(m)[n*3];
            p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
            p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
			inputCloud.points.push_back(p);
		}
	}
	inputCloud.height = ceil(imDepth.rows/float(cloudDis));
	inputCloud.width = ceil(imDepth.cols/float(cloudDis));

    // // check if we need to transform point cloud to world coordinates
	pcl::PointCloud<pcl::PointXYZRGBL>  raw_points_world;
	pcl::transformPointCloud(inputCloud, raw_points_world, Twc);
	rawCloudPoints = raw_points_world;


    if(vis_enable)
    {
        viewer = new pcl::visualization::PCLVisualizer("pointcloud_raw_viewer");
        viewer->addCoordinateSystem(1);
		viewer->setBackgroundColor(1.0, 1.0, 1.0);
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = rawCloudPoints.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud");
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }
        viewer->close();
    }
    return true;

}

void ObjectDetector::SaveCuboidParam(std::string& filename)
{
	// save cuboid to file
    ofstream online_stream_cube;
    online_stream_cube.open(filename.c_str());
	for (size_t i = 0; i < object_param_list.size(); i++)
	{
		int class_name = bbox_name_list[i];
		Eigen::Vector3f cuboid_loc = object_param_list[i].head(3);
		Eigen::Vector3f cuboid_rpy = object_param_list[i].segment<3>(3);
		Eigen::Vector3f cuboid_dim = object_param_list[i].tail(3);
		Eigen::Vector4f cuboid_xywh = bbox_xywh_list[i];
		// Eigen::Matrix4d cuboid_mat = cuboid_mat_list[i].cast<double>();
		// Eigen::Matrix3d cuboid_rot = cuboid_mat.block(0,0,3,3);
		// Eigen::Vector3d rpy;
		// quat_to_euler_zyx(Eigen::Quaterniond(cuboid_rot), rpy(0), rpy(1), rpy(2));
		online_stream_cube << class_name << " " << cuboid_xywh(0)
			<< " " << cuboid_xywh(1) << " " << cuboid_xywh(2) << " " << cuboid_xywh(3)
            << " " << cuboid_loc(0) << " " << cuboid_loc(1)  << " " << cuboid_loc(2)
			<< " " << cuboid_rpy(0) << " " << cuboid_rpy(1) << " " << cuboid_rpy(2)
			<< " " << cuboid_dim(0) << " " << cuboid_dim(1) << " " << cuboid_dim(2)
			<< endl;

        // void plot_3d_box_with_loc_dim_camera(cv::Mat &img, Eigen::Matrix3d& Kalib, Eigen::Vector3d& location, Eigen::Vector3d& dimension, Eigen::Matrix3d& local_rot_mat)

	}
    online_stream_cube.close();
	std::cout << "save cuboid param to " << filename << std::endl;
}

void ObjectDetector::SaveFrameObjects()
{
	// std::cout << "SaveFrameObjects: " << std::endl;
	for (size_t i = 0; i < bbox_xywh_list.size(); i++)
	{
		MapObject *mvObj = new MapObject();
		mvObj->mnClass = bbox_name_list[i];
		mvObj->bbox = bbox_xywh_list[i];
		mvObj->loc = object_param_list[i].head(3);
		mvObj->rpy = object_param_list[i].segment<3>(3);
		mvObj->dim = object_param_list[i].tail(3);
		mvObj->hsv = object_hsv[i].clone();
		mvObj->points = bbox_points_filter[i];

		mspFrameObjects.push_back(mvObj);
	}
}

bool ObjectDetector::AssociateWithMapObject(std::vector<MapObject*>& mspMapObjects)
{
	std::cout << "AssociateWithMapObject: " << std::endl;
	for (size_t i = 0; i < mspFrameObjects.size(); i++)
	{
		MapObject* mFO = mspFrameObjects[i];
		mFO->find_asso = false;
		mFO->asso_id = 0;
		float min_dist = 1000;
		for (size_t j = 0; j < mspMapObjects.size(); j++)
		{
			MapObject* mMO = mspMapObjects[j];
			// step 1: check class
			if(mFO->mnClass != mMO->mnClass)
				continue;
			// step 2 : HSV
			double correl = cv::compareHist(mFO->hsv, mMO->hsv, CV_COMP_CORREL);
			// std::cout << "correl: " << correl << std::endl;
			if(correl < 0.3)
				continue;
			// step3: distance
			double dist = sqrt(pow(mFO->loc(0) - mMO->loc(0) , 2) +
								pow(mFO->loc(1) - mMO->loc(1) , 2) +
								pow(mFO->loc(2) - mMO->loc(2) , 2) );
			// std::cout << "dist: " << dist << std::endl;
			if(dist > 1.0)
				continue;
			
			// step 4 find best association
			if(dist < min_dist)
			{
				min_dist = dist;
				mFO->find_asso = true;
				mFO->asso_id = mMO->mnId;
			}
		} // loop map object
		std::cout << "mspFrameObjects " << i << " find_asso " << mFO->find_asso << " id " << mFO->asso_id<< std::endl;
	} // loop frame object
	return true;

// association
// class
// id frame interval > 20
// 2d iou, current and last bbox
// if (all_objects[i]->mnLastAddID > mCurrentFrame.mnId - 20)// seen with in 20 frame
//     // step 1.2 compute IoU, record the max IoU and the map object ID.
//     float Iou = bboxOverlapratio(RectCurrent, RectPredict);
//     if ((Iou > IouThreshold) && Iou > IouMax)
//     {
//         IouMax = Iou;
//         IouMaxObjID = i;
//     }
// float bboxOverlapratio(const cv::Rect &rect1, const cv::Rect &rect2)
// {
//     // using opencv with cv::Rect
//     int overlap_area = (rect1 & rect2).area();
//     return (float)overlap_area / ((float)(rect1.area() + rect2.area() - overlap_area));
// }

}

void ObjectDetector::UpdateMapObject(std::vector<MapObject*>& mspMapObjects)
{
	std::cout << "UpdateMapObject: " << std::endl;
	for (size_t i = 0; i < mspFrameObjects.size(); i++)
	{
		MapObject* mFO = mspFrameObjects[i];
		if (mFO->find_asso == false)
		{
			std::cout << "init map object" << std::endl;
			mFO->InitializeMapObject(mspMapObjects);
		}
		else
		{
			MapObject* mMO = mspMapObjects[mFO->asso_id];
			std::cout << "merge object" << std::endl;
			mMO->MergeWithFrameObject(mFO);
			Eigen::VectorXf obj_param = ComputeCuboidParam(mMO->points);
			mMO->loc = obj_param.head(3);
			mMO->rpy = obj_param.segment<3>(3);
			mMO->dim = obj_param.tail(3);
			pcl::VoxelGrid<pcl::PointXYZRGBL> vgFilter;
			vgFilter.setInputCloud(mMO->points.makeShared());
			vgFilter.setLeafSize(0.02f, 0.02f, 0.02f);
			vgFilter.setSaveLeafLayout(true);
			vgFilter.filter(mMO->points);
			std::cout << "mMO->points: "<<mMO->points.size() << std::endl;
		}
	}
}

bool ObjectDetector::ExtractObjectFromOrganizedPlanes(pcl::PointCloud<pcl::PointXYZRGBL>& bbox_pcl,
													pcl::PointCloud<pcl::PointXYZRGBL>& obj_pcl)
{

	int min_plane = MultiPlane_SizeMin;//500;
	float AngTh = MultiPlane_AngleThre;//2.0;
	float DisTh = MultiPlane_DistThre;//0.02;

	// // firstly, compute normal
	pcl::PointCloud<PointT>::Ptr  input_cloud = bbox_pcl.makeShared();
	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.05f); // 0.05
	ne.setNormalSmoothingSize(10.0f); // 10.0
	ne.setInputCloud(input_cloud);
	ne.compute(*cloud_normals);

	// secondly, compute region, label, coefficient, inliners, ...
	pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
	pcl::PointCloud<pcl::Label>::Ptr labels ( new pcl::PointCloud<pcl::Label> );
	vector<pcl::ModelCoefficients> coefficients;
	vector<pcl::PointIndices> inliers;
	vector<pcl::PointIndices> label_indices;
	vector<pcl::PointIndices> boundary;
	std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
	mps.setMinInliers(min_plane);	//int min_plane = 1000;
	mps.setAngularThreshold (0.017453 * AngTh); //float AngleThreshold = 3.0 (0.017453=pi/180)
	mps.setDistanceThreshold (DisTh); // float DistanceThreshold = 0.05
	mps.setInputNormals (cloud_normals);
	mps.setInputCloud (input_cloud);
	mps.segmentAndRefine (regions, coefficients, inliers, labels, label_indices, boundary);
	std::cout << "inliers: " << inliers.size() << std::endl;
	if (inliers.size()==0)
	{
		std::cout << "no object point cloud detected" << std::endl;
		return false;
	}

	// // thirdly, exact and filter point cloud
	// pcl::PointCloud <pcl::PointXYZRGBL> obj_pcl; 
	std::vector<cv::Mat> mvPlaneCoefficients; // plane normal 
	std::vector<pcl::PointIndices > mvPlaneIndices; // plane points 
	std::vector<pcl::PointCloud <pcl::PointXYZRGBL> > mvPlanePoints; // plane points 
    std::vector<pcl::PointCloud <pcl::PointXYZRGBL> > mvBoundaryPoints; // plane boundary, just for visualization
	for (int i = 0; i < inliers.size(); ++i)
	{
		cv::Mat coef = (cv::Mat_<float>(4,1) << coefficients[i].values[0], 
						coefficients[i].values[1], 
						coefficients[i].values[2], 
						coefficients[i].values[3]);
		if(coef.at<float>(3) < 0)
				coef = -coef;
		// std::cout << "plane: " << i <<" "<< coef.t() << std::endl;

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(input_cloud);
		extract.setNegative(false);
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
		extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers[i])); // #include <boost/make_shared.hpp>
		extract.filter(*planeCloud);
		// std::cout << "plane: " << i <<" "<< coef.t() << planeCloud->size() << std::endl;

		// save for visualization
		pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices(inliers[i]));
		PointCloud::Ptr boundaryPoints(new PointCloud());
		boundaryPoints->points = regions[i].getContour();
		mvPlanePoints.push_back(*planeCloud);
		mvPlaneCoefficients.push_back(coef);
		mvBoundaryPoints.push_back(*boundaryPoints);
		mvPlaneIndices.push_back(*planeIndices);
		
		// just collect all plane points;
		obj_pcl += *planeCloud;
	}

	if(vis_enable)
	{
		// pcl::visualization::PCLVisualizer::Ptr viewer_plane(new pcl::visualization::PCLVisualizer("plane viewer"));
		// viewer_plane->addCoordinateSystem(1);
		// viewer_plane->setBackgroundColor(0.0, 0.0, 0.0);
		// // add plane points
		// PointCloud::Ptr showingCloud(new PointCloud);
		// for (int x = 0; x < mvPlanePoints.size(); ++x) {
		// 	*showingCloud += mvPlanePoints[x];
		// }
		// pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> plane_add(showingCloud);
		// viewer_plane->addPointCloud<pcl::PointXYZRGBL> (showingCloud, plane_add, "sample cloud");
		// viewer_plane->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

		// // add plane boundary
		// PointCloud::Ptr showingBorder(new PointCloud);
		// for (int j = 0; j < mvBoundaryPoints.size(); ++j) {
		// 	for (auto &p : mvBoundaryPoints[j].points) {
		// 		p.r = 0;
		// 		p.g = 255;
		// 		p.b = 0;
		// 	}
		// 	*showingBorder += mvBoundaryPoints[j];
		// }
		// pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> border_add(showingBorder);
		// viewer_plane->addPointCloud<pcl::PointXYZRGBL> (showingBorder, border_add, "sample boader");
		// viewer_plane->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample boader");

		// while (!viewer_plane->wasStopped ())
		// {
		// 	viewer_plane->spinOnce (100);
		// }
	}

    return true;
}

Eigen::VectorXf ObjectDetector::ComputeCuboidParam(pcl::PointCloud <pcl::PointXYZRGBL>& obj_pcl)
{
	Eigen::VectorXf obj_param(9);
	pcl::PointCloud <pcl::PointXYZRGBL>::Ptr obj_cloud = obj_pcl.makeShared();

	//compute 3d centroid of object, get centroid and height
	//centroid is not the center, and average value of points
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*obj_cloud, centroid);
	pcl::PointXYZRGBL whd_min, whd_max;
	pcl::getMinMax3D(*obj_cloud, whd_min, whd_max);
	float height = whd_max.z - whd_min.z;
	float center_z = (whd_max.z + whd_min.z)/2.0;
	// std::cout << "whd_max: " << whd_max.x << " " << whd_max.y << " " << whd_max.z << std::endl;
	// std::cout << "whd_min: " << whd_min.x << " " << whd_min.y << " " << whd_min.z << std::endl;
	// std::cout << "center_z: " << center_z << std::endl;

	//set plane coef, and  project points on plane
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 0.0;
	coefficients->values[1] = 0.0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0.0;
	PointCloud::Ptr obj_projected_cloud(new PointCloud);
	pcl::ProjectInliers<pcl::PointXYZRGBL> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setModelCoefficients(coefficients);
	proj.setInputCloud(obj_cloud);
	proj.filter(*obj_projected_cloud);
	pcl::getMinMax3D(*obj_projected_cloud, whd_min, whd_max);
	// std::cout << "whdcxy: " << whd_max.z << " " << whd_min.z << std::endl;
	// std::cout << "whdcyx: " << whd_max.y - whd_min.y << std::endl;
	// std::cout << "whdcyy: " << whd_max.x - whd_min.x << std::endl;

	float best_area = 99999.9;
	Eigen::Vector3f best_whd;
	best_whd << 0.0, 0.0, height; //fix height
	float best_theta;
	Eigen::Vector3f bbox_trans = centroid.block<3,1>(0,0);
	Eigen::Matrix3f bbox_rot;
	Eigen::Matrix4f Tow = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f Two = Eigen::Matrix4f::Identity();
	Eigen::Vector3f two = centroid.block<3,1>(0,0);
	for (size_t cnt = 0; cnt <= 360; cnt++)
	{
		float theta = cnt * M_PI / 360.0;;
		Eigen::AngleAxisf r_v(theta, Eigen::Vector3f(0, 0, 1));
		Eigen::Matrix3f Rwo = r_v.toRotationMatrix(); //rotation matrix from object coordinate to world coordinate
		Eigen::Matrix3f Row = Rwo.transpose(); //rotation matrix from world coordinate to obj coordinate
		// std::cout << "Rwo: " << Rwo << std::endl;
		// std::cout << "r_v: " << r_v.angle() << std::endl;

		Eigen::Vector3f tow = -1.0f * Rwo.transpose() * two;
		Tow.block<3,3>(0,0) = Row;
		Tow.block<3,1>(0,3) = tow;
		Two.block<3,3>(0,0) = Rwo;
		Two.block<3,1>(0,3) = two;

		// rotate projected cloud plane
		PointCloud::Ptr cloud_trans(new PointCloud);
		pcl::transformPointCloud(*obj_projected_cloud, *cloud_trans, Tow);
		pcl::PointXYZRGBL min_p, max_p;
		// get the min and max of rotated plane, get center
		pcl::getMinMax3D(*cloud_trans, min_p, max_p);
		Eigen::Vector3f co, cw;
		co = 0.5f * (min_p.getVector3fMap() + max_p.getVector3fMap()); //geometric center of obj in obj coordinate system
		Eigen::Affine3f Tco_aff(Two);
		pcl::transformPoint(co, cw, Tco_aff); //get obj centroid in world coordinate
		// std::cout << "min_p: " << min_p.x << " " << min_p.y << " " << min_p.z << std::endl;
		// std::cout << "max_p: " << max_p.x << " " << max_p.y << " " << max_p.z << std::endl;

		Eigen::Vector3f whd;
		whd = max_p.getVector3fMap() - min_p.getVector3fMap(); //get width, height, depth of bounding box
		float area = whd(0) * whd(1); // area = width * depth
		// std::cout << "area: " << area << std::endl;

		// find a minimum bbox that can cover all points
		if(area < best_area)
		{
			best_area = area;
			best_whd(0) = whd(0);
			best_whd(1) = whd(1);
			best_theta = theta;
			// Eigen::Quaternionf q(Rwo);
			// bbox_r = q;
			bbox_rot = Row;
			bbox_trans(0) = cw(0); // update the center
			bbox_trans(1) = cw(1);
		}
	}

	//tx, ty, tz, yaw, w, h, l
	obj_param(0) = bbox_trans(0);
	obj_param(1) = bbox_trans(1);
	obj_param(2) = center_z;
	obj_param(3) = 0.0f;
	obj_param(4) = 0.0f;
	obj_param(5) = best_theta;
	obj_param(6) = best_whd(0);
	obj_param(7) = best_whd(1);
	obj_param(8) = height;
	std::cout << "obj_param: " << obj_param.transpose() << std::endl;
	return obj_param;
}

bool ObjectDetector::ComputeLabelCloud(string& filename)
{
	std::cout << "start reading point cloud in 2d bounding box "  << std::endl;
	Eigen::MatrixXd truth_data(2,6);
	if (!read_all_number_txt(filename, truth_data))
		return -1;
	if(truth_data.rows()==0) // no object
		return false;

	std::cout << "truth_data " << truth_data << std::endl;

	// filter bbox?
	filterBBox2D(truth_data);

	pcl::PointCloud<pcl::PointXYZRGBL> inputCloud;
	int cloudDis = 1; // use 1 to exact more planes
	for ( int m=0; m<imDepth.rows; m+=cloudDis )
	{
		for ( int n=0; n<imDepth.cols; n+=cloudDis )
		{
			int obj_label = checkBbox(n, m, truth_data);
			if(obj_label!=0)
				continue;
			float d = imDepth.ptr<float>(m)[n];
			pcl::PointXYZRGBL p;
			p.z = d;
			p.x = ( n - cx) * p.z / fx;
			p.y = ( m - cy) * p.z / fy;

            p.b = imRGB.ptr<uchar>(m)[n*3];
            p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
            p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
			p.label = 0;
			inputCloud.points.push_back(p);

			// if(obj_label!=0)
			// {
			// 	p.r = 177;
			// 	p.g = 0;
			// 	p.b = 0;
			// 	p.label = obj_label;
			// 	obj_points_cam.points.push_back(p);
			// }
		}
	}
	
	
    for (size_t i = 0; i < truth_data.rows(); i++)
    {
		if(truth_data(i,0)!= 1)
			continue;
		if(truth_data(i,5) < 0.5)
			continue;
        Eigen::Vector4d obj_bbox = truth_data.row(i).segment<4>(1);
        int x_min = int(obj_bbox(0));
        int y_min = int(obj_bbox(1));
        int x_max = min(int(obj_bbox(0)+obj_bbox(2)), imDepth.cols);
        int y_max = min(int(obj_bbox(1)+obj_bbox(3)), imDepth.rows);
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points_cam;
		int cloudDis = 1; // use 1 to exact more planes
		for ( int m=y_min; m<y_max; m+=cloudDis )
		{
			for ( int n=x_min; n<x_max; n+=cloudDis )
			{
				float d = imDepth.ptr<float>(m)[n];
				pcl::PointXYZRGBL p;
				p.z = d;
				p.x = ( n - cx) * p.z / fx;
				p.y = ( m - cy) * p.z / fy;
				p.b = imRGB.ptr<uchar>(m)[n*3];
				p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
				p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
				obj_points_cam.points.push_back(p);
			}
		}
		obj_points_cam.height = ceil((y_max-y_min)/float(cloudDis));
		obj_points_cam.width = ceil((x_max-x_min)/float(cloudDis));

		// estimate plane before transfrom
		// filter outlier by planes (from exprience, it works very good)
		// reason, from camera capture principle, points of outlier are further than objects
		// 由于相机采集数据原理，平台相比于物体散射严重，点云质量不行，无法提取平面，相反通过提取平面反而能提出所有物体点云
		pcl::PointCloud<pcl::PointXYZRGBL> obj_points_new;
		bool obj_exact = ExtractObjectFromOrganizedPlanes(obj_points_cam, obj_points_new);
		if(obj_exact == false) // no object point are detected
			continue;
		std::cout << "obj points raw: " << obj_points_cam.points.size() << " after filter: " << obj_points_new.points.size() << std::endl;

		for (size_t kk = 0; kk < obj_points_new.points.size(); kk++)
		{
			obj_points_new.points[kk].label = truth_data(i,0);
			obj_points_new.points[kk].b = 0;
			obj_points_new.points[kk].g = 0;
			obj_points_new.points[kk].r = 177;
		}
		
		inputCloud = inputCloud + obj_points_new;
		// // save info
		// pcl::PointCloud<pcl::PointXYZRGBL>  obj_points_world;
		// pcl::transformPointCloud(obj_points_new, obj_points_world, Twc);
		// bbox_points_raw.push_back(obj_points_world);
        // bbox_name_list.push_back(truth_data(i,0));
		// bbox_xywh_list.push_back(obj_bbox.cast<float>());

        // if(vis_enable)
        // {
        //     viewer = new pcl::visualization::PCLVisualizer("object_raw_cloud_viewer");
        //     viewer->addCoordinateSystem(1);
        //     viewer->setBackgroundColor(0.0, 0.0, 0.0);
        //     pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = obj_points_world.makeShared();
        //     pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        //     viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show cloud");
        //     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud");
        //     while (!viewer->wasStopped ())
        //     {
        //         viewer->spinOnce (100);
        //     }
        //     viewer->close();
        // }
    }

    // // check if we need to transform point cloud to world coordinates
	pcl::PointCloud<pcl::PointXYZRGBL>  raw_points_world;
	pcl::transformPointCloud(inputCloud, raw_points_world, Twc);


    if(vis_enable)
    {
        viewer = new pcl::visualization::PCLVisualizer("label pcl");
        viewer->addCoordinateSystem(1);
		viewer->setBackgroundColor(1.0, 1.0, 1.0);
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = raw_points_world.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show cloud");
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }
        viewer->close();
    }
	
}

int ObjectDetector::checkBbox(const int& x, const int& y, const Eigen::MatrixXd& bboxes)
{
	int label = 0;
	for (size_t i = 0; i < bboxes.rows(); i++)
	{
		if(bboxes(i,0)!= 1)
			continue;
		if(bboxes(i,5) < 0.5)
			continue;
		if(x>bboxes(i,1) && x<bboxes(i,1)+bboxes(i,3) &&
			y>bboxes(i,2) && y<bboxes(i,2)+bboxes(i,4)	)
		{
			label = bboxes(i,0);
		}
	}
	return label;
}


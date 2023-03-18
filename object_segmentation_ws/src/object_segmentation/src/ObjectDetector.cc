#include "ObjectDetector.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <boost/make_shared.hpp>


#include <cmath>



using namespace std;


namespace object_segmentation {

bool ObjectDetector::ComputePointCloud()
{
	std::cout << "\n----- ComputePointCloud -----" << std::endl;
	// translate to point cloud
	pcl::PointCloud<pcl::PointXYZRGB> inputCloud;
	int cloudDis = 1; // use 1 to exact more planes
	for ( int m=0; m<imDepth.rows; m+=cloudDis )
	{
		for ( int n=0; n<imDepth.cols; n+=cloudDis )
		{
			float d = imDepth.ptr<float>(m)[n];
			if(isnan(d))
			{
				continue;
			}
			PointT p;
			p.z = d;
			p.x = ( n - cx) * p.z / fx;
			p.y = ( m - cy) * p.z / fy;
            p.b = imRGB.ptr<uchar>(m)[n*3];
            p.g = imRGB.ptr<uchar>(m)[n*3 + 1];
            p.r = imRGB.ptr<uchar>(m)[n*3 + 2];
			inputCloud.points.push_back(p);
		}
	}
    // // check if we need to transform point cloud to world coordinates
	pcl::PointCloud<pcl::PointXYZRGB>  raw_points_world;
	pcl::transformPointCloud(inputCloud, raw_points_world, Twc);
	rawCloudPoints = raw_points_world;

    return true;
}

bool ObjectDetector::ClearData()
{
	std::cout << "----- frame dataset reset -----" << std::endl;
	bbox_name_list.clear();
	bbox_xywh_list.clear();
	rawCloudPoints.clear();
	bbox_points_raw.clear();
	bbox_points_filter.clear();
	object_param_list.clear();
	object_hsv.clear();
	mspFrameObjects.clear();
    return true;
}


bool ObjectDetector::ReadBBox2DCloud(std::string& filename)
{
	std::cout << "start reading point cloud in 2d bounding box "  << std::endl;
	Eigen::MatrixXd truth_data(2,6);
	if (!read_all_number_txt(filename, truth_data))
		return -1;
	if(truth_data.rows()==0) // no object
		return false;

	// std::cout << "truth_data:\n " << truth_data << std::endl;

	// filter bbox?
	filterBBox2D(truth_data);

    for (size_t i = 0; i < truth_data.rows(); i++)
    {
		if(truth_data(i,0)!= 1)
			continue;
		if(truth_data(i,5) < 0.90)
			continue;
        Eigen::Vector4d obj_bbox = truth_data.row(i).segment<4>(1);
        int x_min = int(obj_bbox(0));
        int y_min = int(obj_bbox(1));
        int x_max = min(int(obj_bbox(0)+obj_bbox(2)), imDepth.cols);
        int y_max = min(int(obj_bbox(1)+obj_bbox(3)), imDepth.rows);
		pcl::PointCloud<pcl::PointXYZRGB> obj_points_cam;
		int cloudDis = 1; // use 1 to exact more planes
		for ( int m=y_min; m<y_max; m+=cloudDis )
		{
			for ( int n=x_min; n<x_max; n+=cloudDis )
			{
				float d = imDepth.ptr<float>(m)[n];
				PointT p;
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


		cout << "obj_points_cam.width :" << obj_points_cam.width << endl;

		// estimate plane before transfrom
		// filter outlier by planes (from exprience, it works very good)
		// reason, from camera capture principle, points of outlier are further than objects
		// 由于相机采集数据原理，平台相比于物体散射严重，点云质量不行，无法提取平面，相反通过提取平面反而能提出所有物体点云
		pcl::PointCloud<pcl::PointXYZRGB> obj_points_new;
		bool obj_exact = ExtractObjectFromOrganizedPlanes(obj_points_cam, obj_points_new);
		if(obj_exact == false) // no object point are detected
			continue;
		std::cout << "obj points raw: " << obj_points_cam.points.size() << " after filter: " << obj_points_new.points.size() << std::endl;

		// save info
		pcl::PointCloud<pcl::PointXYZRGB>  obj_points_world;
		pcl::transformPointCloud(obj_points_new, obj_points_world, Twc);
		bbox_points_raw.push_back(obj_points_world);
        bbox_name_list.push_back(truth_data(i,0));
		bbox_xywh_list.push_back(obj_bbox.cast<float>());
    }
    return true;
}

bool ObjectDetector::FilterOutliers()
{
	std::cout << "start FilterOutliers "  << std::endl;
	std::cout << "ground value: " << ground_min << " " << ground_max << std::endl;
	std::cout << "wall value: " << wall_min << " " << wall_max << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_wall(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_outlier(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_euclidean(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (size_t i = 0; i < bbox_points_raw.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB> obj_points = bbox_points_raw[i];
		// std::cout << "obj_points.size() " << obj_points.points.size() << std::endl;
		
		// // filter voxel
		pcl::VoxelGrid<pcl::PointXYZRGB> vgFilter;
		vgFilter.setInputCloud(obj_points.makeShared());
		vgFilter.setLeafSize(0.02f, 0.02f, 0.02f);
		vgFilter.setSaveLeafLayout(true);
		vgFilter.filter(obj_points);
		std::cout << "obj_points.size() " << obj_points.points.size() << std::endl;

		// filter ground
	  	pcl::PassThrough<pcl::PointXYZRGB> pass;
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


		// filter StatisticalOutlierRemoval
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud (cloud_without_ground);
		sor.setMeanK(50);
		sor.setStddevMulThresh (1.0);
		sor.filter(*cloud_without_outlier);

		*cloud_euclidean = *cloud_without_outlier;
		// // filter EuclideanClusterExtraction
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud (cloud_euclidean);
		std::vector<pcl::PointIndices> cluster_indices;
		//Do eucledean clustering
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
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
		pcl::PointIndices::Ptr temp_indices(new pcl::PointIndices);
		temp_indices->indices = cluster_indices[max_indice_id].indices;
		pcl::ExtractIndices<pcl::PointXYZRGB> extractIndices;
		extractIndices.setInputCloud(cloud_euclidean);
		extractIndices.setIndices(temp_indices);
		extractIndices.setNegative (false);
		extractIndices.filter(*cloud_euclidean);

        bbox_points_filter.push_back(*cloud_euclidean);

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
	}

}

void ObjectDetector::GetObjectHSV()
{
	for (size_t obj_id = 0; obj_id < bbox_xywh_list.size(); obj_id++)
	{
		Eigen::Vector4f cuboid_xywh = bbox_xywh_list[obj_id];
		// std::cout << "cuboid_xywh " << cuboid_xywh.transpose() << std::endl;
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
	std::cout << "SaveFrameObjects: " << std::endl;
	for (size_t i = 0; i < bbox_xywh_list.size(); i++)
	{
		MapObject *mvObj = new MapObject();
		mvObj->mnClass = bbox_name_list[i];
		mvObj->bbox = bbox_xywh_list[i];
		mvObj->loc = object_param_list[i].head(3);
		mvObj->rpy = object_param_list[i].segment<3>(3);
		mvObj->dim = object_param_list[i].tail(3);
		mvObj->hsv = object_hsv[i].clone();
		Eigen::Vector4f  corner_point(mvObj->loc[0]-0.5*mvObj->dim[0],mvObj->loc[1]+0.5*mvObj->dim[1],
								mvObj->loc[0]+0.5*mvObj->dim[0],mvObj->loc[1]-0.5*mvObj->dim[1]);
		mvObj->bbox_projected = corner_point;
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
		float min_dist = 1.0;
		float bad_dist = 0.20;
		for (size_t j = 0; j < mspMapObjects.size(); j++)
		{
			MapObject* mMO = mspMapObjects[j];
			// step 1: check class
			if(mFO->mnClass != mMO->mnClass)
				continue;
			// step 2 : HSV
			double correl = cv::compareHist(mFO->hsv, mMO->hsv, CV_COMP_CORREL);
			std::cout << "correl: " << correl << std::endl;
			if(correl < 0.1)
				continue;
			// step3: distance
			double dist = sqrt(pow(mFO->loc(0) - mMO->loc(0) , 2) +
								pow(mFO->loc(1) - mMO->loc(1) , 2) +
								pow(mFO->loc(2) - mMO->loc(2) , 2) );
			if(dist > 1.0)
				continue;
			// step: cal IOU
		    float IoU_cal = IoUCal(mFO->bbox_projected,mMO->bbox_projected);			
			// cout << "IoU Cal: " << IoUCal(mFO->bbox_projected,mMO->bbox_projected) << endl;
			// cout << "mFO->bbox_projected: " << mFO->bbox_projected << "mMO->bbox_projected: " <<mMO->bbox_projected << endl;

			// step 4 find  association
			if(dist < min_dist)
			{
				min_dist = dist;
				mFO->find_asso = true;
				mFO->asso_id = mMO->mnId;
			}
			// step 5 check best association quality, if association object has big distance bias, then delete it in update step
			if(dist < min_dist && dist > bad_dist && IoU_cal <0.7)
			{	
				mFO->asso_bad =true;
			}

			// cal area, initial value should have stable and whole observation
			float mF0_width = mFO->dim[0];
			float mF0_depth = mFO->dim[1];
			if(mF0_width * mF0_depth < 0.2){
				mFO->asso_bad =true;
			}


		} // loop map object
		std::cout << "mspFrameObjects: " << i << " find_asso" << mFO->find_asso << std::endl;
	} // loop frame object
	return true;
}

void ObjectDetector::UpdateMapObject(std::vector<MapObject*>& mspMapObjects)
{	
	float area_threshold_down = 0.18;
	float area_threshold_up = 0.26;

	std::cout << "UpdateMapObject: " << std::endl;
	for (size_t i = 0; i < mspFrameObjects.size(); i++)
	{
		MapObject* mFO = mspFrameObjects[i];
		if (mFO->find_asso == false)
		{
			//check object area, if object's area too small or too big, skip that
			float mF0_width = mFO->dim[0];
			float mF0_depth = mFO->dim[1];
			if(mF0_width * mF0_depth < area_threshold_down || mF0_width * mF0_depth > area_threshold_up )
			{
				continue;
			}
			std::cout << "init map object" << std::endl;
			mFO->InitializeMapObject(mspMapObjects);
		}
		else if(mFO->asso_bad == false)
		{
			MapObject* mMO = mspMapObjects[mFO->asso_id];
			std::cout << "mMO->points: "<< mMO->points.size() << std::endl;
			pcl::VoxelGrid<pcl::PointXYZRGB> vgFilter;
			vgFilter.setInputCloud(mMO->points.makeShared());
			vgFilter.setLeafSize(0.02f, 0.02f, 0.02f);
			vgFilter.setSaveLeafLayout(true);
			vgFilter.filter(mMO->points);
			mMO->MergeWithFrameObject(mFO);
			Eigen::VectorXf obj_param = ComputeCuboidParam(mMO->points);
			mMO->loc = obj_param.head(3);
			mMO->rpy = obj_param.segment<3>(3);
			mMO->dim = obj_param.tail(3);
			std::cout << "mMO->points: "<<mMO->points.size() << std::endl;
		}
	}
}

Eigen::VectorXf ObjectDetector::ComputeCuboidParam(pcl::PointCloud <pcl::PointXYZRGB>& obj_pcl)
{
	Eigen::VectorXf obj_param(9);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr obj_cloud = obj_pcl.makeShared();

	//compute 3d centroid of object, get centroid and height
	//centroid is not the center, and average value of points
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*obj_cloud, centroid);
	pcl::PointXYZRGB whd_min, whd_max;
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
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
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
		pcl::PointXYZRGB min_p, max_p;
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

bool ObjectDetector::ExtractObjectFromOrganizedPlanes(pcl::PointCloud<pcl::PointXYZRGB>& bbox_pcl,
													pcl::PointCloud<pcl::PointXYZRGB>& obj_pcl)
{

	int min_plane = MultiPlane_SizeMin;//500;
	float AngTh = MultiPlane_AngleThre;//2.0;
	float DisTh = MultiPlane_DistThre;//0.02;

	// // firstly, compute normal
	pcl::PointCloud<PointT>::Ptr  input_cloud = bbox_pcl.makeShared();
	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.01f); // 0.05
	ne.setNormalSmoothingSize(15.0f); // 10.0
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
	std::cout << "plane inliers: " << inliers.size() << std::endl;
	if (inliers.size()==0)
	{
		std::cout << "no object point cloud detected" << std::endl;
		return false;
	}

	// // thirdly, exact and filter point cloud
	// pcl::PointCloud <pcl::PointXYZRGB> obj_pcl; 
	std::vector<cv::Mat> mvPlaneCoefficients; // plane normal 
	std::vector<pcl::PointIndices > mvPlaneIndices; // plane points 
	std::vector<pcl::PointCloud <pcl::PointXYZRGB> > mvPlanePoints; // plane points 
    std::vector<pcl::PointCloud <pcl::PointXYZRGB> > mvBoundaryPoints; // plane boundary, just for visualization
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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

    return true;
}

bool ObjectDetector::computeConvexHullPcl(pcl::PointCloud<pcl::PointXYZRGB>& obj_pcl,
										pcl::PointCloud<pcl::PointXYZRGB>& proj_pcl)
{
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setModelCoefficients (coefficients);
	proj.setInputCloud (obj_pcl.makeShared());
	proj.filter (*cloud_projected);

	if (cloud_projected->points.size() == 0){
		return false;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ConvexHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud (cloud_projected);
	chull.setDimension(2);
	chull.reconstruct(*cloud_hull);

	proj_pcl = *cloud_hull;
      
}

}
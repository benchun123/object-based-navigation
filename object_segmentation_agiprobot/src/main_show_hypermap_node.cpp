#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <matrix_utils.h>


using namespace std;
typedef pcl::PointXYZRGBL PointT;
typedef pcl::PointCloud <PointT> PointCloud;

void LoadGridMap(const string &grid_map_folder, pcl::PointCloud<pcl::PointXYZRGBL>& grid_pcl);
void ShowHyperMap(pcl::PointCloud<pcl::PointXYZRGBL>& gridPoints, pcl::PointCloud<pcl::PointXYZRGBL>& rawpoints, 
				Eigen::MatrixXd& obj_data);
int main(int argc, char** argv)
{
	if (argc != 2 )
	{
		cout << "Usage: ./show_hypermap_node path/to/data" << endl;
		return -1;
	}

    std::string based_folder = std::string(argv[1]);

    // // load offline grid map
	pcl::PointCloud<pcl::PointXYZRGBL> gridCloudPoints; // convert from grid map
    std::string map_folder = based_folder + "/maps/";
	std::cout << "load grip map at " << map_folder << std::endl;
    LoadGridMap(map_folder, gridCloudPoints);

    // // raw point cloud
	std::string ply_filename = based_folder + "/result_pcl.ply";
	std::cout << "load raw point cloud at " << ply_filename << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_raw (new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::PLYReader Reader;
    Reader.read(ply_filename, *cloud_raw);

    // // load object info
	std::string txt_filename = based_folder + "/gt_obj.txt";
	std::cout << "load object result at " << txt_filename << std::endl;
	Eigen::MatrixXd truth_data(2,10);
	if (!read_all_number_txt(txt_filename, truth_data))
		return -1;
	std::cout << "truth_data\n " << truth_data << std::endl;

    // // visualization
	ShowHyperMap(gridCloudPoints, *cloud_raw, truth_data);

	return 0;

}


void LoadGridMap(const string &grid_map_folder, pcl::PointCloud<pcl::PointXYZRGBL>& grid_pcl)
{
    std::string map_param = grid_map_folder + "/map.yaml";
    cv::FileStorage fSettings(map_param, cv::FileStorage::READ);
    std::string map_img = grid_map_folder + fSettings["image"];
    float map_resolution = fSettings["resolution"];
    float map_cam_origin_x = fSettings["origin.x"];
    float map_cam_origin_y = fSettings["origin.y"];
    float map_cam_origin_z = fSettings["origin.z"];
    int map_negate = fSettings["negate"];
    float map_occ_thre = fSettings["occupied_thresh"];
    float map_free_thre = fSettings["free_thresh"];

	cv::Mat imGrid = cv::imread(map_img, CV_LOAD_IMAGE_UNCHANGED);
	// std::cout << "imGrid " << imGrid.rows << " " << imGrid.cols << " " << imGrid.size << std::endl;
	// cv::Mat plot_2d_img = imGrid.clone();
	// cv::imshow("2d bounding box", plot_2d_img);
	// cv::waitKey(0);

	pcl::PointCloud<pcl::PointXYZRGBL> inputCloud;
	int cloudDis = 1; // use 1 to exact more planes
	for ( int m=0; m<imGrid.rows; m+=cloudDis )
	{
		for ( int n=0; n<imGrid.cols; n+=cloudDis )
		{
			uchar color = imGrid.ptr<uchar>(m)[n];
            float occ = (255 - color) / 255.0;
            if(occ > map_free_thre)
                continue;
			PointT p;
			p.x = n*map_resolution + map_cam_origin_x;
			p.y = m*map_resolution + map_cam_origin_y;
			p.z = 0.0;
			p.r = 177;
			p.g = 177;
			p.b = 177;
			inputCloud.points.push_back(p);
		}
	}

    // convert from image coordinates to map coordinates
    Eigen::VectorXd trans_im_map(7);
    trans_im_map << 0, 0, 0.0, 1, 0, 0, 0 ;
	Eigen::Matrix4d Tim;	// odom to base_link
	Tim.setIdentity();
	Tim.block(0,0,3,3) = Eigen::Quaterniond(trans_im_map(6),trans_im_map(3),trans_im_map(4),trans_im_map(5)).toRotationMatrix();
	Tim.col(3).head(3) = Eigen::Vector3d(trans_im_map(0), trans_im_map(1), trans_im_map(2));
    // std::cout << "Tim:\n " << Tim << std::endl;

    // convert from map to odom coordinates, data comes from tf tree
    Eigen::VectorXd trans_odom_map(7);
    trans_odom_map << 5.3140, 3.5875, 0, 0, 0, 0, 1 ;
	Eigen::Matrix4d Tom;	// odom to base_link
	Tom.setIdentity();
	Tom.block(0,0,3,3) = Eigen::Quaterniond(trans_odom_map(6),trans_odom_map(3),trans_odom_map(4),trans_odom_map(5)).toRotationMatrix();
	Tom.col(3).head(3) = Eigen::Vector3d(trans_odom_map(0), trans_odom_map(1), trans_odom_map(2));
    // std::cout << "Tom:\n " << Tom << std::endl;

	Eigen::Matrix4d Tio = Tom.inverse()*Tim;	// odom to base_link
    // std::cout << "Tio:\n " << Tio << std::endl;
	pcl::transformPointCloud(inputCloud, inputCloud, Tio);
    grid_pcl = inputCloud;
}

void ShowHyperMap(pcl::PointCloud<pcl::PointXYZRGBL>& gridPoints, pcl::PointCloud<pcl::PointXYZRGBL>& rawpoints, 
				Eigen::MatrixXd& obj_data)
{
	// // visualization
	pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("object_map");
	viewer->addCoordinateSystem(1);
	viewer->setBackgroundColor(1.0, 1.0, 1.0);
    // add grid map to viewer
    if(gridPoints.points.size()>0)
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = gridPoints.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "grid_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "grid_cloud");
    }

    if(rawpoints.points.size()>0)
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = rawpoints.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "raw_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "raw_cloud");
    }

    // // add object bounding box
    for (size_t obj_id = 0; obj_id < obj_data.rows(); obj_id++)
	{
		// add cuboid
		Eigen::Vector3f bbox_trans = obj_data.row(obj_id).segment<3>(1).cast<float>();
		Eigen::Vector3f bbox_rpy = obj_data.row(obj_id).segment<3>(4).cast<float>();
		Eigen::Vector3f bbox_whd = obj_data.row(obj_id).tail(3).cast<float>();
		Eigen::AngleAxisf obj_roll(bbox_rpy(0), Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf obj_pitch(bbox_rpy(1), Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf obj_yaw(bbox_rpy(2), Eigen::Vector3f::UnitZ());
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



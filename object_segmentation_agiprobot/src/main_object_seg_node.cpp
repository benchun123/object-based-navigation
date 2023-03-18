#include <matrix_utils.h>
#include <Parameters.h>
#include <ObjectDetector.h>
#include <MapObject.h>

#include <opencv2/core/eigen.hpp>

using namespace std;
void LoadImagesIndex(const string &strAssociationFilename, vector<string> &vRGBIndex,
                    vector<string> &vDepthIndex, vector<Eigen::VectorXd>& vOdomIndex);
int ReadAdditionalParam (string& strSettingPath);
void LoadGridMap(const string &grid_map_folder, pcl::PointCloud<pcl::PointXYZRGBL>& grid_pcl);
void ShowMapObjectPoints(std::vector<MapObject*>& mspMapObjects, pcl::PointCloud<pcl::PointXYZRGBL>& rawPoints,
                        pcl::PointCloud<pcl::PointXYZRGBL>& gridPoints);

int main(int argc, char** argv)
{
	if (argc != 2 )
	{
		cout << "Usage: parcel_detector_node path/to/data" << endl;
		return -1;
	}

    // Retrieve paths to images
    string based_folder = string(argv[1]);
    vector<string> vRGBIndex;
    vector<string> vDepthIndex;
    vector<Eigen::VectorXd> vOdomIndex;
    string strAssociationFilename = based_folder + "/associate_laser_odom.txt";
    LoadImagesIndex(strAssociationFilename, vRGBIndex, vDepthIndex, vOdomIndex);

    // // load offline grid map
	pcl::PointCloud<pcl::PointXYZRGBL> gridCloudPoints; // convert from grid map
    string map_folder = based_folder + "/maps/";
    LoadGridMap(map_folder, gridCloudPoints);

    // // visualization // collect info and show
    std::vector<MapObject*> mspMapObjects;
	pcl::PointCloud<pcl::PointXYZRGBL> mapCloudPoints; // convert from depth image

    // Main loop
    // load additional parameters, for debug
    int nImages = vRGBIndex.size();
    string file_name = string(argv[1]) + "/Additional.yaml";
    nImages = ReadAdditionalParam(file_name);
    for(int ni=image_start; ni<nImages; ni+=image_fps)
    {
		std::cout << "------------ frame_index " << ni << "------------" << std::endl;
        // Read image and depthmap from file
        string color_filename = string(argv[1]) + "/rgb/" + vRGBIndex[ni] + ".png";
        string depth_filename = string(argv[1]) + "/depth/" + vDepthIndex[ni] + ".png";
        string label_filename = string(argv[1]) + "/labels/" + vRGBIndex[ni] + ".txt";
        // string additional_param_file = string(argv[1]) + "/Additional.yaml";
        std::cout << "color_filename: " << color_filename << std::endl;
        std::cout << "depth_filename: " << depth_filename << std::endl;
        // string output_filename = string(argv[1]) + "/result_depth/"+ vRGBIndex[ni] + ".txt";
        // std::cout << "output_folder: " << output_filename << std::endl;

		ObjectDetector obj_det;
		obj_det.ReadColorImage(color_filename);
		obj_det.ReadDepthImage(depth_filename);  // require depth param
		obj_det.ReadOdomFile(vOdomIndex[ni]);

        // collect all raw point cloud for visualization
	    obj_det.ComputePointCloud();
        mapCloudPoints += obj_det.rawCloudPoints;
        pcl::VoxelGrid<pcl::PointXYZRGBL> vgFilter; // add a filter to decrease the number
        vgFilter.setInputCloud(mapCloudPoints.makeShared());
        // vgFilter.setLeafSize(0.04f, 0.04f, 0.04f); // make it bigger to accelerate
        vgFilter.setLeafSize(0.02f, 0.02f, 0.02f); // make it bigger to accelerate
        vgFilter.setSaveLeafLayout(true);
        vgFilter.filter(mapCloudPoints);
        // mapCloudPoints = obj_det.rawCloudPoints;

        if(vis_opencv_enable)
	        obj_det.ShowBBox2D(label_filename); //show bbox in image

        double start_time = what_time_is_it_now();

        // bool detect_object = obj_det.ComputeLabelCloud(label_filename);// transform to world cooridnates
        bool detect_object = obj_det.ReadBBox2DCloud(label_filename);// transform to world cooridnates
        if(detect_object)
        {
            obj_det.FilterOutliers();
            obj_det.ClusterObjects();
            obj_det.GetObjectHSV();
            // obj_det.ProjectObjectOntoImage();
            obj_det.SaveFrameObjects();
            obj_det.AssociateWithMapObject(mspMapObjects);
            obj_det.UpdateMapObject(mspMapObjects);
        }
        std::cout << "mspMapObjects.size(): " << mspMapObjects.size() << std::endl;
        // for (size_t i = 0; i < mspMapObjects.size(); i++)
        // {
        //     std::cout << "map object " << i << " " << mspMapObjects[i]->loc.transpose()
        //     << mspMapObjects[i]->rpy.transpose()   << mspMapObjects[i]->dim.transpose() << std::endl;
        // }
        double segment_time = what_time_is_it_now() - start_time;
        std::cout << "segment_time " << segment_time << std::endl;

        // if(vis_result_enable)
        //     ShowMapObjectPoints(mspMapObjects, mapCloudPoints);
    }
    
    if(save_result_enable)
    {
        std::string txt_filename = string(argv[1]) + "/"+ save_obj_file;
        ofstream f_cuboid;
        f_cuboid.open(txt_filename.c_str());
        for (size_t id = 0; id < mspMapObjects.size(); id++)
        {
            MapObject* Obj_tmp = mspMapObjects[id];
            f_cuboid << Obj_tmp->mnClass << " " << Obj_tmp->loc(0) << " " << Obj_tmp->loc(1) << " " << Obj_tmp->loc(2)
                    << " " << Obj_tmp->rpy(0) << " " << Obj_tmp->rpy(1) << " " << Obj_tmp->rpy(2)
                    << " " << Obj_tmp->dim(0) << " " << Obj_tmp->dim(1) << " " << Obj_tmp->dim(2)
                    << endl;
            std::cout << Obj_tmp->mnClass << " " << Obj_tmp->loc(0) << " " << Obj_tmp->loc(1) << " " << Obj_tmp->loc(2)
                    << " " << Obj_tmp->rpy(0) << " " << Obj_tmp->rpy(1) << " " << Obj_tmp->rpy(2)
                    << " " << Obj_tmp->dim(0) << " " << Obj_tmp->dim(1) << " " << Obj_tmp->dim(2)
                    << std::endl;
        }
        f_cuboid.close();
        std::string ply_filename = string(argv[1]) + "/"+ save_pcl_file;
        pcl::io::savePLYFile(ply_filename, mapCloudPoints);
        std::string obj_ply_filename = string(argv[1]) + "/result_obj_pcl.ply";
        pcl::PointCloud<pcl::PointXYZRGBL> mapObjPonts;
        for (size_t id = 0; id < mspMapObjects.size(); id++)
            mapObjPonts += mspMapObjects[id]->points;
        pcl::io::savePLYFile(obj_ply_filename, mapObjPonts);

        std::cout << "save final object to " << txt_filename << std::endl;
        std::cout << "save raw pcl file to " << ply_filename << std::endl;
        std::cout << "save obj pcl file to " << obj_ply_filename << std::endl;
    }

    if(vis_result_enable)
        ShowMapObjectPoints(mspMapObjects, mapCloudPoints, gridCloudPoints);
	
    return 0;

}



void LoadImagesIndex(const string &strAssociationFilename, vector<string> &vRGBIndex,
                    vector<string> &vDepthIndex, vector<Eigen::VectorXd>& vOdomIndex)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string id;
            string rgb;
            string depth;
            ss >> id;
            vRGBIndex.push_back(id);
            ss >> rgb;
            ss >> id;
            ss >> depth;
            vDepthIndex.push_back(id);
            Eigen::VectorXd odom(7);
            ss >> id;
            for (size_t i = 0; i < 7; i++)
                ss >> odom(i);
            vOdomIndex.push_back(odom);
        }
    }
}

int ReadAdditionalParam (string& strSettingPath)
{
	// Load camera parameters from settings file
	std::cout << "start reading additional param "  << std::endl;
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
	mDepthMapFactor = fSettings["DepthMapFactor"];
	if(fabs(mDepthMapFactor)<1e-5)
		mDepthMapFactor=1;
	else
		mDepthMapFactor = 1.0f/mDepthMapFactor;
    Kalib << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- mDepthMapFactor: " << mDepthMapFactor << endl;

    // Option 1: get tf base to cam manually 
	Eigen::VectorXd tf_base_cam(7); 
	tf_base_cam << fSettings["T_base_cam.x"], fSettings["T_base_cam.y"],
					fSettings["T_base_cam.z"], fSettings["T_base_cam.qx"],
					fSettings["T_base_cam.qy"], fSettings["T_base_cam.qz"],
					fSettings["T_base_cam.qw"];
	// Eigen::Matrix4d Tbc;	// baselink to cam
	Tbc.setIdentity();
	Tbc.block(0,0,3,3) = Eigen::Quaterniond(tf_base_cam(6),tf_base_cam(3),tf_base_cam(4),tf_base_cam(5)).toRotationMatrix();
	Tbc.col(3).head(3) = Eigen::Vector3d(tf_base_cam(0), tf_base_cam(1), tf_base_cam(2));
    cout << "- Tbc: \n" << Tbc << endl;

    // // Option 2: get tf base to cam from calibration
    // // tf laser to camera: pay attention !!! from laser to camera
    // // calibration result: https://github.com/MegviiRobot/CamLaserCalibraTool/blob/master/README.md
    // cv::Mat extrinsicTlc(4,4,CV_32FC1);
    // fSettings["extrinsicTlc"] >> extrinsicTlc;
    // std::cout <<"Load extrinsicTlc: \n"<<extrinsicTlc<<std::endl;
    // Eigen::Matrix4d Tlc;
    // Tlc.setIdentity();
    // if(!extrinsicTlc.empty())
    //     cv::cv2eigen(extrinsicTlc, Tlc);
    // else
    //     std::cerr <<" You Do not have calibra result Tlc. We use a Identity matrix." << std::endl;
    // std::cout <<"Tlc: \n"<<Tlc<<std::endl;
	// Eigen::Matrix4d rotate_z_180; // in agiprobot setting, the camera is opposite
	// rotate_z_180 << -1.0, 0.0, 0.0, 0.0,
	// 				0.0, -1.0, 0.0, 0.0,
	// 				0.0, 0.0, 1.0, 0.0,
	// 				0.0, 0.0, 0.0, 1.0;
    // Tlc = Tlc * rotate_z_180;
    // std::cout <<"Tlc: \n"<<Tlc<<std::endl;

	// Eigen::VectorXd tf_base_laser(7); // tf from base to laser
	// tf_base_laser << fSettings["T_base_laser.x"], fSettings["T_base_laser.y"],
	// 				fSettings["T_base_laser.z"], fSettings["T_base_laser.qx"],
	// 				fSettings["T_base_laser.qy"], fSettings["T_base_laser.qz"],
	// 				fSettings["T_base_laser.qw"];
	// Eigen::Matrix4d Tbl;	// baselink to laser
	// Tbl.setIdentity();
	// Tbl.block(0,0,3,3) = Eigen::Quaterniond(tf_base_laser(6),tf_base_laser(3),tf_base_laser(4),tf_base_laser(5)).toRotationMatrix();
	// Tbl.col(3).head(3) = Eigen::Vector3d(tf_base_laser(0), tf_base_laser(1), tf_base_laser(2));
    // std::cout <<"Tbl: \n"<<Tbl<<std::endl;

	// // Eigen::Matrix4d Tbc;	// baselink to cam
    // Tbc = Tbl * Tlc;

    MultiPlane_SizeMin = int(fSettings["MultiPlane.SizeMin"]);
    MultiPlane_AngleThre = float(fSettings["MultiPlane.AngleThre"]);
    MultiPlane_DistThre = float(fSettings["MultiPlane.DistThre"]);
    ground_min = fSettings["Ground.min"];
    ground_max = fSettings["Ground.max"];
    wall_min = fSettings["Wall.min"];
    wall_max = fSettings["Wall.max"];

	image_start = int(fSettings["Debug.Image_start"]);
    image_fps = int(fSettings["Debug.Image_fps"]);
    image_end = int(fSettings["Debug.Image_end"]);
    vis_enable = int(fSettings["Vis.showpcl"]);
    vis_opencv_enable = int(fSettings["Vis.showopencv"]);
    vis_result_enable = int(fSettings["Vis.showresult"]);
    save_result_enable = int(fSettings["Save.result"]);
    save_obj_file = std::string(fSettings["Save.obj_file"]);
    save_pcl_file = std::string(fSettings["Save.pcl_file"]);

    return image_end;
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

void ShowMapObjectPoints(std::vector<MapObject*>& mspMapObjects, pcl::PointCloud<pcl::PointXYZRGBL>& rawPoints,
                        pcl::PointCloud<pcl::PointXYZRGBL>& gridPoints)
{
	// // visualization
	pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("object_map");
	// viewer->addCoordinateSystem(1);
	viewer->setBackgroundColor(1.0, 1.0, 1.0);
    // add points to viewer
    if(rawPoints.points.size()>0)
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = rawPoints.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show_cloud");
    }

    if(gridPoints.points.size()>0)
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = gridPoints.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
        viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "grid_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "grid_cloud");
    }

    for (size_t obj_id = 0; obj_id < mspMapObjects.size(); obj_id++)
	{
		// // add object point cloud
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show_raw = mspMapObjects[obj_id]->points.makeShared();
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZRGBL>);
		pcl::VoxelGrid<pcl::PointXYZRGBL> vgFilter;
		vgFilter.setInputCloud(cloud_show_raw);
		vgFilter.setLeafSize(0.02f, 0.02f, 0.02f);
		vgFilter.setSaveLeafLayout(true);
		vgFilter.filter(*cloud_show);

		// pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_show = mspMapObjects[obj_id]->points.makeShared();
		pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGBL> rgb(cloud_show);
		viewer->addPointCloud<pcl::PointXYZRGBL> (cloud_show, rgb, "show_cloud"+to_string(obj_id));
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "show_cloud"+to_string(obj_id));
		// add cuboid
		Eigen::Vector3f bbox_trans = mspMapObjects[obj_id]->loc;
		Eigen::Vector3f bbox_whd = mspMapObjects[obj_id]->dim;
		Eigen::Vector3f bbox_rpy = mspMapObjects[obj_id]->rpy;
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






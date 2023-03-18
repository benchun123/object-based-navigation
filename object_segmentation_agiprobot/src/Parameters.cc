#include "Parameters.h"

std::string dataset_name;
std::string truth_camera_file;
std::string rgb_list_file;
std::string bbox_2d_list_file;
std::string truth_cuboid_file;
std::string offline_cuboid_list;
std::string offline_plane_list;
std::string association_file;

// camera parameter
float fx;
float fy;
float cx;
float cy;
float mDepthMapFactor;
int imWidth;
int imHeight;

Eigen::Matrix3d Kalib;
Eigen::Matrix4d Tbc;  //calibration result, tf base_link to camera

// Eigen::Matrix4d Twc; // maybe more to frame

// global object detector param 
int MultiPlane_SizeMin; // param for MultiPlaneSegmentation
float MultiPlane_AngleThre;
float MultiPlane_DistThre;

float ground_min;
float ground_max;
float wall_min;
float wall_max;

// global visualization param 
int image_start;
int image_fps;
int image_end;
int vis_enable;
int vis_opencv_enable;
int vis_result_enable;
int save_result_enable;
std::string save_obj_file;
std::string save_pcl_file;
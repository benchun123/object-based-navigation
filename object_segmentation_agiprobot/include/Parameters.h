#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "opencv2/opencv.hpp"


extern std::string dataset_name;
extern std::string truth_camera_file;
extern std::string rgb_list_file;
extern std::string bbox_2d_list_file;
extern std::string truth_cuboid_file;
extern std::string offline_cuboid_list;
extern std::string offline_plane_list;
extern std::string association_file;

extern float fx;
extern float fy;
extern float cx;
extern float cy;
extern float mDepthMapFactor;
extern int imWidth;
extern int imHeight;

extern Eigen::Matrix3d Kalib;
extern Eigen::Matrix4d Tbc;  //calibration result, tf base_link to camera
// extern Eigen::Matrix4d Twc; // move to object detector

extern int MultiPlane_SizeMin; // param for MultiPlaneSegmentation
extern float MultiPlane_AngleThre;
extern float MultiPlane_DistThre;

extern float ground_min;
extern float ground_max;
extern float wall_min;
extern float wall_max;

// for debug
extern int image_start;
extern int image_fps;
extern int image_end;
extern int vis_enable;
extern int vis_opencv_enable;
extern int vis_result_enable;
extern int save_result_enable;
extern std::string save_obj_file;
extern std::string save_pcl_file;

#endif // PARAMETERS_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <ros/package.h> // use function ros::package::getPath("acdu_numberplate")

using namespace std;
using namespace Eigen;

class HyperMapNode {
public:
    HyperMapNode(): node_handle_("~")
    {

    node_handle_.param<std::string>("save_obj_file", save_obj_file_,
                                    save_obj_file_);
    node_handle_.param<std::string>("gt_file", gt_file_,
                                    gt_file_);

    pub_map_object_cuboid =
        node_handle_.advertise<visualization_msgs::MarkerArray>("/map_object_cuboid", 1, true);

    pub_map_truth_cuboid =
        node_handle_.advertise<visualization_msgs::MarkerArray>("/map_truth_cuboid", 1, true);


    bool show_det_obj = true;
    if(show_det_obj)
    {
        // // load object info
        std::string txt_filename = save_obj_file_;
        std::cout << "load object result at " << txt_filename << std::endl;
        Eigen::MatrixXd obj_data(2,10);
        if (!read_all_number_txt(txt_filename, obj_data))
            ROS_ERROR("!!! could not load file !!!");
        std::cout << "detect_data\n " << obj_data << std::endl;

        visualization_msgs::MarkerArray show_markers;
        for (size_t obj_id = 0; obj_id < obj_data.rows(); obj_id++)
        {
            Eigen::Vector3f bbox_trans = obj_data.row(obj_id).segment<3>(1).cast<float>();
            Eigen::Vector3f bbox_rpy = obj_data.row(obj_id).segment<3>(4).cast<float>();
            Eigen::Vector3f bbox_whd = obj_data.row(obj_id).tail(3).cast<float>();
            Eigen::AngleAxisf obj_roll(bbox_rpy(0), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf obj_pitch(bbox_rpy(1), Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf obj_yaw(bbox_rpy(2), Eigen::Vector3f::UnitZ());
            Eigen::Quaternion<float> bbox_quaternion = obj_roll * obj_pitch * obj_yaw;
            Eigen::Matrix3f bbox_rot = bbox_quaternion.matrix().transpose();

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = obj_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = bbox_trans(0);
            marker.pose.position.y = bbox_trans(1);
            // marker.pose.position.z = bbox_trans(2);
            marker.pose.position.z = 0.05;
            marker.pose.orientation.x = bbox_quaternion.x();
            marker.pose.orientation.y = bbox_quaternion.y();
            marker.pose.orientation.z = bbox_quaternion.z();
            marker.pose.orientation.w = bbox_quaternion.w();
            marker.scale.x = bbox_whd(0);
            marker.scale.y = bbox_whd(1);
            // marker.scale.z = bbox_whd(2);
            marker.scale.z = 0.02;
            marker.color.a = 0.8; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            std::cout << "obj " << obj_id << " loc" << bbox_trans.transpose() <<  std::endl;
            show_markers.markers.push_back(marker);
        }
        pub_map_object_cuboid.publish(show_markers);
    }
    ros::Duration(1).sleep();
    bool show_gt_obj = true;
    if(show_gt_obj)
    {
        // // load object info
        std::string txt_filename = gt_file_;
        std::cout << "load object result at " << txt_filename << std::endl;
        Eigen::MatrixXd obj_data(2,10);
        if (!read_all_number_txt(txt_filename, obj_data))
            ROS_ERROR("!!! could not load file !!!");
        std::cout << "truth_data\n " << obj_data << std::endl;

        visualization_msgs::MarkerArray show_markers;
        for (size_t obj_id = 0; obj_id < obj_data.rows(); obj_id++)
        {
            Eigen::Vector3f bbox_trans = obj_data.row(obj_id).segment<3>(1).cast<float>();
            Eigen::Vector3f bbox_rpy = obj_data.row(obj_id).segment<3>(4).cast<float>();
            Eigen::Vector3f bbox_whd = obj_data.row(obj_id).tail(3).cast<float>();
            Eigen::AngleAxisf obj_roll(bbox_rpy(0), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf obj_pitch(bbox_rpy(1), Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf obj_yaw(bbox_rpy(2), Eigen::Vector3f::UnitZ());
            Eigen::Quaternion<float> bbox_quaternion = obj_roll * obj_pitch * obj_yaw;
            Eigen::Matrix3f bbox_rot = bbox_quaternion.matrix().transpose();

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = obj_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = bbox_trans(0);
            marker.pose.position.y = bbox_trans(1);
            // marker.pose.position.z = bbox_trans(2);
            marker.pose.position.z = 0.05;
            marker.pose.orientation.x = bbox_quaternion.x();
            marker.pose.orientation.y = bbox_quaternion.y();
            marker.pose.orientation.z = bbox_quaternion.z();
            marker.pose.orientation.w = bbox_quaternion.w();
            marker.scale.x = bbox_whd(0);
            marker.scale.y = bbox_whd(1);
            // marker.scale.z = bbox_whd(2);
            marker.scale.z = 0.02;
            marker.color.a = 0.8; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            std::cout << "obj " << obj_id << " loc" << bbox_trans.transpose() <<  std::endl;
            show_markers.markers.push_back(marker);
        }
        pub_map_truth_cuboid.publish(show_markers);
    }


  }

private:
    ros::NodeHandle node_handle_;

private:
    std::string save_obj_file_;
    std::string gt_file_;
    ros::Publisher pub_map_object_cuboid;
    ros::Publisher pub_map_truth_cuboid;

    // make sure column size is given. no checks here. row will be adjusted automatically. if more cols given, will be zero.
    bool read_all_number_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat)
    {
        if (!std::ifstream(txt_file_name))
        {
            std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
            return false;
        }
        std::ifstream filetxt(txt_file_name.c_str());
        int row_counter = 0;
        std::string line;
        if (read_number_mat.rows() == 0)
            read_number_mat.resize(100, 10);

        while (getline(filetxt, line))
        {
            double t;
            if (!line.empty())
            {
                std::stringstream ss(line);
                int colu = 0;
                while (ss >> t)
                {
                    read_number_mat(row_counter, colu) = t;
                    colu++;
                }
                row_counter++;
                if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                    read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
            }
        }
        filetxt.close();
        read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows
        return true;
    }

};



int main( int argc, char *argv[]) {
    // google::InitGoogleLogging(argv[0]);
    // LOG(INFO) << "Starting object segmentation ... ";
    ros::init(argc, argv, "show_hypermap_node");
    ROS_INFO("show_hypermap_node");
    HyperMapNode hyper_map_node;

    while (ros::ok()) {
        ros::spin();
    }
    return 0;
}


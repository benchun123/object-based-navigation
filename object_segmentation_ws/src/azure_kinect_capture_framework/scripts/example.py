#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import cv2
import os
import rospkg
import ros_numpy  # sudo apt-get install ros-melodic-ros-numpy
import numpy

# topics:      /depth/camera_info   9 msgs    : sensor_msgs/CameraInfo 
#              /depth/image_raw     9 msgs    : sensor_msgs/Image      
#              /points2             8 msgs    : sensor_msgs/PointCloud2
#              /rgb/camera_info     9 msgs    : sensor_msgs/CameraInfo 
#              /rgb/image_raw       9 msgs    : sensor_msgs/Image      
#              /tf_static           2 msgs    : tf2_msgs/TFMessage   

class capture_node:
    def __init__(self):
        self.name = "Hello world"
        self.bridge = CvBridge()
        self.color_info_sub = rospy.Subscriber("/rgb/camera_info", CameraInfo, self.receive_color_info)
        self.color_img_sub = rospy.Subscriber("/rgb/image_raw", Image, self.receive_color_img)
        self.depth_info_sub = rospy.Subscriber("/depth_to_rgb/camera_info", CameraInfo, self.receive_depth_info)
        self.depth_img_sub = rospy.Subscriber("/depth_to_rgb/image_raw", Image, self.receive_depth_img)
        self.save_cmd_sub = rospy.Subscriber("/bool_save", Bool, self.save_all_info)
        self.depth_img_sub = rospy.Subscriber("/points2", PointCloud2, self.receive_point_cloud)
        self.save_folder = rospkg.RosPack().get_path('azure_kinect_capture_framework') + "/output"
        self.color_info = None
        self.depth_info = None
        self.color_img = None
        self.depth_img = None
        self.point_cloud = None
        self.image_cnt = 0

    def save_all_info(self, msg):
        self.image_cnt += 1
        folder_name = self.save_folder+"/"+str(self.image_cnt).zfill(4)
        if not os.path.isdir(folder_name):
            os.mkdir(folder_name)
        rospy.loginfo("save all info to %s", folder_name)
        cv2.imwrite( folder_name+'/rgb.png', self.color_img)
        cv2.imwrite( folder_name+'/depth.png', self.depth_img)
        # numpy.savetxt(folder_name+"/pcl_data.txt", self.point_cloud, fmt="%f")
        if self.image_cnt == 1:
            numpy.savetxt(folder_name+"/camera_K.txt", self.cam_K, fmt="%f")
            numpy.savetxt(folder_name+"/camera_D.txt", self.cam_D, fmt="%f")

    def receive_color_info(self, msg):
        # std_msgs/Header header
        # uint32 height
        # uint32 width
        # string distortion_model
        # float64[] D
        # float64[9] K
        # float64[9] R
        # float64[12] P
        # uint32 binning_x
        # uint32 binning_y
        # sensor_msgs/RegionOfInterest roi
        img_height = msg.height
        img_width = msg.width
        self.cam_K = numpy.asarray(msg.K).reshape((3, 3))
        self.cam_D = numpy.asarray(msg.D)
        # rospy.loginfo("color camera info %d %d", img_height, img_width)
        # rospy.loginfo("color camera K ", type(cam_K))
        # print("rgb_cam_info ", self.cam_K)
        # print("rgb_cam_info ", self.cam_D)

    def receive_color_img(self, msg):
        # std_msgs/Header header
        # uint32 height
        # uint32 width
        # string encoding
        # uint8 is_bigendian
        # uint32 step
        # uint8[] data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.color_img = cv_image
            rospy.loginfo("receive color image")
            # print("color image size",cv_image.shape)
            # cv2.imshow("Image window", cv_image)
            # cv2.imwrite(self.save_folder +'/rgb2222.png', cv_image)
        except CvBridgeError as e:
            print(e)
        # else:
            # cv2.imwrite('camera_image.jpeg', cv2_img)

    def receive_depth_info(self, msg):
        img_height = msg.height
        img_width = msg.width
        cam_K = numpy.asarray(msg.K).reshape((3, 3))
        # rospy.loginfo("depth camera info %d %d", img_height, img_width)
        # print("depth_cam_info \n", cam_K)

    def receive_depth_img(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_img = cv_image
            rospy.loginfo("receive depth image")
            # cv2.imshow("Image window", cv_image)
            # cv2.imwrite('depth2222.png', cv_image)
            # print("depth image size",cv_image.shape)
        except CvBridgeError as e:
            print(e)
        # # else:
        #     # cv2.imwrite('camera_image.jpeg', cv2_img)

    def receive_point_cloud(self, msg):
        pc = ros_numpy.numpify(msg)
        points=numpy.zeros((pc.shape[0],3))
        points[:,0]=pc['x']
        points[:,1]=pc['y']
        points[:,2]=pc['z']
        self.point_cloud = points
        # p = pcl.PointCloud(np.array(points, dtype=np.float32))
        rospy.loginfo("!!!!!!!!!!!!!!!points %d", points.size)



if __name__ == '__main__':
    rospy.init_node('capture_framework')
    try:
        test = capture_node()
    except rospy.ROSInterruptException: 
        pass
    rospy.spin()

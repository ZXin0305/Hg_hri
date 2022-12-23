#pragma once
#include<iostream>
#include<sstream>
#include<algorithm>
#include<string>
/*       opencv & aruco      */
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/aruco.hpp>
#include<opencv2/aruco/dictionary.hpp>
/*       ros                */
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
/*       Eigen             */
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
/*       toCv              */
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>

/*       发布必要marker信息 */
#include<std_msgs/Int32MultiArray.h>

using namespace std;
using namespace cv;

class Object{
public:
    Object(string camera_topic, string camera_param_path):it(nh){
        this->camera_topic = camera_topic;
        this->camera_param_path = camera_param_path;
        this->load_camera_param();

        // 订阅图像  这个图像是不是要从RGBA转到RGB呢　？？
        this->image_sub = this->it.subscribe(this->camera_topic, 1, &Object::image_callback, this);
        this->marker_id_pub = this->nh.advertise<std_msgs::Int32MultiArray>("marker_detect", 1);
    }

    void load_camera_param();
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void run();
    void locate_marker();
    void senMarkerID();


private:
    string camera_topic;   // 订阅图像
    string camera_param_path;  // 相机内参

    cv::Mat camera_matrix;
    cv::Mat camera_dist;

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    cv::Mat img;

    //marker  DICT_4X4_50
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    vector<int> marker_ids;
    vector<vector<cv::Point2f>> marker_corners;
    vector<cv::Vec3d> rvecs;
    vector<cv::Vec3d> tvecs;

    std_msgs::Int32MultiArray marker_id_list;
    vector<int32_t> tmp_marker_vec;
    ros::Publisher marker_id_pub;
};
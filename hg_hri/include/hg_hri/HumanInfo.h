#pragma once
#include<iostream>
#include<vector>
#include<string>
#include<map>
#include<ros/ros.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

/*       　接收＆发布人员信息        */
#include<hg_hri/Human.h>
#include<hg_hri/HumanList.h>
#include<hg_hri/PointCoors.h>
#include<std_msgs/Int32MultiArray.h>  // 存储并发布人员的身份id和手势信息　形式：[human_id, is_operator, left_hg, right_hg; human_id, is_operator, left, right; ....]
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
/* 　　订阅marker, 得到相机到世界的坐标转换关系  　*/
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
using namespace std;


const int N = 15;  // 关节点
const int MAX_MEO = 8;
const int pairNum = 14;
int bodyEages[pairNum][2] = {
    {0,1}, {0,2}, {0,9}, {0,3}, {3,4}, {4,5},
    {9,10}, {10,11}, {2,6}, {6,7},
    {7,8}, {2,12}, {12,13}, {13,14}
};

// 前面四个不知道干什么的　。。。忘了
double min_x = 0.01;
double min_y = 0.01;
double max_x = 0.2;
double max_y = 0.2;
double z_thres = 0.12;

Eigen::Matrix<double, 3, 1> world_xyz;

struct KeyPoint{
    double x;
    double y;
    double z;
};

struct HumanWithIdHg{
    int human_id;
    int left_hg;
    int right_hg;
    int is_operator;
    vector<KeyPoint> point;
};


class PointPub{
public:
    PointPub(){
        this->points.header.frame_id = this->world_name;
        this->points.header.stamp = ros::Time::now();
        this->points.ns = this->name_space;
        this->points.action = visualization_msgs::Marker::ADD;
        this->points.pose.orientation.w = 1.0;
        this->points.pose.orientation.x = 0.0;
        this->points.pose.orientation.y = 0.0;
        this->points.pose.orientation.z = 0.0;
        this->points.id = 0;
        this->points.type = visualization_msgs::Marker::POINTS;

        this->points.scale.x = 0.03;
        this->points.scale.y = 0.03;
        this->points.scale.z = 0.03;
        this->points.color.r = 1.0;
        this->points.color.a = 1.0;
        this->points.lifetime = ros::Duration(1);
    }

    void store_points(vector<vector<Eigen::Matrix<double,3,1>>>& pointList);
public:
    visualization_msgs::Marker points;
private:
    string world_name = "base";
    string name_space = "marker_node";
};

// 这个是将人体的姿态信息连成线并发布到Rviz空间中，是一种视觉反馈
class LinePub
{
public:
    LinePub(){
        this->line.header.frame_id = this->world_name;
        this->line.header.stamp = ros::Time::now();
        this->line.ns = this->name_space;
        this->line.action = visualization_msgs::Marker::ADD;
        this->line.pose.orientation.w = 1.0;
        this->line.pose.orientation.x = 0.0;
        this->line.pose.orientation.y = 0.0;
        this->line.pose.orientation.z = 0.0;

        this->line.id = 1;
        this->line.type = visualization_msgs::Marker::LINE_LIST;
        this->line.scale.x = 0.01;
        this->line.scale.y = 0.01;
        this->line.scale.z = 0.01;
        //color
        this->line.color.g = 2.0;
        this->line.color.a = 2.0;
        this->line.color.r = 0.0;  //0.8
        this->line.color.b = 0.0;  //1.0
        this->line.lifetime = ros::Duration(1);
    }

    void store_points(vector<vector<Eigen::Matrix<double,3,1>>>& pointList);
public:
    visualization_msgs::Marker line;
private:
    string world_name = "marker_0";  //marker_0
    string name_space = "marker_node";  
};


class Human{
public:
    Human(string& camera_name):camera_name(camera_name){
        this->human_sub = this->nh.subscribe(this->pose_topic, 5, &Human::human_pose_callback, this);
        this->human_idHg_pub = this->nh.advertise<std_msgs::Int32MultiArray>(this->human_id_hg, 5);
        point_pub = new PointPub();
        line_pub = new LinePub();
        tf_listener();
    }

    void get_human_pose(vector<HumanWithIdHg>& human_pose_with_id_hg);
    void change_pose_format(vector<vector<Eigen::Matrix<double, 3, 1>>>& point_list);
    void pub_human_info(vector<vector<Eigen::Matrix<double, 3, 1>>>& point_list);
    void human_pose_callback(const hg_hri::HumanList& human_list);
    void run();
    void tf_listener();
    void cal_cross_point();
    bool intersectionLinePlane(Eigen::Vector3f &p1, Eigen::Vector3f p2, const Eigen::Vector4f &plane, Eigen::Vector3f &crossP);

    ~Human(){
        delete point_pub;
        delete line_pub;
    }

public:
    vector<int32_t> tmp_id_hg;
    std_msgs::Int32MultiArray human_id_with_hg;
    tf::Transform current_joint;
    string joint_name;
    double X, Y, Z;
    Eigen::Matrix3d cam2world;
    Eigen::Vector3d trans;

    //两个空间点 实验２
    Eigen::Vector3f point_1;
    Eigen::Vector3f point_2;
    Eigen::Vector4f plane;
    Eigen::Vector3f cross_point; //待求解的点
    tf::StampedTransform task_point;

    bool cal_cross = false;
    int corss_hg = 12;

private:

    vector<HumanWithIdHg> human_points_vector;
    ros::NodeHandle nh;
    ros::Subscriber human_sub;
    ros::Publisher human_idHg_pub;
    tf::TransformBroadcaster human_info_br;
    vector<tf::Transform> human_info_trans;
    string pose_topic = "/pub_human";
    string human_id_hg = "/human_id_hg";
    string camera_name;
    PointPub *point_pub;
    LinePub *line_pub;

};




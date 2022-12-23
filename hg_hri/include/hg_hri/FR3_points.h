#pragma once
#include<iostream>
#include<vector>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<cmath>
#include<queue>
#include<std_msgs/String.h>
#include<string>
#include<std_msgs/Int32MultiArray.h>
#include<hg_hri/robot2global.h>
#include<hg_hri/point2robot.h>
#include<ctime>
#include<unordered_map>
#include<ctime>

using namespace std;

class Fr3Points{
public:
    Fr3Points(){
        this->init();   // 初始化一系列的点....
        this->human_sub = this->nh.subscribe(this->topic_name_human, 5, &Fr3Points::humanCallback, this);
    }
    void init();
    void pub_points();  // 这个是发布几个固定的点
    void pub_obj_points();
    void run();
    void listen_point();
    void humanCallback(const std_msgs::Int32MultiArray& human_id_hg);  //人

    // 旋转末端关节
    void rotate_tool();

    void decode_human_info();

private:
    string obj_correct_name = "obj_correct_";
    string obj_target_name = "obj_target_";
    string node_name = "base_link";    // marker_0
    string base_name = "base_link";
    string tip_name = "wrist3_Link";

    // 接收人员信息，　主要是为了旋转角度
    int lhg = -1, rhg = -1;
    int info_len = 4;
    string topic_name_human = "/human_id_hg";
    ros::Subscriber human_sub;
    vector<int32_t> human_info;
    vector<vector<int>> human_hg_vec;
    float rotate_angle = (float)(M_PI / 2);

    int object_num = 3;
    ros::NodeHandle nh;
    vector<tf::Transform> cap_init, obj_target;
    tf::Transform tmp, tmp_correct, tmp_target, tmp_record, task_point, rotate_point;
    tf::StampedTransform current_tool_pose;
    tf::TransformBroadcaster br;
    tf::TransformListener tf_listener;
};
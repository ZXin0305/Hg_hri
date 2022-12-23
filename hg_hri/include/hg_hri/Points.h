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

using namespace std;

struct lastInfo{
    int last_mode;
    //mode1
    int last_mode1_idx;
    int last_mode1_step;

    //mode2
    int last_mode2_direct;

    //mode3  record_point_idx
    int last_mode3_record;
};



class Points{
public:
    Points(){
        this->init();
        this->robot_sub = this->nh.subscribe(this->topic_name_robot, 5, &Points::robotCallback, this);
        this->human_sub = this->nh.subscribe(this->topic_name_human, 5, &Points::humanCallback, this);
        this->status_client = this->nh.serviceClient<hg_hri::point2robot>(srv_name);
    }

    void init();
    void pub_points();
    void pub_obj_points(const int obj_idx);
    void robotCallback(const hg_hri::robot2global& robot_msg);         // 机械臂
    void humanCallback(const std_msgs::Int32MultiArray& human_id_hg);  // 人
    void run();
    void mode1(const int idx);

    void mode2();
    void cal_next();
    void listen_point();
    void decode_human_info();

    void mode3();
    void record_point();

    void reset_info();

    void last_memory(int mode);

    void rotate_tool();

    // 返回上次的信息
    void pub_status();

public:
    int lhg = -1, rhg = -1;
    int info_len = 4;
    int mode1_step = 0, mode1_process = 9; // mode1，　记录mode1进行到那一步了
    int mode1_idx = 0, mode1_obj_idx = 0;  //　前面：矫正点和目标点的idx, 后面：当前正在进行的是哪一个obj(后面这个没有用了)  mode1
    lastInfo last_info;
    int direct = -1;           // mode2
    int record_point_idx = 0;  // mode3

    float rotate_angle = (float)M_PI / 9;

    unordered_map<int, int> un_map;

private:
    int cap_size, obj_size;
    string node_name = "marker_0";
    string topic_name_robot = "/robot2global";
    string topic_name_human = "/human_id_hg";
    ros::NodeHandle nh;

    ros::Subscriber robot_sub;
    int is_arrived = 0;
    int mode1_finish = 0;
    ros::Subscriber human_sub;
    vector<int32_t> human_info;   // 用来接收
    vector<vector<int>> human_hg_vec;  //如果有多个操作人员，那么就需要判断

    // service
    ros::ServiceClient status_client;  //返回上次的信息
    hg_hri::point2robot srv;
    string srv_name = "/last_status";
    float delay_time = 3.0;
    bool is_accept = false;    // Robot那边接收到了
    
    vector<tf::Transform> cap_init, obj_target;
    vector<tf::Transform> record_list;
    vector<vector<tf::Transform>> obj_target_new;   // 将每一个物体的矫正点和抓取点给放在同一个内嵌vector中

    tf::Transform tmp, tmp_correct, tmp_target, tmp_record, task_point;
    tf::TransformBroadcaster br;
    tf::TransformListener tf_listener;


    // 不同的mode
    /*
    lhg: 0-9都是抓取物块的, 10是停止
    rhg: 10　是停止
    */
    bool mode1_flag = false;   // 抓取物块
    bool mode2_flag = false;   // 上下左右前后
    bool mode3_flag = false, mode3_flag2 = false;   // 记录点  1：是记录，　２是发布


    // 模式下的监听的点
    tf::StampedTransform current_tool_pose;
    double step = 2.0;   // 控制方向时每次的增加的长度
    tf::Transform next_point;
};
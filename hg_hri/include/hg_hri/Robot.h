#pragma once
#include<boost/scoped_ptr.hpp>
// ------------ kdl运动学库 ----------------
#include<kdl/chain.hpp>
#include<kdl/jacobian.hpp>
#include<kdl/chainjnttojacsolver.hpp>
#include<kdl/jntarray.hpp>
#include<kdl_parser/kdl_parser.hpp>
// ------------ 逆运动学库 -----------------
#include<trac_ik/trac_ik.hpp>
// ------------ ROS ----------------------
#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<tf_conversions/tf_kdl.h>
// ------------ UR5 ----------------------
#include<std_msgs/String.h>
#include<sensor_msgs/JointState.h>
#include<pthread.h>
#include<iostream>
#include<cmath>
#include<vector>
#include<string>

//　接收人的消息
#include<std_msgs/Int32MultiArray.h>
#include<algorithm>
#include<cmath>
#include<ctime>

#include<hg_hri/robot2global.h>  // 发布是否到达以及mode１是否完成
#include<hg_hri/point2robot.h>   // 这个是返回接收到上次的的状态

using namespace std;

const int JOINT_NUM = 6;  // 机器人关节
const int PUB_RATE = 20;  // 机器人运动指令发布频率
const float TIME_STEP = 1.0 / PUB_RATE; 

// 参数初始化
bool JointState_available;  // 能够获取机器人的关节状态
KDL::JntArray q(JOINT_NUM);  //　当前的关节状态的向量
KDL::JntArray q_desired(JOINT_NUM); // 期望的关节状态的向量
KDL::Frame correct_pose, desired_pose;

vector<string> joint_names(JOINT_NUM);  // 机器人关节名称
vector<double> joint_angles(JOINT_NUM); // 机器人的关节角度
vector<double> joint_speed(JOINT_NUM);  // 机器人的关节速度

string urdf_param = "/robot_description";  // ur5机器人模型
string base_name = "base";
string tip_name = "tool0";

struct LastInfo{
    int last_mode;
    //mode1
    int last_mode1_idx;
    int last_mode1_step;

    //mode2
    int last_mode2_direct;

    //mode3  record_point_idx
    int last_mode3_record;
};


class Robot{
public:
    Robot(){
        string robot_topic1 = "/ur_driver/URScript";
        string robot_topic2 = "/joint_states";
        this->init();
        this->ur_command_publisher = this->nh.advertise<std_msgs::String>(robot_topic1, 3);
        this->ur_joint_publisher = this->nh.advertise<sensor_msgs::JointState>(robot_topic2, 3);
        this->joint_state_sub = this->nh.subscribe(robot_topic2, 3, &Robot::jointStateCallback, this);

        this->human_id_hg_sub = nh.subscribe(this->human_id_hg, 3, &Robot::humanIdHgCallback, this);

        this->robot_info_pub = nh.advertise<hg_hri::robot2global>(this->robot_info_name, 3);   // 发布是否到达任务点

        this->status_server = nh.advertiseService(this->srv_name, &Robot::serverCallback, this);
    
    }
    void init();

    // 机器人控制
    void jointStateCallback(const sensor_msgs::JointState& joint_state);
    void sendCmd(TRAC_IK::TRAC_IK &ik_solver, tf::StampedTransform& tf_point);  //发布控制指令
    double cal_distance(tf::StampedTransform& tf_point, string& robot_joint_name);  //optional -->　在过程中判断人体和机器人的距离，调整速度
    tf::Vector3 listener_robot_point(string& robot_joint_name);  //监听机器人自身的点的tf信息, 便于计算距离
    void adjust_speed(vector<double>& speed_vector);   //根据手腕和末端的距离进行速度的调整
    void sort_distance();// 找到最近的点
    void listener_object();// 物块的点
    void listener_point();// 这个要监听４个点
    void listener_mode_point(string& point_name); // mode2中计算出来的下一个点: next_point;;;　mode3中的记录的点: record_point

    bool serverCallback(hg_hri::point2robot::Request &req, hg_hri::point2robot::Response &res);

    void reset();
    
    // 人员信息
    void humanIdHgCallback(const std_msgs::Int32MultiArray& id_hg_array);
    void decode_human_info();

    void reach_point(string current_case);
    bool judge(tf::StampedTransform& tf_point);

public:

    // 接收人员信息
    vector<int32_t> tmp_id_hg;  // 每个人有四个信息,　用来接收
    vector<vector<int>> human_hg_vec;  //如果有多个操作人员，那么就需要判断
    int info_len = 4;
    int human_num;
    int tf_joint_num = 5;  //设定的接收人体的关节点的个数
    int lhg = -1, rhg = -1;   // 左右两边的手势
    vector<tf::StampedTransform> all_human_joint_tf;

    //在comman中更改
    bool mode1_flag = false;
    int mode1_step = 0;
    int mode1_process = 9;
    int mode1_finish = 0;

    bool mode2_flag = false;
    bool mode3_flag = false;

    LastInfo last_info;
    bool last_info_update = false;

    // 调整速度
    float speed_factor = 1;
    double closest_dis;
    vector<double> distance_vec;
    double kD;  //速度的尺寸因子
    tf::StampedTransform base_human_dis, base_tool_dis;

private:
    string joint_name_tool0 = "tool0";
    string joint_name_controller = "tool0_controller";
    string joint_name_base = "base";

    pthread_t rectify_thread;
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf;

    //用来订阅human发布的hg和id，但是这是个列表,
    //需要利用这个得到人员的id从而去监听对应的关节点的信息

    ros::Subscriber human_id_hg_sub;
    string human_id_hg = "/human_id_hg";

    ros::Publisher robot_info_pub;
    string robot_info_name = "/robot2global";
    hg_hri::robot2global robot_msg;   // 是否达到任务点

    // service
    ros::ServiceServer status_server;
    string srv_name = "/last_status";



    // /ur_driver/URScript：
    // 话题名称，类型是std_msgs::String，
    // 允许我们向该话题发布URScript脚本命令
    ros::Publisher ur_command_publisher;

    // /joint_states：
    // 话题名称，类型是：sensor_msgs::JointState，
    // 实现机械臂的运动，就是向话题joint_states添加有关关节的消息
    ros::Publisher ur_joint_publisher;
    ros::Subscriber joint_state_sub; //实时订阅机器人关节角度的信息

    std_msgs::String command_msg;
    double Kp = 2.0;
    double Td = 0.2;

    // 设置阈值调整速度
    double min_thres = 0.2;
    double max_thres = 0.4;
    double max_speed = 1.5;

    double arrive_thres = 0.001; // 判断机器人到达位置

    string urdf_param = "/robot_description";  //ur5的机器人模型参数
    string base_name = "base";
    string tip_name = "tool0";

    bool toPlace, update;

    //相关的点的信息
    tf::StampedTransform cap_point, init_point, mid_point, assist_point;
    string cap_name = "cap_point";
    string init_name = "init_point";
    string middle_name = "mid_point"; 
    vector<tf::StampedTransform> cap_init;//机器人的初始点和夹取物件的地方
    bool tf_available = false;


    tf::StampedTransform obj_correct, obj_target;    // mode1
    vector<tf::StampedTransform> obj_list;

    string next_point_name = "next_point";
    // tf::StampedTransform next_point;    // mode2

    string record_point_name = "record_point";  // mode3
    // tf::StampedTransform record_point;

    tf::StampedTransform other_mode_point;
};

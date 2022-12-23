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
#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<tf_conversions/tf_kdl.h>
// -------------- fr3 -------------------
#include<std_msgs/String.h>
#include<sensor_msgs/JointState.h>
#include<control_msgs/FollowJointTrajectoryAction.h>
#include<actionlib/client/simple_action_client.h>
// -------------- 爪子 ------------------
// #include<hg_hri/Gripper.h>
#include <byp80/ByStatus.h>
#include <byp80/ShutdownGripper.h>
#include <byp80/RestartGripper.h>
#include <byp80/MoveTo.h>
#include <byp80/GetStatus.h>
#include <byp80/GetCalibrated.h>
#include <byp80/CalibrateGripper.h>
// ------------- cpp header -------------
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

//　用来记录点的数据
#include<ostream>
#include<fstream>

using namespace std;
// global
const int JOINT_NUM = 6;       // 机器人关节

bool JointState_available = false;     // 能够获取机器人的关节状态
KDL::JntArray q(JOINT_NUM);    //　当前的关节状态的向量
KDL::JntArray q_desired(JOINT_NUM);  // 期望的关节状态的向量
KDL::Frame desired_pose;

vector<string> joint_names{"j1", "j2", "j3", "j4", "j5", "j6"};   // 机器人关节名称
vector<double> joint_angles(JOINT_NUM);  //　机器人关节角度
vector<double> joint_speed(JOINT_NUM);   // 机器人关节速度
vector<double> speed_vector;
vector<double> desired_goal;
vector<double> speed_goal;

// // fr3需要和底层驱动建立连接使用的
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
const string trajectory_action_name = "/frrobot/position_trajectory_controller/follow_joint_trajectory";
Client *client_ptr;

// TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);

struct LastInfo{
    int last_mode;
    //mode1
    int last_mode1_idx;
    int last_mode1_step;

    //mode2
    int last_mode2_direct;

    //mode3  record_point_idx
    int last_mode3_record;
    int last_mode3_idx;
};

class Fr3Robot{
public:
    Fr3Robot(){
        this->init();
        move_to_client = nh.serviceClient<byp80::MoveTo>("move_to");
        move_to_client.call(srv_move);
        string robot_topic_name = "/frrobot/joint_states";
        this->joint_state_sub = this->nh.subscribe(robot_topic_name, 3, &Fr3Robot::jointStateCallback, this);
        this->human_id_hg_sub = this->nh.subscribe(human_id_hg, 3, &Fr3Robot::humanIdHgCallback, this);
    }

    void init();
    // 机器人控制
    void jointStateCallback(const sensor_msgs::JointState& joint_state);
    void sendCmd(TRAC_IK::TRAC_IK &ik_solver, tf::StampedTransform& tf_point);   // 发布控制指令, 不需要机器人的点的信息
    double cal_distance(tf::StampedTransform& tf_point1, tf::StampedTransform& tf_point2);
    void listen_robot_point(string robot_joint_name);   // 让这个更新wrist3_Link和base之间的信息
    void sort_distance();   // 找到最近的点
    void listener_object();   // 物块的点
    void listener_point();   //几个固定点 (4个)
    bool stop_robot();       //　根据距离判断是否停止机器人
    void judge_status(int mode_idx, tf::StampedTransform tf_point);     // 公共函数，判定机器人的状态 执行完毕之后会退出来
    void adjust_speed(string current_case);
    void listener_mode_point();

    // 人员信息
    void humanIdHgCallback(const std_msgs::Int32MultiArray& id_hg_array);
    void decode_human_info();

    void reset();
    
    bool have_arrived();    //判断是否到达目标位置
    void reach_point(string current_case);  // mode1的函数
    void move_direction(string current_case);   // mode2的函数
    void record_waypoints();   // mode3的函数
    void display_waypoints();  // 复现轨迹

    void build_connection();

    void clamp();  //夹紧爪子
    void loose();  //松开
    void stop_motion();

    // void main_step();

    ~Fr3Robot(){
    }

public:
    // vector<float> positions{30, }
    // 爪子
    ros::ServiceClient status_client;
    ros::ServiceClient calibrated_client;
    ros::ServiceClient move_to_client;
	byp80::GetStatus srv_getstatus;
	byp80::CalibrateGripper srv_calibarate;
	byp80::MoveTo srv_move;
    double delay_time_start = 0.2;
    double delay_time_end = 0.8;


    // 接收人员信息
    vector<int32_t> tmp_id_hg;   // 每个人有四个信息,　用来接收
    vector<vector<int>> human_hg_vec;   //如果有多个操作人员，那么就需要判断
    int info_len = 4;
    int human_num = 0;
    int tf_joint_num = 5;            //设定的接收人体的关节点的个数
    int lhg = -1, rhg = -1;          // 左右两边的手势
    vector<tf::StampedTransform> all_human_joint_tf;

    // mode1
    bool mode1_flag = false;
    int mode1_step = 0;
    int mode1_process = 7;   // 步骤
    int mode1_finish = 0;
    int mode1_idx = 0;   // 这个是当前的第几个物块
    int obj_num = 3;      // 物块的个数，人为设定

    // mode2
    // mode2中每个方向的最大位置  单位: m
    bool mode2_flag = false;
    double x_max = 0.2;
    double x_min = -0.2;
    double y_max = 0.55;    // 最大限度55cm
    double y_min = 0.25;    // 这个0.18
    double z_max = 0.4;
    double z_min = 0.2;     // 这个差不多刚刚好
    int mode2_direct = -1;

    // mode3
    bool mode3_flag = false;
    int mode3_idx = 0;
    vector<tf::StampedTransform> waypoints;
    //记录是否保存了上一次的运动
    bool last_info_update = false;
    int last_mode_ = -1;  // 重启时标志着自己需要重启哪一步
    //
    /*
    ***********************************************************************
                              
    ***********************************************************************
    */

    LastInfo last_info;

    // 本来是调整速度的，现在直接停止就好了．．．
    double closest_dis, stop_thres = 0.01;
    vector<double> distance_vec;
    tf::StampedTransform base_human_dis, base_tool_dis;

    string common_case = "";


private:

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

    // fr3
    ros::Subscriber joint_state_sub;   // 实时订阅机器人关节角度信息

    //　机器人参数
    string urdf_param = "/robot_description";  // 机器人模型
    string base_name = "base_link";            // 基座   没有启动机器人的时候，这里就暂时用base_link代替
    string tip_name = "wrist3_Link";           // 法兰
    double Kp = 2.0;
    double Td = 0.2;

    // 根据空间距离调整速度
    double trajectory_dis = 10;
    double max_speed = 1.5;
    double fixed_dis = 100.0;  // 单位：cm
    int speed_idx = 0;
    vector<double> speed_time{50.0, 25.0, 10.0};   // slow  middle  quick   // 需要修改速度

    bool toPlace, update;

    // 相关点的信息
    tf::StampedTransform cap_point, init_point, mid_point, assist_point, rotate_point;
    string cap_name = "cap_point";
    string init_name = "init_point";
    string middle_name = "mid_point";
    string rotate_name = "rotate_point";
    vector<tf::StampedTransform> cap_init;  // 机器人的初始点和夹取点的列表
    bool tf_available = false;

    tf::StampedTransform obj_correct, obj_target;   // mode1
    vector<tf::StampedTransform> obj_list;          // 物件的列表

    // 直接就在robot.h这里将这两个模式实现，不用在point.h那里计算和发布了
    tf::StampedTransform next_point;                // mode2

    tf::StampedTransform record_point;              // mode3

    tf::StampedTransform other_mode_point;          // ????  感觉没有什么用．．． 应该是没有用

};

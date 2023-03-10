#pragma once
#include <iostream>
#include <string>
#include <ros/ros.h>
// 爪子的一些头文件，需要先进行编译
#include <byp80/ByStatus.h>
#include <byp80/ShutdownGripper.h>
#include <byp80/RestartGripper.h>
#include <byp80/MoveTo.h>
#include <byp80/GetStatus.h>
#include <byp80/GetCalibrated.h>
#include <byp80/CalibrateGripper.h>

using namespace std;

class Gripper
{
public:
    Gripper(){
        // this->init();
        // this->status_client = nh.serviceClient<byp80::GetStatus>("get_status");
        // this->calibrated_client = nh.serviceClient<byp80::CalibrateGripper>("calibrate_gripper");
        this->move_to_client = nh.serviceClient<byp80::MoveTo>("move_to");

        // status_client.call(srv_getstatus);
        // calibrated_client.call(srv_calibarate);
        move_to_client.call(srv_move);
    }
    // void init();   //设置放置的目标点 、 放置之后的回到的点
    void clamp();  //夹紧爪子
    void loose();  //松开

private:
    ros::NodeHandle nh;
    ros::ServiceClient status_client;
    ros::ServiceClient calibrated_client;
    ros::ServiceClient move_to_client;
	byp80::GetStatus srv_getstatus;
	byp80::CalibrateGripper srv_calibarate;
	byp80::MoveTo srv_move;
    double delay_time_start = 0.2;
    double delay_time_end = 0.8;
};
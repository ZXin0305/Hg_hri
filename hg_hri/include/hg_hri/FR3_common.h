#pragma once
#include<iostream>
#include<hg_hri/Gripper.h>
#include<hg_hri/FR3_robot.h>
#include<ros/ros.h>

// for test
// #include<control_msgs/FollowJointTrajectoryAction.h>
// #include<actionlib/client/simple_action_client.h>

using namespace std;

class Fr3Modules{
public:
    Fr3Modules(){
        robot = new Fr3Robot();
        gripper = new Gripper();
    }

    // void run();
    // void catch_obj();

    // void track_next_right(int num);
    // void track_next_left(int num);

    // void track_record();

    void test();

    // ~Fr3Modules(){
    //     delete robot;
    //     delete gripper;
    // }

public:
//     Fr3Robot *robot;
//     Gripper *gripper;

    Fr3Robot *robot;
    Gripper *gripper;
};

#include<ros/ros.h>
#include<iostream>
//　用来记录点的数据
#include<ostream>
#include<fstream>
#include<sensor_msgs/JointState.h>
#include<control_msgs/JointTrajectoryControllerState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<string>
#include<vector>

using namespace std;

vector<double> joint_angles, joint_speed;
vector<string> joint_names{"j1", "j2", "j3", "j4", "j5", "j6"};
bool JointState_available = false;

ofstream position_file("/home/zx/hg_ws/src/hg_hri/data/pos.csv", ios_base::out | ios_base::app);
ofstream speed_file("/home/zx/hg_ws/src/hg_hri/data/speed.csv", ios_base::out | ios_base::app);

void jointStateCallback(const sensor_msgs::JointState& joint_state){
    joint_angles.clear();
    joint_speed.clear();

    vector<string> joint_names_recv = joint_state.name;

    for(auto it = joint_names.begin(); it != joint_names.end(); ++it){
        for (auto it_recv = joint_names_recv.begin(); it_recv != joint_names.end(); ++it_recv){
            if(*it_recv == *it){
                int idx = it_recv - joint_names_recv.begin();
                // int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);    //关节角度 ，其微分就是角速度 ， 即下面的joint_speed
                joint_speed.push_back(joint_state.velocity[idx]);     //关节角速度
                
                break;
            }
        }
    }


    // cout << joint_speed[0] << "," <<
    //               joint_speed[1] << "," <<
    //               joint_speed[2] << "," <<
    //               joint_speed[3] << "," <<
    //               joint_speed[4] << "," <<
    //               joint_speed[5] << "," << endl;
    
    position_file << joint_angles[0] << "," << 
                     joint_angles[1] << "," <<
                     joint_angles[2] << "," <<
                     joint_angles[3] << "," <<
                     joint_angles[4] << "," <<
                     joint_angles[5] << "," << endl;

    speed_file << joint_speed[0] << "," <<
                  joint_speed[1] << "," <<
                  joint_speed[2] << "," <<
                  joint_speed[3] << "," <<
                  joint_speed[4] << "," <<
                  joint_speed[5] << "," << endl;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "record_trac");
    ros::NodeHandle nh;

    ros::Rate rate(60);
    // /frrobot/position_trajectory_controller/state
    // /move_group/display_planned_path
    // 
    string robot_topic_name = "/frrobot/joint_states";  // /frrobot/position_trajectory_controller/state
    ros::Subscriber joint_state_sub = nh.subscribe(robot_topic_name, 3, jointStateCallback);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
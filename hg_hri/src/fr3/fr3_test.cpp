#include<iostream>
#include<vector>

#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf_conversions/tf_kdl.h>
#include<sensor_msgs/JointState.h>

#include<kdl/chain.hpp>
#include<kdl/jacobian.hpp>
#include<kdl/chainjnttojacsolver.hpp>
#include<kdl/jntarray.hpp>
#include<kdl_parser/kdl_parser.hpp>
#include<trac_ik/trac_ik.hpp>

#include<control_msgs/FollowJointTrajectoryAction.h>
#include<actionlib/client/simple_action_client.h>

using namespace std;

// global
// ***************************************
const int JOINT_NUM = 6;
const int PUB_RATE = 20;
const float TIME_STEP = 1.0 / PUB_RATE;

double trajectory_dis = 0.5;
const double fixed_dis = 100.0;  // 单位：cm
double speed_time = 50;          // 单位：s   slow: 50 middle: 25 quick:10
double dis_every_second = fixed_dis / speed_time; 
float speed_factor = 0.5;

KDL::JntArray q(JOINT_NUM);
KDL::JntArray q_desired(JOINT_NUM);
KDL::Frame correct_pose, desired_pose;

vector<string> joint_names{"j1", "j2", "j3", "j4", "j5", "j6"};
vector<double> joint_angles(JOINT_NUM);
vector<double> joint_speed(JOINT_NUM);
string urdf_param = "/robot_description";
string base_name = "world";
string tcp_name = "wrist3_Link";
uint32_t stamp_seq = 0;
// vector<string> JOINTNAMES{"J1", "J2", "J3", "J4", "J5", "J6"};

bool JointState_available = false;
sensor_msgs::JointState command_msg;

ros::Publisher joint_state_pub;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
static Client *client_ptr;

std::vector<double> speed_vector;
std::vector<double> desired_goal;
std::vector<double> speed_goal;

// ***************************************
void fr3DoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryActionResultConstPtr& result){
    ROS_INFO("have arrived ..");
}

void fr3ActiveCb(){
    ROS_INFO("action is active ..");
}

void fr3FeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
    
    if (stamp_seq == 30){
        client_ptr->cancelGoal();
    }
    // cout << "feedback" << endl;
    // if (stamp_seq >= 10)
    //     client_ptr->cancelGoal();
}

double cal_distance(tf::StampedTransform& tf_point1, tf::StampedTransform& tf_point2){
    tf::Vector3 base_point_vector;  //机器人base到需要测量的点的距离向量
    tf::Vector3 base_tool_vector;
    
    base_point_vector = tf_point1.getOrigin();
    base_tool_vector = tf_point2.getOrigin();

    tf::Vector3 robot_point_vec = base_point_vector - base_tool_vector;
    double distance = robot_point_vec.length();
    return distance;
}

void jointStateCallback(const sensor_msgs::JointState& joint_state){
    joint_angles.clear();
    joint_speed.clear();
    vector<string> joint_names_rvec = joint_state.name;
    // for (auto xx : joint_names_rvec)
    //     cout << xx << "  ";
    // cout << endl;
    // for (auto xx : joint_names)
    //     cout << xx << "  ";
    // cout << endl;

    // cout << joint_names.size() << "  " << joint_names_rvec.size() << endl;
    for (auto it = joint_names.begin(); it != joint_names.end(); ++it){
        // cout << *it << endl;
        for (auto it_revc = joint_names_rvec.begin(); it_revc != joint_names_rvec.end(); ++it_revc){
            // cout << *it_revc << endl;
            if (*it_revc == *it){
                int idx = it_revc - joint_names_rvec.begin();
                int i = it - joint_names_rvec.begin();

                joint_angles.push_back(joint_state.position[idx]);
                joint_speed.push_back(joint_state.velocity[idx]);
                // cout << joint_state.position[idx] << "   " << joint_state.velocity[idx] << endl;
                break;
            }
        }
    }

    // cout << joint_angles[0] << endl;

    // ROS_INFO("finished callback ..");
}


void sendcmd(TRAC_IK::TRAC_IK &ik_solver, tf::StampedTransform& tf_point){
    transformTFToKDL(tf_point, desired_pose);
    if (JointState_available){
        for(int i = 0; i < JOINT_NUM; ++i){
            q(i) = joint_angles[i];
            
            // printf("q_current[%d]: %f \n", i, joint_angles[i]);
        }
        if(ik_solver.CartToJnt(q, desired_pose, q_desired)){
            
            speed_vector.clear();
            desired_goal.clear();  // 用来存储转换后的值
            speed_goal.clear();
            for(int j = 0; j < JOINT_NUM; ++j){
                double delta = q_desired(j) - q(j);    //误差（q_desired:系统输入，q：上一时刻的“输出”）
                double speed = 2.0 * delta + 0.2 * joint_speed[j];  // PD控制--比例环节:当前偏差，微分环节：最近偏差

                // 检查速度
                if(speed > 1.5){
                    speed = 1.5;
                }
                if(speed < -1.5){
                    speed = -1.5;
                }

                // speed_vector.push_back(speed);
                speed_goal.push_back(speed * speed_factor);
                desired_goal.push_back(q_desired(j));
                // printf("desired goal[%d]:%f\n", j, q_desired(j));
                // printf("speed[%d]:%f\n",j,speed * speed_factor);   //速度控制
                
            }


            // float command[JOINT_NUM];
            // for (int i = 0; i < JOINT_NUM; ++i){
            //     command[i] = speed_vector[i];
            // }

            // float desired_goal[JOINT_NUM];
            // for (int i = 0; i < JOINT_NUM; ++i){
            //     desired_goal[i] = q_desired(i);
            // }
            
            // ROS_INFO("send command ..");
            // command_msg.header.seq = stamp_seq;
            // command_msg.header.stamp = ros::Time::now();
            // command_msg.name = joint_names;
            // command_msg.position = speed_vector;
            // command_msg.velocity = speed_vector;
            // // command_msg.effort = {};
            // joint_state_pub.publish(command_msg);

            control_msgs::FollowJointTrajectoryGoal goal;
            
            // goal.path_tolerance = 0.001;
            // goal.path_tolerance = 0.01;   // wrong
            // useful 20221207 ---------------------------------------------
            goal.trajectory.header.stamp = ros::Time::now();
            goal.trajectory.joint_names = joint_names;
            goal.trajectory.points.resize(1);

            goal.trajectory.points[0].positions.resize(6);
            goal.trajectory.points[0].positions[0] = desired_goal[0];
            goal.trajectory.points[0].positions[1] = desired_goal[1];
            goal.trajectory.points[0].positions[2] = desired_goal[2];
            goal.trajectory.points[0].positions[3] = desired_goal[3];
            goal.trajectory.points[0].positions[4] = desired_goal[4];
            goal.trajectory.points[0].positions[5] = desired_goal[5];
            cout << goal.trajectory.points[0].positions[0] << " " <<
                    goal.trajectory.points[0].positions[1] << " " <<
                    goal.trajectory.points[0].positions[2] << " " <<
                    goal.trajectory.points[0].positions[3] << " " <<
                    goal.trajectory.points[0].positions[4] << " " <<
                    goal.trajectory.points[0].positions[5] << " " << endl;


            goal.trajectory.points[0].velocities.resize(6);
            for (int j = 0; j < 6; ++j)
            {
                goal.trajectory.points[0].velocities[j] = 0.0;
            }

            double execute_time = trajectory_dis / dis_every_second * 100.0;
            cout << "true time: " << execute_time << endl;
            goal.trajectory.points[0].time_from_start = ros::Duration(execute_time);  // 更改这个元素，可以更改机械臂轨迹运动时间
            // useful 20221207 ---------------------------------------------
            
            // goal.trajectory.points[0].positions = joint_angles;
            // goal.trajectory.points[0].velocities = joint_speed;
            // goal.trajectory.points[0].time_from_start = ros::Duration(0.0);

            // goal.trajectory.points[1].positions = desired_goal;
            // goal.trajectory.points[1].velocities = speed_goal;
            // goal.trajectory.points[1].time_from_start = ros::Duration(1.0);
            // goal.trajectory.points[0].positions = desired_goal;
            // goal.trajectory.points[0].velocities = speed_goal;

            // useful 20221207
            client_ptr->sendGoal(goal);
            goal.trajectory.points.clear();
            // cout << "here" << endl;
            // useful 20221207 ---------------------------------------------
            
            // int count = 0;
            // while(!(client_ptr->getState().isDone()) && ros::ok())
            // {
            //     cout << "is_done: " << client_ptr->getState().isDone() << endl;
            //     cout << "here" << endl;
            //     ros::Duration(2).sleep(); // sleep for four seconds
            //     if (count >= 2)
            //         client_ptr->cancelGoal();
            //     count++;
            // }
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "fr3_test_code");
    ros::NodeHandle nh;
    string robot_topic = "/frrobot/joint_states";
    ros::Subscriber joint_state_sub = nh.subscribe(robot_topic, 3, jointStateCallback);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>(robot_topic, 3);

    tf::StampedTransform stamp_tmp1, stamp_fr3;
    tf::TransformListener tf_listener;

    // stamp_tmp1.setRotation(tf::Quaternion(0,0,0,1));
    // stamp_tmp1.setOrigin(tf::Vector3(0.1, 0.1, 0.1));
    
    // tf::Quaternion qq;
    // stamp_fr3.setRotation(tf::Quaternion(0,0,0,1));
    
    // stamp_fr3.setOrigin(tf::Vector3(stamp_tmp1.getOrigin().x() - 0.1, 
    //                                 stamp_tmp1.getOrigin().y(),
    //                                 stamp_tmp1.getOrigin().z()));
    
    // tf::Vector3 xxx = stamp_tmp1.getOrigin() - stamp_fr3.getOrigin();
    // cout << xxx.length() << endl;
    // cout << "here" << endl;

    // /frrobot/position_trajectory_controller/follow_joint_trajectory
    Client client("/frrobot/position_trajectory_controller/follow_joint_trajectory");
    client_ptr = &client;
    // cout << "here" << endl;
    client_ptr->waitForServer();
    // cout << "have connected" << endl;
    // cout << "222" << endl;

    
    // command_msg.header.frame_id = "FR3-V5.0";   // non-useful 20221207
    while(ros::ok()){
        try{
            tf_listener.lookupTransform(base_name, "init_point", ros::Time(0), stamp_tmp1);
            // cout << "X: " << stamp_tmp.getOrigin().getX() << ", Y: " << stamp_tmp.getOrigin().getY()<< ", Z: " << stamp_tmp.getOrigin().getZ() << endl;
            tf_listener.lookupTransform(base_name, tcp_name, ros::Time(0), stamp_fr3);
            trajectory_dis = cal_distance(stamp_tmp1, stamp_fr3);
            cout << trajectory_dis << endl;
            ROS_INFO("receive the point ..");
            JointState_available = true;
            break;
        }
        catch(tf::TransformException &ex){
            continue;
        }
    }

    TRAC_IK::TRAC_IK ik_solver(base_name, tcp_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
    ros::Rate rate(60);
    ros::spinOnce();

    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory.joint_names = joint_names;
    // goal.trajectory.points.resize(2);
    // // goal.trajectory.points[0].positions = joint_angles;
    // // goal.trajectory.points[0].velocities = joint_speed;
    // goal.trajectory.points[0].positions = {joint_angles[0]-0.1, joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5]};
    // goal.trajectory.points[0].velocities = {0.1,0.1,0.1,0.1,0.1};
    // goal.trajectory.points[1].positions = {joint_angles[0]-0.2, joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5]};
    // goal.trajectory.points[1].velocities = {0.1,0.1,0.1,0.1,0.1};
    // client_ptr->sendGoal(goal);

    // sendcmd(ik_solver, stamp_tmp1);

    // for test 
    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    // goal.trajectory.joint_names = joint_names;
    // goal.trajectory.points.resize(2);

    // // First trajectory point
    // // Positions
    // int index = 0;
    // goal.trajectory.points[index].positions.resize(6);
    // goal.trajectory.points[index].positions[0] = 0.2;
    // goal.trajectory.points[index].positions[1] = 0.0;
    // goal.trajectory.points[index].positions[2] = -1.5;
    // goal.trajectory.points[index].positions[3] = 1.94;
    // goal.trajectory.points[index].positions[4] = -1.57;
    // goal.trajectory.points[index].positions[5] = 0.0;

    // goal.trajectory.points[index].velocities.resize(6);
    // for (int j = 0; j < 6; ++j)
    // {
    //     goal.trajectory.points[index].velocities[j] = 1.0;
    // }
    // // To be reached 2 second after starting along the trajectory
    // goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
    // // Second trajectory point
    // // Positions
    // index += 1;
    // goal.trajectory.points[index].positions.resize(6);
    // goal.trajectory.points[index].positions[0] = 2.5;
    // goal.trajectory.points[index].positions[1] = 0.2;
    // goal.trajectory.points[index].positions[2] = -2.1;
    // goal.trajectory.points[index].positions[3] = 1.9;
    // goal.trajectory.points[index].positions[4] = 1.0;
    // goal.trajectory.points[index].positions[5] = 0.0;
    // // Velocities
    // goal.trajectory.points[index].velocities.resize(6);
    // for (int j = 0; j < 6; ++j)
    // {
    //     goal.trajectory.points[index].velocities[j] = 0.0;
    // }
    // goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
    // cout << "111" << endl;
    // client_ptr->sendGoal(goal);
    
    // // cout << client_ptr->getState().isDone() << endl;

    // while(!(client_ptr->getState().isDone()) && ros::ok())
    // {
    //     ros::Duration(4).sleep(); // sleep for four seconds
    // }

    sensor_msgs::JointState sensor_joint_state;    // 可以试一下这个
    
    sendcmd(ik_solver, stamp_tmp1);   // useful 20221207
    // JointState_available = false;
    int count_num = 0;

    // useful 20221207 ---------------------------------------------
    while(ros::ok()){
        
        ros::spinOnce();
        // sendcmd(ik_solver, stamp_tmp1);
        if (client_ptr->getState().isDone()){
            ROS_INFO("have finished !!");
            break;
        }
        // cout << count_num << endl;
        // if (count_num >= 200){
        //     client_ptr->cancelGoal();  // 直接使用这个取消函数
        //     break;
        //     // while(ros::ok()){
        //     //     try{
        //     //         tf_listener.lookupTransform(base_name, tcp_name, ros::Time(0), stamp_fr3);

        //     //         // cout << "X: " << stamp_tmp.getOrigin().getX() << ", Y: " << stamp_tmp.getOrigin().getY()<< ", Z: " << stamp_tmp.getOrigin().getZ() << endl;
        //     //         JointState_available = true;
        //     //         break;
        //     //     }
        //     //     catch(tf::TransformException &ex){
        //     //         continue;
        //     //     }
        //     //     // ROS_INFO("receive the point ..");
        //     // }
        //     // if (JointState_available){
                
        //     //     sendcmd(ik_solver, stamp_fr3);
        //     //     cout << "hi" << endl;
        //     //     break;
        //     // }
        // }
        // count_num++;
        // cout << "have stop it" << endl;
        rate.sleep();
    }
    // useful 20221207 ---------------------------------------------

    // int count = 0;
    // while(ros::ok()){
    //     ros::spinOnce();
    //     // cout << stamp_tmp1.getOrigin().getX() << " " <<
    //     //         stamp_tmp1.getOrigin().getY() << " " <<
    //     //         stamp_tmp1.getOrigin().getZ() << endl;
                
    //     sendcmd(ik_solver, stamp_tmp1);

    //     // control_msgs::FollowJointTrajectoryGoal goal;
    //     // goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    //     // goal.trajectory.joint_names = joint_names;
    //     // goal.trajectory.points.resize(1);
            
    //     // goal.trajectory.points[0].positions = desired_goal;
    //     // goal.trajectory.points[0].velocities = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    //     // goal.trajectory.points[0].time_from_start = ros::Duration(2);
    //     // goal.trajectory.points[0].accelerations = {};

    //     // goal.trajectory.points[1].positions = desired_goal;
    //     // goal.trajectory.points[1].velocities = speed_goal;
    //     // goal.trajectory.points[1].time_from_start = ros::Duration(0.2);
    //     // // goal.trajectory.points[1].accelerations = {};

    //     // // goal.trajectory.points[0].positions = {};
    //     // // goal.trajectory.points[0].velocities = {};
    //     // // goal.trajectory.points[0].positions = desired_goal;
    //     // // goal.trajectory.points[0].velocities = speed_goal;
    //     // client_ptr->sendGoal(goal);
    //     // goal.trajectory.points.clear();
    //     // client_ptr->sendGoal(goal);
    //     rate.sleep();
    //     // cout << count << endl;
    //     if (++count >= 1)
    //         break;
    //     stamp_seq++;
    // }


    // ros::spin();

    return 0;
}
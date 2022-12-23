#include<iostream>
#include<string>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<vector>
#include<math.h>
#include<moveit/move_group_interface/move_group.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit_msgs/PlanningScene.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit_msgs/MoveGroupAction.h>
#include<actionlib/client/simple_action_client.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "fr3_test3");
    ros::NodeHandle nh;
    ROS_INFO_ONCE("this is a demo to get joint pose ..");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //　定义moveit组件
    moveit::planning_interface::MoveGroupInterface arm("fr3_arm");
    string end_effector_link = arm.getEndEffectorLink();
    cout << end_effector_link << endl;

    // 定义机械臂的信息
    // arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.02);
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setPlanningTime(5.0);

    string reference_frame = "world";
    arm.setPoseReferenceFrame(reference_frame);
    geometry_msgs::Pose curPose;
    // cout << "now Robot poeition: [X, Y, Z]: [" << curPose.position.x << ", " << 
    //                                               curPose.position.y << ", " << 
    //                                               curPose.position.z << "]" << endl;

    // cout << "now Robot orientation: [x, y, z, w]: [" << curPose.orientation.x << ", " << 
    //                                                     curPose.orientation.y << ", " << 
    //                                                     curPose.orientation.z << ", " <<
    //                                                     curPose.orientation.w << "]" << endl;

    // geometry_msgs::Pose desired_pose;
    float step_dis = 0.025;
    float end_dis = 0.3;

    geometry_msgs::Pose init_pose = arm.getCurrentPose(end_effector_link).pose;
    // geometry_msgs::Pose desired_pose;
    // desired_pose.orientation = curPose.orientation;
    // desired_pose.position.x = end_dis;
    // desired_pose.position.y = curPose.position.y;
    // desired_pose.position.z = curPose.position.z;
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    // arm.plan(my_plan1);
    // int size_ = my_plan1.trajectory_.joint_trajectory.points.size();
    // cout << size_ << endl;
    // cout << my_plan1.trajectory_.joint_trajectory.points[0] << endl;
    // cout << my_plan1.trajectory_.joint_trajectory.points[1] << endl;
    // cout << my_plan1.trajectory_.joint_trajectory.points[2] << endl;


    // for (int i = 0; i < size_; ++i){
    //     vector<double> targetPose = my_plan1.trajectory_.joint_trajectory.points[i].positions;
    //     // cout << my_plan1.trajectory_.joint_trajectory.points[i].positions[0] << endl;
    //     arm.setJointValueTarget(targetPose);
        
    //     arm.move();
    // }

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    // vector<geometry_msgs::Pose> way_point;
    // way_point.emplace_back(curPose);
    // int steps = (int)( abs(end_dis - curPose.position.x) / step_dis + 0.5);
    // for (int i = 0; i < steps; ++i){
    //     geometry_msgs::Pose tmp_pose;
    //     tmp_pose.orientation.x = curPose.orientation.x;
    //     tmp_pose.orientation.y = curPose.orientation.y;
    //     tmp_pose.orientation.z = curPose.orientation.z;
    //     tmp_pose.orientation.w = curPose.orientation.w;

    //     if (end_dis > 0)
    //         tmp_pose.position.x = curPose.position.x + step_dis * i;
    //     else
    //         tmp_pose.position.x = curPose.position.x - step_dis * i;
    //     cout << tmp_pose.position.x << endl;
    //     tmp_pose.position.y = curPose.position.y;
    //     tmp_pose.position.z = curPose.position.z;
    //     way_point.emplace_back(tmp_pose);
    // }

    // // moveit_msgs::RobotTrajectory trajectory;
    // // const double jump_thres = 0.0;
    // // const double eef_step = 0.01;
    // // double fraction = 0.0;
    // // int maxtries = 100;
    // // int attempts = 0;

    // // while(fraction < 1.0 && attempts < maxtries){
    // //     fraction = arm.computeCartesianPath(way_point, eef_step, jump_thres, trajectory);
    // //     attempts++;
    // //     if (attempts % 10 == 0)
    // //         ROS_INFO("Still trying after %d attempts", attempts);
    // // }


    int count = 0;
    while(ros::ok()){
        // cout << "working .." << endl;
        curPose = arm.getCurrentPose(end_effector_link).pose;
        geometry_msgs::Pose desired_pose;
        // if (count < steps){
        //     desired_pose = way_point[count];
        //     count++;
        //     // cout << desired_pose.position.x << endl;
        // }
        // else{
        //     way_point.clear();
        //     ros::shutdown();
        // }
        desired_pose.orientation = init_pose.orientation;
        if (end_dis > 0){
            desired_pose.position.x = curPose.position.x + step_dis;
            cout << desired_pose.position.x << endl;
            if (desired_pose.position.x >= end_dis){
                // cout << "here .." << endl;
                ros::shutdown();
            }
                
        }
        else{
            desired_pose.position.x = curPose.position.x - step_dis;
            cout << desired_pose.position.x << endl;
            if (desired_pose.position.x <= end_dis)
                ros::shutdown();
        }
        
        desired_pose.position.y = curPose.position.y;
        desired_pose.position.z = curPose.position.z;
        arm.setPoseTarget(desired_pose);
        // arm.setApproximateJointValueTarget(desired_pose, end_effector_link);
        // arm.move();
        // arm.plan(my_plan1);
        // cout << count << endl;
        // if (count == 10)
        //     arm.stop();
        arm.move();
        // arm.asyncExecute(my_plan1);
        // arm.asyncMove();
        // actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>& tmp = arm.getMoveGroupClient();

    }

    return 0;
}
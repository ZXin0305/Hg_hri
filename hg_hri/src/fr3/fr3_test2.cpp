#include<iostream>
#include<string>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>

#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit_msgs/PlanningScene.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "fr3_test");
    ros::NodeHandle nh;
    ROS_INFO_ONCE("this is a demo to get joint pose ..");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("fr3_arm");
    string end_effector_link = arm.getEndEffectorLink();
    cout << end_effector_link << endl;
    arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);

    string reference_frame = "world";
    arm.setPoseReferenceFrame(reference_frame);
    geometry_msgs::Pose curPose = arm.getCurrentPose(end_effector_link).pose;
    cout << "now Robot poeition: [X, Y, Z]: [" << curPose.position.x << ", " << 
                                                  curPose.position.y << ", " << 
                                                  curPose.position.z << "]" << endl;

    cout << "now Robot orientation: [x, y, z, w]: [" << curPose.orientation.x << ", " << 
                                                        curPose.orientation.y << ", " << 
                                                        curPose.orientation.z << ", " <<
                                                        curPose.orientation.w << "]" << endl;
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.369603;
    target_pose.orientation.y = -0.600847;
    target_pose.orientation.z = 0.651715;
    target_pose.orientation.w = -0.278643;

    target_pose.position.x = -0.1;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.28;

    arm.setApproximateJointValueTarget(target_pose, end_effector_link);
    arm.move();
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit_msgs::MotionPlanRequest response;
    // arm.plan(my_plan);
    // arm.execute(my_plan);

    // ---------------------------------------
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.x = 0.369603;
    target_pose1.orientation.y = -0.600847;
    target_pose1.orientation.z = 0.651715;
    target_pose1.orientation.w = -0.278643;

    target_pose1.position.x = -0.05;
    target_pose1.position.y = 0.3;
    target_pose1.position.z = 0.28;
    arm.setApproximateJointValueTarget(target_pose1, end_effector_link);
    // arm.move();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    moveit_msgs::MotionPlanRequest response1;
    arm.plan(my_plan1);
    // arm.execute(my_plan1);

    // arm.setPoseTarget(target_pose);
    // arm.setApproximateJointValueTarget(target_pose, "wrist3_Link");
    // arm.move();
    
    return 0;
}
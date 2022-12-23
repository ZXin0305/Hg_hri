#include<hg_hri/HumanInfo.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "sub_human_pose");
    ros::NodeHandle nh;
    string camera_name = "camera_base_1";
    Human human(camera_name);
    human.run();
}
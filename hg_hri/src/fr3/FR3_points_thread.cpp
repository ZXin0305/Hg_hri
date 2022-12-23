#include<hg_hri/FR3_points.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_points_fr3");
    ROS_INFO_ONCE("pub_points_fr3");
    ros::NodeHandle nh;
    Fr3Points points;
    points.run();
}
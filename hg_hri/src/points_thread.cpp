#include<hg_hri/Points.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_points");
    ros::NodeHandle nh;
    Points points;
    points.run();
}
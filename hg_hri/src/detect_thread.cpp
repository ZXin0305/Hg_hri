#include<hg_hri/Detect.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "marker_detect_thread");
    ros::NodeHandle nh;
    string camera_topic = "/kinectSDK/color";  // /camera/color/image_raw   kinectSDK/color
    string camera_param_path = "/home/zx/hg_ws/src/hg_hri/cam_data/cam.yml";  // cam.yml  rs.yml
    Object marker_detector(camera_topic, camera_param_path);
    marker_detector.run();
}
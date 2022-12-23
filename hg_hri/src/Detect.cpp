#include<hg_hri/Detect.h>

void Object::load_camera_param(){
    cv::FileStorage fs(this->camera_param_path, cv::FileStorage::READ);  // 相机内参
    if(!fs.isOpened()){
        ROS_INFO("cannot open the configuration file !!!");
        exit(1);
    }
    fs["camera_matrix"] >> this->camera_matrix;
    fs.release();

    cv::Mat distCoeffs_origin = (Mat_<double>(1,5) << 0,0,0,0,0);
    this->camera_dist = distCoeffs_origin.clone();
    ROS_INFO("load camera param sucessfully !! ");
}

void Object::image_callback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);   // 直接用RGB8就好了
        cv_ptr->image.copyTo(this->img);
    }
    catch(cv_bridge::Exception& ex){
        ROS_INFO("cv_bridge exception: %s", ex.what());
        return;
    }
}

void Object::locate_marker(){
    this->marker_ids.clear();
    this->marker_corners.clear();
    this->tvecs.clear();
    this->rvecs.clear();

    if(!this->img.empty()){
        cv::aruco::detectMarkers(img, dictionary, marker_corners, marker_ids);

        // 只检测的话，下面的就可以不用了
        // int marker_id_size = marker_ids.size();
        // if(marker_id_size > 0){
        //     cv::aruco::drawDetectedMarkers(img, marker_corners, marker_ids);

        //     for(int idx = 0; idx < marker_id_size; ++idx){
        //         int marker_id = marker_ids[idx];

        //         if(marker_id >= 0){
        //             cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.05, camera_matrix, camera_dist, rvecs, tvecs);
        //             cv::aruco::drawAxis(img, camera_matrix, camera_dist, rvecs[idx], tvecs[idx], 0.1);
        //         }
        //     }
        // }
        // for show result
        // char window_name[50];
        // sprintf(window_name, "camera_%d", 1);
        // cv::imshow(window_name, this->img);
        // cv::waitKey(1);
    }
}

void Object::senMarkerID(){
    this->tmp_marker_vec.clear();
    int marker_id_size = marker_ids.size();
    for(int i = 0; i < marker_id_size; ++i){
        int marker_id = marker_ids[i];
        if(marker_id >= 0){
            tmp_marker_vec.emplace_back(marker_id);
        }
    }
    marker_id_list.data = tmp_marker_vec;
    marker_id_pub.publish(marker_id_list);
}

void Object::run(){
    ros::Rate rate(100);

    while(ros::ok()){
        ros::spinOnce();

        locate_marker();
        senMarkerID();
        // rate.sleep();
    }
}
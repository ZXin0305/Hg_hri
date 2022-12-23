#include<hg_hri/FR3_points.h>

void Fr3Points::init(){
    // tf::Quaternion q_init(0.709983,-0.702785,0.0213792,0.0396129);  //初始点  X,Y,Z,W 
    // q_init = q_init.normalize();
    // tmp.setOrigin(tf::Vector3(-0.2,-0.2,0.50));    //-0.4, -0.55, 0.35
    // tmp.setRotation(q_init);
    // cap_init.push_back(tmp);

    // for fr3
    // tf::Quaternion q_init(-0.459429,-0.54028,0.4785,0.517733);  //初始点  X,Y,Z,W 
    // q_init = q_init.normalize();
    // tmp.setOrigin(tf::Vector3(-0.0738,0.408,0.2795));    //-0.4, -0.55, 0.35
    // tmp.setRotation(q_init);
    // cap_init.push_back(tmp);

    tf::Quaternion q_init(0.3131, 0.6456, -0.6169, -0.3232);  //初始点  X,Y,Z,W   init point
    q_init = q_init.normalize();
    tmp.setOrigin(tf::Vector3(0.061, 0.427, 0.4));    //-0.4, -0.55, 0.35
    tmp.setRotation(q_init);
    cap_init.push_back(tmp);

    tf::Quaternion q_cap(-0.04715, 0.7156, -0.6964, 0.0241);  //抓取上方  cap point
    q_cap = q_cap.normalize();
    tmp.setOrigin(tf::Vector3(-0.351, 0.27, 0.4));   //0.0, -0.55, 0.35
    tmp.setRotation(q_cap);
    cap_init.push_back(tmp);

    tf::Quaternion q_mid(0.3131, 0.6456, -0.6169, -0.3232);  //过渡点 middle point
    q_mid = q_mid.normalize();
    tmp.setOrigin(tf::Vector3(-0.1, 0.27, 0.4));  //0.4, -0.55, 0.35
    tmp.setRotation(q_mid);     
    cap_init.push_back(tmp);

    tf::Quaternion q_assist(0.1063, -0.6899, 0.7066, -0.1153);  //　固定协助点  assist point
    q_assist = q_assist.normalize();
    tmp.setOrigin(tf::Vector3(0.058, 0.58, 0.23));  //0.4, -0.55, 0.35
    tmp.setRotation(q_assist);     
    cap_init.push_back(tmp);



    /*
    　　　　　物块的固定抓取位置
    */
    tf::Quaternion q_obj0_cor(0.0049, 0.7204, -0.6934, -0.0146);  //obj0 correct
    q_obj0_cor = q_obj0_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.29, 0.40, 0.30));
    tmp_correct.setRotation(q_obj0_cor);
    obj_target.push_back(tmp_correct);     

    tf::Quaternion q_obj0_tar(0.0049, 0.7204, -0.6934, -0.0146);  //obj0 target 
    q_obj0_tar = q_obj0_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.29, 0.40, 0.236));
    tmp_target.setRotation(q_obj0_tar);     
    obj_target.push_back(tmp_target);

    tf::Quaternion q_obj1_cor(-0.0998, 0.7112, -0.6909, 0.0732);  //obj1 correct
    q_obj1_cor = q_obj1_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.32, 0.24, 0.30));
    tmp_correct.setRotation(q_obj1_cor);
    obj_target.push_back(tmp_correct);      

    tf::Quaternion q_obj1_tar(-0.0998, 0.7112, -0.6909, 0.0732);  //obj1 target
    q_obj1_tar = q_obj1_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.32, 0.24, 0.24));
    tmp_target.setRotation(q_obj1_tar);     
    obj_target.push_back(tmp_target);


    tf::Quaternion q_obj2_cor(-0.095, 0.7164, -0.6871, 0.0732);  //obj2 correct
    q_obj2_cor = q_obj2_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.398, 0.34, 0.30));
    obj_target.push_back(tmp_correct);   

    tf::Quaternion q_obj2_tar(-0.095, 0.7164, -0.6871, 0.0732);  //obj2 target
    q_obj2_tar = q_obj2_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.398, 0.34, 0.25));
    tmp_target.setRotation(q_obj2_tar);  
    obj_target.push_back(tmp_target);
}


void Fr3Points::pub_points(){
    // ----------------------------------- //
    // 固定的过渡点、初始点、放置点

    br.sendTransform(tf::StampedTransform(this->cap_init[0], ros::Time::now(), this->node_name, "init_point"));
    br.sendTransform(tf::StampedTransform(this->cap_init[1], ros::Time::now(), this->node_name, "cap_point"));
    br.sendTransform(tf::StampedTransform(this->cap_init[2], ros::Time::now(), this->node_name, "mid_point"));
    br.sendTransform(tf::StampedTransform(this->cap_init[3], ros::Time::now(), this->node_name, "assist_point"));
}

void Fr3Points::pub_obj_points(){
    for (int i = 0; i < object_num; ++i){
        auto tmp1 = obj_correct_name + to_string(i);
        auto tmp2 = obj_target_name + to_string(i);
        br.sendTransform(tf::StampedTransform(obj_target[i * 2 + 0],
                                              ros::Time::now(),
                                              node_name,
                                              tmp1));
        br.sendTransform(tf::StampedTransform(obj_target[i * 2 + 1],
                                              ros::Time::now(),
                                              node_name,
                                              tmp2));
    }
}


// 人员的信息
void Fr3Points::humanCallback(const std_msgs::Int32MultiArray& human_id_hg){
    this->human_info.clear();
    this->human_info = human_id_hg.data;
    int human_num = this->human_info.size() / info_len;
    if (human_num == 0){
        ROS_INFO("no people ..");
    }
    else {
        this->human_hg_vec.clear();
        for (int i = 0; i < human_num; ++i){
            if (human_info[i * info_len + 1] == 1){     // 是操作人员,这里只需要判断是不是操作人员就好了，robot那么需要全部接收
                this->human_hg_vec.push_back({(int)human_info[i * info_len],
                                              (int)human_info[i * info_len + 2],
                                              (int)human_info[i * info_len + 3]
                                             });
            }
        }
        sort(this->human_hg_vec.begin(), this->human_hg_vec.end(), 
            [](vector<int>& v1, vector<int>& v2){return v1[0] < v2[0];}
            );
    }
}

void Fr3Points::decode_human_info(){
    // 取出第一个人的，如果第一个操作者的hg没有(左右同时没有)，那么就依次在后面找
    // 更新的是在类中的，即类中所有成员函数共享 
    lhg = -1;
    rhg = -1;  // 左，右
    for (auto xx : human_hg_vec){
        if (xx[1] != -1 || xx[2] != -1){
            lhg = xx[1];
            rhg = xx[2];
            break;
        }
    }
}

void Fr3Points::listen_point(){
    // 监听当前的tool的位置
    while (ros::ok()){
        try{
            this->tf_listener.lookupTransform(this->base_name, this->tip_name, ros::Time(0), current_tool_pose); 
            break;
        }
        catch (tf::TransformException& ex){
            continue;
        }
    }
}

// 旋转
void Fr3Points::rotate_tool(){
    ros::Rate rate(60);
    float angle_val = 0;
    this->listen_point();
    if (lhg == 6)
        angle_val = this->rotate_angle;   // 正转
    else if (lhg == 7)
        angle_val = this->rotate_angle * (-1);
    // angle_val = this->rotate_angle * (1);   // test 20221217
    auto q_rotate = current_tool_pose.getRotation();
    q_rotate.setRPY(0, angle_val, 0.0);
    tmp.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tmp.setRotation(q_rotate);

    clock_t st = clock();
    while(ros::ok()){
        ros::spinOnce();
        decode_human_info();
        if (lhg == 10 || rhg == 10)
            break;
        clock_t et = clock();
        cout << (double)(et - st) / (double)CLOCKS_PER_SEC << endl;
        if ((double)(et - st)/(double)CLOCKS_PER_SEC >= 0.3)   // 就循环10s
            break;
        br.sendTransform(tf::StampedTransform(tmp, ros::Time::now(), tip_name, "rotate_point"));
        rate.sleep();
    }
}

void Fr3Points::run(){
    ros::Rate rate(60);
    bool flag = 0;  // 测试用的
    while(ros::ok()){
        ros::spinOnce();
        decode_human_info();

        if (lhg == 6 || lhg == 7){
            rotate_tool();   // for test other functions 20221209
            flag = 1;
        }
        pub_points();
        pub_obj_points();
        
        rate.sleep();
        ROS_INFO("pub points");
    }
}
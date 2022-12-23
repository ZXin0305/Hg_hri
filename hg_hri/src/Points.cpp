#include<hg_hri/Points.h>

void Points::init(){
    //初始点，过渡点，放置点
    //都是相较于marker_0的
    // tf::Quaternion q_init(-0.696,0.715,-0.022,-0.0393);  //初始点  X,Y,Z,W 
    // q_init = q_init.normalize();
    // tmp.setOrigin(tf::Vector3(-0.18,-0.18,0.417));
    // tmp.setRotation(q_init);
    // cap_init.push_back(tmp);

    // tf::Quaternion q_cap(0.942,-0.331,0.0195,0.03);  //过渡点
    // q_cap = q_cap.normalize();
    // tmp.setOrigin(tf::Vector3(-0.42, 0.13, 0.417));
    // tmp.setRotation(q_cap);
    // cap_init.push_back(tmp);

    // tf::Quaternion q_place(0.999,0.009,0.019,0.018);  //放置点
    // q_place = q_place.normalize();
    // tmp.setOrigin(tf::Vector3(-0.779, 0.214, 0.417));
    // tmp.setRotation(q_place);     
    // cap_init.push_back(tmp);

    // ********************************************************************* // 
    // tf::Quaternion q_init(0.709983,-0.702785,0.0213792,0.0396129);  //初始点  X,Y,Z,W 
    // q_init = q_init.normalize();
    // tmp.setOrigin(tf::Vector3(-0.2,-0.2,0.50));    //-0.4, -0.55, 0.35
    // tmp.setRotation(q_init);
    // cap_init.push_back(tmp);

    // for test fr3
    // tf::Quaternion q_init(-0.459429,-0.54028,0.4785,0.517733);  //初始点  X,Y,Z,W 
    // q_init = q_init.normalize();
    // tmp.setOrigin(tf::Vector3(-0.0738,0.408,0.2795));    //-0.4, -0.55, 0.35
    // tmp.setRotation(q_init);
    // cap_init.push_back(tmp);

    // tf::Quaternion q_cap(0.709983,-0.702785,0.0213792,0.0396129);  //抓取上方
    // q_cap = q_cap.normalize();
    // tmp.setOrigin(tf::Vector3(0.299,0.21,0.27));   //0.0, -0.55, 0.35
    // tmp.setRotation(q_cap);
    // cap_init.push_back(tmp);

    // tf::Quaternion q_mid(0.709983,-0.702785,0.0213792,0.0396129);  //过渡点
    // q_mid = q_mid.normalize();
    // tmp.setOrigin(tf::Vector3(-0.024,-0.24,0.30));  //0.4, -0.55, 0.35
    // tmp.setRotation(q_mid);     
    // cap_init.push_back(tmp);

    // tf::Quaternion q_assist(0.709983,-0.702785,0.0213792,0.0396129);  //　固定协助点
    // q_assist = q_assist.normalize();
    // tmp.setOrigin(tf::Vector3(-0.024,-0.24,0.30));  //0.4, -0.55, 0.35
    // tmp.setRotation(q_assist);     
    // cap_init.push_back(tmp);    

/* ----------------------------------------------------------------------------- */
    //物件的固定抓取位置
    //６个物件
    // tf::Quaternion q_obj0_cor(0.998, -0.0409, 0.0192, 0.0059);  //obj0 correct
    // q_obj0_cor = q_obj0_cor.normalize();
    // tmp.setOrigin(tf::Vector3(-0.83,0.37,0.25));
    // tmp.setRotation(q_obj0_cor);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj0_tar(0.998, -0.0409, 0.0192, 0.0059);  //obj0 target 
    // q_obj0_tar = q_obj0_tar.normalize();
    // tmp.setOrigin(tf::Vector3(-0.83,0.37,0.08));
    // tmp.setRotation(q_obj0_tar);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj1_cor(0.998, -0.0409, 0.0192, 0.0059);  //obj1 correct
    // q_obj1_cor = q_obj1_cor.normalize();
    // tmp.setOrigin(tf::Vector3(-0.9,0.37,0.25));
    // tmp.setRotation(q_obj1_cor);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj1_tar(0.998, -0.0409, 0.0192, 0.0059);  //obj1 target
    // q_obj1_tar = q_obj1_tar.normalize();
    // tmp.setOrigin(tf::Vector3(-0.9,0.37,0.08));
    // tmp.setRotation(q_obj1_tar);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj2_cor(0.714, -0.7, -0.002, 0.0004);  //obj2 correct
    // q_obj2_cor = q_obj2_cor.normalize();
    // tmp.setOrigin(tf::Vector3(-0.125, -0.120, 0.264));
    // tmp.setRotation(q_obj2_cor);     
    // obj_target.push_back(tmp);

    // tf::Quaternion q_obj2_tar(0.714, -0.7, -0.002, 0.0004);  //obj2 target
    // q_obj2_tar = q_obj2_tar.normalize();
    // tmp.setOrigin(tf::Vector3(-0.125, -0.120, 0.264));
    // tmp.setRotation(q_obj2_tar);     
    // obj_target.push_back(tmp); 

    // ***************************************************************** //
    tf::Quaternion q_obj0_cor(-0.683361,0.729727,0.002334114,-0.0226279);  //obj0 correct
    q_obj0_cor = q_obj0_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.21,-0.01,0.45));
    tmp_correct.setRotation(q_obj0_cor);     

    tf::Quaternion q_obj0_tar(-0.683361,0.729727,0.002334114,-0.0226279);  //obj0 target 
    q_obj0_tar = q_obj0_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.21,-0.01,0.26));
    tmp_target.setRotation(q_obj0_tar);     
    obj_target_new.push_back({tmp_correct, tmp_target});

    un_map[0]++;  // 这个是说明存在这个点

    tf::Quaternion q_obj1_cor(-0.683361,0.729727,0.002334114,-0.0226279);  //obj1 correct
    q_obj1_cor = q_obj1_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.28,-0.01,0.45));
    tmp_correct.setRotation(q_obj1_cor);     

    tf::Quaternion q_obj1_tar(-0.683361,0.729727,0.002334114,-0.0226279);  //obj1 target
    q_obj1_tar = q_obj1_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.28,-0.01,0.26));
    tmp_target.setRotation(q_obj1_tar);     
    obj_target_new.push_back({tmp_correct, tmp_target});

    un_map[1]++;  // 这个是说明存在这个点

    tf::Quaternion q_obj2_cor(-0.683361,0.729727,0.002334114,-0.0226279);  //obj2 correct
    q_obj2_cor = q_obj2_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.35,-0.01,0.45));
    tmp_correct.setRotation(q_obj2_cor);     

    tf::Quaternion q_obj2_tar(-0.683361,0.729727,0.002334114,-0.0226279);  //obj2 target
    q_obj2_tar = q_obj2_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.35,-0.01,0.26));
    tmp_target.setRotation(q_obj2_tar);  
    obj_target_new.push_back({tmp_correct, tmp_target});

    un_map[2]++;  // 这个是说明存在这个点   

    tf::Quaternion q_obj3_cor(-0.683361,0.729727,0.002334114,-0.0226279);  //obj3 correct
    q_obj3_cor = q_obj3_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.21,0.06,0.45));
    tmp_correct.setRotation(q_obj3_cor);     

    tf::Quaternion q_obj3_tar(-0.683361,0.729727,0.002334114,-0.0226279);  //obj3 target
    q_obj3_tar = q_obj3_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.21,0.06,0.26));
    tmp_target.setRotation(q_obj3_tar);   
    obj_target_new.push_back({tmp_correct, tmp_target}); 

    un_map[3]++;  // 这个是说明存在这个点        

    tf::Quaternion q_obj4_cor(-0.683361,0.729727,0.002334114,-0.0226279);  //obj4 correct
    q_obj4_cor = q_obj4_cor.normalize();
    tmp_correct.setOrigin(tf::Vector3(-0.28,0.06,0.45));
    tmp_correct.setRotation(q_obj4_cor);     

    tf::Quaternion q_obj4_tar(-0.683361,0.729727,0.002334114,-0.0226279);  //obj4 target
    q_obj4_tar = q_obj4_tar.normalize();
    tmp_target.setOrigin(tf::Vector3(-0.28,0.06,0.26));
    tmp_target.setRotation(q_obj4_tar);     
    obj_target_new.push_back({tmp_correct, tmp_target});  

    un_map[4]++;  // 这个是说明存在这个点    

    cap_size = this->cap_init.size();
    obj_size = this->obj_target_new.size();

    // last_info  相当于初始化了
    last_info.last_mode = -1;
    last_info.last_mode1_idx = 0;
    last_info.last_mode1_step = 0;
    last_info.last_mode2_direct = -1;
    last_info.last_mode3_record = 0;
}

void Points::pub_points(){
    // ----------------------------------- //
    // 固定的过渡点、初始点、放置点

    // for test fr3
    br.sendTransform(tf::StampedTransform(this->cap_init[0], ros::Time::now(), "base_link", "init_point"));


    // br.sendTransform(tf::StampedTransform(this->cap_init[0], ros::Time::now(), this->node_name, "init_point"));
    // br.sendTransform(tf::StampedTransform(this->cap_init[1], ros::Time::now(), this->node_name, "cap_point"));
    // br.sendTransform(tf::StampedTransform(this->cap_init[2], ros::Time::now(), this->node_name, "mid_point"));
    // br.sendTransform(tf::StampedTransform(this->cap_init[3], ros::Time::now(), this->node_name, "assist_point"));
}

void Points::pub_obj_points(const int obj_idx){
    // 发布物件的位姿信息
    string obj_correct_name = "obj_correct";
    string obj_target_name = "obj_target";
    if(obj_idx > this->obj_size || un_map[obj_idx] == 0){  //id不能越界，且必须有这个
        ROS_INFO("incorrect obj idx, please check it !!!!!!");
    }
    else{
       br.sendTransform(tf::StampedTransform(this->obj_target_new[obj_idx][0], 
                                             ros::Time::now(), 
                                             this->node_name, 
                                             obj_correct_name)); 
       br.sendTransform(tf::StampedTransform(this->obj_target_new[obj_idx][1], 
                                             ros::Time::now(), 
                                             this->node_name, 
                                             obj_target_name)); 
       
    }
}


void Points::robotCallback(const hg_hri::robot2global& robot_msg){
    this->is_arrived = robot_msg.is_arrived;
    this->mode1_finish = robot_msg.mode1_finish;  // mode1需要这样判定一下
}

void Points::humanCallback(const std_msgs::Int32MultiArray& human_id_hg){
    this->human_info.clear();
    this->human_info = human_id_hg.data;
    int human_num = this->human_info.size() / info_len;
    if (human_num == 0){
        ROS_INFO("no people ..");
    }
    else{
        this->human_hg_vec.clear();
        for(int i = 0; i < human_num; ++i){
            if(human_info[i * info_len + 1] == 1){   // 是操作人员,这里只需要判断是不是操作人员就好了，robot那么需要全部接收
                this->human_hg_vec.push_back({(int)human_info[i * info_len],
                                                (int)human_info[i * info_len + 2],
                                                (int)human_info[i * info_len + 3]
                                             }
                                            );
            }
        }
        sort(this->human_hg_vec.begin(), this->human_hg_vec.end(), 
            [](vector<int>& v1, vector<int>& v2){return v1[0] < v2[0];}
            );

    }
}

void Points::mode1(const int idx){
    if(idx != -1)
        this->mode1_idx = idx;   // 这个mode1_idx只有当lhg不为-1时，才会进行更改, 因为lhg/rhg在每次decode_human_info的时候都会重置
    pub_obj_points(this->mode1_idx);
    if(this->is_arrived == 1){
       mode1_step = (mode1_step + 1) % mode1_process;  // 为了更新当前mode1进行到了哪一步了
    }
}

void Points::cal_next(){
    if(rhg != -1){
        direct = rhg;
    }
    // 计算新的点：
    double X = current_tool_pose.getRotation().getX();
    double Y = current_tool_pose.getRotation().getY();
    double Z = current_tool_pose.getRotation().getZ();
    double W = current_tool_pose.getRotation().getW();
    tf::Quaternion q_cal(X, Y, Z, W);
    q_cal = q_cal.normalize();
    next_point.setRotation(q_cal);

    auto x = current_tool_pose.getOrigin().getX();
    auto y = current_tool_pose.getOrigin().getY();
    auto z = current_tool_pose.getOrigin().getZ();
    if (direct == 1){  // 上
        next_point.setOrigin(tf::Vector3(x + 0.0 , y + 0.0, z + step));
    }
    else if(direct == 2){ // 下
        next_point.setOrigin(tf::Vector3(x + 0.0 , y + 0.0, z - step));
    }
    else if(direct == 3){ // 左
        next_point.setOrigin(tf::Vector3(x + 0.0 , y - step, z + 0.0));
    }
    else if(direct == 4){ // 右
        next_point.setOrigin(tf::Vector3(x + 0.0 , y + step, z + 0.0));
    }
    else if(direct == 5){ // 前
        next_point.setOrigin(tf::Vector3(x + step , y + 0.0, z + 0.0));
    }
    else if(direct == 6){  // 后
        next_point.setOrigin(tf::Vector3(x - step, y + 0.0, z - step));
    }
    else{
        next_point.setOrigin(tf::Vector3(x, y, z));
        ROS_INFO("don't change ..");
    }
}

void Points::listen_point(){
    // 监听当前tool的位置
    while(ros::ok()){
        try{
            this->tf_listener.lookupTransform("marker_0", "tool0", ros::Time(0), current_tool_pose);
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

void Points::decode_human_info(){
    // 取出第一个人的，如果第一个操作者的hg没有(左右同时没有)，那么就依次在后面找
    // 更新的是在类中的，即类中所有成员函数共享
    lhg = -1;
    rhg = -1;  // 左　右
    for (auto xx : human_hg_vec){
        if(xx[1] != -1 || xx[2] != -1){
            lhg = xx[1];
            rhg = xx[2];
            break;
        }
    }
}

void Points::mode2(){
    ros::Rate rate(60);
    while(ros::ok()){
        ros::spinOnce();
        decode_human_info();
        if(lhg == 10 || rhg == 10)  // 到达之后或者中途停止都会break掉(暂时这样)  //  this->is_arrived || 
        {
            mode2_flag = false;
            break;
        }
        if(this->is_arrived || (rhg != -1)){  // 当rhg在指定范围才会进入函数体，或者到达指定位置以后，也会进入函数体中，计算新的next_point...
            listen_point();   //监听当前的
            cal_next();
        }
        br.sendTransform(tf::StampedTransform(next_point, ros::Time::now(), "marker_0", "next_point"));
        rate.sleep();
    }
    // br.sendTransform(tf::StampedTransform(next_point, ros::Time::now(), "marker_0", "next_point"));
}

void Points::mode3(){
    // check
    if(this->record_list.size() == 0){
        ROS_INFO("the record list is empty !!!");
        mode3_flag = false;
    }
    else{
        ros::Rate rate(60);
        record_point_idx = 0;
        int record_point_size = this->record_list.size();
        while(ros::ok()){
            ros::spinOnce();
            decode_human_info();
            if(lhg == 10 || rhg == 10)  // 到达之后或者中途停止都会break掉
            {
                mode3_flag = false;
                break;
            }

            if(record_point_size > 0){
                if(is_arrived)
                    record_point_idx = (record_point_idx + 1) % record_point_size;  // 更新record_point的发布索引

                br.sendTransform(tf::StampedTransform(this->record_list[record_point_idx], 
                                                    ros::Time::now(), 
                                                    "marker_0", "record_point"));
            }
            rate.sleep();
        }
    }
}

void Points::record_point(){
    while(ros::ok()){
        try{
            this->listen_point();
            // this->tf_listener.lookupTransform("marker_0", "tool0", ros::Time(0), current_tool_pose);
            double X = current_tool_pose.getRotation().getX();
            double Y = current_tool_pose.getRotation().getY();
            double Z = current_tool_pose.getRotation().getZ();
            double W = current_tool_pose.getRotation().getW();

            tf::Quaternion q_record(X, Y, Z, W);
            q_record = q_record.normalize();
            tf::Vector3 origin_vec = current_tool_pose.getOrigin();
            tmp.setOrigin(origin_vec);
            tmp.setRotation(q_record);
            this->record_list.push_back(tmp);
            ROS_INFO("have recorded current point ..");
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

void Points::reset_info(){
    this->rhg = -1;
    this->rhg = -1;
    this->is_arrived = 0;
    this->human_info.clear();
    this->human_hg_vec.clear();
    this->record_list.clear();
    this->mode1_flag = false;
    this->mode2_flag = false;
    this->mode3_flag = false;

    this->mode1_idx = 0;
    this->mode1_obj_idx = 0;

    this->direct = -1;

    this->record_point_idx = 0;

    // last_info
    this->last_info.last_mode = -1;
    this->last_info.last_mode1_idx = 0;
    this->last_info.last_mode1_step = 0;
    this->last_info.last_mode2_direct = -1;
    this->last_info.last_mode3_record = 0;

    this->is_accept = false;
    ROS_INFO("status info have beed reset ..");
}

void Points::last_memory(int mode){
    last_info.last_mode = mode;
    if(mode == 1){
        last_info.last_mode1_idx = mode1_idx;
        last_info.last_mode1_step = this->mode1_step;
    }
    else if(mode == 2){
        last_info.last_mode2_direct = direct;
    }
    else if(mode == 3){
        last_info.last_mode3_record = record_point_idx;
    }
}

void Points::pub_status(){
    clock_t st = clock();
    while(ros::ok()){
        ros::service::waitForService(srv_name);
        srv.request.last_mode = last_info.last_mode;   // 模式

        //　mode1
        srv.request.last_mode1_idx = last_info.last_mode1_idx;
        srv.request.last_mode1_step = last_info.last_mode1_step;  // 如果是模式１,还需要告知进行到哪一步啦

        // mode2
        srv.request.last_mode2_direct = last_info.last_mode2_direct;

        // mode3
        srv.request.last_mode3_record = last_info.last_mode3_record;  // 这个是上次记录的点的索引
        this->status_client.call(srv);
        if (srv.response.result == "ok"){
            ROS_INFO("have finished call ..");
            this->is_accept = true;
            break;
        }
        ROS_INFO("call for srv ..");
        clock_t et = clock();
        if(((et - st) / CLOCKS_PER_SEC) >= this->delay_time)
            break;
    }
}

void Points::rotate_tool(){
    ros::Rate rate(60);
    float angle_val = 0;
    while(ros::ok()){
        ros::spinOnce();  // 更新好需要的东西
        this->listen_point();
        if(lhg == 6)
            angle_val = this->rotate_angle;
        else if (lhg == 7)
            angle_val = (-1) * this->rotate_angle;
        auto q_rotate = current_tool_pose.getRotation();
        q_rotate.setRPY(0, 0, angle_val);   // 应该是绕Z轴把
        tmp.setRotation(q_rotate);
        tmp.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

        // 相对于tool0的
        br.sendTransform(tf::StampedTransform(tmp, ros::Time::now(), "tool0", "rotate_point"));

        if(this->lhg == 10 || this->rhg == 10)
            break;
    }
}

void Points::run(){
    ros::Rate rate(60);
    
    while (ros::ok())
    {
        // ros::spinOnce();
        pub_points();  // 发布必要的点

        // cout << human_hg_vec.size() << endl;

        // if(human_hg_vec.size() != 0){
        //     decode_human_info();
        //     if (mode1_flag || ((lhg >= 0 && lhg <= 5) || lhg == 10) || rhg == 10){  // 1. 抓取物块, 这个要确定两个点是否都达到了，并且要返回了
        //         /*
        //         0 <= lhg <= 10 : 10个物块，还有一个停止
        //         rhg：10停止
        //         */
        //         if (lhg != 10 || rhg != 10){
        //             mode1_flag = true;
        //             mode1(lhg);
        //         }
        //         else if(lhg == 10 || rhg == 10){
        //             last_memory(1);
        //             mode1_flag = false;
        //             direct = -1;
        //         }

        //         if(this->mode1_finish){
        //             mode1_flag = false;
        //             ROS_INFO("mode1 finished ..");
        //         }
        //         else{
        //             ROS_INFO("mode1 working ..");
        //         }
                
        //     }
        //     else if (mode2_flag || (lhg == 10 || rhg == 10) || (rhg >= 1 && rhg <= 6)){  // 2. 上下左右前后
                
        //         if (lhg != 10 || rhg != 10){
        //             // 监听当前末端位置
        //             // 计算新的点
        //             // 发布
        //             mode2_flag = true;  // 这个貌似没有用
        //             mode2();   //会陷入循环中
        //         }
        //         else if(lhg == 10 || rhg == 10){
        //             last_memory(2);
        //             mode2_flag = false;
        //             record_point_idx = 0;  // 如果停止之后，需要重新从第一个点开始的话
        //         }

        //         ROS_INFO("mode2 working ..");
        //     }
        //     else if(mode3_flag || lhg == 10 || rhg == 10 || rhg == 12 || rhg == 13){
                
        //         if(rhg == 12){
        //             // 记录
        //             mode3_flag = false;
        //             record_point();
        //         }
        //         if (rhg == 13 || mode3_flag){
        //             // 循环发布
        //             mode3_flag = true;
        //             // 陷入循环？？  其实可以不用陷入循环，　只要mode3_flag保证就好了
        //             mode3();
        //         }
                
        //         if (lhg == 10 || rhg == 10){
        //             last_memory(3);
        //             mode3_flag = false;
        //         }

        //         ROS_INFO("mode3 working ..");
        //     }
        //     else if(rhg == 8){   // reset的标志
        //         reset_info();
        //     }
        //     else if(rhg == 11){  // 重新启动
        //         pub_status();
        //         if(this->is_accept){
        //             if(last_info.last_mode == 1){
        //                 mode1_flag = true;
        //                 mode1_step = last_info.last_mode1_step;
        //                 mode1_idx = last_info.last_mode1_idx;
        //             }
        //             else if(last_info.last_mode == 2){
        //                 mode2_flag = true;
        //                 direct = last_info.last_mode2_direct;
        //             }
        //             else if(last_info.last_mode == 3){
        //                 mode3_flag = true;
        //                 record_point_idx = last_info.last_mode3_record;
        //             }
        //             this->is_accept = false;
        //         }
        //     }
        //     else if(lhg == 6 || lhg == 7){
        //         rotate_tool();
        //     }
        //     else{
        //         ROS_INFO("working ..");
        //     }
        // }
        rate.sleep();
    }
}
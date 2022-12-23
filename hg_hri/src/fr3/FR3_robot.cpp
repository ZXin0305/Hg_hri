#include<hg_hri/FR3_robot.h>

void Fr3Robot::init(){

    human_num = 0;

    last_info.last_mode = -1;
    last_info.last_mode1_idx = 0;
    last_info.last_mode1_step = 0;
    last_info.last_mode2_direct = -1;
    last_info.last_mode3_record = 0;
    last_info.last_mode3_idx = 0;

    last_info_update = false;
    last_mode_ = -1;
    mode1_idx = 0;
    mode1_step = 0;
    mode2_direct = -1;
    mode2_flag = false;  // 这个无关紧要

    mode3_idx = 0;

    cout << "fr3 初始化完成 .." << endl;

}

void Fr3Robot::clamp(){
    sleep(delay_time_start);
    srv_move.request.position = 25;
    srv_move.request.speed = 100;
    srv_move.request.acceleration = 50;
    srv_move.request.torque = 1;
    srv_move.request.tolerance = 100;
    srv_move.request.waitFlag = true;
    std::cout<<"进行抓取........"<<std::endl;
    move_to_client.call(srv_move);    // 在这里必须call两次？？？离谱
    move_to_client.call(srv_move);
    sleep(delay_time_end);
}

void Fr3Robot::loose(){
    sleep(delay_time_start);
    srv_move.request.position = 80;
	srv_move.request.speed = 100;
	srv_move.request.acceleration = 50;
	srv_move.request.torque = 1;
	srv_move.request.tolerance = 100;
	srv_move.request.waitFlag = true;
    std::cout<<"放下.........."<<std::endl;
	move_to_client.call(srv_move);
    move_to_client.call(srv_move);
    sleep(delay_time_end);
}

void Fr3Robot::jointStateCallback(const sensor_msgs::JointState& joint_state){
    joint_angles.clear();
    joint_speed.clear();
    vector<string> joint_names_recv = joint_state.name;

    for(auto it = joint_names.begin(); it != joint_names.end(); ++it){
        for (auto it_recv = joint_names_recv.begin(); it_recv != joint_names.end(); ++it_recv){
            if(*it_recv == *it){
                int idx = it_recv - joint_names_recv.begin();
                // int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);    //关节角度 ，其微分就是角速度 ， 即下面的joint_speed
                joint_speed.push_back(joint_state.velocity[idx]);     //关节角速度
                break;
            }
        }
    }
    
    JointState_available = true;
    // cout << "joint state: " << JointState_available << endl;
}


void Fr3Robot::sendCmd(TRAC_IK::TRAC_IK &ik_solver, tf::StampedTransform& tf_point){
    cout << tf_point.getOrigin().getX() << " " << 
            tf_point.getOrigin().getY() << " " << 
            tf_point.getOrigin().getZ() << " " << endl;
    transformTFToKDL(tf_point, desired_pose);
    if (JointState_available){
        
        for (int i = 0; i < JOINT_NUM; ++i){
            q(i) = joint_angles[i];
            cout << joint_angles[i] << endl;
        }
        if (ik_solver.CartToJnt(q, desired_pose, q_desired)){
            
            speed_vector.clear();
            desired_goal.clear();    // 用来存储转换后的值
            speed_goal.clear();

            for (int j = 0; j < JOINT_NUM; ++j){
                double delta = q_desired(j) - q(j);
                double speed = Kp * delta + Td * joint_speed[j];
                
                // 检查速度
                if(speed > max_speed){
                    speed = max_speed;
                }
                if(speed < -max_speed){
                    speed = -max_speed;
                }

                speed_goal.push_back(speed * 0.5);
                desired_goal.push_back(q_desired(j));
            }
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.header.stamp = ros::Time::now();
            goal.trajectory.joint_names = joint_names;
            // cout << joint_names[0] << " " << 
            //         joint_names[1] << " " <<
            //         joint_names[2] << " " <<
            //         joint_names[3] << " " <<
            //         joint_names[4] << " " <<
            //         joint_names[5] << " " << endl;
 
            goal.trajectory.points.resize(1);    // 一次只走一个点
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
            for (int j = 0; j < 6; ++j){
                goal.trajectory.points[0].velocities[j] = 0.0;  //　这里设置为0也没有事情
            }

            cout << goal.trajectory.points[0].velocities[0] << " " <<
                    goal.trajectory.points[0].velocities[1] << " " <<
                    goal.trajectory.points[0].velocities[2] << " " <<
                    goal.trajectory.points[0].velocities[3] << " " <<
                    goal.trajectory.points[0].velocities[4] << " " <<
                    goal.trajectory.points[0].velocities[5] << " " << endl;
            
            /* ===================================================  */
            // 相当于调整速度．．．
            double dis_every_second = fixed_dis / speed_time[speed_idx];
            cout << "dis_every_second: " << dis_every_second << endl;
            //　计算距离, 这个时候的tf_point就是新的点了
            trajectory_dis = cal_distance(tf_point, base_tool_dis);
            cout << "trajectory_dis: " << trajectory_dis << endl;
            double execute_time = trajectory_dis / dis_every_second * 100.0;  //换算成cm单位
            if (common_case == "rotate_left" || common_case == "rotate_right")
                execute_time = 10.0;
            else if (execute_time <= 0.0)
                execute_time = 5.0;
            cout << "execute time: " << execute_time << endl;
            /* ===================================================  */
            // if (client_ptr == nullptr)
            //     cout << "========" << endl;
            // else{
            //     cout << "^^^^^^^^" << endl;
            //     cout << goal.trajectory.points[0].positions.size() << endl;
            // }

            
            
            goal.trajectory.points[0].time_from_start = ros::Duration(execute_time);
            client_ptr->sendGoal(goal);   // 发送指令
            goal.trajectory.points.clear();
        }
    }
}


double Fr3Robot::cal_distance(tf::StampedTransform& tf_point1, tf::StampedTransform& tf_point2){
    tf::Vector3 base_point_vector;  //机器人base到需要测量的点的距离向量
    tf::Vector3 base_tool_vector;

    base_point_vector = tf_point1.getOrigin();
    base_tool_vector = tf_point2.getOrigin();

    tf::Vector3 robot_point_vec = base_point_vector - base_tool_vector;
    double distance = robot_point_vec.length();
    return distance;
}

void Fr3Robot::listen_robot_point(string robot_joint_name){
    while(ros::ok()){
        try{
            this->tf_listener.lookupTransform(base_name, robot_joint_name, ros::Time(0), base_tool_dis);  // base --> tool0
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

void Fr3Robot::listener_mode_point(){
    while(ros::ok()){
        try{
            this->tf_listener.lookupTransform(base_name, rotate_name, ros::Time(0), rotate_point);  // base --> tool0
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

void Fr3Robot::decode_human_info(){
    // 取出第一个人的，如果第一个操作者的hg没有(左右同时没有)，那么就依次在后面找
    // 注意这里和point那边不一样，这里的human_hg_vec还有非操作人员的信息,因此需要进行判断
    lhg = -1;
    rhg = -1;  // 左　右
    for (auto xx : human_hg_vec){
        if(xx[1]){   // 是操作人员
            if(xx[2] != -1 || xx[3] != -1){
                lhg = xx[2];
                rhg = xx[3];
                break;
            }
        }
    }
}

// 订阅人员信息
void Fr3Robot::humanIdHgCallback(const std_msgs::Int32MultiArray& id_hg_array){
    this->tmp_id_hg.clear();
    this->human_hg_vec.clear();
    tmp_id_hg = id_hg_array.data;
    human_num = tmp_id_hg.size() / info_len;
    // cout << "human number: " << human_num << endl;
    if (human_num == 0){
        return;  // 没有人就直接返回，不会进入下面的了
    }
    else{
        // 这里也只是将得到的人员信息分类了，并没有操作，取出数据是在decode_human_info中
        for (int i = 0; i < human_num; ++i){
            this->human_hg_vec.push_back({
                (int)tmp_id_hg[i * info_len],  // id
                (int)tmp_id_hg[i * info_len + 1],  // is_operator
                (int)tmp_id_hg[i * info_len + 2],  // left hg
                (int)tmp_id_hg[i * info_len + 3]   // right hg
            });
        }
        sort(human_hg_vec.begin(), human_hg_vec.end(), 
             [](vector<int>& v1, vector<int>& v2){return v1[0] < v2[0];});

        // 这样会把所有人的关节点的信息都放在了一个vector中
        string human_joint_name;
        tf::StampedTransform tmp;
        while(ros::ok()){
            all_human_joint_tf.clear();
            try{
                for (int i = 0; i < human_num; ++i){
                    int human_id = human_hg_vec[i][0];  // id, 应该是从0开始的
                    human_joint_name = to_string(human_id) + "_root";
                    this->tf_listener.lookupTransform(base_name, human_joint_name, ros::Time(0), tmp);
                    this->all_human_joint_tf.push_back(tmp);

                    human_joint_name = to_string(human_id) + "_lelbow";
                    this->tf_listener.lookupTransform(base_name, human_joint_name, ros::Time(0), tmp);
                    this->all_human_joint_tf.push_back(tmp);

                    human_joint_name = to_string(human_id) + "_lwrist";
                    this->tf_listener.lookupTransform(base_name, human_joint_name, ros::Time(0), tmp);
                    this->all_human_joint_tf.push_back(tmp);

                    human_joint_name = to_string(human_id) + "_relbow";
                    this->tf_listener.lookupTransform(base_name, human_joint_name, ros::Time(0), tmp);
                    this->all_human_joint_tf.push_back(tmp);

                    human_joint_name = to_string(human_id) + "_rwrist";
                    this->tf_listener.lookupTransform(base_name, human_joint_name, ros::Time(0), tmp);
                    this->all_human_joint_tf.push_back(tmp);
                }
                // ROS_INFO("received human info ..");
                break;
            }
            catch(tf::TransformException &ex){
                // ROS_INFO("wait for human info ..");
                continue;
            }
        }
    }
}

void Fr3Robot::sort_distance(){
    if (human_num <= 0){
        closest_dis = 100.0;
        return;
    }
    this->distance_vec.clear();
    this->listen_robot_point(tip_name);
    for (int i = 0; i < this->human_num; ++i){   // 第一层是人数
        // 如果想要只对非操作人员进行避障就可以用  ?? 这里面什么意思??
        // if(this->[human_hg_vec][i][1])
        //     continue;
        for(int j = 0; j < this->tf_joint_num; ++j){
            double dis = this->cal_distance(this->all_human_joint_tf[i * tf_joint_num + j], base_tool_dis);
            distance_vec.push_back(dis);
        }
    }
    sort(distance_vec.begin(), distance_vec.end());  //从小到大
    this->closest_dis = distance_vec[0];  // 之前这里就会爆出段错误，因为如果由于刚开始没有人，distance_vec就是空的，那么就无法使用指针了．
}

bool Fr3Robot::stop_robot(){
    sort_distance();
    if (closest_dis <= stop_thres)
        return true;
    else
        return false;
}

void Fr3Robot::listener_object(){
    while(ros::ok()){
        this->obj_list.clear();
        /*
        之前是将所有的点发布出来，然后这里接收，用vector保存，现在这里只用一次监听一个就好了
        */
       try{
           for (int i = 0; i < obj_num; ++i){
                this->tf_listener.lookupTransform(this->base_name, "obj_correct_" + to_string(i), ros::Time(0), obj_correct);
                this->tf_listener.lookupTransform(this->base_name, "obj_target_" + to_string(i), ros::Time(0), obj_target);
                obj_list.push_back(obj_correct);
                obj_list.push_back(obj_target);
           }
           break;
       }
       catch(tf::TransformException& ex){
           continue;
       }
    }
}

void Fr3Robot::listener_point(){
    while(ros::ok()){
        this->cap_init.clear();
        try{
            tf_listener.lookupTransform(this->base_name, "cap_point", ros::Time(0), cap_point);   // 0
            cap_init.push_back(cap_point);
            tf_listener.lookupTransform(this->base_name, "init_point", ros::Time(0), init_point); //1
            cap_init.push_back(init_point);
            tf_listener.lookupTransform(this->base_name, "mid_point", ros::Time(0), mid_point);   //2
            cap_init.push_back(mid_point);
            tf_listener.lookupTransform(this->base_name, "assist_point", ros::Time(0), assist_point); //3
            cap_init.push_back(assist_point);
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

void Fr3Robot::reset(){
    tmp_id_hg.clear();
    human_hg_vec.clear();

    human_num = 0;
    lhg = -1;
    rhg = -1;

    all_human_joint_tf.clear();
    mode1_idx= 0;
    mode1_step = 0;
    mode1_finish = 0;
    mode1_flag = false;

    mode2_direct = -1;
    mode2_flag = false;

    mode3_idx = 0;
    waypoints.clear();
    mode3_flag = false;
    
    last_info.last_mode = -1;
    last_info.last_mode1_idx = 0;
    last_info.last_mode1_step = 0;
    last_info.last_mode2_direct = -1;
    last_info.last_mode3_record = 0;
    last_info.last_mode3_idx = 0;
    last_info_update = false;
    last_mode_ = -1;

    speed_idx = 0;  // 速度
    common_case = "";
}

// bool Fr3Robot::have_arrived(){
//     return client_ptr.getState().isDone();
// }

// 停止机器人的运动，统一放在了这个函数中
// 包括上次信息的保留
// 最后没有用了．．．
void Fr3Robot::judge_status(int mode_idx, tf::StampedTransform tf_point){
}

// mode1:: 定点抓取
void Fr3Robot::reach_point(string current_case){
    TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
    ros::Rate rate(60);
    tf::StampedTransform tf_point;
    string case_tmp = current_case;
    common_case = current_case;
    cout << "current case: " << common_case << endl;

    while(ros::ok()){
        ros::spinOnce();
        
        decode_human_info();
        if (case_tmp == "cap_point"){
            this->listener_point();
            this->listen_robot_point(tip_name);
            tf_point = this->cap_init[0];
        }
        else if(case_tmp == "init_point"){
            this->listener_point();
            this->listen_robot_point(tip_name);
            tf_point = this->cap_init[1];
        }
        else if (case_tmp == "mid_point"){    //这个可能不需要了
            this->listener_point();
            this->listen_robot_point(tip_name);
            tf_point = this->cap_init[2];
        }
        else if(case_tmp == "assist_point"){
            this->listener_point();
            this->listen_robot_point(tip_name);
            tf_point = this->cap_init[3];
        }
        else if(case_tmp == "obj_correct"){
            this->listener_object();
            this->listen_robot_point(tip_name);
            tf_point = this->obj_list[mode1_idx * 2 + 0];
        }
        else if (case_tmp == "obj_target"){
            this->listener_object();
            this->listen_robot_point(tip_name);
            tf_point = this->obj_list[mode1_idx * 2 + 1];
        }
        this->sendCmd(ik_solver, tf_point);
        // judge_status(1, tf_point);

        // 判断
        if (lhg == 10 || rhg == 10){
            clock_t st = clock();
            decode_human_info();
            while(ros::ok()){
                client_ptr->cancelAllGoals();  // 停止
                clock_t et = clock();
                if ((double)(et - st) >= 0.1){
                    break;
                }
            }
            last_info.last_mode1_idx = mode1_idx;   // ???? 记录停止时物块的ID
            last_info.last_mode = 1;                // 记录停止时模式信息
            last_info.last_mode1_step = mode1_step; // 记录当前的步骤
            break;  // 直接break掉
        }
        // 根据距离来判断是否停止
        // if(stop_robot()){
        //     while(stop_robot() && ros::ok()){
        //         ros::spinOnce();
        //         ROS_INFO("waiting ...");
        //     }
        // }

        /*
        这个有问题，，，　在main函数中就没问题，不知道为什么？？？
        */
        // cout << client_ptr->getState().isDone() << endl;
        // if (client_ptr->getState().isDone()){
        //     ROS_INFO("have finished ..");
        //     break;   // 达到目标点
        // }

        // 这个也可以用,就是简单判定是否达到目标点，退出这个循环
        if (cal_distance(tf_point, base_tool_dis) <= stop_thres){
            ROS_INFO("have finished ..");
            break;
        }
        rate.sleep();
    }
}

// mode2:: 旋转/平移
void Fr3Robot::move_direction(string current_case){
    TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
    ros::Rate rate(60);
    tf::StampedTransform tf_point, rotate_tmp;
    string case_tmp = current_case;
    common_case = current_case;
    cout << "current case: " << common_case << endl;
    bool rotate_flag = false;
    if (case_tmp == "left" ){
        this->listen_robot_point(tip_name);
        tf_point.setOrigin(tf::Vector3(x_max,
                                    this->base_tool_dis.getOrigin().y(),
                                    this->base_tool_dis.getOrigin().z()));
        tf_point.setRotation(tf::Quaternion(this->base_tool_dis.getRotation().x(),
                                    this->base_tool_dis.getRotation().y(),
                                    this->base_tool_dis.getRotation().z(),
                                    this->base_tool_dis.getRotation().w()));
    }
    else if (case_tmp == "right"){
        this->listen_robot_point(tip_name);
        tf_point.setOrigin(tf::Vector3(x_min,
                                    this->base_tool_dis.getOrigin().y(),
                                    this->base_tool_dis.getOrigin().z())); 
        tf_point.setRotation(tf::Quaternion(this->base_tool_dis.getRotation().x(),
                                    this->base_tool_dis.getRotation().y(),
                                    this->base_tool_dis.getRotation().z(),
                                    this->base_tool_dis.getRotation().w()));   
    }
    else if (case_tmp == "up"){
        this->listen_robot_point(tip_name);
        tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
                                    this->base_tool_dis.getOrigin().y(),
                                    z_max));   
        tf_point.setRotation(tf::Quaternion(this->base_tool_dis.getRotation().x(),
                                    this->base_tool_dis.getRotation().y(),
                                    this->base_tool_dis.getRotation().z(),
                                    this->base_tool_dis.getRotation().w()));
    }
    else if (case_tmp == "down"){
        this->listen_robot_point(tip_name);
        tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
                                    this->base_tool_dis.getOrigin().y(),
                                    z_min));
        tf_point.setRotation(tf::Quaternion(this->base_tool_dis.getRotation().x(),
                                    this->base_tool_dis.getRotation().y(),
                                    this->base_tool_dis.getRotation().z(),
                                    this->base_tool_dis.getRotation().w()));
    }
    else if (case_tmp == "forward"){
        this->listen_robot_point(tip_name);
        tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
                                    y_max,
                                    this->base_tool_dis.getOrigin().z()));
        tf_point.setRotation(tf::Quaternion(this->base_tool_dis.getRotation().x(),
                                    this->base_tool_dis.getRotation().y(),
                                    this->base_tool_dis.getRotation().z(),
                                    this->base_tool_dis.getRotation().w()));
    }
    else if (case_tmp == "backward"){
        this->listen_robot_point(tip_name);
        tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
                                    y_min,
                                    this->base_tool_dis.getOrigin().z()));
        tf_point.setRotation(tf::Quaternion(this->base_tool_dis.getRotation().x(),
                                    this->base_tool_dis.getRotation().y(),
                                    this->base_tool_dis.getRotation().z(),
                                    this->base_tool_dis.getRotation().w()));
    }
    else if(case_tmp == "rotate_left" || case_tmp == "rotate_right"){
        this->listener_mode_point();
        tf_point = rotate_point;
    }
    rotate_tmp = tf_point;
    sendCmd(ik_solver, tf_point);

    while(ros::ok()){
        ros::spinOnce();
        decode_human_info();
        this->listen_robot_point(tip_name);
        cout << "motion .. " << endl;
        if (lhg == 10 || rhg == 10){
            clock_t st = clock();
            while(ros::ok()){
                client_ptr->cancelGoal();  // 停止
                clock_t et = clock();
                if ((double)(et - st) >= 0.1){
                    break;
                }
                rate.sleep();
            }
            last_info.last_mode2_direct = mode2_direct;
            last_info.last_mode = 2;
            break;  // 直接break掉
        }
        if (case_tmp != "rotate_left" && case_tmp != "rotate_right"){
            if (cal_distance(rotate_tmp, base_tool_dis) <= stop_thres){
                ROS_INFO("have finished ..");
                break;
            }
        }
        else {
            this->listen_robot_point(tip_name);
            if (abs(base_tool_dis.getRotation().x() - tf_point.getRotation().x()) <= (double)0.01 && 
                abs(base_tool_dis.getRotation().y() - tf_point.getRotation().y()) <= (double)0.01 &&
                abs(base_tool_dis.getRotation().z() - tf_point.getRotation().z()) <= (double)0.01 &&
                abs(base_tool_dis.getRotation().w() - tf_point.getRotation().w()) <= (double)0.01){
                    ROS_INFO("have finished ..");
                    break;
                }
        }
    }


    // while(ros::ok()){
    //     ros::spinOnce();
    //     decode_human_info();
    //     if (case_tmp == "left" ){
    //         this->listen_robot_point(tip_name);
    //         tf_point.setOrigin(tf::Vector3(x_max,
    //                                     this->base_tool_dis.getOrigin().y(),
    //                                     this->base_tool_dis.getOrigin().z()));
    //     }
    //     else if (case_tmp == "right"){
    //         this->listen_robot_point(tip_name);
    //         tf_point.setOrigin(tf::Vector3(x_min,
    //                                     this->base_tool_dis.getOrigin().y(),
    //                                     this->base_tool_dis.getOrigin().z()));    
    //     }
    //     else if (case_tmp == "up"){
    //         this->listen_robot_point(tip_name);
    //         tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
    //                                     this->base_tool_dis.getOrigin().y(),
    //                                     z_max));   
    //     }
    //     else if (case_tmp == "down"){
    //         this->listen_robot_point(tip_name);
    //         tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
    //                                     this->base_tool_dis.getOrigin().y(),
    //                                     z_min)); 
    //     }
    //     else if (case_tmp == "forward"){
    //         this->listen_robot_point(tip_name);
    //         tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
    //                                     y_max,
    //                                     this->base_tool_dis.getOrigin().z()));
    //     }
    //     else if (case_tmp == "backward"){
    //         this->listen_robot_point(tip_name);
    //         tf_point.setOrigin(tf::Vector3(this->base_tool_dis.getOrigin().x(),
    //                                     y_min,
    //                                     this->base_tool_dis.getOrigin().z()));
    //     }
    //     else if(case_tmp == "rotate_left" || case_tmp == "rotate_right"){
    //         if (!rotate_flag){
    //             this->listener_mode_point();
    //             tf_point = rotate_point;
    //             rotate_tmp = rotate_point;
    //             rotate_flag = true;
    //         }
    //         else if (rotate_flag){
    //             tf_point = rotate_tmp;
    //         }
    //     }

    //     if (case_tmp != "rotate_left" && case_tmp != "rotate_right"){
    //         tf_point.setRotation(tf::Quaternion(this->base_tool_dis.getRotation().x(),
    //                                     this->base_tool_dis.getRotation().y(),
    //                                     this->base_tool_dis.getRotation().z(),
    //                                     this->base_tool_dis.getRotation().w()));
    //     }


    //     this->sendCmd(ik_solver, tf_point);

    //     // 判断
    //     if (lhg == 10 || rhg == 10){
    //         clock_t st = clock();
    //         while(ros::ok()){
    //             client_ptr->cancelAllGoals();  // 停止
    //             clock_t et = clock();
    //             if ((double)(et - st) >= 0.1){
    //                 break;
    //             }
    //             rate.sleep();
    //         }
    //         last_info.last_mode2_direct = mode2_direct;
    //         last_info.last_mode = 2;
    //         break;  // 直接break掉
    //     }
    //     // // 根据距离来判断是否停止
    //     // if(stop_robot()){
    //     //     client_ptr->cancelGoal();   // 不知道管不管用
    //     //     while(stop_robot() && ros::ok()){
    //     //         ros::spinOnce();
    //     //         ROS_INFO("waiting ...");
    //     //     }
    //     // }

    //     if (case_tmp != "rotate_left" && case_tmp != "rotate_right"){
    //         if (cal_distance(tf_point, base_tool_dis) <= stop_thres){
    //             ROS_INFO("have finished ..");
    //             break;
    //         }
    //     }
    //     else {
    //         // cout << tf_point.getRotation().x() << " " << 
    //         //         tf_point.getRotation().y() << " " << 
    //         //         tf_point.getRotation().z() << " " << 
    //         //         tf_point.getRotation().w() << " " << endl;
    //         this->listen_robot_point(tip_name);

    //         if (abs(base_tool_dis.getRotation().x() - tf_point.getRotation().x()) <= (double)0.01 && 
    //             abs(base_tool_dis.getRotation().y() - tf_point.getRotation().y()) <= (double)0.01 &&
    //             abs(base_tool_dis.getRotation().z() - tf_point.getRotation().z()) <= (double)0.01 &&
    //             abs(base_tool_dis.getRotation().w() - tf_point.getRotation().w()) <= (double)0.01){
    //                 ROS_INFO("have finished ..");
    //                 break;
    //             }
    //     }
    // }
}


// mode3:: 轨迹复现
void Fr3Robot::record_waypoints(){
    this->listen_robot_point(tip_name);
    if (waypoints.size() != 0){
        if (cal_distance(base_tool_dis, waypoints[waypoints.size() - 1]) <= 0.02)
            ROS_INFO("cannot store the waypoint ..");
        else{
            waypoints.push_back(this->base_tool_dis);  // 将末端的信息保存进去
            ROS_INFO("have store the waypoint ..");
        }
    }
    else if (waypoints.size() == 0){
        waypoints.push_back(this->base_tool_dis);  // 将末端的信息保存进去
        ROS_INFO("have store the waypoint ..");
    }
}

void Fr3Robot::display_waypoints(){
    int waypoint_num = waypoints.size();
    if (waypoint_num < 2){
        ROS_INFO("cannot display trajectory, add more points ..");
    }
    else{
        ros::Rate rate(60);
        TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
        bool stop_flag = false;
        tf::StampedTransform tf_point, point_tmp;
        while(ros::ok()){

            tf_point = waypoints[mode3_idx];  // 这个会实时更新的,
            point_tmp = tf_point;
            this->sendCmd(ik_solver, tf_point);
            while(ros::ok()){
                ros::spinOnce();
                decode_human_info();
                this->listen_robot_point(tip_name);
                // this->sendCmd(ik_solver, tf_point);
                // 判断
                if (lhg == 10 || rhg == 10){
                    clock_t st = clock();
                    while(ros::ok()){
                        client_ptr->cancelGoal();  // 停止
                        clock_t et = clock();
                        if ((double)(et - st) >= 0.1){
                            break;
                        }
                        rate.sleep();
                    }
                    last_info.last_mode3_idx = mode3_idx;
                    last_info.last_mode = 3;
                    stop_flag = true;
                    break;  // 直接break掉
                }
                // 根据距离来判断是否停止
                // if(stop_robot()){
                //     while(stop_robot() && ros::ok()){
                //         ros::spinOnce();
                //         ROS_INFO("waiting ...");
                //     }
                // } 
                if (cal_distance(point_tmp, base_tool_dis) <= stop_thres){
                    ROS_INFO("have finished ..");
                    break;
                }
                rate.sleep();
            }
            mode3_idx = (mode3_idx + 1) % waypoint_num;

            if (stop_flag){   //退出mode3
                stop_flag = false;
                ROS_INFO("mode3 quit ..");
                break;
            }
        }
    }
}

// 更新速度
void Fr3Robot::adjust_speed(string current_case){
    if (current_case == "speed_up"){
        if (speed_idx + 1 < speed_time.size())
            speed_idx += 1;
    }
    else if (current_case == "speed_down"){
        if (speed_idx - 1 >= 0)
            speed_idx -= 1;
    }
}

void Fr3Robot::build_connection(){   // 这个函数没有用
    // client_ptr->waitForServer();
    cout << mode1_idx << endl;
    // last_info.last_mode = mode1_idx;
    ROS_INFO("have connect to server ..");
}

void Fr3Robot::stop_motion(){
    cout << lhg << "  " << rhg << endl;
    clock_t st = clock();
    
    while(ros::ok()){
        ros::spinOnce();
        decode_human_info();
        client_ptr->cancelAllGoals();  // 停止
        clock_t et = clock();
        if ((double)(et - st) >= 0.05){
            break;
        }
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "main_step");
    ros::NodeHandle nh;
    Client client(trajectory_action_name);
    client_ptr = &client;
    client_ptr->waitForServer();
    ROS_INFO("have connected ..");
    Fr3Robot fr3;

    ros::spinOnce();
    ros::Rate rate(60);

    // 测试用
    // fr3.decode_human_info();
    // fr3.clamp();
    // fr3.loose();
    // fr3.reach_point("cap_point");
    // fr3.move_direction("down");  //rotate_left
    // test in 20221210
    // fr3.mode1_idx = 10;
    // fr3.build_connection();
    // cout << fr3.last_info.last_mode << " " << fr3.last_info.last_mode1_idx << "  " << fr3.last_info_update << endl;


/* =================================================================================================== */
/* =================================================================================================== */
/* =================================================================================================== */
/* =================================================================================================== */
// 主要程序
// 初始化
    // fr3.clamp();
    fr3.loose();
    sleep(0.2);
    fr3.reach_point("init_point");
    ROS_INFO("robot is ready ..");

    while(ros::ok()){
        ros::spinOnce();
        fr3.decode_human_info();
        int current_lhg = fr3.lhg;
        int current_rhg = fr3.rhg;

        if (current_lhg == 10 || current_rhg == 10)
        {
            fr3.stop_motion();
        }
        else if ((fr3.last_info_update && fr3.last_mode_ == 2) || (current_rhg >= 1 && current_rhg <= 6)){  //平移 mode2  // right hand
            if (fr3.last_info_update){
                current_rhg = fr3.mode2_direct;  // 通过这里，利用数字选择相应的string
                fr3.last_info_update = false;
            }
            else{
                fr3.mode2_direct = (int)current_rhg;  //这里需要记录一下，以便停止的时候能够保存mode2的数据
            }
            switch (current_rhg)
            {
            case 1:
                fr3.move_direction("up");
                break;
            case 2:
                fr3.move_direction("down");
                break;
            case 3:
                fr3.move_direction("forward");
                break;
            case 4:
                fr3.move_direction("backward");
                break;
            case 5:
                fr3.move_direction("left");
                break;
            case 6:
                fr3.move_direction("right");
                break;
            default:
                break;
            }
        }
        else if (current_rhg == 7){   // 回到原点
            fr3.reach_point("init_point");
            sleep(0.2);
            fr3.loose();
        }
        else if (current_rhg == 15){   // 重置fr3中的一些信息
            fr3.reset();
        }
        else if (current_rhg == 9){   // 到达协助点
            fr3.reach_point("assist_point");
        }
        else if (current_rhg == 11){  // 恢复上一次的运动  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            fr3.last_mode_ = fr3.last_info.last_mode;
            if (fr3.last_mode_ == 1){
                fr3.mode1_step = fr3.last_info.last_mode1_step;   // 上一次运行到抓取物块的哪一步了
                fr3.mode1_idx = fr3.last_info.last_mode1_idx;     // 上一次抓取的物块的idx
            }
            else if (fr3.last_mode_ == 2){
                fr3.mode2_direct = fr3.last_info.last_mode2_direct;  // 上一次mode2的方向
            }
            else if (fr3.last_mode_ == 3){
                fr3.mode3_idx = fr3.last_info.last_mode3_idx;  // 轨迹复现中的步骤
            }
            fr3.last_info_update = true;   //说明已经重启
        }
        else if (current_rhg == 12){  // 记录路径点
            fr3.record_waypoints();
        }
        else if ((fr3.last_info_update && fr3.last_mode_ == 3) || current_rhg == 13){  //　重现路径点  mode3
            if (fr3.last_info_update)
                fr3.last_info_update = false;
            fr3.display_waypoints();
        }
        else if (current_rhg == 14){  // 爪子闭合
            fr3.clamp();
        }
        else if (current_rhg == 8){  // 爪子张开
            sleep(3.0);
            fr3.loose();
        }

        // left hand
        if ((fr3.last_info_update && fr3.last_mode_ == 1) || (current_lhg >= 1 && current_lhg <= 3)){   // mode1
            if (!fr3.last_info_update)
                fr3.mode1_idx = current_lhg - 1;  // 如果不是重启，那么就按照当前的物块idx
            else 
                fr3.last_info_update = false;
            bool jump_loop = false;
            bool mode1_finish_flag = false;
            while (ros::ok()){
                ros::spinOnce();
                fr3.decode_human_info();
                switch (fr3.mode1_step)
                {
                case 0:
                    /* code */
                    fr3.reach_point("init_point");
                    if (fr3.rhg != 10 || fr3.lhg != 10){
                        sleep(0.2);
                        fr3.loose();
                        fr3.mode1_step = (fr3.mode1_step + 1) % fr3.mode1_process;  // 1
                    }
                    else {
                        jump_loop = true;
                    }
                    break;
                case 1:
                    fr3.reach_point("cap_point");
                    if (fr3.rhg != 10 || fr3.rhg != 10){
                        fr3.mode1_step = (fr3.mode1_step + 1) % fr3.mode1_process;  // 2
                    }
                    else 
                        jump_loop = true;
                    break;
                case 2:
                    fr3.reach_point("obj_correct");
                    if (fr3.rhg != 10 || fr3.rhg != 10){
                        sleep(0.1);
                        fr3.mode1_step = (fr3.mode1_step + 1) % fr3.mode1_process;  // 3
                    }
                    else 
                        jump_loop = true;
                    break;
                case 3:
                    fr3.reach_point("obj_target");
                    if (fr3.rhg != 10 || fr3.rhg != 10){
                        sleep(0.1);
                        fr3.clamp();
                        sleep(0.2);
                        fr3.mode1_step = (fr3.mode1_step + 1) % fr3.mode1_process;  // 4
                    }
                    else 
                        jump_loop = true;
                    break;
                case 4:
                    fr3.reach_point("obj_correct");
                    if (fr3.rhg != 10 || fr3.rhg != 10){
                        fr3.mode1_step = (fr3.mode1_step + 1) % fr3.mode1_process;  // 5
                    }
                    else 
                        jump_loop = true;
                    break;
                case 5:
                    fr3.reach_point("cap_point");
                    if (fr3.rhg != 10 || fr3.rhg != 10){
                        fr3.mode1_step = (fr3.mode1_step + 1) % fr3.mode1_process;  // 6
                    }
                    else 
                        jump_loop = true;
                    break;
                case 6:
                    mode1_finish_flag = true;
                    fr3.reach_point("init_point");
                    if (fr3.rhg != 10 || fr3.rhg != 10){
                        fr3.mode1_step = (fr3.mode1_step + 1) % fr3.mode1_process;   // 也不用
                    }
                    else 
                        jump_loop = true;
                    break;
                default:
                    break;
                }
                if (mode1_finish_flag || jump_loop){
                    mode1_finish_flag = false;
                    jump_loop = false;
                    break;
                }
                rate.sleep();
            }
        }
        else if (current_lhg == 6 || current_lhg == 7){
            switch (current_lhg)
            {
            case 6:
                fr3.move_direction("rotate_right");  // 顺时针
                break;
            case 7:
                fr3.move_direction("rotate_left");   // 逆时针
                break;
            default:
                break;
            }
        }
        else if (current_lhg == 8 || current_lhg == 9){
            switch (current_lhg)
            {
            case 8:
                fr3.adjust_speed("speed_down");
                break;
            case 9:
                fr3.adjust_speed("speed_up");
                break;
            default:
                break;
            }
        }
        rate.sleep();
    }

/* =================================================================================================== */
/* =================================================================================================== */
/* =================================================================================================== */
/* =================================================================================================== */

    return 0;

}
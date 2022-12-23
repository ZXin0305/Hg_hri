#include<hg_hri/Robot.h>

void Robot::init(){
    // 关节名称
    joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");
    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");

    this->kD = pow((max_thres * min_thres), 2) / pow((max_thres - min_thres), 2);

    human_num = 0;

    last_info.last_mode = -1;
    last_info.last_mode1_idx = 0;
    last_info.last_mode1_step = 0;
    last_info.last_mode2_direct = -1;
    last_info.last_mode3_record = 0;
}


void Robot::jointStateCallback(const sensor_msgs::JointState& joint_state){
    joint_angles.clear();
    joint_speed.clear();
    vector<string> joint_names_recv = joint_state.name;   //关节名称  .name是一个列表

    for(auto it = joint_names.begin(); it != joint_names.end(); ++it){
        for (auto it_recv = joint_names_recv.begin(); it_recv != joint_names.end(); ++it_recv){
            if(*it_recv == *it){
                int idx = it_recv - joint_names_recv.begin();
                int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);    //关节角度 ，其微分就是角速度 ， 即下面的joint_speed
                joint_speed.push_back(joint_state.velocity[idx]);     //关节角速度
                break;
            }
        }
    }

    JointState_available = true;
}

void Robot::sendCmd(TRAC_IK::TRAC_IK &ik_solver, tf::StampedTransform& tf_point){
    transformTFToKDL(tf_point, desired_pose);

    if(JointState_available){
        for(int i = 0; i < JOINT_NUM; ++i){
            q(i) = joint_angles[i];
            printf("q_current[%d]: %f \n", i, joint_angles[i]);
        }

        if(ik_solver.CartToJnt(q, desired_pose, q_desired)){
            std::vector<double> speed_vector;

            for(int j = 0; j < JOINT_NUM; ++j){
                double delta = q_desired(j) - q(j);    //误差（q_desired:系统输入，q：上一时刻的“输出”）
                double speed = Kp * delta + Td * joint_speed[j];  // PD控制--比例环节:当前偏差，微分环节：最近偏差

                // 检查速度
                if(speed > max_speed){
                    speed = max_speed;
                }
                if(speed < -max_speed){
                    speed = -max_speed;
                }

                speed_vector.push_back(speed);
                printf("speed[%d]:%f\n",j,speed);   //速度控制
            }

            // 调整速度
            this->adjust_speed(speed_vector);

            char command[100];  // command
            sprintf(command, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0, 0.05)",
                              speed_vector[0], speed_vector[1], speed_vector[2],
                              speed_vector[3], speed_vector[4], speed_vector[5]);
            command_msg.data = command;
            ur_command_publisher.publish(command_msg);
        }
    }
}

void Robot::sort_distance(){
    this->distance_vec.clear();

    for(int i = 0; i < this->human_num; ++i){  // 第一层是人数
        // 如果想要只对非操作人员进行避障就可以用
        // if(this->[human_hg_vec][i][1])
        //     continue;
        for(int j = 0; j < this->tf_joint_num; ++j){
            double dis = this->cal_distance(this->all_human_joint_tf[i * tf_joint_num + j], joint_name_controller);
            distance_vec.push_back(dis);
        }
    }

    sort(distance_vec.begin(), distance_vec.end());
    this->closest_dis = distance_vec[0];
}

void Robot::adjust_speed(vector<double>& speed_vector){   //根据手腕和末端的距离进行速度的调整

    if(this->human_num > 0){
        // 最近的点
        this->sort_distance();
        double closest_dis_tmp = this->closest_dis;  // 最近的距离
        if(this->lhg == 9)   // 增加速度
            speed_factor = 1.5;
        else if(this->lhg == 8)
            speed_factor = 1.0;


        if(this->lhg == 10 && this->rhg == 10)  // 直接停止
            speed_factor = 0;
        // 调整速度
        if(closest_dis_tmp > max_thres)
            return;
        else if(closest_dis_tmp <= max_thres && closest_dis_tmp > min_thres){
            double tmp = pow((1/closest_dis_tmp - 1/max_thres), 2);
            speed_factor = exp((-1) * (kD * tmp));
        }
        else if(closest_dis_tmp >= 0 && closest_dis_tmp <= min_thres){
            speed_factor = 0;
        }

        for(int i = 0; i < JOINT_NUM; ++i){
            speed_vector[i] *= speed_factor;
            if(speed_vector[i] > max_speed)
                speed_vector[i] = max_speed;
            else if(speed_vector[i] < -max_speed)
                speed_vector[i] = -max_speed;
        }
    }

    return;
}

void Robot::listener_object(){
    while(ros::ok()){
        this->obj_list.clear();
        /*
        之前是将所有的点发布出来，然后这里接收，用vector保存，现在这里只用一次监听一个就好了
        */
        try{
            this->tf_listener.lookupTransform("base", "obj_correct", ros::Time(0), obj_correct);
            this->tf_listener.lookupTransform("base", "obj_target", ros::Time(0), obj_target);
            obj_list.push_back(obj_correct);
            obj_list.push_back(obj_target);
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

void Robot::listener_point(){
    while(ros::ok()){
        this->cap_init.clear();
        try{
            this->tf_listener.lookupTransform("base", "cap_point", ros::Time(0), cap_point);  //0
            this->cap_init.push_back(cap_point);
            this->tf_listener.lookupTransform("base", "init_point", ros::Time(0), init_point); //1
            this->cap_init.push_back(init_point);
            this->tf_listener.lookupTransform("base", "mid_point", ros::Time(0), mid_point); //2
            this->cap_init.push_back(mid_point); 
            this->tf_listener.lookupTransform("base", "assist_point", ros::Time(0), assist_point); //3
            this->cap_init.push_back(assist_point);           
            // ROS_INFO("get point..");
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

void Robot::listener_mode_point(string& point_name){
    while(ros::ok()){
        try{
            this->tf_listener.lookupTransform("base", point_name, ros::Time(0), other_mode_point);  //0         
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
}

tf::Vector3 Robot::listener_robot_point(string& robot_joint_name){
    tf::Vector3 tf_vec(0, 0, 0);
    while(ros::ok()){
        try{
            this->tf_listener.lookupTransform("base", robot_joint_name, ros::Time(0), base_tool_dis);  // base --> tool0
            tf_vec = base_tool_dis.getOrigin();
            break;
        }
        catch(tf::TransformException& ex){
            continue;
        }
    }
    return tf_vec;
}


double Robot::cal_distance(tf::StampedTransform& tf_point, string& robot_joint_name){
    tf::Vector3 base_point_vector;  //机器人base到需要测量的点的距离向量
    tf::Vector3 base_tool_vector;

    base_point_vector = tf_point.getOrigin();
    base_tool_vector = this->listener_robot_point(robot_joint_name);

    tf::Vector3 robot_point_vec = base_point_vector - base_tool_vector;
    double distance = robot_point_vec.length();
    return distance;
}


void Robot::decode_human_info(){
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
void Robot::humanIdHgCallback(const std_msgs::Int32MultiArray& id_hg_array){
    this->tmp_id_hg.clear();
    this->human_hg_vec.clear();
    tmp_id_hg = id_hg_array.data;
    human_num = tmp_id_hg.size() / info_len;   // 四个信息

    if(human_num == 0){
        ROS_INFO("here is no people");
        return;
    }
    else{
        // 这里也只是将得到的人员信息分类了，并没有操作，取出数据是在decode_human_info中
        for (int i = 0; i < human_num; ++i){
            this->human_hg_vec.push_back({
                (int)tmp_id_hg[i * info_len],  // id
                (int)tmp_id_hg[i * info_len + 1],  // is_operator
                (int)tmp_id_hg[i * info_len + 2],  // left hg
                (int)tmp_id_hg[i * info_len + 3]  // right hg
            });
        }
        sort(human_hg_vec.begin(), human_hg_vec.end(), 
            [](vector<int>& v1, vector<int>& v2){return v1[0] < v2[0];});
    }

    //这样会把所有人的关节点的信息都放在了一个vector中

    string human_joint_name;
    tf::StampedTransform tmp;

    while(ros::ok()){
        all_human_joint_tf.clear();
        try{
            for(int i = 0; i < human_num; ++i){
                int human_id = human_hg_vec[i][0];  // id, 应该是从0开始的
                human_joint_name = to_string(human_id) + "_root";
                this->tf_listener.lookupTransform("base", human_joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                human_joint_name = to_string(human_id) + "_lelbow";
                this->tf_listener.lookupTransform("base", human_joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                human_joint_name = to_string(human_id) + "_lwrist";
                this->tf_listener.lookupTransform("base", human_joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                human_joint_name = to_string(human_id) + "_relbow";
                this->tf_listener.lookupTransform("base", human_joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);

                human_joint_name = to_string(human_id) + "_rwrist";
                this->tf_listener.lookupTransform("base", human_joint_name, ros::Time(0), tmp);
                this->all_human_joint_tf.push_back(tmp);
            }
            break;
        }
        catch(tf::TransformException &ex){
            ROS_INFO("wait for human info ..");
            continue;
        }
    }

    // this->decode_human_info();   // 解析当前的手势  在需要的时候解析一下
}

bool Robot::judge(tf::StampedTransform& tf_point){
    double length = this->cal_distance(tf_point, joint_name_tool0);
    if(length <= this->arrive_thres)
        return true;
    return false;
}

void Robot::reach_point(string current_case){
    TRAC_IK::TRAC_IK ik_solver(base_name, tip_name, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);
    ros::Rate rate(60);
    tf::StampedTransform tf_point;
    string case_tmp = current_case;

    while(ros::ok()){
        ros::spinOnce();
        decode_human_info();  // 这里是个死循环，需要监听人员的信息
        if(this->human_num != 0){
            if(case_tmp == "cap_point"){
                this->listener_point();
                tf_point = this->cap_init[0];
            }
            else if(case_tmp == "init_point"){
                this->listener_point();
                tf_point = this->cap_init[1];
            }
            else if(case_tmp == "mid_point"){
                this->listener_point();
                tf_point = this->cap_init[2];
            }
            else if(case_tmp == "assist_point"){
                this->listener_point();
                tf_point = this->cap_init[3];
            }
            else if(case_tmp == "obj_correct"){
                this->listener_object();
                tf_point = this->obj_list[0];
            }
            else if(case_tmp == "obj_target"){
                this->listener_object();
                tf_point = this->obj_list[1];    
            }
            else if(case_tmp == "next_point"){
                this->listener_mode_point(case_tmp);
                tf_point = this->other_mode_point;
            }
            else if(case_tmp == "record_point"){
                this->listener_mode_point(case_tmp);
                tf_point = this->other_mode_point;
            }
            else if(case_tmp == "rotate_point"){
                this->listener_mode_point(case_tmp);
                tf_point = this->other_mode_point;
            }
            
            // sendCmd中的adjust_speed已经包含了：如果手势为10(stop),那么机器人直接停止
            this->sendCmd(ik_solver, tf_point);

            bool is_arrived_tmp = this->judge(tf_point);
            robot_msg.is_arrived = is_arrived_tmp ? 1 : 0;
            robot_msg.mode1_finish = (is_arrived_tmp && mode1_flag && mode1_finish) ? 1 : 0;
            robot_info_pub.publish(robot_msg);

            if(is_arrived_tmp || lhg == 10 || rhg == 10){
                break;
            }
        }

        rate.sleep();
    }
}

bool Robot::serverCallback(hg_hri::point2robot::Request &req, hg_hri::point2robot::Response &res){
    // 接受信息
    this->last_info.last_mode = (int)req.last_mode;
    this->last_info.last_mode1_idx = (int)req.last_mode1_idx;
    this->last_info.last_mode1_step = (int)req.last_mode1_step;

    this->last_info.last_mode2_direct = (int)req.last_mode2_direct;

    this->last_info.last_mode3_record = (int)req.last_mode3_record;
    
    this->last_info_update = true;

    res.result = "ok";
    return true;
}

void Robot::reset(){
    tmp_id_hg.clear();
    human_hg_vec.clear();

    human_num = 0;
    lhg = -1;
    rhg = -1;

    all_human_joint_tf.clear();

    mode1_finish = 0;
    mode1_flag = false;
    mode2_flag = false;
    mode3_flag = false;

    last_info.last_mode = -1;
    last_info.last_mode1_idx = 0;
    last_info.last_mode1_step = 0;
    last_info.last_mode2_direct = -1;
    last_info.last_mode3_record = 0;
    last_info_update = false;

    distance_vec.clear();
    obj_list.clear();
    cap_init.clear();

    ROS_INFO("status info have been reset ..");
}



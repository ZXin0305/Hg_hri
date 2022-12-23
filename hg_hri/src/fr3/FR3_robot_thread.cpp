#include<hg_hri/FR3_common.h>

// void Fr3Modules::run(){
//     // 初始化
//     robot->build_connection();   // 和服务端创建连接
//     gripper->loose();
//     sleep(0.2);
//     robot->reach_point("init_point");

//     ros::Rate rate(60);
//     ROS_INFO("robot is ready ..");

//     while(ros::ok()){
//         ros::spinOnce();
//         robot->decode_human_info();
//         int current_lhg = robot->lhg;
//         int current_rhg = robot->rhg;

//         // mode1
//         // right
//         if (current_rhg >= 1 && current_rhg <= 6){
//             this->track_next_right(current_rhg);   // 平移
//         }
//         else if (current_rhg == 7){  // 回到原点
//             robot->reach_point("init_point");
//             gripper->loose();
//         }
//         else if (current_rhg == 8){   //重置
//             robot->reset();
//         }
//         else if (current_rhg == 9){   //到达协助点
//             robot->reach_point("assist_point");
//         }
//         else if (current_rhg == 11){  // resume
//             int last_mode = robot->last_info.last_mode;
//             if (last_mode == 1){
//                 robot->mode1_step = robot->last_info.last_mode1_step;
//                 robot->mode1_idx = robot->last_info.last_mode1_idx;
//             }
//             else if (last_mode == 2){
//                 robot->mode2_direct = robot->last_info.last_mode2_direct;
//             }
//             else if (last_mode == 3){
//                 robot->mode3_idx = robot->last_info.last_mode3_idx;
//             }
//         }
//         else if (current_rhg == 12){   // 记录路径点
//             robot->record_waypoints();
//         }
//         else if (current_rhg == 13){   // 重现
//             robot->display_waypoints();
//         }
//         else if (current_rhg == 14){
//             gripper->clamp();
//         }
//         else if (current_rhg == 15){
//             gripper->loose();
//         }

//         // left
//         if (current_lhg >= 1 && current_lhg <= 5){
//             robot->mode1_idx = current_lhg;
//             this->catch_obj();
//         }
//         else if (current_lhg == 6 || current_lhg == 7){
//             this->track_next_left(current_rhg);
//         }
//         else if (current_lhg == 8 || current_lhg == 9){
//             if (current_lhg == 8)
//                 robot->adjust_speed("speed_down");
//             else
//                 robot->adjust_speed("speed_up");
//         }
//     }
// }

// void Fr3Modules::track_next_right(int num){
//     if (num == 1)
//         robot->move_direction("up");
//     else if (num == 2)
//         robot->move_direction("down");
//     else if (num == 3)
//         robot->move_direction("forward");
//     else if (num == 4)
//         robot->move_direction("backward");
//     else if (num == 5)
//         robot->move_direction("left"); 
//     else if (num == 6)
//         robot->move_direction("right");  
// }

// void Fr3Modules::track_next_left(int num){
//     if (num == 6)
//         robot->move_direction("right");
//     else if (num == 7)
//         robot->move_direction("left");
// }

// void Fr3Modules::catch_obj(){
//     switch (robot->mode1_step)
//     {
//     case 0:
//         robot->reach_point("init_point");
//         sleep(0.1);
//         gripper->loose();
//         robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//         break;
//     case 1:
//         robot->reach_point("cap_point");
//         robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//         break;
//     case 2:
//         robot->reach_point("obj_correct");
//         sleep(0.1);
//         gripper->loose();
//         robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//         break;
//     case 3:
//         robot->reach_point("obj_target");
//         gripper->clamp();
//         sleep(0.2);
//         robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//         break;
//     case 4:
//         robot->reach_point("obj_correct");
//         robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//         break;
//     case 5:
//         robot->reach_point("cap_point");
//         robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//         break;
//     case 6:
//         robot->mode1_finish = true;
//         robot->reach_point("init_point");
//         robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//         break;
//     default:
//         break;
//     }
// }

void Fr3Modules::test(){

    cout << "finish" << endl;
    // delete client_ptr;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "fr3_robot_thread");
    ros::NodeHandle nh;
    Fr3Modules module;
    
    // module.run();
    cout << "run test " << endl;
    module.test();
}
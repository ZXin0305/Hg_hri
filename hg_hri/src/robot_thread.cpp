#include<hg_hri/common.h>

void Modules::run(){

    //　初始化
    gripper->clamp();   // 爪子张开
    gripper->loose();
    sleep(0.2);
    // robot->reach_point("init_point");

    // ros::Rate rate(60);
    // ROS_INFO("robot is ready ..");

    /*
    设定中：mode1可以自己停止（但是是在完成当前任务后）
        　　mode2以及mode3则需要人为的停止
    */

//     while(ros::ok()){
//         ros::spinOnce();
//         robot->decode_human_info();
//         int current_lhg = robot->lhg;
//         int current_rhg = robot->rhg;

//         // mode1
//         if(robot->mode1_flag || ((current_lhg >= 0 && current_lhg <= 5) || current_lhg == 10) || current_rhg == 10){
//             if(current_lhg != 10 || current_rhg != 10){
//                 robot->mode1_flag = true;
//                 this->catch_obj();
//             }
//             else{
//                 robot->mode1_flag = false;
//             }
//         }
//         else if(robot->mode2_flag || (current_lhg == 10 || current_rhg == 10) || (current_rhg >= 1 && current_rhg <= 6)){
            
//             if(current_lhg != 10 || current_rhg != 10){
//                 robot->mode2_flag = true;
//                 this->track_next();
//             }
//             else{
//                 robot->mode2_flag = false;
//             }
//         }
//         else if(robot->mode3_flag || current_lhg == 10 || current_rhg == 10 || current_rhg == 13){
//             if(current_rhg == 13){
//                 robot->mode3_flag = true;
//                 this->track_record();
//             }
//             if(current_lhg == 10 || current_rhg == 10){
//                 robot->mode3_flag = false;
//             }
//             ROS_INFO("mode3 working ..");
//         }
//         else if(current_rhg == 8){  // 重置
//             robot->reset();
//         }
//         else if(current_rhg == 7){  // 回到原位置
//             robot->reach_point("init_point");
//             gripper->loose();
//         }
//         else if(current_rhg == 9){  // 到达指定的协助点
//             robot->reach_point("assist_point");
//         }
//         else if(current_rhg == 11){  // resume
//             if(robot->last_info_update){
//                 int last_mode = robot->last_info.last_mode;
//                 if(last_mode == 1){
//                     robot->mode1_flag = true;
//                     robot->mode1_step = robot->last_info.last_mode1_step;
//                 }
//                 else if(last_mode == 2){
//                     robot->mode2_flag = true;
//                 }
//                 else if(last_mode == 3){
//                     robot->mode3_flag = true;
//                 }
//             }
//         }
//         else if (current_lhg == 6 || current_rhg == 7){
//             robot->reach_point("rotate_point");
//         }
//         else if(current_rhg == 14){
//             gripper->clamp();
//         }
//         else if(current_rhg == 15){
//             gripper->loose();
//         }

//         rate.sleep();
//     }

// }

// // mode1
// void Modules::catch_obj(){
    
//     if(robot->mode1_flag){
//         switch (robot->mode1_step)
//         {
//         case 0:
//             robot->reach_point("init_point");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 1:
//             robot->reach_point("mid_point");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 2:
//             robot->reach_point("cap_point");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 3:
//             robot->reach_point("obj_correct");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 4:
//             robot->reach_point("obj_target");
//             gripper->clamp();
//             sleep(0.2);
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 5:
//             robot->reach_point("obj_correct");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 6:
//             robot->reach_point("cap_point");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 7:
//             robot->reach_point("mid_point");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             break;
//         case 8:
//             robot->mode1_finish = true;
//             robot->reach_point("init_point");
//             robot->mode1_step = (robot->mode1_step + 1) % robot->mode1_process;
//             robot->mode1_flag = false;
//             robot->mode1_finish = false;
//             break;
//         default:
//             break;
//         }
//     } 
}


// void Modules::track_next(){
//     robot->reach_point("next_point");
// }

// void Modules::track_record(){
//     robot->reach_point("record_point");  // 不用在这里循环，mode3_flag 可以保证循环
// }


int main(int argc, char** argv){
    ros::init(argc, argv, "robot_thread");
    ros::NodeHandle nh;
    Modules modules;
    modules.run();
    return 0;
}
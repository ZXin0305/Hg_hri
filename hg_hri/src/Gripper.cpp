#include<hg_hri/Gripper.h>
void Gripper::clamp(){
    sleep(delay_time_start);
    srv_move.request.position = 5;
    srv_move.request.speed = 50;
    srv_move.request.acceleration = 50;
    srv_move.request.torque = 1;
    srv_move.request.tolerance = 100;
    srv_move.request.waitFlag = true;
    std::cout<<"进行抓取........"<<std::endl;
    move_to_client.call(srv_move);
    // cout << move_to_client.call(srv_move) << endl;
    sleep(delay_time_end);
}

void Gripper::loose(){
    sleep(delay_time_start);
    srv_move.request.position = 80;
	srv_move.request.speed = 100;
	srv_move.request.acceleration = 50;
	srv_move.request.torque = 1;
	srv_move.request.tolerance = 100;
	srv_move.request.waitFlag = true;
    std::cout<<"放下.........."<<std::endl;
	move_to_client.call(srv_move);
    sleep(delay_time_end);
}


// int main(int argc, char** argv){
//     ros::init(argc, argv, "test_gripper");
//     ros::NodeHandle nh;

//     Gripper gg;
//     gg.clamp();

//     return 0;
// }

#include <iostream>
#include <ros/ros.h>
#include <byp80/ByStatus.h>
#include <byp80/ShutdownGripper.h>
#include <byp80/RestartGripper.h>
#include <byp80/MoveTo.h>
#include <byp80/GetStatus.h>
#include <byp80/GetCalibrated.h>
#include <byp80/CalibrateGripper.h>

using namespace std;

int main(int argc, char** argv)
{   
        cout << "testing .." << endl;
	ros::init(argc, argv, "byp80_test");
	ros::NodeHandle nh;
	ros::ServiceClient status_client = nh.serviceClient<byp80::GetStatus>("get_status");
	ros::ServiceClient calibrated_client = nh.serviceClient<byp80::CalibrateGripper>("calibrate_gripper");
	ros::ServiceClient move_to_client = nh.serviceClient<byp80::MoveTo>("move_to");
	byp80::GetStatus srv_getstatus;
	byp80::CalibrateGripper srv_calibarate;
	byp80::MoveTo srv_move;
	status_client.call(srv_getstatus);
	calibrated_client.call(srv_calibarate);
	move_to_client.call(srv_move);
	
	while(ros::ok()){
         cout << "testing .." << endl;
	//if(srv_calibarate.response.successful == true){
		srv_move.request.position = 20;
		srv_move.request.speed = 50;
		srv_move.request.acceleration = 50;
		srv_move.request.torque = 1;
		srv_move.request.tolerance = 100;
		srv_move.request.waitFlag = true;
		move_to_client.call(srv_move);

		srv_move.request.position = 50;
		srv_move.request.speed = 50;
		srv_move.request.acceleration = 50;
		srv_move.request.torque = 1;
		srv_move.request.tolerance = 100;
		srv_move.request.waitFlag = true;
		move_to_client.call(srv_move);
	//}
	}
	//cout << srv_getstatus.response.current_speed << endl;
	//cout << srv_getstatus.response.current_voltage << endl;
	return 0;
}

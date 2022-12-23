#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Backyard Automation, Inc.
# All rights reserved.
#

import rospy
import xmlrpclib
import time
#from std_msgs.msg import String
from byp80.msg import *
from byp80.srv import *
from pyModbusTCP.client import ModbusClient

def getStatus(bygripper):
    retStatus=ByStatus()
    retStatus.motor_error=0
    retStatus.current_position=0
    retStatus.current_speed=0
    retStatus.current_force=0
    retStatus.current_voltage=0
    retStatus.current_temperature=0
    try:
        retv=bygripper.read_holding_registers(60,8)
        if(retv):
            retStatus.communication_error=0
            retStatus.motor_error=retv[0]
            retStatus.current_position=retv[1]/100
            retStatus.current_speed=retv[2]/10
            retStatus.current_force=retv[3]/1000
            retStatus.current_voltage=retv[4]/10
            retStatus.current_temperature=retv[5]
            if(retv[6]==1):
                retStatus.completed=True
            else:
                retStatus.completed=False
            if(retv[7]==1):
                retStatus.calibrated=True
            else:
                retStatus.calibrated=False
            return retStatus
        else:
            retStatus.communication_error=1
            retStatus.completed=False
            retStatus.calibrated=False
            return retStatus
    except:
        retStatus.communication_error=1
        retStatus.completed=False
        retStatus.calibrated=False
        return retStatus

def moveTo(bygripper,cmd,pos,spd,acc,torq):
    try:
        pos=round(pos*100)
        spd=round(spd*10)
        acc=round(acc*10)
        torq=round(torq*1000)
        r0=bygripper.write_multiple_registers(50,[pos,spd,acc,torq,cmd])
        return r0
    except:
        raise


# by gripper server
def handleCalibrateGripper(req):
    global gripper
    global gripperStatus
    try:
        suc=gripper.write_multiple_registers(54,[10])
        gripperStatus=getStatus(gripper)
        while(not gripperStatus.completed):
            time.sleep(0.02)
            gripperStatus=getStatus(gripper)
        suc=gripper.write_multiple_registers(54,[0])
        gripperStatus=getStatus(gripper)
        return CalibrateGripperResponse(gripperStatus.calibrated)
    except Exception as e:
        print(e)
        return CalibrateGripperResponse(False)

def handleGetCalibrated(req):
    global gripper
    try:
        return GetCalibratedResponse(gripper.getCalibrated())
    except:
        return GetCalibratedResponse(False)

def handleGetStatus(req):
    #global gripperStatus
    try:
        stat=getStatus(gripper)
        return GetStatusResponse(stat.communication_error,stat.motor_error,stat.current_position,
                                 stat.current_speed,stat.current_force,stat.current_voltage,
                                 stat.current_temperature,stat.completed,stat.calibrated)
    except:
        return GetStatusResponse(1,0,0,0,0,0,0,0,0)

def handleMoveTo(req):
    global gripper
    global gripperStatus
    try:
        suc=gripper.write_multiple_registers(54,[0])
        suc=moveTo(gripper,1,req.position,req.speed,req.acceleration,req.torque)
        if(req.waitFlag):
            gripperStatus=getStatus(gripper)
            while(not gripperStatus.completed):
                time.sleep(0.02)  
                gripperStatus=getStatus(gripper)
            suc=gripper.write_multiple_registers(54,[0])
            if(abs(gripperStatus.current_position-req.position)<=req.tolerance):
                return MoveToResponse(True)
            else:
                return MoveToResponse(False)
        else:
            return MoveToResponse(True)
    except Exception as e:
        return MoveToResponse(False)

def handleRestartGripper(req):
    global gripper
    try:
        pass #gripper.restart()
    except:
        pass

def handleShutdownGripper(req):
    global gripper
    try:
        pass#gripper.shutdown()
    except:
        pass

def byDriver_server():
    rospy.init_node('byp80_driver', anonymous=True)
    s_calib_gripper=rospy.Service('calibrate_gripper',CalibrateGripper,handleCalibrateGripper)
    s_get_calibrated=rospy.Service('get_calibrated',GetCalibrated,handleGetCalibrated)
    s_get_status=rospy.Service('get_status',GetStatus,handleGetStatus)
    s_moveto=rospy.Service('move_to',MoveTo,handleMoveTo)
    s_restart=rospy.Service('restart_gripper',RestartGripper,handleRestartGripper)
    s_shutdown=rospy.Service('shutdown_gripper',ShutdownGripper,handleShutdownGripper)
    

if __name__ == '__main__':
    #print(sys.argv[1])
    #print(len(sys.argv))
    try:
        if len(sys.argv)==4:
            ip=str(sys.argv[1])
            print('using Gripper IP', ip)
            global gripper
            global gripperStatus
            gripperStatus=ByStatus()
            gripper=ModbusClient(host=ip, port=502, unit_id=1, auto_open=True, auto_close=True)
            gripper.open()
            byDriver_server()
            rospy.spin()
        else:
            print("No IP assigned")
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Backyard Automation, Inc.
# All rights reserved.
#

import threading
import rospy
import xmlrpclib
#from std_msgs.msg import String
from byp80.msg import *
from byp80.srv import *
#from numpy import *

global gripper
global gripperStatus

#gripperStatus=ByStatus()
gripper=xmlrpclib.ServerProxy("http://192.168.137.9:33000/")
pub = rospy.Publisher('by_status',ByStatus, queue_size=10)
rospy.init_node('byp80_driver', anonymous=True)



def fillByStatusMsg(inputdata):
    retv=ByStatus()
    retv.communication_error=inputdata[0]
    retv.motor_error=inputdata[1]
    retv.current_position=inputdata[2]
    retv.current_speed=inputdata[3]
    retv.current_force=inputdata[4]
    retv.current_voltage=inputdata[5]
    retv.current_temperature=inputdata[6]
    retv.calibrated=inputdata[7]
    retv.current_stroke=inputdata[8]
    return retv

# status publisher
def statusTalker():
    global gripper
    global gripperStatus
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            retv=fillByStatusMsg(gripper.getStatus())
            gripperStatus=retv
            #rospy.loginfo(retv)  
            pub.publish(retv)
        except Exception as e: 
            print(e)
        finally:
            rate.sleep()

# by gripper server
def handleCalibrateGripper(req):
    global gripper
    try:
        return CalibrateGripperResponse(gripper.calibrateGripper())
    except Exception as e:
        print(e)
        return CalibrateGripperResponse(False)

def handleGetCalibrated(req):
    global gripper
    try:
        return GetCalibratedResponse(gripper.getCalibrated())
    except:
        pass

def handleGetStatus(req):
    global gripperStatus
    try:
        return GetStatusResponse(gripperStatus)
    except:
        pass

def handleMoveTo(req):
    global gripper
    try:
        return MoveToResponse(gripper.moveTo(req.position,req.speed,req.acceleration,req.torque,req.tolerance,req.waitFlag))
    except Exception as e:
        print(e)
        return MoveToResponse(False)

def handleRestartGripper(req):
    global gripper
    try:
        gripper.restart()
    except:
        pass

def handleShutdownGripper(req):
    global gripper
    try:
        gripper.shutdown()
    except:
        pass

def byDriver_server():
    #rospy.init_node('by_driver_server')
    s_calib_gripper=rospy.Service('calibrate_gripper',CalibrateGripper,handleCalibrateGripper)
    s_get_calibrated=rospy.Service('get_calibrated',GetCalibrated,handleGetCalibrated)
    s_get_status=rospy.Service('get_status',GetStatus,handleGetStatus)
    s_moveto=rospy.Service('move_to',MoveTo,handleMoveTo)
    s_restart=rospy.Service('restart_gripper',RestartGripper,handleRestartGripper)
    s_shutdown=rospy.Service('shutdown_gripper',ShutdownGripper,handleShutdownGripper)
    

if __name__ == '__main__':
    try:

        t=threading.Thread(target=statusTalker)
        t.start()
        byDriver_server()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Backyard Automation, Inc.
# All rights reserved.
#


import rospy
import xmlrpclib
import time
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


# status publisher
def statusTalker():
    global gripper
    pub = rospy.Publisher('by_status',ByStatus, queue_size=10)
    rospy.init_node('bystatus',anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            retv=getStatus(gripper)
            #rospy.loginfo(retv)  
            pub.publish(retv)
        except Exception as e: 
            print(e)
        finally:
            rate.sleep()

if __name__ == '__main__':
    try:
        if len(sys.argv)==4:
            ip=str(sys.argv[1])
            global gripper
            gripper=ModbusClient(host=ip, port=502, unit_id=1, auto_open=True, auto_close=True)
            gripper.open()
            statusTalker()
        else:
            print("No IP Assigned")
    except rospy.ROSInterruptException:
        pass

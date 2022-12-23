#!/usr/bin/env python

import sys
import rospy
from byp80.srv import *
from byp80.msg import *
import time

def test():
    #rospy.wait_for_service('shutdown_gripper')
    try:
        calib_gripper = rospy.ServiceProxy('calibrate_gripper', CalibrateGripper)
	print("1")
        moveTo = rospy.ServiceProxy('move_to', MoveTo)
	print("2")
        getStatus=rospy.ServiceProxy('get_status',GetStatus)
	print("3")
        a=calib_gripper()
	a=moveTo(50,200,500,1,100,True)
        print(a)
        if(a.successful):
		while(1):
            		print(moveTo(5,200,500,1,100,True))
            		a=moveTo(50,200,500,1,100,True)
        time.sleep(1)
        print(getStatus())
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    test()

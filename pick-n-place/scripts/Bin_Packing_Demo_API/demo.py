#!/usr/bin/env python
import sys 
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import math

from box_class import *
from database import *
from function import *
import move as mv
import vision_server as vs
import eef_server as es

# Initialization
pub_pose = rospy.Publisher('moveAPI_cmd', Float32MultiArray, queue_size=1000)
pub_plan = rospy.Publisher('plan_type', Float32MultiArray, queue_size=1)
rospy.init_node('demo', anonymous=True)

# Functions
def syn():
    plantype = Float32MultiArray()
    plantype.data = [0, 2]
    pub_plan.publish(plantype)
    rospy.sleep(1)
    plantype.data = [0, 1]
    pub_plan.publish(plantype)

def debug():
    es.off()
    mv.initial_pose()
    sys.exit()

# Synchronization
syn()

# Initial Setup
mv.initial_pose()

# Plan
rospy.logwarn("START!")
given_bin_list, box_list = database("data.xlsx") # Load database
i = box_list[0]
num = 0

while not rospy.is_shutdown():
    if vs.pick(i):
        rospy.logwarn("Object detected!")
        if mv.grasp_pose():
            if mv.pick(i):
                if mv.place(i):
                    num += 1
                    rospy.loginfo("Finish: %d", num)
                else:
                    debug()
            else:
                debug()
        else:
            debug()
        #raw_input("Continue...")
        mv.initial_pose()
    else:
        rospy.logwarn("No object detected!")
        break
rospy.logwarn("FINISH!")

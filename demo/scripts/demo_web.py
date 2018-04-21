#!/usr/bin/env python
import sys 
import rospy
from std_msgs.msg import Float32MultiArray, Int8
from geometry_msgs.msg import Point
import math

from box_class import *
from database import *
from function import *
import move as mv
import vision_server as vs
import eef_server as es

signal = 0

def callback(data):
    global signal
    signal = data.data

# Initialization
pub_pose = rospy.Publisher('moveAPI_cmd', Float32MultiArray, queue_size=1000)
pub_plan = rospy.Publisher('plan_type', Float32MultiArray, queue_size=1)
rospy.Subscriber("states", Int8, callback)
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

def wait(sig):
    global signal
    while sig is 0:
        rospy.sleep(0.1)
        sig = signal

# Synchronization
syn()
rospy.sleep(5.0)

# Initial Setup
mv.initial_pose()

# Plan
rospy.logwarn("START!")
given_bin_list, box_list = database("/home/wcy/catkin_ws/src/demo/scripts/data.xlsx") # Load database
i = box_list[0]
num = 0

while not rospy.is_shutdown():
    if signal is -1:
        mv.initial_pose()
        signal = 0
    wait(signal)
    if signal is 1:
        if vs.pick(i):
            rospy.logwarn("Object detected!")
            wait(signal)
            if signal is -1:
                continue
            if mv.grasp_pose():
                wait(signal)
                if signal is -1:
                    continue
                if mv.pick(i):
                    wait(signal)
                    if signal is -1:
                        continue
                    if mv.place(i):
                        wait(signal)
                        if signal is -1:
                            continue
                        num += 1
                        rospy.loginfo("Finish: %d", num)
                    else:
                        debug()
                else:
                    debug()
            else:
                debug()
            #raw_input("Continue...")
            wait(signal)
            if signal is -1:
                continue
            mv.wait_pose()
            wait(signal)
            if signal is -1:
                continue
        else:
            rospy.logwarn("No object detected!")
            break
mv.initial_pose()
rospy.logwarn("FINISH!")

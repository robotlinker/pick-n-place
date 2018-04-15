#!/usr/bin/env python
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

# Parameters


# Functions
def syn():
    plantype = Float32MultiArray()
    plantype.data = [0, 2]
    pub_plan.publish(plantype)
    rospy.sleep(1)
    plantype.data = [0, 1]
    pub_plan.publish(plantype)

def BPplanner(given_bin_list, box_list):
    # Goal pose of box
    for i in box_list:
        i.rot_goal = 1
        i.goal_position = [0.6, -0.2, -0.5]

# Synchronization
syn()

# Initial Setup
mv.initial_pose()

# Plan
rospy.logwarn("START!")
given_bin_list, box_list = database("data.xlsx") # Load database
BPplanner(given_bin_list, box_list)
i = box_list[0]

while not rospy.is_shutdown():
    mv.detection_pose()
    if cs.pick(i) is 0:
        print "No Object detected!"
        continue
    else:
        print "Object detected!"
    mv.grasp_pose()
    mv.pick(i)
    mv.place(i)
    raw_input("Continue...")
rospy.logwarn("FINISH!")



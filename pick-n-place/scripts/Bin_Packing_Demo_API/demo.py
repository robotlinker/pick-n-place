#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import math

from move import *
from box_class import *
from database import *
from function import *
import camera_server as cs
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

def initial_pose():
    rospy.logwarn("INITIAL POSE")  
    pub_pose.publish(joint_cmd([0, 0, 0, 0, 0, 0]))
    rospy.sleep(7)

def grasp_pose():  
    rospy.logwarn("GRASP POSE")  
    pub_pose.publish(joint_cmd([-30, -31.46, 112.90, 59.11, 91.27, -53.87]))

def detection_pose():
    rospy.logwarn("DETECTION POSE")  
    pub_pose.publish(joint_cmd([-47.2, -32.0, 78.6, 6.7, 110.2, -78.7]))
    rospy.sleep(7)

def BPplanner(given_bin_list, box_list):
    # Goal pose of box
    for i in box_list:
        i.rot_goal = 1
        i.goal_position = [0.6, -0.2, -0.5]

def pick(box_i):
    target = box_i.pick()
    pub_pose.publish(target[0])
    rospy.sleep(1)
    pub_pose.publish(target[2])
    rospy.sleep(5)
    es.on()
    rospy.sleep(3)
    pub_pose.publish(target[3])
    rospy.sleep(5)    

def place(box_i):
    target = box_i.place(given_bin_list[0])
    pub_pose.publish(target[0])
    rospy.sleep(1)
    pub_pose.publish(target[2])
    rospy.sleep(7)
    es.off()
    pub_pose.publish(target[3])
    rospy.sleep(2)  

# Synchronization
syn()

# Initial Setup
initial_pose()

# Plan
rospy.logwarn("START!")
given_bin_list, box_list = database("data.xlsx") # Load database
BPplanner(given_bin_list, box_list)
i = box_list[0]

while not rospy.is_shutdown():
    detection_pose()
    if cs.pick(i) is 0:
        print "No Object detected!"
        continue
    else:
        print "Object detected!"
    grasp_pose()
    pick(i)
    place(i)
    raw_input("Continue...")
rospy.logwarn("FINISH!")



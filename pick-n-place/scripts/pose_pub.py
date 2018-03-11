#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import math

# Initialization
pub_pose = rospy.Publisher('moveAPI_cmd', Float32MultiArray, queue_size=1000)
pub_plan = rospy.Publisher('plan_type', Float32MultiArray, queue_size=1)
rospy.init_node('aubo_pose', anonymous=True)

# Parameters
EPS = 3

# Functions
def syn():

    plantype = Float32MultiArray()
    plantype.data = [0, 2]
    pub_plan.publish(plantype)
    rospy.sleep(1)
    plantype.data = [0, 1]
    pub_plan.publish(plantype)

def joint_cmd(joints):

    target = Float32MultiArray()
    joint_values = [0] + joints
    target.data = joint_values
    pub_pose.publish(target)

def rotation_cmd(angle, axis = "Z"):
    
    target = Float32MultiArray()
    if axis is "X":
        target_values = [2] + [math.radians(angle), 1, 0, 0]
    elif axis is "Y":
        target_values = [2] + [math.radians(angle), 0, 1, 0]
    elif axis is "Z":
        target_values = [2] + [math.radians(angle), 0, 0, 1]
    target.data = target_values
    pub_pose.publish(target)

def pose_cmd(pose):
    
    target = Float32MultiArray()
    pose_value = [pose.x, pose.y, pose.z]
    target_value = [4] + pose_value
    target.data = target_value
    pub_pose.publish(target)

def init_pose():

    joint_cmd([0,0,0,0,0,0])

def deg2rad(joints):

    return [math.radians(i) for i in joints]

def pick_and_place():

    # Move to A
    A_up = [-68.2,-46.4,85.9,42.8,89.6,-36.4]
    A_down = [-68.2,-68.11,80.7,59.2,89.6,-36.4]
    joint_cmd(deg2rad(A_up))
    rospy.sleep(2)    
    joint_cmd(deg2rad(A_down))
    joint_cmd(deg2rad(A_up))
    rospy.sleep(5)
    # Move to B
    B_up = [35.1,-50.2,78.6,39.3,89.6,-37.2]
    B_down = [35.1,-78.8,67.9,57.2,89.6,-37.2]
    joint_cmd(deg2rad(B_up))
    rospy.sleep(2)
    joint_cmd(deg2rad(B_down))
    joint_cmd(deg2rad(B_up))    
    rospy.sleep(5)


# Synchronization
syn()

# Plan
#init_pose()
#for i in range(0, EPS):
#    pick_and_place()
target_pose = Point()
target_pose.x = 0.3
target_pose.y = -0.6
target_pose.z = 0.2
pose_cmd(target_pose)
rotation_cmd(90 ,"X")




#init_pose()



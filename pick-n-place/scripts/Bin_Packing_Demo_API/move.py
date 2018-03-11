#!/usr/bin/env python
from std_msgs.msg import Float32MultiArray
import math

def deg2rad(joints):

    return [math.radians(i) for i in joints]

def joint_cmd(joints):

    target = Float32MultiArray()
    joint_values = [0] + deg2rad(joints)
    target.data = joint_values
    return target

def rotation_cmd(angle, axis = "Z"):
    
    target = Float32MultiArray()
    if axis is "X":
        target_values = [2] + [math.radians(angle), 1, 0, 0]
    elif axis is "Y":
        target_values = [2] + [math.radians(angle), 0, 1, 0]
    elif axis is "Z":
        target_values = [2] + [math.radians(angle), 0, 0, 1]
    target.data = target_values
    return target

def pose_cmd(pose):
    
    target = Float32MultiArray()
    pose_value = [pose.position.x, pose.position.y, pose.position.z]
    target_value = [4] + pose_value
    target.data = target_value
    return target

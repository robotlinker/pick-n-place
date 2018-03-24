#!/usr/bin/env python
import rospy
import math
from aubo_msgs.srv import *
import eef_server as es


def move(typ, data):
    
    rospy.wait_for_service("aubo_driver/move")
    Move_proxy = rospy.ServiceProxy("aubo_driver/move", Move)
    req = MoveRequest(typ, data)
    res = Move_proxy.call(req)


def deg2rad(joints):

    return [math.radians(i) for i in joints]

def joint_cmd(joints):

    move(0, deg2rad(joints))


def rotation_cmd(angle, axis = "Z"):
    
    if axis is "X":
        move(2, [math.radians(angle), 1, 0, 0])
    elif axis is "Y":
        move(2, [math.radians(angle), 0, 1, 0])
    elif axis is "Z":
        move(2, [math.radians(angle), 0, 0, 1])

def pose_cmd(pose):
    
    move(4, [pose.position.x, pose.position.y, pose.position.z])

def initial_pose():
    rospy.logwarn("INITIAL POSE")  
    joint_cmd([0, 0, 0, 0, 0, 0])

def grasp_pose():  
    rospy.logwarn("GRASP POSE")  
    joint_cmd([-30, -31.46, 112.90, 59.11, 91.27, -53.87])

def detection_pose():
    rospy.logwarn("DETECTION POSE")  
    joint_cmd([-47.2, -32.0, 78.6, 6.7, 110.2, -78.7])

def pick(box_i):
    target = box_i.pick()
    pose_cmd(target[0])
    pose_cmd(target[2])
    es.on()
    pose_cmd(target[3])

def place(box_i):
    target = box_i.place()
    pose_cmd(target[0])
    pose_cmd(target[2])
    es.off()
    pose_cmd(target[3])

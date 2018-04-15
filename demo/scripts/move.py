#!/usr/bin/env python
import rospy
from math import *
from aubo_msgs.srv import *
import eef_server as es

def feedback(state):
    if state is 1:
        rospy.loginfo("DONE!")
    elif state is -1:
        rospy.logerr("Fail!")
    return state

def move(typ, data):
    
    rospy.wait_for_service("aubo_driver/move")
    Move_proxy = rospy.ServiceProxy("aubo_driver/move", Move)
    req = MoveRequest(typ, data)
    res = Move_proxy.call(req)
    return res.success

def deg2rad(joints):

    return [radians(i) for i in joints]

def joint_cmd(joints):

    return move(0, deg2rad(joints))

def pose_cmd(pose):
    
    return move(4, [pose.position.x, pose.position.y, pose.position.z])

def initial_pose():
    rospy.logwarn("INITIAL POSE")  
    if joint_cmd([0, 0, 0, 0, 0, 0]):
        return feedback(1)
    else:
        return feedback(-1)

def grasp_pose():  
    rospy.logwarn("GRASP POSE")  
    if joint_cmd([59.39, -21.76, 77.36, 9.18, 87.78, 4.42]):
        return feedback(1)
    else:
        return feedback(-1)

def wait_pose():  
    rospy.logwarn("WAIT POSE")  
    if joint_cmd([32.42, 14.49, 62.79, -16.89, 78.80, -87.13]):
        return feedback(1)
    else:
        return feedback(-1)

def pick(box_i):
    rospy.logwarn("PICKING...") 
    target = box_i.pick()
    if pose_cmd(target[0]):
        if pose_cmd(target[2]):
            es.on()
            if pose_cmd(target[3]):
                return feedback(1)
            else:
                return feedback(-1)
        else:
            return feedback(-1)
    else:
        return feedback(-1)

def place(box_i):
    rospy.logwarn("PLACING...") 
    target = box_i.place()
    if pose_cmd(target[0]):
        if pose_cmd(target[2]):
            es.off()
            if pose_cmd(target[3]):
                return feedback(1)
            else:
                return feedback(-1)
        else:
            return feedback(-1)
    else:
        return feedback(-1)

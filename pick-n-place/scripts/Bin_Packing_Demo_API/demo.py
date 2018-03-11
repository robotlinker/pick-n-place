#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import math

from move import *
from box_class import *
from database import *
from function import *

# Initialization
pub_pose = rospy.Publisher('moveAPI_cmd', Float32MultiArray, queue_size=1000)
pub_plan = rospy.Publisher('plan_type', Float32MultiArray, queue_size=1)
rospy.init_node('demo', anonymous=True)

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

def init_pose():  
    pub_pose.publish(joint_cmd([-52.6, -41.9, 91.05, 42.6, 89.9, 37.0]))

def end_pose():
    target = Pose()
    target.position.x = 0.3
    target.position.y = -0.6
    target.position.z = 0.2
    pub_pose.publish(pose_cmd(target))

def vision(obj):
    # Initial pose of box
    object_pose = Pose()
    object_pose.position.x = 0.3
    object_pose.position.y = -0.6
    object_pose.position.z = -0.5
    object_pose.orientation = RPY2q([0, 0, 0])
    obj.current_pose = object_pose
    rospy.sleep(5)

def BPplanner(given_bin_list, box_list):
    # Goal pose of box
    for i in box_list:
        i.rot_goal = 1
        i.goal_position = [0.6, -0.2, -0.5]

def pick(box_i):
    target = box_i.pick()
    pub_pose.publish(target[0])
    rospy.sleep(1)
    #pub_pose.publish(target[1])
    pub_pose.publish(target[2])
    rospy.sleep(1)
    pub_pose.publish(target[3])
    rospy.sleep(2)    

def place(box_i):
    target = box_i.place(given_bin_list[0])
    pub_pose.publish(target[0])
    rospy.sleep(1)
    #pub_pose.publish(target[1])
    pub_pose.publish(target[2])
    rospy.sleep(1)
    pub_pose.publish(target[3])
    rospy.sleep(1)  

# Synchronization
syn()

# Initial Setup
init_pose()

# Plan
rospy.logwarn("START!")
given_bin_list, box_list = database("data.xlsx") # Load database
BPplanner(given_bin_list, box_list)
i = box_list[0]
vision(i)
pick(i)
#place(i)
end_pose()
rospy.logwarn("FINISH!")

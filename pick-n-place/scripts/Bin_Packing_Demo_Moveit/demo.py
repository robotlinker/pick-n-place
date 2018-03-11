#!/usr/bin/env python
# ROS
import rospy
import moveit_commander

# Msg
from geometry_msgs.msg import Pose, PoseStamped

# Manipulation
import env

# Bin packing
from box_class import *
from database import *

# Parameters


# ROS node
rospy.init_node("demo", anonymous=True)

# Initiation of Moveit
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander() 
arm = moveit_commander.MoveGroupCommander("arm_group")
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(1)

# Functions
def init_pose():
    arm.set_joint_value_target([0,0,0,0,0,0])

# Env setup
init_pose()
env.init(scene)

# Plan
rospy.logwarn("START!")
given_bin_list, box_list = database("data.xlsx") # Load database
for i in box_list:
    box_pose = PoseStamped() # get box_pose from vision system
    env.generate_box(scene, i, box_pose) # generate a box in env.
    i.pick(scene, arm)
    
    i.place(scene, arm, given_bin_list[0])
    env.clear(scene, i)
init_pose()
rospy.logwarn("FINISH!")





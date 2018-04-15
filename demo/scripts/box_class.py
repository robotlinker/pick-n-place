#!/usr/bin/python
import rospy
from geometry_msgs.msg import Pose
from function import *
import copy


eef_length = 0.235
pre = 0.15
post = 0.15

class box_class:
    def __init__(self, name, position, size, rot_goal=1):
        self.id = str(name)
        self.size = size
        self.width = size[0]
        self.length = size[1]
        self.height = size[2]
        self.rot_goal = rot_goal
        self.goal_position = position
        self.current_pose = Pose()
        self.goal_pose = Pose()
        if self.rot_goal is 1:
            self.inc_goal = [size[0], size[1], size[2]]
        elif self.rot_goal is 2:
            self.inc_goal = [size[1], size[0], size[2]]


    def pick(self):
        pre_grasp_pose = copy.deepcopy(self.current_pose)
        pre_grasp_pose.position.z += eef_length + pre
        grasp_pose = copy.deepcopy(self.current_pose)
        grasp_pose.position.z += eef_length
        post_grasp_pose = copy.deepcopy(self.current_pose)
        post_grasp_pose.position.z += eef_length + post
        theta = q2RPY(self.current_pose.orientation)[2]
        return [pre_grasp_pose, theta, grasp_pose, post_grasp_pose]

    def place(self):
        self.goal_pose.position.x = 0.358
        self.goal_pose.position.y = -0.369
        self.goal_pose.position.z = 0.05
        pre_place_pose = copy.deepcopy(self.goal_pose)
        pre_place_pose.position.z += pre
        place_pose = copy.deepcopy(self.goal_pose)
        post_place_pose = copy.deepcopy(self.goal_pose)
        post_place_pose.position.z += post
        if self.rot_goal is 1:
            theta = 0
        elif self.rot_goal is 2:
            theta = 90
        return [pre_place_pose, theta, place_pose, post_place_pose]

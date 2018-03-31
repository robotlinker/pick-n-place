#!/usr/bin/python
import rospy
from geometry_msgs.msg import Pose
from function import *
import copy


eef_length = 0.23
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
        self.px = position[0]
        self.py = position[1]
        self.pz = position[2]
        self.vol = size[0]*size[1]*size[2]
        self.current_pose = Pose()
        self.goal_pose = Pose()
        if self.rot_goal is 1:
            self.inc_goal = [size[0], size[1], size[2]]
        elif self.rot_goal is 2:
            self.inc_goal = [size[1], size[0], size[2]]

    def generate_goal_pose(self, given_bin):
        self.goal_pose.position.x = self.goal_position[0]# + 0.5*self.inc_goal[0] + given_bin.goal_position[0] - 0.5*given_bin.width
        self.goal_pose.position.y = self.goal_position[1]# + 0.5*self.inc_goal[1] + given_bin.goal_position[1] - 0.5*given_bin.length
        self.goal_pose.position.z = self.goal_position[2] + 0.3# + 0.5*self.inc_goal[2] + given_bin.goal_position[2]

    def pick(self):
        rospy.logwarn("Picking Box: %s", self.id)
        pre_grasp_pose = copy.deepcopy(self.current_pose)
        pre_grasp_pose.position.z += eef_length + pre
        grasp_pose = copy.deepcopy(self.current_pose)
        grasp_pose.position.z += eef_length
        post_grasp_pose = copy.deepcopy(self.current_pose)
        post_grasp_pose.position.z += eef_length + post
        theta = q2RPY(self.current_pose.orientation)[2]
        return [pre_grasp_pose, theta, grasp_pose, post_grasp_pose]

    def place(self):
        rospy.logwarn("Placing Box: %s", self.id)
        self.goal_pose.position.x = 0.55
        self.goal_pose.position.y = 0.245
        self.goal_pose.position.z = -0.15
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

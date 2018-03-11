#!/usr/bin/python
from geometry_msgs.msg import PoseStamped, Pose
from function import *
import rospy
import env
import grasp_pose as gp
import place_pose as pp

eef_length = 0.2
pre = 0.03
post = 0.1

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
        self.goal_pose = PoseStamped()
        self.rpy_goal = [0, 0, 0]
        if self.rot_goal is 1:
            self.inc_goal = [size[0], size[1], size[2]]
            self.rpy_goal = [0, 0, 0]
        elif self.rot_goal is 2:
            self.inc_goal = [size[1], size[0], size[2]]
            self.rpy_goal = [0, 0, 90]
        self.pre_grasp_pose = Pose()
        self.pre_grasp_pose.position.z = eef_length + pre
        self.pre_grasp_pose.orientation = CalOrientation(0, 180, 0)
        self.grasp_pose = Pose()
        self.grasp_pose.position.z = eef_length
        self.grasp_pose.orientation = CalOrientation(0, 180, 0)
        self.post_grasp_pose = Pose()
        self.post_grasp_pose.position.z = eef_length + post
        self.post_grasp_pose.orientation = CalOrientation(0, 180, 0)
        self.pre_place_pose = Pose()
        self.pre_place_pose.position.z = eef_length + pre
        self.pre_place_pose.orientation = CalOrientation(0, 180, 0)
        self.place_pose = Pose()
        self.place_pose.position.z = eef_length
        self.place_pose.orientation = CalOrientation(0, 180, 0)
        self.post_place_pose = Pose()
        self.post_place_pose.position.z = eef_length + post
        self.post_place_pose.orientation = CalOrientation(0, 180, 0)

    def generate_goal_pose(self, given_bin):
        self.goal_pose.header.frame_id = "world"  
        self.goal_pose.pose.position.x = self.goal_position[0] + 0.5*self.inc_goal[0] + given_bin.goal_position[0] - 0.5*given_bin.width
        self.goal_pose.pose.position.y = self.goal_position[1] + 0.5*self.inc_goal[1] + given_bin.goal_position[1] - 0.5*given_bin.length
        self.goal_pose.pose.position.z = self.goal_position[2] + 0.5*self.inc_goal[2] + given_bin.goal_position[2]
        self.goal_pose.pose.orientation = CalOrientation(self.rpy_goal[0], self.rpy_goal[1], self.rpy_goal[2])

    def update_current_pose(self, scene):
        self.current_pose = scene.get_object_poses([self.id])[self.id]

    def pick(self, scene, group):
        rospy.logwarn("Picking Box: %s", self.id)
        group.set_pose_target(gp.PreGraspPoseCal(self))
        group.go()
        group.set_pose_target(gp.GraspPoseCal(self))
        group.go()
        scene.attach_box("ee_Link", self.id)
        group.set_pose_target(gp.PostGraspPoseCal(self))
        while scene.get_attached_objects() is {}:
            rospy.sleep(0.1)        
        else:
            group.go()
            rospy.logwarn("Done!")

    def place(self, scene, group, given_bin):
        rospy.logwarn("Placing Box: %s", self.id)
        self.generate_goal_pose(given_bin)
        group.set_pose_target(pp.PrePlacePoseCal(self))
        group.go()
        group.set_pose_target(pp.PlacePoseCal(self))
        group.go()
        scene.remove_attached_object("ee_Link", self.id)
        group.set_pose_target(pp.PostPlacePoseCal(self))
        group.go()
        rospy.logwarn("Done!")
        
        

import rospy
import geometry_msgs.msg
import tf
import function as func
import math
import numpy as np

def PrePlacePoseCal(target):

    return func.GetTransformPose(target.goal_pose.pose, target.pre_place_pose)

def PlacePoseCal(target):

    return func.GetTransformPose(target.goal_pose.pose, target.place_pose)

def PostPlacePoseCal(target):

    return func.GetTransformPose(target.goal_pose.pose, target.post_place_pose)

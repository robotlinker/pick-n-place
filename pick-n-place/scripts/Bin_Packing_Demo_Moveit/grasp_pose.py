import rospy
import geometry_msgs.msg
import tf
import function as func
import math
import numpy as np

def PreGraspPoseCal(target):

    return func.GetTransformPose(target.current_pose, target.pre_grasp_pose)

def GraspPoseCal(target):

    return func.GetTransformPose(target.current_pose, target.grasp_pose)

def PostGraspPoseCal(target):

    return func.GetTransformPose(target.current_pose, target.post_grasp_pose)

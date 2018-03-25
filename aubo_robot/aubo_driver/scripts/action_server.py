#!/usr/bin/env python
import rospy
import time
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from aubo_msgs.srv import *

def CB(goal):

    rospy.wait_for_service("aubo_driver/moveit")
    Moveit_proxy = rospy.ServiceProxy("aubo_driver/moveit", Moveit)
    req = MoveitRequest(goal)
    res = Moveit_proxy.call(req)
    result = FollowJointTrajectoryResult()
    if result.error_code is 0:
        server.set_succeeded(result)
    else:
        server.set_aborted(result)


rospy.init_node('aubo_action_server')
server = actionlib.SimpleActionServer('/follow_joint_trajectory', FollowJointTrajectoryAction, CB, False)
server.start()
rospy.spin()

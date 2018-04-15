import rospy
from perception_msgs.srv import *
from function import *
from geometry_msgs.msg import PoseStamped
from math import *

def REC_ERROR():
    print "recognition error!"

def pick(box):

    flag = 0
    rospy.logwarn("DETECTING...")
    while flag is 0:
        rospy.wait_for_service("Square_detection")
        SquareDetection_proxy = rospy.ServiceProxy("Square_detection", SquareDetection)
        req = SquareDetectionRequest(1)
        res = SquareDetection_proxy.call(req)
        box.current_pose = res.Square_pose.pose
        if isnan(box.current_pose.position.x):
            REC_ERROR()
            continue
        if box.current_pose.position.x < 0:
            REC_ERROR()
            continue
        if box.current_pose.position.y < 0:
            REC_ERROR()
            continue
        if box.current_pose.position.z > 0.1:
            REC_ERROR()
            continue
        flag = res.flag

    if flag is 1:
        print box.current_pose
        return True
    else:
        return False

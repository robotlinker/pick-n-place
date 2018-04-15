import rospy
from perception_msgs.srv import *
from function import *
from geometry_msgs.msg import PoseStamped
import copy

def pick(box):

    flag = 0
    rospy.logwarn("Get %s Pose (3D) ...", box.id)
    while flag is 0:
        rospy.wait_for_service("Square_detection")
        SquareDetection_proxy = rospy.ServiceProxy("Square_detection", SquareDetection)
        req = SquareDetectionRequest(1)
        res = SquareDetection_proxy.call(req)
        flag = res.flag
        box.current_pose = res.Square_pose

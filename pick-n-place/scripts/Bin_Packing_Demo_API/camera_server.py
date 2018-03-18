import rospy
from perception_msgs.srv import *
from function import *
from geometry_msgs.msg import PoseStamped
import copy

def pick(box):
    
    rospy.logwarn("Get %s Pose ...", box.id)
    rospy.wait_for_service("Get_pose")
    GetPose_proxy = rospy.ServiceProxy("Get_pose", GetPose)
    req = GetPoseRequest(1)
    res = GetPose_proxy.call(req)
    
    box.current_pose.position.x = 0.41 + res.dx - 0.03
    box.current_pose.position.y = -0.37 + res.dy + 0.04
    box.current_pose.position.z = -0.3
    box.current_pose.orientation = RPY2q([0, 0, 0])

    return res.flag

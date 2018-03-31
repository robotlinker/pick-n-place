import rospy
from perception_msgs.srv import *
from function import *
from geometry_msgs.msg import PoseStamped
import copy

def pick(box):
    
    rospy.logwarn("Get %s Pose (2D) ...", box.id)
    rospy.wait_for_service("Get_pose")
    GetPose_proxy = rospy.ServiceProxy("Get_pose", GetPose)
    req = GetPoseRequest(1)
    res = GetPose_proxy.call(req)
    
    box.current_pose.position.x = 0.41 + res.dx - 0.03
    box.current_pose.position.y = -0.37 + res.dy + 0.04
    box.current_pose.position.z = -0.3
    box.current_pose.orientation = RPY2q([0, 0, 0])

    rospy.logwarn("Get %s Pose (3D) ...", box.id)
    rospy.wait_for_service("Square_detection")
    SquareDetection_proxy = rospy.ServiceProxy("Square_detection", SquareDetection)
    req2 = SquareDetectionRequest(1)
    res2 = SquareDetection_proxy.call(req2)

    box.current_pose.position.z = res2.Square_pose.pose.position.z

    return res.flag

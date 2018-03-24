#
import rospy
from aubo_msgs.srv import *

def on():
    
    rospy.wait_for_service("aubo_driver/set_io")
    SetIO_proxy = rospy.ServiceProxy("aubo_driver/set_io", SetIO)
    req = SetIORequest(1, 0, 1)
    res = SetIO_proxy.call(req)

def off():
    
    rospy.wait_for_service("aubo_driver/set_io")
    SetIO_proxy = rospy.ServiceProxy("aubo_driver/set_io", SetIO)
    req = SetIORequest(1, 0, 0)
    res = SetIO_proxy.call(req)

#!/usr/bin/env python
import rospy
import os
import sys
from std_msgs.msg import Int8

aubo_flag = 0
kinect2_flag = 0
ensenso_flag = 0


def aubo_callback(data):
    global aubo_flag
    if data.data is 1 and aubo_flag is 0:
        os.system("./Aubo_real.sh&")
        aubo_flag = 1
    if data.data is -1 and aubo_flag is 1:
        os.system("rosnode kill /rviz")
        os.system("rosnode kill /robot_state_publisher")
        os.system("rosnode kill /move_group")
        os.system("rosnode kill /aubo_moveit_action_server")
        os.system("rosnode kill /aubo_driver")
        aubo_flag = 0

def kinect2_callback(data):
    global kinect2_flag
    if data.data is 1 and kinect2_flag is 0:
        os.system("./kinect.sh&")
        kinect2_flag = 1
    if data.data is -1 and kinect2_flag is 1:
        os.system("rosnode kill /kinect2")
        kinect2_flag = 0

def ensenso_callback(data):
    global ensenso_flag
    if data.data is 1 and ensenso_flag is 0:
        os.system("./ensenso.sh&")
        ensenso_flag = 1
    if data.data is -1 and ensenso_flag is 1:
        os.system("rosnode kill /request_data")
        os.system("rosnode kill /ensenso_to_world")
        os.system("rosnode kill /ensenso_camera_node")
        ensenso_flag = 0
    

rospy.init_node('launch_controller', anonymous=True)

os.system("sudo chmod +x *.sh")
os.system("./web.sh&")
os.system("./tf.sh&")

rospy.Subscriber("rosbridge_launch", Int8, rosbridge_callback)
rospy.Subscriber("tf2_launch", Int8, tf2_callback)
rospy.Subscriber("aubo_launch", Int8, aubo_callback)
rospy.Subscriber("kinect2_launch", Int8, kinect2_callback)
rospy.Subscriber("ensenso_launch", Int8, ensenso_callback)

rospy.spin()

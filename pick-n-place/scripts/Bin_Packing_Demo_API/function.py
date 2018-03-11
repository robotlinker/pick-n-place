import tf.transformations
import geometry_msgs.msg
import math
import numpy as np


def RPY2q(rpy):
    
    p = geometry_msgs.msg.Pose()
    q = tf.transformations.quaternion_from_euler(math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2]))
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    
    return p.orientation

def q2RPY(q):
    (r,p,y)=tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    return [r,p,y]

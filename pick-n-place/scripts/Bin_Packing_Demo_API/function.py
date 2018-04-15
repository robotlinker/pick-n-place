import tf.transformations
from geometry_msgs.msg import *
from math import *


def RPY2q(rpy):
    Q = Quaternion()
    q = tf.transformations.quaternion_from_euler(radians(rpy[0]), radians(rpy[1]), radians(rpy[2]))
    Q.orientation.x = q[0]
    Q.orientation.y = q[1]
    Q.orientation.z = q[2]
    Q.orientation.w = q[3]
    
    return Q

def q2RPY(q):
    (r,p,y) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    return [r,p,y]

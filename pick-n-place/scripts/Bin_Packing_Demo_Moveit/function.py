import tf.transformations
import geometry_msgs.msg
import math
import numpy as np


def CalOrientation(roll, pitch, yaw):
    
    p = geometry_msgs.msg.Pose()
    q = tf.transformations.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    
    return p.orientation

def CalRPY(q):
    (r,p,y)=tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    return [r,p,y]

def GetTransformPose(M1, M2):

    trans1 = tf.transformations.translation_matrix([M1.position.x, M1.position.y, M1.position.z])
    rot1 = tf.transformations.quaternion_matrix([M1.orientation.x, M1.orientation.y, M1.orientation.z, M1.orientation.w])
    trans2 = tf.transformations.translation_matrix([M2.position.x, M2.position.y, M2.position.z])
    rot2 = tf.transformations.quaternion_matrix([M2.orientation.x, M2.orientation.y, M2.orientation.z, M2.orientation.w])
    T1 = tf.transformations.concatenate_matrices(trans1, rot1)
    T2 = tf.transformations.concatenate_matrices(trans2, rot2)
    T = T1.dot(T2)
    p = geometry_msgs.msg.Pose()
    p.position.x = tf.transformations.translation_from_matrix(T)[0]
    p.position.y = tf.transformations.translation_from_matrix(T)[1]
    p.position.z = tf.transformations.translation_from_matrix(T)[2]
    p.orientation.x = tf.transformations.quaternion_from_matrix(T)[0]
    p.orientation.y = tf.transformations.quaternion_from_matrix(T)[1]
    p.orientation.z = tf.transformations.quaternion_from_matrix(T)[2]
    p.orientation.w = tf.transformations.quaternion_from_matrix(T)[3]

    return p




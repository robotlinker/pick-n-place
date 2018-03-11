from geometry_msgs.msg import PoseStamped
import rospy
   

def init(scene):
       
    scene.remove_world_object()
    rospy.logwarn("Environment Initialization Done!")

def clear(scene, obj):

    scene.remove_world_object(obj.id)
    rospy.logwarn("Environment Updated!")

def generate_box(scene, box_i, p):

    p.header.frame_id = "world"
    p.pose.position.x = 0
    p.pose.position.y = -0.4
    p.pose.position.z = 0.5*box_i.size[2] - 0.5
    scene.add_box(box_i.id, p, box_i.size)
    rospy.sleep(0.5)    
    box_i.update_current_pose(scene)
